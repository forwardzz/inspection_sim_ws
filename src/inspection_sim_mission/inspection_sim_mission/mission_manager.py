import math
import os

import yaml

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robot_mission_utils.inspection_planner import (
    preview_current_order,
    validate_mission_points,
)
from robot_mission_utils.grid_map import GridMap
from robot_monitor_interfaces.msg import InspectionPoint
from robot_monitor_interfaces.srv import ConfirmInspectionPoints, Localize, StartNavigation

from .mission_regions import (
    InspectionRegion,
    assign_path_headings,
    generate_points_for_region,
    generate_region_points,
    regions_from_yaml,
    regions_to_yaml_data,
    sweep_positions,
)
from .qos import latched_qos
from .robot_config import (
    ACTION_NAVIGATE_TO_POSE,
    FRAME_MAP,
    INSPECTION_REGIONS_PATH,
    SERVICE_ABORT_MISSION,
    SERVICE_CLEAR_INSPECTION_REGIONS,
    SERVICE_CLEAR_RVIZ_POINTS,
    SERVICE_CONFIRM_INSPECTION_POINTS,
    SERVICE_LOAD_INSPECTION_REGIONS,
    SERVICE_LOCALIZE_ROBOT,
    SERVICE_SAVE_INSPECTION_REGIONS,
    SERVICE_SET_REGION_MODE,
    SERVICE_START_NAVIGATION,
    SERVICE_UNDO_LAST_INSPECTION_REGION,
    SERVICE_UNDO_LAST_RVIZ_POINT,
    TOPIC_AMCL_POSE,
    TOPIC_CLICKED_POINT,
    TOPIC_CMD_VEL_NAV,
    TOPIC_GOAL_POSE,
    TOPIC_MAP,
    TOPIC_MISSION_GOAL_POSE,
    TOPIC_MISSION_POINTS_MARKERS,
    TOPIC_MISSION_PREVIEW_PATH,
    TOPIC_MISSION_STATUS,
    TOPIC_ODOM,
)
from .ros_utils import inspection_point_to_pose, make_inspection_point, quat_to_yaw


class MissionManager(Node):
    DEFAULT_WAYPOINT_PAUSE_SEC = 2.0
    MAX_WAYPOINT_PAUSE_SEC = 60.0
    RELAXED_GOAL_DISTANCE_M = 0.20

    def __init__(self):
        super().__init__("mission_manager")

        self.sweep_spacing = float(self.declare_parameter("sweep_spacing", 0.50).value)
        self.region_margin = float(self.declare_parameter("region_margin", 0.15).value)
        self.regions_path = str(
            self.declare_parameter(
                "inspection_regions_path",
                INSPECTION_REGIONS_PATH,
            ).value
        )

        self.current_odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.current_map_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.have_odom = False
        self.have_map_pose = False
        self.have_map = False
        self.map_msg = None
        self.confirmed_points = []
        self.rviz_points = []
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
        self.rviz_recompute_throttle_sec = 0.5
        self.last_rviz_recompute_time = 0.0
        self.region_mode = False
        self.pending_region_corner = None
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.goal_handle = None
        self.mission_active = False
        self.mission_points = []
        self.mission_index = 0
        self.mission_source = ""
        self.mission_wait_timer = None
        self.mission_run_id = 0
        self.mission_waypoint_pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None
        self.direct_goal_handle = None
        self.direct_nav_active = False
        self.last_direct_feedback_log_time = 0.0

        self.create_subscription(Odometry, TOPIC_ODOM, self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, TOPIC_AMCL_POSE, self._amcl_pose_cb, 10
        )
        self.create_subscription(OccupancyGrid, TOPIC_MAP, self._map_cb, latched_qos())
        self.create_subscription(PointStamped, TOPIC_CLICKED_POINT, self._clicked_point_cb, 10)
        self.create_subscription(
            PoseStamped, TOPIC_MISSION_GOAL_POSE, self._goal_pose_cb, 10
        )
        self.create_subscription(PoseStamped, TOPIC_GOAL_POSE, self._direct_goal_pose_cb, 10)

        self.preview_pub = self.create_publisher(Path, TOPIC_MISSION_PREVIEW_PATH, latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, TOPIC_MISSION_POINTS_MARKERS, latched_qos())
        self.cmd_vel_nav_pub = self.create_publisher(Twist, TOPIC_CMD_VEL_NAV, 10)
        self.mission_status_pub = self.create_publisher(String, TOPIC_MISSION_STATUS, 10)

        self.create_service(Localize, SERVICE_LOCALIZE_ROBOT, self._handle_localize)
        self.create_service(
            ConfirmInspectionPoints,
            SERVICE_CONFIRM_INSPECTION_POINTS,
            self._handle_confirm_points,
        )
        self.create_service(StartNavigation, SERVICE_START_NAVIGATION, self._handle_start_navigation)
        self.create_service(Trigger, SERVICE_CLEAR_RVIZ_POINTS, self._handle_clear_rviz_points)
        self.create_service(SetBool, SERVICE_SET_REGION_MODE, self._handle_set_region_mode)
        self.create_service(
            Trigger,
            SERVICE_CLEAR_INSPECTION_REGIONS,
            self._handle_clear_inspection_regions,
        )
        self.create_service(
            Trigger,
            SERVICE_SAVE_INSPECTION_REGIONS,
            self._handle_save_inspection_regions,
        )
        self.create_service(
            Trigger,
            SERVICE_LOAD_INSPECTION_REGIONS,
            self._handle_load_inspection_regions,
        )
        self.create_service(Trigger, SERVICE_ABORT_MISSION, self._handle_abort_mission)
        self.create_service(
            Trigger,
            SERVICE_UNDO_LAST_INSPECTION_REGION,
            self._handle_undo_last_inspection_region,
        )
        self.create_service(
            Trigger,
            SERVICE_UNDO_LAST_RVIZ_POINT,
            self._handle_undo_last_rviz_point,
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, ACTION_NAVIGATE_TO_POSE)
        self.mission_position_timer = self.create_timer(
            0.25, self._check_mission_early_transition
        )
        self.get_logger().info("Mission manager ready")

    def _odom_cb(self, msg):
        self.current_odom["x"] = msg.pose.pose.position.x
        self.current_odom["y"] = msg.pose.pose.position.y
        self.current_odom["theta"] = quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def _amcl_pose_cb(self, msg):
        self.current_map_pose["x"] = msg.pose.pose.position.x
        self.current_map_pose["y"] = msg.pose.pose.position.y
        self.current_map_pose["theta"] = quat_to_yaw(msg.pose.pose.orientation)
        self.have_map_pose = True
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_rviz_recompute_time >= self.rviz_recompute_throttle_sec:
            self.last_rviz_recompute_time = now_sec
            if self.rviz_points and not self.mission_active:
                self._recompute_rviz_plan()
            elif self.inspection_regions and self.have_map and self.map_msg is not None:
                self._publish_rviz_plan_visuals()

    def _map_cb(self, msg):
        self.have_map = True
        self.map_msg = msg
        if self.rviz_points:
            self._recompute_rviz_plan()
        elif self.inspection_regions:
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()

    def _clicked_point_cb(self, msg):
        frame_id = msg.header.frame_id or FRAME_MAP
        if frame_id != FRAME_MAP:
            self.get_logger().warn(
                f"Ignoring RViz point in frame {frame_id}; use RViz Publish Point with Fixed Frame=map"
            )
            return

        if self.region_mode:
            self._add_region_corner(float(msg.point.x), float(msg.point.y))
            return

        point = make_inspection_point(
            f"RVIZ_{len(self.rviz_points) + 1}",
            float(msg.point.x),
            float(msg.point.y),
            0.0,
        )
        valid, reason = self._validate_points_for_setting(
            self.rviz_points + [point],
            f"RViz mission point {point.point_name}",
        )
        if not valid:
            self._warn_safety(reason)
            return

        self.rviz_points.append(point)
        self.get_logger().info(
            f"Added RViz mission point {point.point_name} at ({point.x:.2f}, {point.y:.2f})"
        )
        self._recompute_rviz_plan()

    def _add_region_corner(self, x, y):
        if self.pending_region_corner is None:
            self.pending_region_corner = (x, y)
            self.get_logger().info(
                f"Stored first region corner at ({x:.2f}, {y:.2f}); click the opposite corner"
            )
            self._publish_rviz_plan_visuals()
            return

        first_x, first_y = self.pending_region_corner
        self.pending_region_corner = None
        region = InspectionRegion(
            name=f"REGION_{len(self.inspection_regions) + 1}",
            min_x=min(first_x, x),
            min_y=min(first_y, y),
            max_x=max(first_x, x),
            max_y=max(first_y, y),
        )
        self.inspection_regions.append(region)
        self._recompute_region_preview()
        if self.region_generation_error is not None:
            self.inspection_regions.pop()
            message = f"Inspection region {region.name} rejected: {self.region_generation_error}"
            self._warn_safety(message)
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()
            return
        if not self.region_preview_points:
            self.inspection_regions.pop()
            message = (
                f"Inspection region {region.name} rejected: no safe sweep waypoint can be generated. "
                f"Check region_margin={self.region_margin:.2f}m."
            )
            self._warn_safety(message)
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()
            return
        valid, reason = self._validate_region_entry(region)
        if not valid:
            self.inspection_regions.pop()
            self._warn_safety(reason)
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()
            return

        self.get_logger().info(
            f"Added inspection region {region.name}: "
            f"({region.min_x:.2f}, {region.min_y:.2f}) to ({region.max_x:.2f}, {region.max_y:.2f})"
        )
        self._publish_rviz_plan_visuals()

    def _goal_pose_cb(self, msg):
        frame_id = msg.header.frame_id or FRAME_MAP
        if frame_id != FRAME_MAP:
            self.get_logger().warn(
                f"Ignoring RViz mission heading pose in frame {frame_id}; use Fixed Frame=map"
            )
            return

        if not self.rviz_points:
            self.get_logger().warn(
                "Received a Mission Heading pose but there are no RViz mission points yet"
            )
            return

        target_x = float(msg.pose.position.x)
        target_y = float(msg.pose.position.y)
        closest_index = None
        closest_distance = None
        for index, point in enumerate(self.rviz_points):
            distance = math.hypot(point.x - target_x, point.y - target_y)
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_index = index

        if closest_index is None or closest_distance is None or closest_distance > 0.75:
            self.get_logger().warn(
                "Ignoring RViz mission heading pose because it is not close to any stored mission point"
            )
            return

        point = self.rviz_points[closest_index]
        point.theta = quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"Updated heading for {point.point_name} to {math.degrees(point.theta):.1f} deg"
        )
        self._publish_rviz_plan_visuals()

    def _direct_goal_pose_cb(self, msg):
        frame_id = msg.header.frame_id or FRAME_MAP
        if frame_id != FRAME_MAP:
            self.get_logger().warn(
                f"Ignoring RViz navigation goal in frame {frame_id}; use Fixed Frame=map"
            )
            return

        if self.mission_active:
            self.get_logger().warn(
                "Ignoring RViz navigation goal because an inspection mission is already running"
            )
            return

        if not self.have_map_pose:
            self.get_logger().warn(
                "Ignoring RViz navigation goal because AMCL pose is unavailable. Set the initial pose first."
            )
            return

        if not self.have_map or self.map_msg is None:
            self.get_logger().warn("Ignoring RViz navigation goal because no /map data has been received")
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 action server /navigate_to_pose is not ready")
            return

        goal_point = make_inspection_point(
            "RVIZ_NAV_GOAL",
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            quat_to_yaw(msg.pose.orientation),
        )
        validation = validate_mission_points(
            self.map_msg,
            (self.current_map_pose["x"], self.current_map_pose["y"]),
            [goal_point],
            min_start_distance_m=0.05,
        )
        if not validation.valid:
            self.get_logger().warn(f"RViz navigation goal rejected: {validation.message}")
            return

        if self.direct_goal_handle is not None:
            try:
                self.direct_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel previous RViz navigation goal: {exc}")

        goal = NavigateToPose.Goal()
        goal.pose = self._inspection_point_to_pose(goal_point)

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self._direct_feedback_cb,
        )
        send_goal_future.add_done_callback(self._direct_goal_response_cb)
        self.direct_nav_active = True
        self.get_logger().info(
            f"RViz navigation goal sent to ({goal_point.x:.2f}, {goal_point.y:.2f}, "
            f"{math.degrees(goal_point.theta):.1f} deg)"
        )

    def _recompute_rviz_plan(self):
        self.rviz_ordered_points = list(self.rviz_points)
        self.rviz_preview_path = []
        if not self.rviz_points:
            self._publish_rviz_plan_visuals()
            return

        if self.map_msg is not None and self.have_map_pose:
            preview = preview_current_order(
                self.map_msg,
                (self.current_map_pose["x"], self.current_map_pose["y"]),
                self.rviz_points,
            )
            if preview:
                self.rviz_preview_path = list(preview.preview_path)
        self._publish_rviz_plan_visuals()

    def _recompute_region_preview(self):
        self.region_preview_points = self._generate_region_points()

    def _generate_region_points(self):
        result = generate_region_points(
            self.inspection_regions,
            self.sweep_spacing,
            self.region_margin,
        )
        self.region_generation_error = result.first_error
        for warning in result.warnings:
            self.get_logger().warn(warning)
        return result.points

    def _generate_points_for_region(self, region):
        return generate_points_for_region(
            region,
            self.sweep_spacing,
            self.region_margin,
        )

    def _validate_points_for_setting(self, points, context):
        if not self.have_map_pose:
            return (
                False,
                f"{context} rejected: AMCL pose unavailable. Set the initial pose before adding mission points.",
            )
        if not self.have_map or self.map_msg is None:
            return False, f"{context} rejected: no /map data has been received."

        validation = validate_mission_points(
            self.map_msg,
            (self.current_map_pose["x"], self.current_map_pose["y"]),
            points,
        )
        if not validation.valid:
            return False, f"{context} rejected: {validation.message}"
        return True, validation.message

    def _validate_region_entry(self, region):
        width = region.max_x - region.min_x
        height = region.max_y - region.min_y
        min_dim = max(self.sweep_spacing + self.region_margin * 2.0, 0.10)
        if width < min_dim or height < min_dim:
            return (
                False,
                f"Region {region.name} rejected: dimensions ({width:.2f}x{height:.2f}m) "
                f"are too small. Minimum is {min_dim:.2f}m per side.",
            )
        if not self.have_map or self.map_msg is None:
            self.get_logger().info(
                f"Region {region.name} accepted without map validation (no /map yet)"
            )
            return True, "Region added (map not yet available)"
        grid_map = GridMap.from_occupancy_grid(self.map_msg)
        corners = [
            (region.min_x, region.min_y),
            (region.max_x, region.min_y),
            (region.max_x, region.max_y),
            (region.min_x, region.max_y),
        ]
        for cx, cy in corners:
            gx, gy = grid_map.world_to_grid(cx, cy)
            if not grid_map.in_bounds(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: corner ({cx:.2f}, {cy:.2f}) "
                    f"is outside map bounds.",
                )
            if not grid_map.is_valid(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: corner ({cx:.2f}, {cy:.2f}) "
                    f"is in an obstacle or unknown area.",
                )
        return True, f"Region {region.name} validated"

    def _publish_mission_status(self, message, safety=False):
        msg = String()
        msg.data = f"[SAFETY] {message}" if safety else message
        self.mission_status_pub.publish(msg)

    def _warn_safety(self, message):
        self.get_logger().warn(message)
        self._publish_mission_status(message, safety=True)

    @staticmethod
    def _sweep_positions(start, end, spacing):
        return sweep_positions(start, end, spacing)

    def _assign_path_headings(self, points):
        assign_path_headings(points)

    def _publish_rviz_plan_visuals(self):
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = FRAME_MAP
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()
        next_marker_id = 1

        for index, point in enumerate(self.rviz_ordered_points, start=1):
            marker_id = next_marker_id
            next_marker_id += 2

            sphere = Marker()
            sphere.header.frame_id = FRAME_MAP
            sphere.header.stamp = stamp
            sphere.ns = "mission_points"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = point.x
            sphere.pose.position.y = point.y
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.14
            sphere.scale.y = 0.14
            sphere.scale.z = 0.14
            sphere.color.r = 1.0
            sphere.color.g = 0.62
            sphere.color.b = 0.11
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = FRAME_MAP
            text.header.stamp = stamp
            text.ns = "mission_labels"
            text.id = marker_id + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point.x
            text.pose.position.y = point.y + 0.10
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color.r = 1.0
            text.color.g = 0.85
            text.color.b = 0.30
            text.color.a = 1.0
            text.text = str(index)
            marker_array.markers.append(text)

        if self.pending_region_corner is not None:
            corner_id = next_marker_id
            next_marker_id += 3

            region_index = len(self.inspection_regions) + 1
            cx, cy = self.pending_region_corner
            corner_sphere = Marker()
            corner_sphere.header.frame_id = FRAME_MAP
            corner_sphere.header.stamp = stamp
            corner_sphere.ns = "inspection_region_pending"
            corner_sphere.id = corner_id
            corner_sphere.type = Marker.SPHERE
            corner_sphere.action = Marker.ADD
            corner_sphere.pose.position.x = cx
            corner_sphere.pose.position.y = cy
            corner_sphere.pose.position.z = 0.08
            corner_sphere.pose.orientation.w = 1.0
            corner_sphere.scale.x = 0.18
            corner_sphere.scale.y = 0.18
            corner_sphere.scale.z = 0.18
            corner_sphere.color.r = 0.47
            corner_sphere.color.g = 0.24
            corner_sphere.color.b = 0.72
            corner_sphere.color.a = 0.95
            marker_array.markers.append(corner_sphere)

            pending_label = Marker()
            pending_label.header.frame_id = FRAME_MAP
            pending_label.header.stamp = stamp
            pending_label.ns = "inspection_region_pending"
            pending_label.id = corner_id + 1
            pending_label.type = Marker.TEXT_VIEW_FACING
            pending_label.action = Marker.ADD
            pending_label.pose.position.x = cx
            pending_label.pose.position.y = cy + 0.20
            pending_label.pose.position.z = 0.40
            pending_label.pose.orientation.w = 1.0
            pending_label.scale.z = 0.16
            pending_label.color.r = 0.80
            pending_label.color.g = 0.70
            pending_label.color.b = 0.95
            pending_label.color.a = 1.0
            pending_label.text = f"\u533a\u57df {region_index} \u7b2c1\u89d2"
            marker_array.markers.append(pending_label)

            dot = Marker()
            dot.header.frame_id = FRAME_MAP
            dot.header.stamp = stamp
            dot.ns = "inspection_region_pending"
            dot.id = corner_id + 2
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = cx
            dot.pose.position.y = cy
            dot.pose.position.z = 0.02
            dot.pose.orientation.w = 1.0
            dot.scale.x = 0.06
            dot.scale.y = 0.06
            dot.scale.z = 0.01
            dot.color.r = 0.80
            dot.color.g = 0.70
            dot.color.b = 0.95
            dot.color.a = 0.90
            marker_array.markers.append(dot)

        region_base_id = next_marker_id + 100
        for index, region in enumerate(self.inspection_regions, start=1):
            self._append_region_markers(marker_array, stamp, index, region, region_base_id)

        self.marker_pub.publish(marker_array)

        path_msg = Path()
        path_msg.header.frame_id = FRAME_MAP
        path_msg.header.stamp = stamp
        if self.inspection_regions and self.have_map_pose and self.have_map and self.map_msg is not None:
            preview_xy = self._compute_inter_region_preview_path()
        elif self.inspection_regions:
            preview_xy = []
        else:
            preview_xy = self.rviz_preview_path
        for x, y in preview_xy:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.preview_pub.publish(path_msg)

    def _compute_inter_region_preview_path(self):
        if not self.region_preview_points or not self.have_map or self.map_msg is None or not self.have_map_pose:
            return []
        start_xy = (self.current_map_pose["x"], self.current_map_pose["y"])
        preview = preview_current_order(
            self.map_msg,
            start_xy,
            self.region_preview_points,
        )
        if preview:
            return preview.preview_path
        return []

    def _append_region_markers(self, marker_array, stamp, index, region, base_id):
        width = region.max_x - region.min_x
        height = region.max_y - region.min_y
        region_marker_id = base_id + (index - 1) * 20

        fill = Marker()
        fill.header.frame_id = FRAME_MAP
        fill.header.stamp = stamp
        fill.ns = "inspection_region_fill"
        fill.id = region_marker_id
        fill.type = Marker.CUBE
        fill.action = Marker.ADD
        fill.pose.position.x = (region.min_x + region.max_x) / 2.0
        fill.pose.position.y = (region.min_y + region.max_y) / 2.0
        fill.pose.position.z = 0.01
        fill.pose.orientation.w = 1.0
        fill.scale.x = max(width, 0.01)
        fill.scale.y = max(height, 0.01)
        fill.scale.z = 0.005
        fill.color.r = 0.47
        fill.color.g = 0.24
        fill.color.b = 0.72
        fill.color.a = 0.18
        marker_array.markers.append(fill)

        border = Marker()
        border.header.frame_id = FRAME_MAP
        border.header.stamp = stamp
        border.ns = "inspection_region_border"
        border.id = region_marker_id + 1
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.pose.orientation.w = 1.0
        border.scale.x = 0.045
        border.color.r = 0.65
        border.color.g = 0.35
        border.color.b = 0.85
        border.color.a = 0.95
        corners = [
            (region.min_x, region.min_y),
            (region.max_x, region.min_y),
            (region.max_x, region.max_y),
            (region.min_x, region.max_y),
            (region.min_x, region.min_y),
        ]
        for cx, cy in corners:
            border.points.append(self._marker_point(cx, cy, 0.06))
        marker_array.markers.append(border)

        for ci, (cx, cy) in enumerate(corners[:4]):
            corner_dot = Marker()
            corner_dot.header.frame_id = FRAME_MAP
            corner_dot.header.stamp = stamp
            corner_dot.ns = "inspection_region_corners"
            corner_dot.id = region_marker_id + 2 + ci
            corner_dot.type = Marker.SPHERE
            corner_dot.action = Marker.ADD
            corner_dot.pose.position.x = cx
            corner_dot.pose.position.y = cy
            corner_dot.pose.position.z = 0.07
            corner_dot.pose.orientation.w = 1.0
            corner_dot.scale.x = 0.10
            corner_dot.scale.y = 0.10
            corner_dot.scale.z = 0.10
            corner_dot.color.r = 0.65
            corner_dot.color.g = 0.35
            corner_dot.color.b = 0.85
            corner_dot.color.a = 0.95
            marker_array.markers.append(corner_dot)

        label = Marker()
        label.header.frame_id = FRAME_MAP
        label.header.stamp = stamp
        label.ns = "inspection_region_labels"
        label.id = region_marker_id + 6
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = (region.min_x + region.max_x) / 2.0
        label.pose.position.y = (region.min_y + region.max_y) / 2.0
        label.pose.position.z = 0.45
        label.pose.orientation.w = 1.0
        label.scale.z = 0.20
        label.color.r = 0.85
        label.color.g = 0.78
        label.color.b = 0.95
        label.color.a = 1.0
        label.text = f"\u533a\u57df {index} ({width:.1f}\u00d7{height:.1f})"
        marker_array.markers.append(label)

        region_points = self._generate_points_for_region(region)
        if region_points:
            region_scan = Marker()
            region_scan.header.frame_id = FRAME_MAP
            region_scan.header.stamp = stamp
            region_scan.ns = "inspection_region_scan"
            region_scan.id = region_marker_id + 7
            region_scan.type = Marker.LINE_STRIP
            region_scan.action = Marker.ADD
            region_scan.pose.orientation.w = 1.0
            region_scan.scale.x = 0.04
            region_scan.color.r = 0.05
            region_scan.color.g = 0.72
            region_scan.color.b = 0.42
            region_scan.color.a = 0.95
            for rx, ry in region_points:
                region_scan.points.append(self._marker_point(rx, ry, 0.08))
            marker_array.markers.append(region_scan)

    @staticmethod
    def _marker_point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _handle_localize(self, _request, response):
        pose = self.current_map_pose if self.have_map_pose else self.current_odom
        response.current_x = pose["x"]
        response.current_y = pose["y"]
        response.current_theta = pose["theta"]
        response.success = self.have_map_pose
        if self.have_map_pose:
            response.message = (
                f"AMCL pose available at ({pose['x']:.2f}, {pose['y']:.2f}, "
                f"{math.degrees(pose['theta']):.1f} deg)"
            )
        elif self.have_odom:
            response.message = (
                "AMCL pose unavailable, returning odom pose. "
                "Set the initial pose in RViz before mission start."
            )
        else:
            response.message = "No odometry received yet."
        return response

    def _handle_confirm_points(self, request, response):
        self.confirmed_points = []
        for point in request.points:
            confirmed = InspectionPoint()
            confirmed.point_name = point.point_name
            confirmed.x = point.x
            confirmed.y = point.y
            confirmed.theta = point.theta
            confirmed.is_confirmed = True
            self.confirmed_points.append(confirmed)

        response.success = True
        response.message = f"Stored {len(self.confirmed_points)} mission points"
        self.get_logger().info(response.message)
        return response

    def _handle_clear_rviz_points(self, _request, response):
        count = len(self.rviz_points)
        self.rviz_points = []
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Cleared {count} RViz mission points"
        self.get_logger().info(response.message)
        return response

    def _handle_set_region_mode(self, request, response):
        self.region_mode = bool(request.data)
        self.pending_region_corner = None
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = (
            "Region mode enabled. Use RViz Publish Point to click two opposite rectangle corners."
            if self.region_mode
            else "Region mode disabled. RViz Publish Point will add normal mission points."
        )
        self.get_logger().info(response.message)
        return response

    def _handle_clear_inspection_regions(self, _request, response):
        count = len(self.inspection_regions)
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.pending_region_corner = None
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Cleared {count} inspection region(s)"
        self.get_logger().info(response.message)
        return response

    def _handle_save_inspection_regions(self, _request, response):
        data = regions_to_yaml_data(
            self.inspection_regions,
            self.sweep_spacing,
            self.region_margin,
        )
        try:
            directory = os.path.dirname(self.regions_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.regions_path, "w", encoding="utf-8") as output:
                yaml.safe_dump(data, output, sort_keys=False)
        except Exception as exc:
            response.success = False
            response.message = f"Failed to save inspection regions: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f"Saved {len(self.inspection_regions)} inspection region(s) to {self.regions_path}"
        )
        self.get_logger().info(response.message)
        return response

    def _handle_load_inspection_regions(self, _request, response):
        try:
            with open(self.regions_path, "r", encoding="utf-8") as input_file:
                data = yaml.safe_load(input_file) or {}
            regions = self._regions_from_yaml(data)
        except Exception as exc:
            response.success = False
            response.message = f"Failed to load inspection regions: {exc}"
            self.get_logger().error(response.message)
            return response

        self.inspection_regions = regions
        self.pending_region_corner = None
        self.sweep_spacing = float(data.get("sweep_spacing", self.sweep_spacing))
        self.region_margin = float(data.get("region_margin", self.region_margin))
        self._recompute_region_preview()
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Loaded {len(regions)} inspection region(s) from {self.regions_path}"
        self.get_logger().info(response.message)
        return response

    def _regions_from_yaml(self, data):
        return regions_from_yaml(data)

    def _publish_zero_cmd(self):
        stop = Twist()
        self.cmd_vel_nav_pub.publish(stop)

    def _handle_abort_mission(self, _request, response):
        self._publish_zero_cmd()
        aborted = []
        errors = []
        mission_was_active = self.mission_active
        self.mission_run_id += 1

        if self.mission_wait_timer is not None:
            self._clear_mission_wait_timer()
            aborted.append("mission wait")

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                errors.append(f"mission: {exc}")
            else:
                aborted.append("mission")
            self.goal_handle = None

        if mission_was_active and not aborted:
            aborted.append("mission")
        self.mission_active = False
        self._clear_mission_state()

        if self.direct_goal_handle is not None:
            try:
                self.direct_goal_handle.cancel_goal_async()
            except Exception as exc:
                errors.append(f"direct navigation: {exc}")
            else:
                aborted.append("direct navigation")
            self.direct_goal_handle = None
            self.direct_nav_active = False

        if errors:
            response.success = False
            response.message = "Failed to cancel " + "; ".join(errors)
            self.get_logger().error(response.message)
            return response

        if aborted:
            response.success = True
            response.message = f"Abort requested for {', '.join(aborted)} and stop command sent"
            self.get_logger().warn(response.message)
        self._publish_mission_status(response.message)
        return response

    def _handle_undo_last_inspection_region(self, _request, response):
        if self.pending_region_corner is not None:
            cx, cy = self.pending_region_corner
            self.pending_region_corner = None
            self._publish_rviz_plan_visuals()
            response.success = True
            response.message = f"Undone pending region corner at ({cx:.2f}, {cy:.2f})"
            self.get_logger().info(response.message)
            return response
        if not self.inspection_regions:
            response.success = False
            response.message = "No inspection regions to undo"
            return response
        removed = self.inspection_regions.pop()
        self._recompute_region_preview()
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Undone region {removed.name}. {len(self.inspection_regions)} region(s) remaining"
        self.get_logger().info(response.message)
        return response

    def _handle_undo_last_rviz_point(self, _request, response):
        if not self.rviz_points:
            response.success = False
            response.message = "No RViz points to undo"
            return response
        removed = self.rviz_points.pop()
        self._recompute_rviz_plan()
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Undone point {removed.point_name}. {len(self.rviz_points)} point(s) remaining"
        self.get_logger().info(response.message)
        return response

    def _handle_start_navigation(self, request, response):
        if self.mission_active:
            response.success = False
            response.message = "A mission is already running"
            return response
        if self.direct_nav_active:
            response.success = False
            response.message = "A direct RViz navigation goal is already running"
            return response

        region_points = []
        if not request.waypoints and self.inspection_regions:
            self._recompute_region_preview()
            region_points = list(self.region_preview_points)
            if self.region_generation_error is not None:
                response.success = False
                response.message = self.region_generation_error
                return response
            if not region_points:
                response.success = False
                response.message = (
                    "Inspection regions are too small to generate sweep waypoints. "
                    f"Check region_margin={self.region_margin:.2f}m."
                )
                return response

        points = list(request.waypoints) if request.waypoints else list(self.confirmed_points)
        source = "request"
        if region_points:
            points = region_points
            source = "region"
        if not points and self.rviz_points:
            points = list(self.rviz_points)
            source = "rviz"
        if not points:
            response.success = False
            response.message = "No mission points available"
            return response

        if not self.have_map_pose:
            response.success = False
            response.message = "AMCL pose unavailable. Set the initial pose before starting a mission."
            return response

        if not self.have_map or self.map_msg is None:
            response.success = False
            response.message = "No /map data received"
            return response

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Nav2 action server /navigate_to_pose is not ready"
            return response

        pause_sec = self._clamp_waypoint_pause(
            getattr(request, "waypoint_pause_sec", self.DEFAULT_WAYPOINT_PAUSE_SEC)
        )

        validation = validate_mission_points(
            self.map_msg,
            (self.current_map_pose["x"], self.current_map_pose["y"]),
            points,
        )
        if not validation.valid:
            response.success = False
            if source == "region":
                response.message = f"Inspection region mission rejected: {validation.message}"
            else:
                response.message = validation.message
            self.get_logger().warn(f"Mission request rejected: {response.message}")
            self._publish_mission_status(response.message, safety=True)
            return response

        ordered_points = list(points)
        if not self._start_sequential_mission(ordered_points, source, pause_sec):
            response.success = False
            response.message = "Failed to start sequential mission"
            return response

        names = " -> ".join(point.point_name for point in ordered_points[:12])
        if len(ordered_points) > 12:
            names += f" -> ... ({len(ordered_points)} total)"
        response.success = True
        if source == "region":
            response.message = (
                f"Sequential region inspection mission started with {len(ordered_points)} sweep waypoint(s) "
                f"from {len(self.inspection_regions)} region(s), pause={pause_sec:.1f}s: {names}"
            )
        else:
            response.message = (
                f"Sequential mission started with {len(ordered_points)} points from {source}, "
                f"pause={pause_sec:.1f}s: {names}"
            )
        self.get_logger().info(response.message)
        self._publish_mission_status(response.message)
        return response

    def _inspection_point_to_pose(self, point):
        return inspection_point_to_pose(point, self.get_clock().now().to_msg())

    @staticmethod
    def _make_inspection_point(name, x, y, theta):
        return make_inspection_point(name, x, y, theta)

    def _clamp_waypoint_pause(self, value):
        try:
            pause_sec = float(value)
        except (TypeError, ValueError):
            pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        if not math.isfinite(pause_sec):
            pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        return max(0.0, min(self.MAX_WAYPOINT_PAUSE_SEC, pause_sec))

    def _start_sequential_mission(self, ordered_points, source, pause_sec):
        self._clear_mission_wait_timer()
        self.goal_handle = None
        self.mission_points = list(ordered_points)
        self.mission_index = 0
        self.mission_source = source
        self.mission_run_id += 1
        self.mission_waypoint_pause_sec = pause_sec
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None
        self.mission_active = True
        return self._send_current_mission_goal()

    def _send_current_mission_goal(self):
        if not self.mission_active:
            return False
        if self.mission_index >= len(self.mission_points):
            self._finish_mission_success()
            return True

        point = self.mission_points[self.mission_index]
        waypoint_number = self.mission_index + 1
        total = len(self.mission_points)
        run_id = self.mission_run_id

        if self.mission_source == "region":
            dx = point.x - self.current_map_pose["x"]
            dy = point.y - self.current_map_pose["y"]
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                theta = self.current_map_pose["theta"]
            else:
                theta = math.atan2(dy, dx)
            goal_point = self._make_inspection_point(point.point_name, point.x, point.y, theta)
        else:
            goal_point = point

        goal = NavigateToPose.Goal()
        goal.pose = self._inspection_point_to_pose(goal_point)
        goal.behavior_tree = ""

        try:
            send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback_msg, run_id=run_id, index=self.mission_index: (
                    self._mission_feedback_cb(feedback_msg, run_id, index)
                ),
            )
        except Exception as exc:
            self._finish_mission_failed(f"Failed to send mission waypoint {waypoint_number}/{total}: {exc}")
            return False

        send_goal_future.add_done_callback(
            lambda future, run_id=run_id, index=self.mission_index: self._mission_goal_response_cb(
                future, run_id, index
            )
        )
        self.get_logger().info(
            f"Mission waypoint {waypoint_number}/{total} sent to {point.point_name} "
            f"({point.x:.2f}, {point.y:.2f}, {math.degrees(point.theta):.1f} deg)"
        )
        return True

    def _mission_goal_response_cb(self, future, run_id, waypoint_index):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._is_current_mission_goal(run_id, waypoint_index):
                self._finish_mission_failed(
                    f"Failed to send mission waypoint {waypoint_index + 1}/{len(self.mission_points)}: {exc}"
                )
            return

        if not self._is_current_mission_goal(run_id, waypoint_index):
            if goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warn(f"Failed to cancel stale mission waypoint goal: {exc}")
            return

        if not goal_handle.accepted:
            self._finish_mission_failed(
                f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} was rejected by Nav2"
            )
            return

        self.goal_handle = goal_handle
        self.get_logger().info(
            f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} accepted by Nav2"
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, handle=goal_handle, run_id=run_id, index=waypoint_index: self._mission_result_cb(
                future, handle, run_id, index
            )
        )

    def _mission_feedback_cb(self, feedback_msg, run_id, waypoint_index):
        if not self._is_current_mission_goal(run_id, waypoint_index):
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_mission_feedback_log_time < 2.0:
            return
        self.last_mission_feedback_log_time = now_sec
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} feedback: "
            f"{feedback.distance_remaining:.2f} m left"
        )

    def _mission_result_cb(self, future, goal_handle, run_id, waypoint_index):
        if run_id != self.mission_run_id:
            return
        if goal_handle is not self.goal_handle:
            return
        if not self._is_current_mission_goal(run_id, waypoint_index):
            return

        early_transition_goal = (run_id, waypoint_index)
        is_early_transition_cancel = self.mission_early_transition_goal == early_transition_goal
        if is_early_transition_cancel:
            self.mission_early_transition_goal = None

        self.goal_handle = None
        try:
            result_msg = future.result()
            result = result_msg.result
        except Exception as exc:
            self._finish_mission_failed(f"Mission result retrieval failed: {exc}")
            return

        if result.error_code != NavigateToPose.Result.NONE:
            if not is_early_transition_cancel:
                point_name = self.mission_points[waypoint_index].point_name
                self._finish_mission_failed(
                    f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} "
                    f"({point_name}) failed with code {result.error_code}: {result.error_msg}"
                )
                return

        self._publish_zero_cmd()
        point_name = self.mission_points[waypoint_index].point_name
        self.get_logger().info(
            f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} "
            f"({point_name}) reached"
        )

        next_index = waypoint_index + 1
        if next_index >= len(self.mission_points):
            self._finish_mission_success()
            return

        self.mission_index = next_index
        next_point = self.mission_points[self.mission_index]
        pause_sec = self.mission_waypoint_pause_sec
        self.get_logger().info(
            f"Waiting {pause_sec:.1f} s before planning to "
            f"mission waypoint {self.mission_index + 1}/{len(self.mission_points)} "
            f"({next_point.point_name})"
        )
        if pause_sec <= 0.0:
            self._send_current_mission_goal()
            return

        self._clear_mission_wait_timer()
        self.mission_wait_timer = self.create_timer(
            pause_sec,
            self._mission_wait_complete_cb,
        )

    def _mission_wait_complete_cb(self):
        self._clear_mission_wait_timer()
        if not self.mission_active:
            return
        self._send_current_mission_goal()

    def _check_mission_early_transition(self):
        if not self.mission_active or self.mission_source != "region":
            return
        if self.goal_handle is None:
            return
        if self.mission_index >= len(self.mission_points):
            return
        if self.mission_index == len(self.mission_points) - 1:
            return
        if not self.have_map_pose:
            return

        waypoint = self.mission_points[self.mission_index]
        dx = self.current_map_pose["x"] - waypoint.x
        dy = self.current_map_pose["y"] - waypoint.y
        distance = math.hypot(dx, dy)
        if distance >= self.RELAXED_GOAL_DISTANCE_M:
            return

        early_transition_goal = (self.mission_run_id, self.mission_index)
        if self.mission_early_transition_goal == early_transition_goal:
            return

        self.get_logger().info(
            f"Early transition on waypoint {self.mission_index + 1}/{len(self.mission_points)} "
            f"({waypoint.point_name}) at {distance:.3f}m (tolerance {self.RELAXED_GOAL_DISTANCE_M:.2f}m)"
        )
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(f"Failed to request early waypoint transition: {exc}")
            return
        self.mission_early_transition_goal = early_transition_goal

    def _is_current_mission_goal(self, run_id, waypoint_index):
        return (
            self.mission_active
            and run_id == self.mission_run_id
            and waypoint_index == self.mission_index
        )

    def _clear_mission_wait_timer(self):
        timer = self.mission_wait_timer
        if timer is None:
            return
        self.mission_wait_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _clear_mission_state(self):
        self.mission_points = []
        self.mission_index = 0
        self.mission_source = ""
        self.mission_waypoint_pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None

    def _finish_mission_success(self):
        self.goal_handle = None
        self.mission_active = False
        self._clear_mission_wait_timer()
        self._publish_zero_cmd()
        self.get_logger().info("Mission completed successfully")
        self._publish_mission_status("Mission completed successfully")
        self._clear_mission_state()

    def _finish_mission_failed(self, message):
        self.goal_handle = None
        self.mission_active = False
        self._clear_mission_wait_timer()
        self._publish_zero_cmd()
        self.get_logger().error(message)
        self._publish_mission_status(message, safety=True)
        self._clear_mission_state()

    def _direct_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.direct_nav_active = False
            self.get_logger().error(f"Failed to send RViz navigation goal: {exc}")
            return

        if not goal_handle.accepted:
            self.direct_nav_active = False
            self.get_logger().error("RViz navigation goal was rejected by Nav2")
            return

        self.direct_goal_handle = goal_handle
        self.get_logger().info("RViz navigation goal accepted by Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, handle=goal_handle: self._direct_result_cb(future, handle)
        )

    def _direct_feedback_cb(self, feedback_msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_direct_feedback_log_time < 2.0:
            return
        self.last_direct_feedback_log_time = now_sec
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"RViz navigation feedback: {feedback.distance_remaining:.2f} m left"
        )

    def _direct_result_cb(self, future, goal_handle):
        if goal_handle is not self.direct_goal_handle:
            return
        self.direct_goal_handle = None
        self.direct_nav_active = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"RViz navigation result retrieval failed: {exc}")
            self._publish_zero_cmd()
            return

        if result.error_code == NavigateToPose.Result.NONE:
            self.get_logger().info("RViz navigation goal completed successfully")
        else:
            self.get_logger().error(
                f"RViz navigation goal failed with code {result.error_code}: {result.error_msg}"
            )
            self._publish_zero_cmd()

    def destroy_node(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.direct_goal_handle is not None:
            self.direct_goal_handle.cancel_goal_async()
        self._clear_mission_wait_timer()
        super().destroy_node()

    @staticmethod
    def _quat_to_yaw(orientation):
        return quat_to_yaw(orientation)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
