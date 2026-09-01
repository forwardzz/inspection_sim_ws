import os
import shlex
import signal

from PyQt5.QtCore import QObject, QProcess, pyqtSignal

from .config import DEFAULT_ROS_SETUP_PATH, DEFAULT_WORKSPACE_PATH


def build_sim_command(
    use_rviz=False,
    headless=False,
    world=None,
    world_name=None,
    spawn_x=0.0,
    spawn_y=0.0,
    spawn_z=0.05,
    spawn_yaw=0.0,
    dynamic_obstacles_config=None,
    dynamic_obstacle_seed=None,
):
    command = (
        "ros2 launch inspection_sim_bringup sim.launch.py "
        f"use_rviz:={str(use_rviz).lower()} "
        f"headless:={str(headless).lower()}"
    )
    if world is not None:
        command += f" world:={shlex.quote(world)}"
    if world_name is not None:
        command += f" world_name:={shlex.quote(world_name)}"
    command += (
        f" spawn_x:={spawn_x} "
        f"spawn_y:={spawn_y} "
        f"spawn_z:={spawn_z} "
        f"spawn_yaw:={spawn_yaw}"
    )
    if dynamic_obstacles_config is not None:
        command += f" dynamic_obstacles_config:={shlex.quote(dynamic_obstacles_config)}"
        if dynamic_obstacle_seed is not None:
            command += f" dynamic_obstacle_seed:={dynamic_obstacle_seed}"
    return command


def build_mapping_command(
    use_rviz=True,
    headless=False,
    world=None,
    world_name=None,
    spawn_x=0.0,
    spawn_y=0.0,
    spawn_z=0.05,
    spawn_yaw=0.0,
    dynamic_obstacles_config=None,
    dynamic_obstacle_seed=None,
):
    command = (
        "ros2 launch inspection_sim_bringup mapping.launch.py "
        f"use_rviz:={str(use_rviz).lower()} "
        f"headless:={str(headless).lower()}"
    )
    if world is not None:
        command += f" world:={shlex.quote(world)}"
    if world_name is not None:
        command += f" world_name:={shlex.quote(world_name)}"
    command += (
        f" spawn_x:={spawn_x} "
        f"spawn_y:={spawn_y} "
        f"spawn_z:={spawn_z} "
        f"spawn_yaw:={spawn_yaw}"
    )
    if dynamic_obstacles_config is not None:
        command += f" dynamic_obstacles_config:={shlex.quote(dynamic_obstacles_config)}"
        if dynamic_obstacle_seed is not None:
            command += f" dynamic_obstacle_seed:={dynamic_obstacle_seed}"
    return command


def build_navigation_command(
    map_path,
    use_rviz=True,
    headless=False,
    world=None,
    world_name=None,
    spawn_x=0.0,
    spawn_y=0.0,
    spawn_z=0.05,
    spawn_yaw=0.0,
    initial_pose_x=0.0,
    initial_pose_y=0.0,
    initial_pose_yaw=0.0,
    regions=None,
    dynamic_obstacles_config=None,
    dynamic_obstacle_seed=None,
):
    command = (
        "ros2 launch inspection_sim_bringup navigation.launch.py "
        f"map:={shlex.quote(map_path)} "
        f"use_rviz:={str(use_rviz).lower()} "
        f"headless:={str(headless).lower()}"
    )
    if world is not None:
        command += f" world:={shlex.quote(world)}"
    if world_name is not None:
        command += f" world_name:={shlex.quote(world_name)}"
    command += (
        f" spawn_x:={spawn_x} "
        f"spawn_y:={spawn_y} "
        f"spawn_z:={spawn_z} "
        f"spawn_yaw:={spawn_yaw}"
    )
    command += (
        f" initial_pose_x:={initial_pose_x} "
        f"initial_pose_y:={initial_pose_y} "
        f"initial_pose_yaw:={initial_pose_yaw}"
    )
    if regions is not None:
        command += f" regions:={shlex.quote(regions)}"
    if dynamic_obstacles_config is not None:
        command += f" dynamic_obstacles_config:={shlex.quote(dynamic_obstacles_config)}"
        if dynamic_obstacle_seed is not None:
            command += f" dynamic_obstacle_seed:={dynamic_obstacle_seed}"
    return command


class LaunchManager(QObject):
    log_line = pyqtSignal(str)
    state_changed = pyqtSignal(str)

    def __init__(self, workspace_path=DEFAULT_WORKSPACE_PATH, ros_setup_path=DEFAULT_ROS_SETUP_PATH):
        super().__init__()
        self.workspace_path = workspace_path
        self.ros_setup_path = ros_setup_path
        self.active_process = None
        self.active_name = "idle"
        self.oneshot_processes = []

    def update_paths(self, workspace_path, ros_setup_path):
        self.workspace_path = workspace_path
        self.ros_setup_path = ros_setup_path

    def start_sim(
        self,
        use_rviz=False,
        headless=False,
        world=None,
        world_name=None,
        spawn_x=0.0,
        spawn_y=0.0,
        spawn_z=0.05,
        spawn_yaw=0.0,
        dynamic_obstacles_config=None,
        dynamic_obstacle_seed=None,
    ):
        command = build_sim_command(
            use_rviz=use_rviz,
            headless=headless,
            world=world,
            world_name=world_name,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_z=spawn_z,
            spawn_yaw=spawn_yaw,
            dynamic_obstacles_config=dynamic_obstacles_config,
            dynamic_obstacle_seed=dynamic_obstacle_seed,
        )
        return self.start("sim", command)

    def start_mapping(
        self,
        use_rviz=True,
        headless=False,
        world=None,
        world_name=None,
        spawn_x=0.0,
        spawn_y=0.0,
        spawn_z=0.05,
        spawn_yaw=0.0,
        dynamic_obstacles_config=None,
        dynamic_obstacle_seed=None,
    ):
        command = build_mapping_command(
            use_rviz=use_rviz,
            headless=headless,
            world=world,
            world_name=world_name,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_z=spawn_z,
            spawn_yaw=spawn_yaw,
            dynamic_obstacles_config=dynamic_obstacles_config,
            dynamic_obstacle_seed=dynamic_obstacle_seed,
        )
        return self.start("mapping", command)

    def start_navigation(
        self,
        map_path,
        use_rviz=True,
        headless=False,
        world=None,
        world_name=None,
        spawn_x=0.0,
        spawn_y=0.0,
        spawn_z=0.05,
        spawn_yaw=0.0,
        initial_pose_x=0.0,
        initial_pose_y=0.0,
        initial_pose_yaw=0.0,
        regions=None,
        dynamic_obstacles_config=None,
        dynamic_obstacle_seed=None,
    ):
        command = build_navigation_command(
            map_path=map_path,
            use_rviz=use_rviz,
            headless=headless,
            world=world,
            world_name=world_name,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_z=spawn_z,
            spawn_yaw=spawn_yaw,
            initial_pose_x=initial_pose_x,
            initial_pose_y=initial_pose_y,
            initial_pose_yaw=initial_pose_yaw,
            regions=regions,
            dynamic_obstacles_config=dynamic_obstacles_config,
            dynamic_obstacle_seed=dynamic_obstacle_seed,
        )
        return self.start("navigation", command)

    def save_map(self, map_path):
        prefix = os.path.splitext(map_path)[0]
        maps_dir = os.path.dirname(map_path) or os.path.join(self.workspace_path, "maps")
        command = (
            f"mkdir -p {shlex.quote(maps_dir)} && "
            f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(prefix)} -t /map"
        )
        self.run_once("save_map", command)

    def start(self, name, command):
        if self.active_process is not None and self.active_process.state() != QProcess.NotRunning:
            self.log_line.emit(f"[WARN] Stop {self.active_name} before starting {name}.")
            return False

        proc = self._make_process(name, track_active=True)
        self.active_process = proc
        self.active_name = name
        self.state_changed.emit(name)
        self.log_line.emit(f"[RUN] {name}: {command}")
        proc.start("setsid", ["bash", "-lc", self._wrap_command(command)])
        return True

    def run_once(self, name, command):
        proc = self._make_process(name, track_active=False)
        self.oneshot_processes.append(proc)
        self.log_line.emit(f"[RUN] {name}: {command}")
        proc.start("setsid", ["bash", "-lc", self._wrap_command(command)])

    def stop(self):
        if self.active_process is None or self.active_process.state() == QProcess.NotRunning:
            self.log_line.emit("[STOP] no active launch")
            self.active_name = "idle"
            self.state_changed.emit("idle")
            return

        pid = int(self.active_process.processId())
        self.log_line.emit(f"[STOP] {self.active_name}")
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        self.active_process.waitForFinished(2500)
        if self.active_process.state() != QProcess.NotRunning:
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.active_process.waitForFinished(2500)
        if self.active_process.state() != QProcess.NotRunning:
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.active_process = None
        self.active_name = "idle"
        self.state_changed.emit("idle")

    def is_running(self):
        return self.active_process is not None and self.active_process.state() != QProcess.NotRunning

    def _wrap_command(self, command):
        setup_path = os.path.join(self.workspace_path, "install", "setup.bash")
        return (
            f"source {shlex.quote(self.ros_setup_path)} && "
            f"source {shlex.quote(setup_path)} && "
            f"cd {shlex.quote(self.workspace_path)} && "
            f"{command}"
        )

    def _make_process(self, name, track_active):
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(lambda p=proc: self._read_output(p))
        proc.finished.connect(lambda code, status, p=proc, n=name, t=track_active: self._finished(p, n, t, code))
        return proc

    def _read_output(self, proc):
        data = bytes(proc.readAllStandardOutput()).decode(errors="replace")
        for line in data.splitlines():
            self.log_line.emit(line)

    def _finished(self, proc, name, track_active, code):
        self.log_line.emit(f"[EXIT] {name} -> {code}")
        if track_active and proc is self.active_process:
            self.active_process = None
            self.active_name = "idle"
            self.state_changed.emit("idle")
        if proc in self.oneshot_processes:
            self.oneshot_processes.remove(proc)
