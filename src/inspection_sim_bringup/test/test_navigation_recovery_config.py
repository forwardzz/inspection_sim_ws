from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_progress_checker_is_explicitly_wired_in_both_behavior_trees():
    for filename in (
        "navigate_replan_if_path_invalid.xml",
        "navigate_through_poses_replan_if_invalid.xml",
    ):
        root = ElementTree.parse(PACKAGE_ROOT / "behavior_trees" / filename).getroot()
        selector = root.find(".//ProgressCheckerSelector")
        follow_path = root.find(".//FollowPath")

        assert selector is not None
        assert selector.attrib["default_progress_checker"] == "progress_checker"
        assert selector.attrib["selected_progress_checker"] == (
            "{selected_progress_checker}"
        )
        assert follow_path is not None
        assert follow_path.attrib["progress_checker_id"] == (
            "{selected_progress_checker}"
        )


def test_progress_timeout_and_position_only_goal_policy():
    with (PACKAGE_ROOT / "config" / "nav2_params.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    controller = config["controller_server"]["ros__parameters"]
    assert controller["progress_checker"]["movement_time_allowance"] == 20.0
    assert controller["FollowPath"]["rotate_to_goal_heading"] is False
    assert controller["general_goal_checker"]["plugin"] == (
        "nav2_controller::PositionGoalChecker"
    )
