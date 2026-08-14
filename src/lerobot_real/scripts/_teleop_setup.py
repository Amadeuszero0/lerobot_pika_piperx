"""Shared setup helpers for real-robot teleoperation workflows."""


def _move_single_robot_to_teleop_base(robot, teleop) -> None:
    robot_base_pose = getattr(teleop.config, "robot_base_pose", None)
    move_to_tcp_pose = getattr(robot, "move_to_tcp_pose", None)
    move_to_base_on_start = getattr(teleop.config, "move_to_base_on_start", True)
    if move_to_base_on_start and robot_base_pose is not None and callable(move_to_tcp_pose):
        move_to_tcp_pose(robot_base_pose)
    else:
        robot.configure()


def move_robot_to_teleop_base(robot, teleop) -> None:
    robots = getattr(robot, "robots", None)
    teleops = getattr(teleop, "teleops", None)
    if (
        isinstance(robots, dict)
        and isinstance(teleops, dict)
        and robots
        and robots.keys() == teleops.keys()
    ):
        for side, child_robot in robots.items():
            _move_single_robot_to_teleop_base(child_robot, teleops[side])
        return

    _move_single_robot_to_teleop_base(robot, teleop)
