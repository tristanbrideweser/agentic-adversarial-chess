#!/usr/bin/env python3
"""Game coordinator behavior tree.

Top-level orchestrator for the agentic adversarial chess robot.

Tree shape
----------
Root: Parallel (SuccessOnAll)
├── DataGathering — subscribers populate the blackboard from ROS topics
│   ├── /board_state       → BOARD_STATE_FEN
│   ├── /game_over         → GAME_OVER
│   ├── /pick_place_tasks  → TASK_QUEUE_MSG (raw msg; parsed by WaitForTaskQueue)
│   └── /chess/occupancy   → OCCUPANCY_MSG (raw msg; consumed by VerifyBoard)
└── GameLoop — Repeat(forever) wrapping the per-turn sequence
    └── PlayOneTurn: Sequence
        ├── CheckGameOver         (FAILURE → exits the Repeat)
        ├── SnapshotPreMoveFen
        ├── DetermineActivePlayer
        ├── RequestMove           [stub: chess_engine]
        ├── PublishApplyMove
        ├── WaitForBoardUpdate
        ├── WaitForTaskQueue      [stub-dep: move_translator]
        ├── IterateTaskQueue
        │   └── ExecuteOneTask: Sequence
        │       ├── PopNextTask
        │       ├── Retry(3) → PlanGrasp         [pre-written: needs chess_interfaces]
        │       └── Retry(3) → ExecutePickPlace  [pre-written: needs chess_interfaces]
        └── VerifyBoard                          [runs once per turn, reads /chess/occupancy]
"""

import rclpy
import py_trees
import py_trees_ros
from std_msgs.msg import String, UInt8MultiArray

from . import behaviors as B
from .behaviors import init_blackboard


def _data_gathering() -> py_trees.behaviour.Behaviour:
    fen_to_bb = py_trees_ros.subscribers.ToBlackboard(
        name="FenToBlackboard",
        topic_name="/board_state",
        topic_type=String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={B.BOARD_STATE_FEN: "data"},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )
    game_over_to_bb = py_trees_ros.subscribers.ToBlackboard(
        name="GameOverToBlackboard",
        topic_name="/game_over",
        topic_type=String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={B.GAME_OVER: "data"},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )
    tasks_to_bb = py_trees_ros.subscribers.ToBlackboard(
        name="TaskQueueToBlackboard",
        topic_name="/pick_place_tasks",
        topic_type=String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={B.TASK_QUEUE_MSG: None},  # whole msg
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )
    occupancy_to_bb = py_trees_ros.subscribers.ToBlackboard(
        name="OccupancyToBlackboard",
        topic_name="/chess/occupancy",
        topic_type=UInt8MultiArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={B.OCCUPANCY_MSG: None},  # whole msg, used by VerifyBoard
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )
    return py_trees.composites.Parallel(
        name="DataGathering",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[fen_to_bb, game_over_to_bb, tasks_to_bb, occupancy_to_bb],
    )


def _execute_one_task(node) -> py_trees.behaviour.Behaviour:
    plan_grasp = py_trees.decorators.Retry(
        name="PlanGraspRetry",
        child=B.PlanGrasp(node),
        num_failures=3,
    )
    pick_place = py_trees.decorators.Retry(
        name="PickPlaceRetry",
        child=B.ExecutePickPlace(node),
        num_failures=3,
    )
    return py_trees.composites.Sequence(
        name="ExecuteOneTask",
        memory=True,
        children=[
            B.PopNextTask(),
            plan_grasp,
            pick_place,
        ],
    )


def _play_one_turn(node) -> py_trees.behaviour.Behaviour:
    return py_trees.composites.Sequence(
        name="PlayOneTurn",
        memory=True,
        children=[
            B.CheckGameOver(),
            B.SnapshotPreMoveFen(),
            B.DetermineActivePlayer(),
            B.RequestMove(node),
            B.PublishApplyMove(node),
            B.WaitForBoardUpdate(node),
            B.WaitForTaskQueue(node),
            B.IterateTaskQueue(_execute_one_task(node)),
            B.VerifyBoard(node),
        ],
    )


def build_tree(node) -> py_trees.behaviour.Behaviour:
    game_loop = py_trees.decorators.Repeat(
        name="GameLoop",
        child=_play_one_turn(node),
        num_success=-1,  # repeat forever; CheckGameOver's FAILURE exits the loop
    )
    return py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[_data_gathering(), game_loop],
    )


def main():
    rclpy.init()
    node = rclpy.create_node("game_coordinator")

    init_blackboard()
    root = build_tree(node)

    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    try:
        tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        node.get_logger().error(f"tree setup timed out: {e}")
        rclpy.try_shutdown()
        return

    def on_post_tick(t: py_trees_ros.trees.BehaviourTree):
        if t.root.status == py_trees.common.Status.FAILURE:
            node.get_logger().info("root FAILURE — shutting down")
            rclpy.try_shutdown()

    tree.add_post_tick_handler(on_post_tick)
    tree.tick_tock(period_ms=200.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
