"""py_trees leaves  and blackboard registry for the chess game coordinator.
"""

import json
import math

import py_trees
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from chess_engine_interfaces.srv import GetMove

# --- blackboard keys -------------------------------------------------------
# Continuously written by the data-gathering parallel branch (subscribers).
BOARD_STATE_FEN = "/board_state_fen"   # str  — live FEN from /board_state
GAME_OVER       = "/game_over"          # str  — termination reason; "" while game continues
TASK_QUEUE_MSG  = "/task_queue_msg"     # std_msgs/String — last raw msg from /pick_place_tasks
OCCUPANCY_MSG   = "/occupancy_msg"      # std_msgs/UInt8MultiArray — last raw msg from /chess/occupancy

# Written/consumed by the per-turn sequence.
PRE_MOVE_FEN    = "/pre_move_fen"       # str  — FEN snapshotted before publishing /apply_move
ACTIVE_PLAYER   = "/active_player"      # "white" | "black"
CURRENT_MOVE    = "/current_move"       # str  — UCI returned by Stockfish
TASK_QUEUE      = "/task_queue"         # list[dict] — parsed JSON payload from move_translator
CURRENT_TASK    = "/current_task"       # dict — task popped off TASK_QUEUE
CURRENT_GRASP   = "/current_grasp"      # PoseStamped (or whatever grasp_planner returns)

_DEFAULTS = {
    BOARD_STATE_FEN: "",
    GAME_OVER:       "",
    TASK_QUEUE_MSG:  None,
    OCCUPANCY_MSG:   None,
    PRE_MOVE_FEN:    "",
    ACTIVE_PLAYER:   "",
    CURRENT_MOVE:    "",
    TASK_QUEUE:      [],
    CURRENT_TASK:    {},
    CURRENT_GRASP:   None,
}


def init_blackboard() -> py_trees.blackboard.Client:
    """Register every key with WRITE access and seed defaults so leaves never
    fault on a "key not found" before the first subscriber callback fires."""
    bb = py_trees.blackboard.Client(name="game_coordinator_init")
    for key in _DEFAULTS:
        bb.register_key(key=key, access=py_trees.common.Access.WRITE)
    for key, value in _DEFAULTS.items():
        bb.set(key, value)
    return bb


def _yaw_to_quat(yaw: float) -> list:
    """Convert a yaw rotation around world Z to an xyzw quaternion."""
    half = float(yaw) / 2.0
    return [0.0, 0.0, math.sin(half), math.cos(half)]


# --- pure-logic leaves ----------------------


class SnapshotPreMoveFen(py_trees.behaviour.Behaviour):
    """Copy the live FEN to PRE_MOVE_FEN so we can later detect that the move applied.

    Reads:  BOARD_STATE_FEN
    Writes: PRE_MOVE_FEN
    Returns FAILURE if no FEN has arrived yet.
    """

    def __init__(self, name: str = "SnapshotPreMoveFen"):
        super().__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(BOARD_STATE_FEN, access=py_trees.common.Access.READ)
        self.bb.register_key(PRE_MOVE_FEN, access=py_trees.common.Access.WRITE)

    def update(self):
        fen = self.bb.get(BOARD_STATE_FEN)
        if not fen:
            self.feedback_message = "no FEN on /board_state yet"
            return py_trees.common.Status.FAILURE
        self.bb.set(PRE_MOVE_FEN, fen)
        return py_trees.common.Status.SUCCESS


class DetermineActivePlayer(py_trees.behaviour.Behaviour):
    """Parse the FEN turn field and write 'white' or 'black' to ACTIVE_PLAYER.

    Reads:  PRE_MOVE_FEN
    Writes: ACTIVE_PLAYER
    """

    def __init__(self, name: str = "DetermineActivePlayer"):
        super().__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(PRE_MOVE_FEN, access=py_trees.common.Access.READ)
        self.bb.register_key(ACTIVE_PLAYER, access=py_trees.common.Access.WRITE)

    def update(self):
        fen = self.bb.get(PRE_MOVE_FEN)
        parts = fen.split() if fen else []
        if len(parts) < 2 or parts[1] not in ("w", "b"):
            self.feedback_message = f"malformed FEN: {fen!r}"
            return py_trees.common.Status.FAILURE
        self.bb.set(ACTIVE_PLAYER, "white" if parts[1] == "w" else "black")
        return py_trees.common.Status.SUCCESS


class CheckGameOver(py_trees.behaviour.Behaviour):
    """Guard leaf: FAILURE when GAME_OVER is set, SUCCESS while the game continues.

    Placed at the top of the per-turn sequence so a FAILURE here breaks the
    Repeat decorator wrapping the loop.

    Reads: GAME_OVER
    """

    def __init__(self, name: str = "CheckGameOver"):
        super().__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(GAME_OVER, access=py_trees.common.Access.READ)

    def update(self):
        reason = self.bb.get(GAME_OVER)
        if reason:
            self.feedback_message = f"game over: {reason}"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class WaitForBoardUpdate(py_trees.behaviour.Behaviour):
    """RUNNING until BOARD_STATE_FEN differs from PRE_MOVE_FEN, then SUCCESS.

    Reads: BOARD_STATE_FEN, PRE_MOVE_FEN
    """

    _TIMEOUT_SEC = 5.0

    def __init__(self, node: Node, name: str = "WaitForBoardUpdate"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(BOARD_STATE_FEN, access=py_trees.common.Access.READ)
        self.bb.register_key(PRE_MOVE_FEN, access=py_trees.common.Access.READ)
        self._deadline = 0

    def initialise(self):
        self._deadline = (
            self.node.get_clock().now().nanoseconds + int(self._TIMEOUT_SEC * 1e9)
        )

    def update(self):
        if self.bb.get(BOARD_STATE_FEN) != self.bb.get(PRE_MOVE_FEN):
            return py_trees.common.Status.SUCCESS
        if self.node.get_clock().now().nanoseconds > self._deadline:
            self.feedback_message = "timeout waiting for /board_state to update"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


class WaitForTaskQueue(py_trees.behaviour.Behaviour):
    """RUNNING until move_translator publishes a JSON task list, then parse it.

    Depends on: move_translator (publishes /pick_place_tasks)

    Reads:  TASK_QUEUE_MSG
    Writes: TASK_QUEUE, TASK_QUEUE_MSG (cleared on entry)
    """

    _TIMEOUT_SEC = 5.0

    def __init__(self, node: Node, name: str = "WaitForTaskQueue"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(TASK_QUEUE_MSG, access=py_trees.common.Access.WRITE)
        self.bb.register_key(TASK_QUEUE, access=py_trees.common.Access.WRITE)
        self._deadline = 0

    def initialise(self):
        self.bb.set(TASK_QUEUE_MSG, None)
        self.bb.set(TASK_QUEUE, [])
        self._deadline = (
            self.node.get_clock().now().nanoseconds + int(self._TIMEOUT_SEC * 1e9)
        )

    def update(self):
        msg = self.bb.get(TASK_QUEUE_MSG)
        if msg is None:
            if self.node.get_clock().now().nanoseconds > self._deadline:
                self.feedback_message = "timeout waiting for /pick_place_tasks"
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        try:
            payload = json.loads(msg.data)
        except (AttributeError, ValueError) as e:
            self.feedback_message = f"bad task queue payload: {e}"
            return py_trees.common.Status.FAILURE
        if not isinstance(payload, dict):
            self.feedback_message = "task queue payload is not a dict"
            return py_trees.common.Status.FAILURE
        queue = payload.get("tasks", [])
        if not isinstance(queue, list) or not queue:
            self.feedback_message = "task queue empty or missing 'tasks' field"
            return py_trees.common.Status.FAILURE
        self.bb.set(TASK_QUEUE, queue)
        return py_trees.common.Status.SUCCESS



class PopNextTask(py_trees.behaviour.Behaviour):
    """Pop the front of TASK_QUEUE into CURRENT_TASK. FAILURE when the queue is empty.

    Reads/Writes: TASK_QUEUE
    Writes:       CURRENT_TASK
    """

    def __init__(self, name: str = "PopNextTask"):
        super().__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(TASK_QUEUE, access=py_trees.common.Access.WRITE)
        self.bb.register_key(CURRENT_TASK, access=py_trees.common.Access.WRITE)

    def update(self):
        queue = list(self.bb.get(TASK_QUEUE) or [])
        if not queue:
            return py_trees.common.Status.FAILURE
        self.bb.set(CURRENT_TASK, queue.pop(0))
        self.bb.set(TASK_QUEUE, queue)
        return py_trees.common.Status.SUCCESS


class IterateTaskQueue(py_trees.decorators.Decorator):
    """Re-tick the child sequence once per task in TASK_QUEUE.

    Returns SUCCESS when TASK_QUEUE drains. Propagates FAILURE if the child
    fails for any reason other than an empty queue (the child is expected to
    not invoke PopNextTask itself; this decorator owns the empty-queue check).
    """

    def __init__(self, child: py_trees.behaviour.Behaviour, name: str = "IterateTaskQueue"):
        super().__init__(name=name, child=child)
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(TASK_QUEUE, access=py_trees.common.Access.READ)

    def update(self):
        if not self.bb.get(TASK_QUEUE):
            return py_trees.common.Status.SUCCESS
        status = self.decorated.status
        if status == py_trees.common.Status.SUCCESS:
            # Task done — keep going if more remain, otherwise we're finished.
            return (py_trees.common.Status.RUNNING
                    if self.bb.get(TASK_QUEUE)
                    else py_trees.common.Status.SUCCESS)
        return status  # RUNNING or FAILURE


# --- ROS-dependent leaves ------------------------------------------


class PublishApplyMove(py_trees.behaviour.Behaviour):
    """Publish CURRENT_MOVE on /apply_move (single shot per turn).

    Depends on: chess_engine (board_state_node subscribes to /apply_move)

    Reads: CURRENT_MOVE
    """

    def __init__(self, node: Node, name: str = "PublishApplyMove"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(CURRENT_MOVE, access=py_trees.common.Access.READ)
        self._pub = None

    def setup(self, **kwargs):
        self._pub = self.node.create_publisher(
            String, "/apply_move",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

    def update(self):
        move = self.bb.get(CURRENT_MOVE)
        if not move:
            self.feedback_message = "CURRENT_MOVE is empty"
            return py_trees.common.Status.FAILURE
        self._pub.publish(String(data=move))
        return py_trees.common.Status.SUCCESS


class RequestMove(py_trees.behaviour.Behaviour):
    """Call /stockfish/{active_player}/get_move and write UCI to CURRENT_MOVE.

    chess_engine (stockfish_node service via chess_engine_interfaces/srv/GetMove)

    Reads:  ACTIVE_PLAYER, PRE_MOVE_FEN
    Writes: CURRENT_MOVE
    """

    def __init__(self, node: Node, name: str = "RequestMove"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(ACTIVE_PLAYER, access=py_trees.common.Access.READ)
        self.bb.register_key(PRE_MOVE_FEN, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_MOVE, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._clients = {
            "white": self.node.create_client(GetMove, "/stockfish/white/get_move"),
            "black": self.node.create_client(GetMove, "/stockfish/black/get_move"),
        }
        self._future = None
    
    def initialise(self):
        self._future = None

    def update(self):
        # Send the request on the first tick
        if self._future is None:
            color = self.bb.get(ACTIVE_PLAYER)
            client = self._clients.get(color)
            if client is None:
                self.feedback_message = f"unknown active player: {color!r}"
                return py_trees.common.Status.FAILURE
            if not client.service_is_ready():
                self.feedback_message = f"/stockfish/{color}/get_move not available"
                return py_trees.common.Status.FAILURE
            request = GetMove.Request()
            request.fen = self.bb.get(PRE_MOVE_FEN)
            self._future = client.call_async(request)
            return py_trees.common.Status.RUNNING
        
        # If we don't recieve the move keep ticking
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        
        # Have recieved a response
        response = self._future.result()
        if response is None:
            self.feedback_message = "service call returned no response"
            return py_trees.common.Status.FAILURE
        if not response.success or not response.uci:
            self.feedback_message = f"stockfish failed: {response.reason}"
            return py_trees.common.Status.FAILURE
        self.bb.set(CURRENT_MOVE, response.uci)
        return py_trees.common.Status.SUCCESS



class PlanGrasp(py_trees.behaviour.Behaviour):
    """Call /grasp_planner/plan (chess_interfaces/action/GraspPlanning) → CURRENT_GRASP.

    Depends on:
      - grasp_planner (action server on /grasp_planner/plan)
      - chess_interfaces (action type — package not built yet, see lazy import in setup)

    Reads:  CURRENT_TASK  (dict from move_translator: pick_square, piece_char, pick_pose, ...)
    Writes: CURRENT_GRASP (dict: position, orientation, finger_separation, source)
    """

    def __init__(self, node: Node, name: str = "PlanGrasp"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(CURRENT_TASK, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_GRASP, access=py_trees.common.Access.WRITE)
        self._client = None
        self._GraspPlanning = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def setup(self, **kwargs):
        # Lazy import: chess_interfaces hasn't been built yet on this branch.
        # Once the interfaces package is done, this resolves and the
        # action client wires up. Until then the leaf returns FAILURE quickly
        # with a clear feedback_message.
        try:
            from chess_interfaces.action import GraspPlanning
        except ImportError:
            self.node.get_logger().warn(
                "chess_interfaces not built — PlanGrasp disabled until package lands"
            )
            return
        self._GraspPlanning = GraspPlanning
        self._client = ActionClient(self.node, GraspPlanning, "/grasp_planner/plan")

    def initialise(self):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def update(self):
        if self._client is None:
            self.feedback_message = "chess_interfaces not available"
            return py_trees.common.Status.FAILURE

        # send the goal on the first tick
        if self._goal_future is None:
            task = self.bb.get(CURRENT_TASK) or {}
            pick_square = task.get("pick_square")
            if not pick_square:
                # place_reserve tasks have pick_square=null. The grasp planner
                # operates on board squares only — reserve pickups need a
                # different code path (TODO: handle separately).
                self.feedback_message = "no pick_square in task (likely place_reserve)"
                return py_trees.common.Status.FAILURE
            if not self._client.server_is_ready():
                self.feedback_message = "/grasp_planner/plan not available"
                return py_trees.common.Status.FAILURE
            goal = self._GraspPlanning.Goal()
            goal.square = pick_square
            goal.fen_char = task.get("piece_char", "")
            goal.use_gpd = False  # lookup mode by default — faster, more reliable per arch §2.5
            goal.yaw_hint = float(task.get("pick_pose", {}).get("yaw", 0.0))
            self._goal_future = self._client.send_goal_async(goal)
            return py_trees.common.Status.RUNNING

        # wait for the server to accept (or reject) the goal
        if self._goal_handle is None:
            if not self._goal_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._goal_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                self.feedback_message = "grasp planning goal rejected"
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Wait for the action result
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        wrapped = self._result_future.result()
        if wrapped is None:
            self.feedback_message = "grasp planning result future returned None"
            return py_trees.common.Status.FAILURE
        result = wrapped.result
        if not result.success:
            self.feedback_message = f"grasp planning failed: {result.failure_reason}"
            return py_trees.common.Status.FAILURE
        self.bb.set(CURRENT_GRASP, {
            "position": list(result.position),
            "orientation": list(result.orientation),
            "finger_separation": float(result.finger_separation),
            "source": result.source,
        })
        return py_trees.common.Status.SUCCESS


class ExecutePickPlace(py_trees.behaviour.Behaviour):
    """Send pick/place goal to the active arm's controller and wait for result.

    Depends on:
      - arm_controller (one action server per arm:
        /white_panda/pick_place, /black_panda/pick_place)
      - chess_interfaces (action type — package not built yet, lazy import in setup)

    Reads: ACTIVE_PLAYER, CURRENT_TASK, CURRENT_GRASP
    """

    def __init__(self, node: Node, name: str = "ExecutePickPlace"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(ACTIVE_PLAYER, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_TASK, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_GRASP, access=py_trees.common.Access.READ)
        self._clients = {}
        self._PickPlace = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def setup(self, **kwargs):
        # Lazy import: chess_interfaces hasn't been built yet on this branch.
        # Once the teammate's interfaces package lands, this resolves and the
        # action clients wire up. Until then the leaf returns FAILURE quickly.
        try:
            from chess_interfaces.action import PickPlace
        except ImportError:
            self.node.get_logger().warn(
                "chess_interfaces not built — ExecutePickPlace disabled until package lands"
            )
            return
        self._PickPlace = PickPlace
        self._clients = {
            "white": ActionClient(self.node, PickPlace, "/white_panda/pick_place"),
            "black": ActionClient(self.node, PickPlace, "/black_panda/pick_place"),
        }

    def initialise(self):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def update(self):
        if not self._clients:
            self.feedback_message = "chess_interfaces not available"
            return py_trees.common.Status.FAILURE

        # send the goal to the active arm
        if self._goal_future is None:
            color = self.bb.get(ACTIVE_PLAYER)
            client = self._clients.get(color)
            if client is None:
                self.feedback_message = f"unknown active player: {color!r}"
                return py_trees.common.Status.FAILURE
            if not client.server_is_ready():
                self.feedback_message = f"/{color}_panda/pick_place not available"
                return py_trees.common.Status.FAILURE

            task = self.bb.get(CURRENT_TASK) or {}
            grasp = self.bb.get(CURRENT_GRASP) or {}

            goal = self._PickPlace.Goal()
            goal.task_type = task.get("task_type", "")
            goal.piece_char = task.get("piece_char", "")
            # Pick pose comes from the grasp planner's computed gripper pose.
            goal.pick_position = list(grasp.get("position", [0.0, 0.0, 0.0]))
            goal.pick_orientation = list(grasp.get("orientation", [0.0, 0.0, 0.0, 1.0]))
            # Place pose comes from the task dict (yaw → xyzw quaternion).
            place_pose = task.get("place_pose", {}) or {}
            goal.place_position = [
                float(place_pose.get("x", 0.0)),
                float(place_pose.get("y", 0.0)),
                float(place_pose.get("z", 0.0)),
            ]
            goal.place_orientation = _yaw_to_quat(place_pose.get("yaw", 0.0))
            goal.finger_separation = float(grasp.get("finger_separation", 0.0))
            goal.grasp_height = float(task.get("grasp_height", 0.0))
            goal.use_cartesian = True  # arch §2.6 default

            self._goal_future = client.send_goal_async(goal)
            return py_trees.common.Status.RUNNING

        # wait for the server to accept the goal
        if self._goal_handle is None:
            if not self._goal_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._goal_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                self.feedback_message = "pick/place goal rejected"
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # wait for the action result
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        wrapped = self._result_future.result()
        if wrapped is None:
            self.feedback_message = "pick/place result future returned None"
            return py_trees.common.Status.FAILURE
        result = wrapped.result
        if not result.success:
            self.feedback_message = f"pick/place failed: {result.failure_reason}"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class VerifyBoard(py_trees.behaviour.Behaviour):
    """Compare /chess/occupancy against the expected occupancy from BOARD_STATE_FEN.

    The perception module publishes an 8x8 UInt8MultiArray at 5 Hz where
    data[rank*8 + file] = 1 if the square is occupied. We derive the expected
    occupancy from the post-move FEN and check for matches.

    Depends on: perception (publishes /chess/occupancy)

    Reads:        BOARD_STATE_FEN, OCCUPANCY_MSG
    Writes (on init): OCCUPANCY_MSG (cleared so we wait for a fresh post-move reading)
    """

    _SETTLE_SEC = 1.0   # let perception's 5 Hz publisher catch up after the arm moves
    _TIMEOUT_SEC = 5.0
    _MAX_MISMATCHES = 0  # tighten/loosen as we learn perception's false-positive rate

    def __init__(self, node: Node, name: str = "VerifyBoard"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(BOARD_STATE_FEN, access=py_trees.common.Access.READ)
        self.bb.register_key(OCCUPANCY_MSG, access=py_trees.common.Access.WRITE)
        self._earliest_check_at = 0
        self._deadline = 0

    def initialise(self):
        # Discard any stale reading so we definitely compare against a sample
        # taken AFTER the arm finished moving.
        self.bb.set(OCCUPANCY_MSG, None)
        now = self.node.get_clock().now().nanoseconds
        self._earliest_check_at = now + int(self._SETTLE_SEC * 1e9)
        self._deadline = now + int(self._TIMEOUT_SEC * 1e9)

    def update(self):
        now = self.node.get_clock().now().nanoseconds
        msg = self.bb.get(OCCUPANCY_MSG)

        # Settle window: even if a fresh msg arrived, give perception a beat
        # to publish a post-arm-motion reading rather than a mid-motion one.
        if now < self._earliest_check_at or msg is None:
            if now > self._deadline:
                self.feedback_message = "timeout waiting for /chess/occupancy"
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        fen = self.bb.get(BOARD_STATE_FEN)
        if not fen:
            self.feedback_message = "no FEN on /board_state yet"
            return py_trees.common.Status.FAILURE

        try:
            expected = self._fen_to_occupancy(fen)
        except (IndexError, ValueError) as e:
            self.feedback_message = f"bad FEN: {e}"
            return py_trees.common.Status.FAILURE

        actual = list(msg.data)
        if len(actual) != 64:
            self.feedback_message = f"unexpected occupancy length: {len(actual)}"
            return py_trees.common.Status.FAILURE

        mismatches = sum(1 for e, a in zip(expected, actual) if (e != 0) != (a != 0))
        if mismatches <= self._MAX_MISMATCHES:
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"board mismatch: {mismatches} square(s) disagree with FEN"
        return py_trees.common.Status.FAILURE

    @staticmethod
    def _fen_to_occupancy(fen: str) -> list:
        """FEN board portion → 64-element occupancy list (a1=0, h1=7, a8=56, h8=63).

        Indexing matches perception's: data[rank_idx * 8 + file_idx], rank_idx=0
        is rank 1, file_idx=0 is file a.
        """
        board_part = fen.split()[0]
        ranks = board_part.split("/")
        if len(ranks) != 8:
            raise ValueError(f"FEN board section has {len(ranks)} ranks, expected 8")
        occupancy = [0] * 64
        for rank_str_idx, rank_str in enumerate(ranks):
            rank_idx = 7 - rank_str_idx  # FEN lists rank 8 first
            file_idx = 0
            for ch in rank_str:
                if ch.isdigit():
                    file_idx += int(ch)
                else:
                    occupancy[rank_idx * 8 + file_idx] = 1
                    file_idx += 1
        return occupancy
