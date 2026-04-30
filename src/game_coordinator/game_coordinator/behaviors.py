"""py_trees leaves  and blackboard registry for the chess game coordinator.
"""

import json

import py_trees
from rclpy.node import Node
from std_msgs.msg import String

from chess_engine_interfaces.srv import GetMove

# --- blackboard keys -------------------------------------------------------
# Continuously written by the data-gathering parallel branch (subscribers).
BOARD_STATE_FEN = "/board_state_fen"   # str  — live FEN from /board_state
GAME_OVER       = "/game_over"          # str  — termination reason; "" while game continues
TASK_QUEUE_MSG  = "/task_queue_msg"     # std_msgs/String — last raw msg from /pick_place_tasks

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
            queue = json.loads(msg.data)
        except (AttributeError, ValueError) as e:
            self.feedback_message = f"bad task queue payload: {e}"
            return py_trees.common.Status.FAILURE
        if not isinstance(queue, list) or not queue:
            self.feedback_message = "task queue empty or not a list"
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
        self._pub = self.node.create_publisher(String, "/apply_move", 10)

    def update(self):
        move = self.bb.get(CURRENT_MOVE)
        if not move:
            self.feedback_message = "CURRENT_MOVE is empty"
            return py_trees.common.Status.FAILURE
        self._pub.publish(String(data=move))
        return py_trees.common.Status.SUCCESS


class RequestMove(py_trees.behaviour.Behaviour):
    """Call /stockfish/{active_player}/get_move and write UCI to CURRENT_MOVE.

    Depends on: chess_engine (stockfish_node service, type TBD)

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
        # TODO(chess_engine branch): create one service client per color:
        #   /stockfish/white/get_move, /stockfish/black/get_move
        # using the request/response type defined by the chess_engine package.
        pass

    def update(self):
        raise NotImplementedError(
            "RequestMove: implement once chess_engine branch defines the service type"
        )


class PlanGrasp(py_trees.behaviour.Behaviour):
    """Call /grasp_planner/plan with CURRENT_TASK → CURRENT_GRASP.

    Depends on: grasp_planner (action server, goal/result types TBD)

    Reads:  CURRENT_TASK
    Writes: CURRENT_GRASP
    """

    def __init__(self, node: Node, name: str = "PlanGrasp"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(CURRENT_TASK, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_GRASP, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        # TODO(grasp_planner branch): create ActionClient for /grasp_planner/plan.
        pass

    def update(self):
        raise NotImplementedError(
            "PlanGrasp: implement once grasp_planner branch defines the action type"
        )


class ExecutePickPlace(py_trees.behaviour.Behaviour):
    """Send the pick/place goal to the active arm's controller and wait for result.

    Depends on: arm_controller (one action server per arm:
                /white_panda/pick_place, /black_panda/pick_place)

    Reads: ACTIVE_PLAYER, CURRENT_TASK, CURRENT_GRASP
    """

    def __init__(self, node: Node, name: str = "ExecutePickPlace"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(ACTIVE_PLAYER, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_TASK, access=py_trees.common.Access.READ)
        self.bb.register_key(CURRENT_GRASP, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        # TODO(arm_controller branch): create two ActionClients keyed by color
        #   so update() can dispatch based on ACTIVE_PLAYER.
        pass

    def update(self):
        raise NotImplementedError(
            "ExecutePickPlace: implement once arm_controller branch defines the action type"
        )


class VerifyBoard(py_trees.behaviour.Behaviour):
    """Call /perception/verify_board with the post-move FEN. SUCCESS iff vision agrees.

    Depends on: perception (verify_board service)

    Reads: BOARD_STATE_FEN
    """

    def __init__(self, node: Node, name: str = "VerifyBoard"):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Client(name=name)
        self.bb.register_key(BOARD_STATE_FEN, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        # TODO(perception branch): create service client for /perception/verify_board.
        pass

    def update(self):
        raise NotImplementedError(
            "VerifyBoard: implement once perception branch defines the service type"
        )
