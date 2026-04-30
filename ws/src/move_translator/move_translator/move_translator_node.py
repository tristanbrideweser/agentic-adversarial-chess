"""
move_translator_node.py
=======================
ROS 2 node that bridges the symbolic chess game (FEN / UCI) and the physical
robot workspace (world-frame pick-and-place task queues).

Subscriptions:
    /board_state   (std_msgs/String)  — FEN string, published by board_state_node
    /apply_move    (std_msgs/String)  — UCI move string, published by coordinator

Publications:
    /pick_place_tasks (std_msgs/String) — JSON-encoded ordered task queue

The node caches the FEN *before* the move is applied (Option B from the spec)
and infers the active arm from the piece colour on the source square.

ROS 2 parameters (all overridable via board_params.yaml or CLI):
    publish_rate_hz   : float  (default 0.0 — only publish on change)
    use_tf_poses      : bool   (default False — use analytic geometry, not TF)
"""

from __future__ import annotations

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

try:
    import chess
except ImportError as exc:
    raise ImportError(
        "python-chess is required: pip install chess"
    ) from exc

from .board_geometry import GraveyardAllocator, ReserveRegistry
from .move_decomposer import decompose_move, task_queue_to_json
from .special_moves import determine_active_arm


# ---------------------------------------------------------------------------
# QoS profile — reliable + transient-local so late joiners get last message
# ---------------------------------------------------------------------------
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)


class MoveTranslatorNode(Node):
    """
    Translates FEN + UCI move into a physical pick-and-place task queue.
    """

    def __init__(self) -> None:
        super().__init__("move_translator")

        # ---------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------
        self.declare_parameter("publish_rate_hz", 0.0)
        self.declare_parameter("use_tf_poses", False)

        # ---------------------------------------------------------------
        # State
        # ---------------------------------------------------------------
        self._lock = threading.Lock()
        self._cached_fen: Optional[str] = None    # FEN before move applied
        self._graveyard = GraveyardAllocator()
        self._reserve = ReserveRegistry()

        # ---------------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------------
        self._board_sub = self.create_subscription(
            String,
            "/board_state",
            self._on_board_state,
            _RELIABLE_QOS,
        )

        self._move_sub = self.create_subscription(
            String,
            "/apply_move",
            self._on_apply_move,
            _RELIABLE_QOS,
        )

        # ---------------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------------
        self._task_pub = self.create_publisher(
            String,
            "/pick_place_tasks",
            _RELIABLE_QOS,
        )

        self.get_logger().info("MoveTranslatorNode started — waiting for board state.")

    # -------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------

    def _on_board_state(self, msg: String) -> None:
        """
        Cache the latest FEN.  This arrives *after* board_state_node has
        already applied the move, but we always store it so we can use the
        pre-move FEN on the next cycle (see _on_apply_move).
        """
        with self._lock:
            self._cached_fen = msg.data.strip()

    def _on_apply_move(self, msg: String) -> None:
        """
        Decompose the UCI move into a task queue and publish it.

        IMPORTANT: We use the FEN cached during the *previous* board_state
        publication, which is the state *before* this move was applied.
        This implements Option B from the architecture spec.
        """
        uci = msg.data.strip()

        with self._lock:
            pre_move_fen = self._cached_fen

        if pre_move_fen is None:
            self.get_logger().warn(
                f"Received UCI move '{uci}' but no FEN cached yet. "
                "Waiting for /board_state to arrive first."
            )
            return

        self.get_logger().info(f"Translating move '{uci}' from FEN: {pre_move_fen}")

        try:
            board = chess.Board(pre_move_fen)
            move = chess.Move.from_uci(uci)

            if move not in board.legal_moves:
                self.get_logger().error(
                    f"Move '{uci}' is not legal in position:\n{pre_move_fen}"
                )
                return

            tasks = decompose_move(board, move, self._graveyard, self._reserve)
            active_arm = determine_active_arm(pre_move_fen, uci)

            payload = {
                "active_arm": active_arm,
                "uci": uci,
                "tasks": [t.to_dict() for t in tasks],
            }
            json_str = json.dumps(payload, indent=2)

            out_msg = String()
            out_msg.data = json_str
            self._task_pub.publish(out_msg)

            self.get_logger().info(
                f"Published {len(tasks)} task(s) for '{uci}' "
                f"(arm: {active_arm})"
            )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to decompose move '{uci}': {exc}",
                exc_info=True,
            )

    # -------------------------------------------------------------------
    # Game control
    # -------------------------------------------------------------------

    def reset(self, new_fen: Optional[str] = None) -> None:
        """
        Reset state for a new game (or board reset mid-game).

        Parameters
        ----------
        new_fen : optional starting FEN (defaults to standard start position).
        """
        with self._lock:
            self._cached_fen = new_fen or chess.STARTING_FEN
            self._graveyard.reset()
            self._reserve.reset()
        self.get_logger().info("MoveTranslatorNode reset for new game.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveTranslatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()