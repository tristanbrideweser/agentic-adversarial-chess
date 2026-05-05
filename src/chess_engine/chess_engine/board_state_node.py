"""Authoritative chess board state.

Publishes:
  /board_state (std_msgs/String)  — current FEN, on every successful apply + at 1 Hz
  /game_over   (std_msgs/String)  — termination reason once the game ends

Subscribes:
  /apply_move  (std_msgs/String)  — UCI move to validate and apply
  /reset_board (std_msgs/String)  — FEN string to reset the board to

All other nodes treat the FEN published on /board_state as the single source
of truth.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import chess
from std_msgs.msg import String

from chess_engine.fen_parser import parse_fen

# /board_state is the authoritative game-state plane. Publish with
# TRANSIENT_LOCAL durability so late-joining subscribers (e.g. move_translator
# coming up after the first move was applied) immediately receive the most
# recent FEN without waiting for the next 1 Hz tick.
_VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class BoardStateNode(Node):
    def __init__(self):
        super().__init__('board_state_node')
        self.board = chess.Board()

        self.state_pub = self.create_publisher(String, '/board_state', _VOLATILE_QOS)
        self.game_over_pub = self.create_publisher(String, '/game_over', 10)

        self.create_timer(1.0, self.publish_board_state)
        self.create_subscription(String, '/apply_move', self.apply_move_callback, 10)
        self.create_subscription(String, '/reset_board', self.apply_reset_callback, 10)

        # Publish the initial FEN so downstream subscribers see the start position
        # without waiting for the 1 Hz timer.
        self.publish_board_state()

    def publish_board_state(self):
        self.state_pub.publish(String(data=self.board.fen()))

    def apply_move_callback(self, msg: String) -> None:
        move_uci = msg.data
        try:
            move = chess.Move.from_uci(move_uci)
        except ValueError:
            self.get_logger().error(f'Invalid UCI format: {move_uci}')
            return

        if not self.board.is_legal(move):
            self.get_logger().warn(f'Illegal move rejected: {move_uci}')
            return

        self.board.push(move)
        # Publish immediately so downstream consumers (move_translator,
        # game_coordinator) don't wait up to 1 s for the next timer tick.
        self.publish_board_state()
        self.get_logger().info(f'Move applied: {move_uci}')

        if self.board.is_game_over():
            outcome = self.board.outcome()
            reason = outcome.termination.name.lower() if outcome else 'unknown'
            self.game_over_pub.publish(String(data=reason))
            self.get_logger().info(f'Game over: {reason}')

    def apply_reset_callback(self, msg: String) -> None:
        try:
            self.board = parse_fen(msg.data)
        except ValueError as e:
            self.get_logger().error(f'Invalid FEN: {e}')
            return
        self.publish_board_state()
        self.get_logger().info(f'Board reset to: {msg.data}')


def main():
    rclpy.init()
    node = BoardStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
