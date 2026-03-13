# Publishes the current board as a FEN string on /board_state topic.
# Exposes two topics:
#   - /reset_board: resets the board from a given FEN string
#   - /apply_move: validates and applies a UCI move string to the board
# All other nodes read from and write to this node to interact with the game state.

import rclpy
from rclpy.node import Node
import chess
from fr3_chess_logic.fen_parser import parse_fen
from std_msgs.msg import String

class BoardStateNode(Node):
    # Constructor
    def __init__(self):
        super().__init__('board_state_node')
        self.board = chess.Board()
        self.publisher = self.create_publisher(String, '/board_state', 10)
        self.create_timer(1.0, self.publish_board_state)
        self.sub_move = self.create_subscription(String, '/move_command', self.apply_move_callback, 10)
        self.sub_reset = self.create_subscription(String, '/reset_board', self.apply_reset_callback, 10)
    
    def publish_board_state(self):
        msg = String()
        msg.data = self.board.fen()
        self.publisher.publish(msg)
    
    def apply_move_callback(self, msg):
        move_uci = msg.data
        try:
            move = chess.Move.from_uci(move_uci)
            if self.board.is_legal(move):
                self.board.push(move)
                self.get_logger().info(f'Move applied: {move_uci}')
                if self.board.is_checkmate():
                    self.get_logger().info('Game over - Checkmate')
                   # TODO: publish to /game_over topic to stop centralized controller and arms
                elif self.board.is_stalemate():
                    self.get_logger().info('Game over - Stalemate')
                    # TODO: publish to /game_over topic to stop centralized controller and arms
                elif self.board.is_insufficient_material():
                    self.get_logger().info('Game over - Draw by insufficient material')
                    # TODO: publish to /game_over topic to stop centralized controller and arms
            else:
                self.get_logger().warn(f'Illegal move rejected: {move_uci}')
        except ValueError:
            self.get_logger().error(f'Invalid UCI format: {move_uci}')
        
    def apply_reset_callback(self, msg):
        move_fen = msg.data
        try:
            self.board = parse_fen(move_fen)
            self.get_logger().info(f'Board reset to: {move_fen}')
        except ValueError as e:
            self.get_logger().error(f'Invalid FEN: {e}')
        
def main():
    rclpy.init()
    node = BoardStateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()