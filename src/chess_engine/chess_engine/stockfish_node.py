"""Stockfish service.

Exposes /stockfish/{color}/get_move (chess_interfaces/srv/GetMove):
  Request : FEN string of the position to evaluate
  Response: best UCI move + success flag + optional reason

Run two instances, one per color:
  ros2 run chess_engine stockfish_node --ros-args -p color:=white -p skill_level:=10
  ros2 run chess_engine stockfish_node --ros-args -p color:=black -p skill_level:=10
"""

import rclpy
from rclpy.node import Node
import chess
import chess.engine

from chess_interfaces.srv import GetMove


class StockfishNode(Node):
    def __init__(self):
        super().__init__('stockfish_node')

        self.declare_parameter('color', 'white')
        self.declare_parameter('skill_level', 10)
        self.declare_parameter('time_limit', 2.0)
        self.declare_parameter('stockfish_path', '/usr/bin/stockfish')

        color = self.get_parameter('color').value
        skill_level = self.get_parameter('skill_level').value
        self._time_limit = self.get_parameter('time_limit').value
        engine_path = self.get_parameter('stockfish_path').value

        self.engine = chess.engine.SimpleEngine.popen_uci(engine_path)
        self.engine.configure({"Skill Level": skill_level})
        
        self.srv = self.create_service(
            GetMove, f'/stockfish/{color}/get_move', self.get_move_callback)

        self.get_logger().info(
            f'Stockfish service ready: /stockfish/{color}/get_move '
            f'(skill={skill_level}, time={self._time_limit:.1f}s)')

    def get_move_callback(self, request, response):
        try:
            board = chess.Board(request.fen)
        except ValueError as e:
            response.uci = ''
            response.success = False
            response.reason = f'invalid fen: {e}'
            return response

        if board.is_game_over():
            response.uci = ''
            response.success = False
            response.reason = 'game over'
            return response

        try:
            result = self.engine.play(board, chess.engine.Limit(time=self._time_limit))
        except chess.engine.EngineError as e:
            response.uci = ''
            response.success = False
            response.reason = f'engine error: {e}'
            return response

        if result.move is None:
            response.uci = ''
            response.success = False
            response.reason = 'no move returned'
            return response

        response.uci = result.move.uci()
        response.success = True
        response.reason = ''
        return response

    def destroy_node(self):
        # Cleanly shut down the Stockfish subprocess so we don't leak it.
        try:
            self.engine.quit()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = StockfishNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
