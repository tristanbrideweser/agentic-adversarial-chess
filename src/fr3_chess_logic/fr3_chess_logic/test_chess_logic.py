# test_chess_logic.py
#
# Pytest unit tests for chess_logic package.
# Tests fen_parser, board_state_node logic, and stockfish_node engine.
#
# Run with:
#   pip install pytest chess
#   pytest test_chess_logic.py -v

import pytest
import chess
import chess.engine


# ─────────────────────────────────────────────
# fen_parser tests
# ─────────────────────────────────────────────

from fr3_chess_logic.fen_parser import parse_fen

class TestFenParser:

    def test_starting_position(self):
        """Standard starting FEN should return a valid board."""
        fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        board = parse_fen(fen)
        assert isinstance(board, chess.Board)
        assert board.fullmove_number == 1

    def test_midgame_position(self):
        """A mid-game FEN should parse correctly and reflect correct turn."""
        fen = "r1bqkb1r/pppp1ppp/2n2n2/4p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4"
        board = parse_fen(fen)
        assert board.turn == chess.WHITE

    def test_black_to_move(self):
        """FEN with black to move should reflect that."""
        fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"
        board = parse_fen(fen)
        assert board.turn == chess.BLACK

    def test_invalid_fen_raises(self):
        """An invalid FEN string should raise a ValueError."""
        with pytest.raises(ValueError):
            parse_fen("this is not a fen")

    def test_empty_string_raises(self):
        """An empty string should raise a ValueError."""
        with pytest.raises(ValueError):
            parse_fen("")


# ─────────────────────────────────────────────
# board_state_node logic tests
# (tests the chess logic directly, no ROS2 needed)
# ─────────────────────────────────────────────

class TestBoardStateLogic:

    def setup_method(self):
        """Fresh board before each test."""
        self.board = chess.Board()

    def test_legal_move_is_accepted(self):
        """A legal move should be pushable onto the board."""
        move = chess.Move.from_uci("e2e4")
        assert self.board.is_legal(move)
        self.board.push(move)
        assert self.board.fullmove_number == 1  # black hasn't moved yet

    def test_illegal_move_is_rejected(self):
        """An illegal move should not be legal on the starting board."""
        move = chess.Move.from_uci("e2e5")  # pawn can't jump 3 squares
        assert not self.board.is_legal(move)

    def test_board_publishes_fen_after_move(self):
        """After a move, board.fen() should reflect the new position."""
        starting_fen = self.board.fen()
        self.board.push(chess.Move.from_uci("e2e4"))
        assert self.board.fen() != starting_fen

    def test_reset_from_fen(self):
        """Board can be reset to a specific FEN position."""
        fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"
        self.board = chess.Board(fen)
        assert self.board.turn == chess.BLACK

    def test_game_over_detection(self):
        """Fool's mate should be detected as game over."""
        # Fool's mate - fastest possible checkmate
        moves = ["f2f3", "e7e5", "g2g4", "d8h4"]
        for uci in moves:
            self.board.push(chess.Move.from_uci(uci))
        assert self.board.is_game_over()
        assert self.board.is_checkmate()

    def test_turn_alternates_after_move(self):
        """After white moves, it should be black's turn."""
        assert self.board.turn == chess.WHITE
        self.board.push(chess.Move.from_uci("e2e4"))
        assert self.board.turn == chess.BLACK


# ─────────────────────────────────────────────
# stockfish_node engine tests
# (tests the Stockfish engine directly, no ROS2 needed)
# ─────────────────────────────────────────────

STOCKFISH_PATH = "/usr/games/stockfish"

class TestStockfishEngine:

    def setup_method(self):
        """Start a fresh engine before each test."""
        self.engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)

    def teardown_method(self):
        """Always shut the engine down cleanly."""
        self.engine.quit()

    def test_engine_returns_a_move(self):
        """Stockfish should return a move for the starting position."""
        board = chess.Board()
        result = self.engine.play(board, chess.engine.Limit(time=0.1))
        assert result.move is not None
        assert isinstance(result.move, chess.Move)

    def test_move_is_legal(self):
        """The move Stockfish returns should be legal."""
        board = chess.Board()
        result = self.engine.play(board, chess.engine.Limit(time=0.1))
        assert board.is_legal(result.move)

    def test_skill_level_can_be_set(self):
        """Engine should accept a skill level without errors."""
        self.engine.configure({"Skill Level": 5})
        board = chess.Board()
        result = self.engine.play(board, chess.engine.Limit(time=0.1))
        assert result.move is not None

    def test_engine_handles_midgame(self):
        """Stockfish should return a valid move from a mid-game position."""
        fen = "r1bqkb1r/pppp1ppp/2n2n2/4p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4"
        board = chess.Board(fen)
        result = self.engine.play(board, chess.engine.Limit(time=0.1))
        assert board.is_legal(result.move)

    def test_engine_move_as_uci_string(self):
        """The move should be expressible as a UCI string like 'e2e4'."""
        board = chess.Board()
        result = self.engine.play(board, chess.engine.Limit(time=0.1))
        uci = result.move.uci()
        assert len(uci) >= 4  # e.g. "e2e4" or "e7e8q" for promotion