"""
test_special_moves.py
=====================
Unit tests for all four special move categories.  No ROS required.

Includes the exact test cases from architecture document §9.3.

Run with:
    pytest move_translator/test/test_special_moves.py -v
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import chess
import pytest

from src.move_translator.move_translator.board_geometry import GraveyardAllocator, ReserveRegistry
from src.move_translator.move_translator.move_decomposer import decompose_move
from src.move_translator.move_translator.special_moves import (
    MoveClass,
    classify_move,
    determine_active_arm,
    get_castling_info,
    get_en_passant_info,
    get_promotion_info,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def fresh() -> tuple[GraveyardAllocator, ReserveRegistry]:
    return GraveyardAllocator(), ReserveRegistry()


# ---------------------------------------------------------------------------
# classify_move
# ---------------------------------------------------------------------------

class TestClassifyMove:
    def test_standard(self):
        board = chess.Board()
        assert classify_move(board, chess.Move.from_uci("e2e4")) == MoveClass.STANDARD

    def test_capture(self):
        # Scholar's mate: Bxf7 is a capture
        fen = "rnb1kbnr/pppp1ppp/8/4p3/2B1P3/8/PPPP1PPP/RNBQK1NR w KQkq - 0 3"
        board = chess.Board(fen)
        assert classify_move(board, chess.Move.from_uci("c4f7")) == MoveClass.CAPTURE

    def test_kingside_castling(self):
        fen = "r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1"
        board = chess.Board(fen)
        assert classify_move(board, chess.Move.from_uci("e1g1")) == MoveClass.CASTLING

    def test_queenside_castling(self):
        fen = "r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1"
        board = chess.Board(fen)
        assert classify_move(board, chess.Move.from_uci("e1c1")) == MoveClass.CASTLING

    def test_en_passant(self):
        fen = "rnbqkbnr/pppp1ppp/8/4pP2/8/8/PPPPP1PP/RNBQKBNR w KQkq e6 0 3"
        board = chess.Board(fen)
        assert classify_move(board, chess.Move.from_uci("f5e6")) == MoveClass.EN_PASSANT

    def test_promotion(self):
        fen = "8/P7/8/8/8/8/8/4K2k w - - 0 1"
        board = chess.Board(fen)
        assert classify_move(board, chess.Move.from_uci("a7a8q")) == MoveClass.PROMOTION

    def test_promotion_capture(self):
        fen = "1b6/P7/8/8/8/8/8/4K2k w - - 0 1"
        board = chess.Board(fen)
        move = chess.Move.from_uci("a7b8q")
        assert classify_move(board, move) == MoveClass.PROMOTION_CAPTURE

    def test_illegal_move_raises(self):
        board = chess.Board()
        with pytest.raises(ValueError, match="not legal"):
            classify_move(board, chess.Move.from_uci("e2e5"))


# ---------------------------------------------------------------------------
# En passant — architecture spec §9.3
# ---------------------------------------------------------------------------

class TestEnPassant:
    # FEN from architecture doc §9.3
    EP_FEN = "rnbqkbnr/pppp1ppp/8/4pP2/8/8/PPPPP1PP/RNBQKBNR w KQkq e6 0 3"

    def test_spec_en_passant(self):
        """Exact test from architecture document §9.3."""
        board = chess.Board(self.EP_FEN)
        move = chess.Move.from_uci("f5e6")
        tasks = decompose_move(board, move, GraveyardAllocator())

        assert len(tasks) == 2
        assert tasks[0].pick_square == "e5"       # captured pawn — NOT e6!
        assert tasks[0].task_type == "remove_piece"
        assert tasks[1].pick_square == "f5"
        assert tasks[1].place_square == "e6"

    def test_captured_pawn_not_on_target_square(self):
        """The captured pawn must be on c5/e5/d5 etc, never the landing square."""
        board = chess.Board(self.EP_FEN)
        move = chess.Move.from_uci("f5e6")
        tasks = decompose_move(board, move, GraveyardAllocator())
        assert tasks[0].pick_square != "e6"

    def test_captured_piece_is_pawn(self):
        board = chess.Board(self.EP_FEN)
        move = chess.Move.from_uci("f5e6")
        tasks = decompose_move(board, move, GraveyardAllocator())
        assert tasks[0].piece_char.lower() == "p"

    def test_get_en_passant_info(self):
        board = chess.Board(self.EP_FEN)
        move = chess.Move.from_uci("f5e6")
        info = get_en_passant_info(board, move)

        assert info.attacker_from == "f5"
        assert info.attacker_to == "e6"
        assert info.captured_sq == "e5"
        assert info.attacker_char == "P"
        assert info.captured_char == "p"

    def test_en_passant_wrong_move_raises(self):
        board = chess.Board()
        with pytest.raises(ValueError, match="not en passant"):
            get_en_passant_info(board, chess.Move.from_uci("e2e4"))

    def test_black_en_passant(self):
        # Black pawn on e4 captures white pawn that just double-advanced to d4.
        # En passant target square is d3; captured white pawn sits on d4.
        # FEN: 3pP3 on rank 4 means d4=black pawn, e4=white pawn.
        # We need the reverse: white pawn just advanced to d4, black pawn on e4.
        # Correct FEN: white pawn on d4, black pawn on e4, ep square = d3
        fen = "rnbqkbnr/ppp1pppp/8/8/3Pp3/8/PPP1PPPP/RNBQKBNR b KQkq d3 0 3"
        board = chess.Board(fen)
        move = chess.Move.from_uci("e4d3")
        tasks = decompose_move(board, move, GraveyardAllocator())

        assert len(tasks) == 2
        assert tasks[0].pick_square == "d4"   # captured white pawn
        assert tasks[0].piece_char == "P"     # uppercase = white pawn
        assert tasks[1].pick_square == "e4"
        assert tasks[1].place_square == "d3"


# ---------------------------------------------------------------------------
# Castling — architecture spec §9.3
# ---------------------------------------------------------------------------

class TestCastling:
    CASTLING_FEN = "r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1"

    def test_spec_kingside_castling(self):
        """Exact test from architecture document §9.3."""
        board = chess.Board(self.CASTLING_FEN)
        move = chess.Move.from_uci("e1g1")
        tasks = decompose_move(board, move, GraveyardAllocator())

        assert len(tasks) == 2
        assert tasks[0].pick_square == "e1"   # king
        assert tasks[0].place_square == "g1"
        assert tasks[1].pick_square == "h1"   # rook
        assert tasks[1].place_square == "f1"

    def test_queenside_castling(self):
        board = chess.Board(self.CASTLING_FEN)
        move = chess.Move.from_uci("e1c1")
        tasks = decompose_move(board, move, GraveyardAllocator())

        assert len(tasks) == 2
        assert tasks[0].pick_square == "e1"
        assert tasks[0].place_square == "c1"
        assert tasks[1].pick_square == "a1"
        assert tasks[1].place_square == "d1"

    def test_black_kingside_castling(self):
        board = chess.Board(self.CASTLING_FEN)
        # Give black the move
        board.push(chess.Move.from_uci("a2a3"))
        move = chess.Move.from_uci("e8g8")
        tasks = decompose_move(board, move, GraveyardAllocator())

        assert len(tasks) == 2
        assert tasks[0].pick_square == "e8"
        assert tasks[0].place_square == "g8"
        assert tasks[1].pick_square == "h8"
        assert tasks[1].place_square == "f8"

    def test_black_queenside_castling(self):
        board = chess.Board(self.CASTLING_FEN)
        board.push(chess.Move.from_uci("a2a3"))
        move = chess.Move.from_uci("e8c8")
        tasks = decompose_move(board, move, GraveyardAllocator())

        assert tasks[0].pick_square == "e8"
        assert tasks[0].place_square == "c8"
        assert tasks[1].pick_square == "a8"
        assert tasks[1].place_square == "d8"

    def test_king_moves_first(self):
        board = chess.Board(self.CASTLING_FEN)
        move = chess.Move.from_uci("e1g1")
        tasks = decompose_move(board, move, GraveyardAllocator())
        # First task must be the king
        assert tasks[0].piece_char.upper() == "K"
        assert tasks[1].piece_char.upper() == "R"

    def test_castling_produces_no_graveyard_tasks(self):
        board = chess.Board(self.CASTLING_FEN)
        move = chess.Move.from_uci("e1g1")
        tasks = decompose_move(board, move, GraveyardAllocator())
        for t in tasks:
            assert t.task_type == "pick_and_place"

    def test_get_castling_info_kingside(self):
        board = chess.Board(self.CASTLING_FEN)
        move = chess.Move.from_uci("e1g1")
        info = get_castling_info(board, move)
        assert info.is_kingside is True
        assert info.king_from == "e1"
        assert info.king_to == "g1"
        assert info.rook_from == "h1"
        assert info.rook_to == "f1"

    def test_get_castling_info_queenside(self):
        board = chess.Board(self.CASTLING_FEN)
        move = chess.Move.from_uci("e1c1")
        info = get_castling_info(board, move)
        assert info.is_kingside is False
        assert info.rook_from == "a1"
        assert info.rook_to == "d1"


# ---------------------------------------------------------------------------
# Promotion
# ---------------------------------------------------------------------------

class TestPromotion:
    PROMO_FEN = "8/P7/8/8/8/8/8/4K2k w - - 0 1"

    def test_promotion_three_tasks(self):
        board = chess.Board(self.PROMO_FEN)
        move = chess.Move.from_uci("a7a8q")
        g, r = fresh()
        tasks = decompose_move(board, move, g, r)

        assert len(tasks) == 2  # remove pawn, place queen

    def test_promotion_remove_pawn_first(self):
        board = chess.Board(self.PROMO_FEN)
        move = chess.Move.from_uci("a7a8q")
        g, r = fresh()
        tasks = decompose_move(board, move, g, r)

        assert tasks[0].task_type == "remove_piece"
        assert tasks[0].piece_char == "P"
        assert tasks[0].pick_square == "a7"

    def test_promotion_place_reserve_last(self):
        board = chess.Board(self.PROMO_FEN)
        move = chess.Move.from_uci("a7a8q")
        g, r = fresh()
        tasks = decompose_move(board, move, g, r)

        assert tasks[1].task_type == "place_reserve"
        assert tasks[1].place_square == "a8"
        assert tasks[1].piece_char == "Q"

    def test_promotion_to_rook(self):
        board = chess.Board(self.PROMO_FEN)
        move = chess.Move.from_uci("a7a8r")
        g, r = fresh()
        tasks = decompose_move(board, move, g, r)
        assert tasks[-1].piece_char == "R"

    def test_black_promotion(self):
        fen = "4k3/8/8/8/8/8/7p/4K3 b - - 0 1"
        board = chess.Board(fen)
        move = chess.Move.from_uci("h2h1q")
        g, r = fresh()
        tasks = decompose_move(board, move, g, r)

        assert tasks[0].piece_char == "p"   # black pawn removed
        assert tasks[-1].piece_char == "q"  # black queen placed

    def test_promotion_with_capture(self):
        fen = "1b6/P7/8/8/8/8/8/4K2k w - - 0 1"
        board = chess.Board(fen)
        move = chess.Move.from_uci("a7b8q")
        g, r = fresh()
        tasks = decompose_move(board, move, g, r)

        # Capture + remove pawn + place queen = 3 tasks
        assert len(tasks) == 3
        # First: remove captured piece from b8
        assert tasks[0].task_type == "remove_piece"
        assert tasks[0].pick_square == "b8"
        # Second: remove promoting pawn
        assert tasks[1].task_type == "remove_piece"
        assert tasks[1].pick_square == "a7"
        # Third: place reserve queen
        assert tasks[2].task_type == "place_reserve"
        assert tasks[2].place_square == "b8"

    def test_get_promotion_info(self):
        board = chess.Board(self.PROMO_FEN)
        move = chess.Move.from_uci("a7a8q")
        info = get_promotion_info(board, move)

        assert info.pawn_from == "a7"
        assert info.promotion_sq == "a8"
        assert info.pawn_char == "P"
        assert info.promoted_char == "Q"
        assert info.is_capture is False
        assert info.captured_char is None

    def test_get_promotion_info_capture(self):
        fen = "1b6/P7/8/8/8/8/8/4K2k w - - 0 1"
        board = chess.Board(fen)
        move = chess.Move.from_uci("a7b8q")
        info = get_promotion_info(board, move)

        assert info.is_capture is True
        assert info.captured_char == "b"

    def test_no_reserve_raises(self):
        """If a reserve is exhausted, decompose_move should raise KeyError."""
        board = chess.Board(self.PROMO_FEN)
        move = chess.Move.from_uci("a7a8q")
        g = GraveyardAllocator()
        r = ReserveRegistry()
        # Drain the white queen reserve
        r.get_reserve("white", "queen")

        with pytest.raises(KeyError, match="No reserve"):
            decompose_move(board, move, g, r)


# ---------------------------------------------------------------------------
# Active arm determination
# ---------------------------------------------------------------------------

class TestActiveArm:
    def test_white_pawn_advance(self):
        arm = determine_active_arm(chess.STARTING_FEN, "e2e4")
        assert arm == "white"

    def test_black_pawn_advance(self):
        # After 1. e4
        fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"
        arm = determine_active_arm(fen, "e7e5")
        assert arm == "black"

    def test_white_castling(self):
        fen = "r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1"
        arm = determine_active_arm(fen, "e1g1")
        assert arm == "white"

    def test_empty_square_raises(self):
        with pytest.raises(ValueError, match="No piece"):
            determine_active_arm(chess.STARTING_FEN, "e4e5")


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_promotion_queen_and_rook_same_game(self):
        """Multiple promotions in one game should use different reserve slots."""
        # First promotion: queen
        fen1 = "8/P7/8/8/8/8/8/4K2k w - - 0 1"
        b1 = chess.Board(fen1)
        g = GraveyardAllocator()
        r = ReserveRegistry()
        tasks1 = decompose_move(b1, chess.Move.from_uci("a7a8q"), g, r)
        assert tasks1[-1].piece_char == "Q"

        # Second promotion: rook
        fen2 = "8/P7/8/8/8/8/8/4K2k w - - 0 1"
        b2 = chess.Board(fen2)
        tasks2 = decompose_move(b2, chess.Move.from_uci("a7a8r"), g, r)
        assert tasks2[-1].piece_char == "R"

    def test_capture_then_standard_move(self):
        """Graveyard allocator should advance after a capture."""
        fen = "rnb1kbnr/pppp1ppp/8/4p3/2B1P3/8/PPPP1PPP/RNBQK1NR w KQkq - 0 3"
        board = chess.Board(fen)
        g, r = fresh()
        tasks_cap = decompose_move(board, chess.Move.from_uci("c4f7"), g, r)

        # Now a normal move in a different position
        board2 = chess.Board()
        tasks_std = decompose_move(board2, chess.Move.from_uci("e2e4"), g, r)

        # Normal move produces only 1 task and does not affect graveyard
        assert len(tasks_std) == 1