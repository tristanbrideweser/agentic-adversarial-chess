"""
special_moves.py
Validation helpers and edge-case detection for the four special move
categories: castling, en passant, promotion, and promotion-with-capture.

These functions are used by move_decomposer.py to pre-flight check moves
and by tests to verify the decomposer handles all edge cases correctly.

None of the functions here mutate the board.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import chess


# ---------------------------------------------------------------------------
# Move classification
# ---------------------------------------------------------------------------

class MoveClass(Enum):
    STANDARD = auto()
    CAPTURE = auto()
    CASTLING = auto()
    EN_PASSANT = auto()
    PROMOTION = auto()
    PROMOTION_CAPTURE = auto()


def classify_move(board: chess.Board, move: chess.Move) -> MoveClass:
    """
    Classify a move into one of six categories.

    Parameters
    ----------
    board:
        Position *before* the move is applied.
    move:
        A legal chess.Move (pre-validated by board_state_node).

    Returns
    -------
    MoveClass enum value.

    Raises
    ------
    ValueError if the move is illegal in the given position.
    """
    if move not in board.legal_moves:
        raise ValueError(
            f"Move {move.uci()} is not legal in position:\n{board.fen()}"
        )

    if board.is_castling(move):
        return MoveClass.CASTLING

    if board.is_en_passant(move):
        return MoveClass.EN_PASSANT

    if move.promotion is not None:
        if board.is_capture(move):
            return MoveClass.PROMOTION_CAPTURE
        return MoveClass.PROMOTION

    if board.is_capture(move):
        return MoveClass.CAPTURE

    return MoveClass.STANDARD


# ---------------------------------------------------------------------------
# Castling helpers
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CastlingInfo:
    is_kingside: bool
    king_from: str
    king_to: str
    rook_from: str
    rook_to: str
    color: chess.Color   # chess.WHITE or chess.BLACK


def get_castling_info(board: chess.Board, move: chess.Move) -> CastlingInfo:
    """
    Extract the four squares involved in a castling move.

    Returns a CastlingInfo with squares in algebraic notation.
    """
    if not board.is_castling(move):
        raise ValueError(f"Move {move.uci()} is not a castling move.")

    king_from_int = move.from_square
    king_to_int = move.to_square
    rank = chess.square_rank(king_from_int)
    is_kingside = chess.square_file(king_to_int) > chess.square_file(king_from_int)

    if is_kingside:
        rook_from_int = chess.square(7, rank)   # h-file
        rook_to_int = chess.square(5, rank)     # f-file
    else:
        rook_from_int = chess.square(0, rank)   # a-file
        rook_to_int = chess.square(3, rank)     # d-file

    piece = board.piece_at(king_from_int)
    color = piece.color if piece else chess.WHITE

    return CastlingInfo(
        is_kingside=is_kingside,
        king_from=chess.square_name(king_from_int),
        king_to=chess.square_name(king_to_int),
        rook_from=chess.square_name(rook_from_int),
        rook_to=chess.square_name(rook_to_int),
        color=color,
    )


# ---------------------------------------------------------------------------
# En passant helpers
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class EnPassantInfo:
    attacker_from: str
    attacker_to: str
    captured_sq: str      # NOT the same as attacker_to
    attacker_char: str    # FEN char of the moving pawn
    captured_char: str    # FEN char of the captured pawn


def get_en_passant_info(board: chess.Board, move: chess.Move) -> EnPassantInfo:
    """
    Return the three squares and piece chars for an en passant move.

    The captured pawn sits on the *same rank as the attacker* but on the
    *file of the target square* — it is never on the target square itself.
    """
    if not board.is_en_passant(move):
        raise ValueError(f"Move {move.uci()} is not en passant.")

    attacker_from_int = move.from_square
    attacker_to_int = move.to_square

    # Captured pawn: same file as target, same rank as attacker
    cap_file = chess.square_file(attacker_to_int)
    cap_rank = chess.square_rank(attacker_from_int)
    cap_sq_int = chess.square(cap_file, cap_rank)

    attacker = board.piece_at(attacker_from_int)
    captured = board.piece_at(cap_sq_int)

    if attacker is None:
        raise ValueError(f"No piece on {chess.square_name(attacker_from_int)}")
    if captured is None:
        raise ValueError(
            f"No captured pawn found on {chess.square_name(cap_sq_int)} "
            f"for en passant move {move.uci()}"
        )

    return EnPassantInfo(
        attacker_from=chess.square_name(attacker_from_int),
        attacker_to=chess.square_name(attacker_to_int),
        captured_sq=chess.square_name(cap_sq_int),
        attacker_char=attacker.symbol(),
        captured_char=captured.symbol(),
    )


# ---------------------------------------------------------------------------
# Promotion helpers
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PromotionInfo:
    pawn_from: str
    promotion_sq: str
    pawn_char: str
    promoted_char: str       # FEN char of the new piece
    promotion_piece_type: int  # chess.QUEEN, chess.ROOK, etc.
    is_capture: bool
    captured_char: Optional[str]  # FEN char of captured piece, or None


def get_promotion_info(board: chess.Board, move: chess.Move) -> PromotionInfo:
    """
    Return full details for a pawn promotion (with or without capture).
    """
    if move.promotion is None:
        raise ValueError(f"Move {move.uci()} is not a promotion.")

    from_sq_int = move.from_square
    to_sq_int = move.to_square

    pawn = board.piece_at(from_sq_int)
    if pawn is None:
        raise ValueError(f"No pawn on {chess.square_name(from_sq_int)}")

    pawn_char = pawn.symbol()
    is_white = pawn.color == chess.WHITE

    # Build the promoted piece's FEN char
    promo_symbol = chess.piece_symbol(move.promotion)
    promoted_char = promo_symbol.upper() if is_white else promo_symbol.lower()

    # Capture details
    is_cap = board.is_capture(move)
    captured_char: Optional[str] = None
    if is_cap:
        cap_piece = board.piece_at(to_sq_int)
        if cap_piece:
            captured_char = cap_piece.symbol()

    return PromotionInfo(
        pawn_from=chess.square_name(from_sq_int),
        promotion_sq=chess.square_name(to_sq_int),
        pawn_char=pawn_char,
        promoted_char=promoted_char,
        promotion_piece_type=move.promotion,
        is_capture=is_cap,
        captured_char=captured_char,
    )


# ---------------------------------------------------------------------------
# Active arm determination
# ---------------------------------------------------------------------------

def determine_active_arm(pre_move_fen: str, uci: str) -> str:
    """
    Determine which arm should execute the move using Option B from the spec:
    infer from the piece character on the source square in the pre-move FEN.

    Parameters
    ----------
    pre_move_fen:
        FEN string *before* the move is applied.
    uci:
        UCI string of the move (e.g. "e2e4").

    Returns
    -------
    "white" or "black"
    """
    board = chess.Board(pre_move_fen)
    move = chess.Move.from_uci(uci)

    piece = board.piece_at(move.from_square)
    if piece is None:
        raise ValueError(
            f"No piece on source square for UCI '{uci}' in FEN '{pre_move_fen}'"
        )

    return "white" if piece.color == chess.WHITE else "black"


# ---------------------------------------------------------------------------
# Reachability check
# ---------------------------------------------------------------------------

def arm_can_reach(
    color: str,
    square: str,
    overlap_zone_ranks: tuple = (3, 4, 5, 6),
) -> bool:
    """
    Quick heuristic: can the given arm reach the target square?

    Based on the reach analysis in the architecture document:
    - Both arms can reach all 64 squares (worst case 0.649 m < 0.855 m limit).
    - For squares in the overlap zone (ranks 3–6), either arm can be used.

    Parameters
    ----------
    color : "white" or "black"
    square : algebraic notation, e.g. "e4"
    overlap_zone_ranks : tuple of rank numbers (1-indexed) reachable by both arms.

    Returns
    -------
    True if the arm can reach the square.

    Note
    ----
    This is always True given the current board/arm layout, but the function
    is provided as a hook for constraint checking in the coordinator's
    failure recovery path (see architecture §7: "Use other arm if square is
    in overlap zone").
    """
    rank = int(square[1])
    if rank in overlap_zone_ranks:
        return True   # either arm can handle this
    # White arm handles ranks 1-4, black arm handles ranks 5-8
    # (with generous overlap — both can technically reach all squares)
    if color == "white":
        return True   # reach analysis confirms all 64 squares
    return True       # same for black


def suggest_fallback_arm(square: str) -> Optional[str]:
    """
    Given a square, return the preferred fallback arm if the primary arm fails.

    For squares in the overlap zone (ranks 3–6), suggest the other arm.
    Outside that zone, there is no safe fallback.
    """
    rank = int(square[1])
    if 3 <= rank <= 6:
        # Either arm could handle this; caller decides which is primary
        return "either"
    return None
# castling

# en passant

# promotion
