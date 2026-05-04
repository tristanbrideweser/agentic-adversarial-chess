"""
move_decomposer.py
==================
Core logic: given a python-chess Board (pre-move state) and a chess.Move,
produce an ordered list of PickPlaceTask objects that the arm controller
can execute sequentially.

All special moves are handled here (normal, capture, castling, en passant,
promotion, promotion-with-capture).  The board is NOT mutated; the caller
is responsible for calling board.push(move) after decomposition.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from typing import List, Optional

import chess

from .board_geometry import (
    GraveyardAllocator,
    ReserveRegistry,
    WorldPose,
    get_piece_geometry,
    square_to_grasp_pose,
    square_to_world,
)


# ---------------------------------------------------------------------------
# Task data model
# ---------------------------------------------------------------------------

@dataclass
class PickPlaceTask:
    """
    A single atomic robot action.

    task_type:
        - "pick_and_place"  — normal move / castling sub-move
        - "remove_piece"    — captured or promoting pawn → graveyard
        - "place_reserve"   — reserve promotion piece → target square

    All pose fields are in the world frame (metres, yaw in radians).
    """
    task_type: str
    piece_char: str                     # FEN char (e.g. "P", "q")
    piece_name: str                     # human-readable (e.g. "white pawn")
    pick_square: Optional[str]          # None for reserve placements
    place_square: Optional[str]         # None for graveyard removals
    pick_pose: WorldPose
    place_pose: WorldPose
    grasp_height: float                 # absolute Z for gripper jaw centre

    def to_dict(self) -> dict:
        d = asdict(self)
        d["pick_pose"] = self.pick_pose.to_dict()
        d["place_pose"] = self.place_pose.to_dict()
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)


def _piece_name(fen_char: str) -> str:
    """Return a human-readable name for a FEN character."""
    color = "white" if fen_char.isupper() else "black"
    names = {"P": "pawn", "R": "rook", "N": "knight",
             "B": "bishop", "Q": "queen", "K": "king"}
    return f"{color} {names[fen_char.upper()]}"


def _color_of(fen_char: str) -> str:
    return "white" if fen_char.isupper() else "black"


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def decompose_move(
    board: chess.Board,
    move: chess.Move,
    graveyard: GraveyardAllocator,
    reserve: Optional[ReserveRegistry] = None,
) -> List[PickPlaceTask]:
    """
    Decompose a chess move into an ordered list of PickPlaceTasks.

    Parameters
    ----------
    board:
        The board in its state *before* the move is applied.
        This function does NOT mutate the board.
    move:
        A legal chess.Move (validated upstream by board_state_node).
    graveyard:
        Stateful allocator for captured-piece graveyard slots.
    reserve:
        Registry of off-board reserve pieces for promotion.
        If None, a default ReserveRegistry is created (useful in unit tests).

    Returns
    -------
    Ordered list of PickPlaceTask objects.  Execute sequentially.

    Raises
    ------
    ValueError if the move or board state is inconsistent.
    """
    if reserve is None:
        reserve = ReserveRegistry()

    # Dispatch to specific handler
    if move.promotion is not None:
        return _decompose_promotion(board, move, graveyard, reserve)
    if board.is_en_passant(move):
        return _decompose_en_passant(board, move, graveyard)
    if board.is_castling(move):
        return _decompose_castling(board, move)
    if board.is_capture(move):
        return _decompose_capture(board, move, graveyard)
    return _decompose_standard(board, move)


# ---------------------------------------------------------------------------
# Move handlers
# ---------------------------------------------------------------------------

def _decompose_standard(
    board: chess.Board,
    move: chess.Move,
) -> List[PickPlaceTask]:
    """Handle a normal (non-capture, non-special) move."""
    from_sq = chess.square_name(move.from_square)
    to_sq = chess.square_name(move.to_square)
    piece = board.piece_at(move.from_square)

    if piece is None:
        raise ValueError(
            f"No piece on {from_sq} in position {board.fen()}"
        )

    fen_char = piece.symbol()
    geom = get_piece_geometry(fen_char)

    pick_pose = square_to_grasp_pose(from_sq, fen_char)
    place_pose = square_to_world(to_sq, z_override=geom.grasp_z)

    task = PickPlaceTask(
        task_type="pick_and_place",
        piece_char=fen_char,
        piece_name=_piece_name(fen_char),
        pick_square=from_sq,
        place_square=to_sq,
        pick_pose=pick_pose,
        place_pose=place_pose,
        grasp_height=geom.grasp_z,
    )
    return [task]


def _decompose_capture(
    board: chess.Board,
    move: chess.Move,
    graveyard: GraveyardAllocator,
) -> List[PickPlaceTask]:
    """
    Handle a standard capture (target piece is on the destination square).

    Order:
        1. Remove captured piece → graveyard
        2. Move attacking piece → destination
    """
    from_sq = chess.square_name(move.from_square)
    to_sq = chess.square_name(move.to_square)

    attacker = board.piece_at(move.from_square)
    captured = board.piece_at(move.to_square)

    if attacker is None:
        raise ValueError(f"No attacker on {from_sq}")
    if captured is None:
        raise ValueError(f"No captured piece on {to_sq} for move {move.uci()}")

    att_char = attacker.symbol()
    cap_char = captured.symbol()

    # --- Task 1: remove captured piece ---
    cap_geom = get_piece_geometry(cap_char)
    graveyard_pose = graveyard.next_slot(cap_char)

    remove_task = PickPlaceTask(
        task_type="remove_piece",
        piece_char=cap_char,
        piece_name=_piece_name(cap_char),
        pick_square=to_sq,
        place_square=None,
        pick_pose=square_to_grasp_pose(to_sq, cap_char),
        place_pose=graveyard_pose,
        grasp_height=cap_geom.grasp_z,
    )

    # --- Task 2: move attacker ---
    att_geom = get_piece_geometry(att_char)
    move_task = PickPlaceTask(
        task_type="pick_and_place",
        piece_char=att_char,
        piece_name=_piece_name(att_char),
        pick_square=from_sq,
        place_square=to_sq,
        pick_pose=square_to_grasp_pose(from_sq, att_char),
        place_pose=square_to_world(to_sq, z_override=att_geom.grasp_z),
        grasp_height=att_geom.grasp_z,
    )

    return [remove_task, move_task]


def _decompose_castling(
    board: chess.Board,
    move: chess.Move,
) -> List[PickPlaceTask]:
    """
    Handle castling (both kingside and queenside, both colours).

    The architecture specifies: king moves first, then rook.
    """
    from_sq = chess.square_name(move.from_square)   # king's origin
    to_sq = chess.square_name(move.to_square)       # king's destination

    king = board.piece_at(move.from_square)
    if king is None or king.piece_type != chess.KING:
        raise ValueError(f"Expected king on {from_sq} for castling move {move.uci()}")

    king_char = king.symbol()
    king_geom = get_piece_geometry(king_char)

    # Determine rook squares from castling direction
    is_kingside = chess.square_file(move.to_square) > chess.square_file(move.from_square)
    rank = chess.square_rank(move.from_square)  # 0 for White, 7 for Black

    if is_kingside:
        rook_from = chess.square_name(chess.square(7, rank))   # h-file
        rook_to = chess.square_name(chess.square(5, rank))     # f-file
    else:
        rook_from = chess.square_name(chess.square(0, rank))   # a-file
        rook_to = chess.square_name(chess.square(3, rank))     # d-file

    rook = board.piece_at(chess.parse_square(rook_from))
    if rook is None or rook.piece_type != chess.ROOK:
        raise ValueError(f"Expected rook on {rook_from} for castling")

    rook_char = rook.symbol()
    rook_geom = get_piece_geometry(rook_char)

    # Task 1: king
    king_task = PickPlaceTask(
        task_type="pick_and_place",
        piece_char=king_char,
        piece_name=_piece_name(king_char),
        pick_square=from_sq,
        place_square=to_sq,
        pick_pose=square_to_grasp_pose(from_sq, king_char),
        place_pose=square_to_world(to_sq, z_override=king_geom.grasp_z),
        grasp_height=king_geom.grasp_z,
    )

    # Task 2: rook
    rook_task = PickPlaceTask(
        task_type="pick_and_place",
        piece_char=rook_char,
        piece_name=_piece_name(rook_char),
        pick_square=rook_from,
        place_square=rook_to,
        pick_pose=square_to_grasp_pose(rook_from, rook_char),
        place_pose=square_to_world(rook_to, z_override=rook_geom.grasp_z),
        grasp_height=rook_geom.grasp_z,
    )

    return [king_task, rook_task]


def _decompose_en_passant(
    board: chess.Board,
    move: chess.Move,
    graveyard: GraveyardAllocator,
) -> List[PickPlaceTask]:
    """
    Handle en passant.

    The captured pawn is on the *same rank as the attacker*, NOT on the
    target square (the only move in chess where this is true).

    Order:
        1. Remove captured pawn from its actual square → graveyard
        2. Move attacking pawn → target square
    """
    from_sq = chess.square_name(move.from_square)
    to_sq = chess.square_name(move.to_square)

    attacker = board.piece_at(move.from_square)
    if attacker is None:
        raise ValueError(f"No piece on {from_sq} for en passant move")

    att_char = attacker.symbol()

    # Captured pawn square: same file as target, same rank as attacker
    captured_file = chess.square_file(move.to_square)
    captured_rank = chess.square_rank(move.from_square)
    captured_sq_int = chess.square(captured_file, captured_rank)
    captured_sq = chess.square_name(captured_sq_int)

    captured = board.piece_at(captured_sq_int)
    if captured is None:
        raise ValueError(
            f"No pawn on {captured_sq} for en passant — "
            f"board FEN: {board.fen()}"
        )

    cap_char = captured.symbol()
    cap_geom = get_piece_geometry(cap_char)
    att_geom = get_piece_geometry(att_char)

    # Task 1: remove captured pawn (NOT on to_sq!)
    remove_task = PickPlaceTask(
        task_type="remove_piece",
        piece_char=cap_char,
        piece_name=_piece_name(cap_char),
        pick_square=captured_sq,
        place_square=None,
        pick_pose=square_to_grasp_pose(captured_sq, cap_char),
        place_pose=graveyard.next_slot(cap_char),
        grasp_height=cap_geom.grasp_z,
    )

    # Task 2: move attacker to target square
    move_task = PickPlaceTask(
        task_type="pick_and_place",
        piece_char=att_char,
        piece_name=_piece_name(att_char),
        pick_square=from_sq,
        place_square=to_sq,
        pick_pose=square_to_grasp_pose(from_sq, att_char),
        place_pose=square_to_world(to_sq, z_override=att_geom.grasp_z),
        grasp_height=att_geom.grasp_z,
    )

    return [remove_task, move_task]


def _decompose_promotion(
    board: chess.Board,
    move: chess.Move,
    graveyard: GraveyardAllocator,
    reserve: ReserveRegistry,
) -> List[PickPlaceTask]:
    """
    Handle pawn promotion (with or without capture).

    Without capture:
        1. Remove promoting pawn → graveyard
        2. Place reserve piece → promotion square

    With capture:
        1. Remove captured piece → graveyard
        2. Remove promoting pawn → graveyard
        3. Place reserve piece → promotion square
    """
    from_sq = chess.square_name(move.from_square)
    to_sq = chess.square_name(move.to_square)

    pawn = board.piece_at(move.from_square)
    if pawn is None:
        raise ValueError(f"No pawn on {from_sq} for promotion")

    pawn_char = pawn.symbol()
    pawn_geom = get_piece_geometry(pawn_char)
    color = _color_of(pawn_char)

    # Promotion piece type
    promo_type_map = {
        chess.QUEEN: "queen", chess.ROOK: "rook",
        chess.BISHOP: "bishop", chess.KNIGHT: "knight",
    }
    promo_type = promo_type_map.get(move.promotion)
    if promo_type is None:
        raise ValueError(f"Unknown promotion type: {move.promotion}")

    # Promotion piece FEN char
    promo_char = chess.piece_symbol(move.promotion)
    if color == "white":
        promo_char = promo_char.upper()

    promo_geom = get_piece_geometry(promo_char)

    tasks: List[PickPlaceTask] = []

    # --- Optional: capture on promotion square ---
    is_capture = board.is_capture(move)
    if is_capture:
        captured = board.piece_at(move.to_square)
        if captured is None:
            raise ValueError(f"Promotion capture: no piece on {to_sq}")
        cap_char = captured.symbol()
        cap_geom = get_piece_geometry(cap_char)

        tasks.append(PickPlaceTask(
            task_type="remove_piece",
            piece_char=cap_char,
            piece_name=_piece_name(cap_char),
            pick_square=to_sq,
            place_square=None,
            pick_pose=square_to_grasp_pose(to_sq, cap_char),
            place_pose=graveyard.next_slot(cap_char),
            grasp_height=cap_geom.grasp_z,
        ))

    # --- Remove the promoting pawn ---
    tasks.append(PickPlaceTask(
        task_type="remove_piece",
        piece_char=pawn_char,
        piece_name=_piece_name(pawn_char),
        pick_square=from_sq,
        place_square=None,
        pick_pose=square_to_grasp_pose(from_sq, pawn_char),
        place_pose=graveyard.next_slot(pawn_char),
        grasp_height=pawn_geom.grasp_z,
    ))

    # --- Place reserve piece ---
    try:
        reserve_pose = reserve.get_reserve(color, promo_type)
    except KeyError:
        # Fallback: use a previously captured piece (best-effort)
        # In practice the operator should ensure reserves are stocked.
        raise KeyError(
            f"No reserve {color} {promo_type} available for promotion. "
            "Ensure reserve pieces are placed in their designated positions "
            "before the game starts."
        )

    tasks.append(PickPlaceTask(
        task_type="place_reserve",
        piece_char=promo_char,
        piece_name=_piece_name(promo_char),
        pick_square=None,
        place_square=to_sq,
        pick_pose=reserve_pose,
        place_pose=square_to_world(to_sq, z_override=promo_geom.grasp_z),
        grasp_height=promo_geom.grasp_z,
    ))

    return tasks


# ---------------------------------------------------------------------------
# Utility: serialise task queue to JSON string for /pick_place_tasks topic
# ---------------------------------------------------------------------------

def task_queue_to_json(tasks: List[PickPlaceTask]) -> str:
    """Serialise a task queue to the JSON format published on /pick_place_tasks."""
    return json.dumps([t.to_dict() for t in tasks], indent=2)


def task_queue_from_json(json_str: str) -> List[dict]:
    """Deserialise a task queue from a /pick_place_tasks JSON string."""
    return json.loads(json_str)