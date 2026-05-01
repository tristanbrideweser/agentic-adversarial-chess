import chess


def parse_fen(fen_string: str) -> chess.Board:
    """Construct a chess.Board from a FEN string.

    Note: python-chess will accept positions that are technically possible to
    set up but not reachable from the start position. Validation of legality
    beyond syntax is the caller's responsibility.
    """
    try:
        return chess.Board(fen_string)
    except ValueError:
        raise ValueError(f"{fen_string} is of illegal format!")
