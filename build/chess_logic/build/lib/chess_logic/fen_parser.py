import chess

# Create a chess board with given position indicated by fen_string
# Note: It is possible to set up and work with invalid positions. 
def parse_fen(fen_string: str):
    try:
        board = chess.Board(fen_string)
        return board
    except ValueError:
        raise ValueError(f"{fen_string} is of illegal format!")
    




