# chess_interfaces/msg/GameState.msg
#
# Published by board_state_node on /board_state (as std_msgs/String for FEN,
# but this richer message is available on /game_state for the coordinator).

# Current board position in FEN notation
string fen

# Whose turn: "white" or "black"
string active_color

# Half-move clock (for 50-move rule)
int32 halfmove_clock

# Full move number
int32 fullmove_number

# Whether the game is over
bool game_over

# Termination reason if game_over is true:
#   "checkmate" | "stalemate" | "insufficient_material" |
#   "threefold_repetition" | "fifty_move_rule" | "draw_agreement"
string termination_reason

# Winner if game_over: "white" | "black" | "draw" | ""
string winner
