# chess_interfaces/msg/PickPlaceTask.msg
#
# A single pick-and-place task from the move_translator.
# The /pick_place_tasks topic carries a JSON string for flexibility,
# but this message type is available for strongly-typed publishing.

# Task type: "pick_and_place" | "remove_piece" | "place_reserve"
string task_type

# FEN character of the piece (e.g. "P", "q")
string piece_char

# Human-readable piece name (e.g. "white pawn")
string piece_name

# Source square in algebraic notation ("" for place_reserve tasks)
string pick_square

# Destination square ("" for remove_piece tasks going to graveyard)
string place_square

# Gripper jaw centre at pick — world frame (m)
geometry_msgs/Pose pick_pose

# Gripper jaw centre at place — world frame (m)
geometry_msgs/Pose place_pose

# Recommended jaw gap at contact (metres)
float64 grasp_height

# Which arm should execute: "white" or "black"
string active_arm
