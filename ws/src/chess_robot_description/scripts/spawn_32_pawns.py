#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from typing import Tuple

FILES = "abcdefgh"

def square_to_xy(square: str, board_center_xy: Tuple[float, float], square_size: float) -> Tuple[float, float]:
    """
    Map chess square (e.g., 'a1') to world (x,y).

    Convention:
      - board is centered at (board_center_xy)
      - rank 1 -> 8 increases along +X
      - file a -> h increases along +Y
      - a1 is bottom-left (lowest x, lowest y)
    """
    square = square.strip().lower()
    if len(square) != 2 or square[0] not in FILES or square[1] not in "12345678":
        raise ValueError(f"Bad square: {square}")

    file_idx = FILES.index(square[0])  # a=0 ... h=7
    rank_idx = int(square[1]) - 1      # 1->0 ... 8->7

    # Center offsets: -3.5, -2.5, ..., +3.5
    dx = (rank_idx - 3.5) * square_size
    dy = (file_idx - 3.5) * square_size

    x0, y0 = board_center_xy
    return (x0 + dx, y0 + dy)

def run(cmd: list) -> int:
    # Print command for debugging
    print(" ".join(cmd))
    return subprocess.call(cmd)

def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn 32 placeholder chess pieces (all pawns) using square names.")
    parser.add_argument("--model-sdf", default="", help="Absolute path to pawn model.sdf. If empty, uses package path.")
    parser.add_argument("--config", default="", help="Path to board_params.yaml. If provided, overrides --board-* defaults.")
    parser.add_argument("--board-x", type=float, default=0.0, help="Board center X in world")
    parser.add_argument("--board-y", type=float, default=0.0, help="Board center Y in world")
    parser.add_argument("--board-z-top", type=float, default=0.780, help="Approx Z where pieces should spawn above board (falls onto board)")
    parser.add_argument("--board-size", type=float, default=0.45, help="Board side length (meters)")
    parser.add_argument("--prefix", default="p_", help="Name prefix for spawned models")
    parser.add_argument("--ranks", default="1,2,7,8", help="Comma-separated ranks to fill (default makes 32 pieces: 1,2,7,8)")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be spawned, do not spawn.")
    args = parser.parse_args()

    if args.config:
        try:
            import yaml  # lazy import; only needed when --config is used
        except ImportError:
            print("ERROR: PyYAML required for --config. pip install pyyaml", file=sys.stderr)
            return 2
        with open(args.config) as f:
            cfg = yaml.safe_load(f).get("board", {})
        if "center_xy" in cfg:
            args.board_x, args.board_y = cfg["center_xy"]
        if "top_z" in cfg:
            args.board_z_top = float(cfg["top_z"])
        if "size" in cfg:
            args.board_size = float(cfg["size"])

    # Board square size
    square_size = args.board_size / 8.0

    # Determine pawn model.sdf path
    model_sdf = args.model_sdf.strip()
    if not model_sdf:
        # Prefer the installed ROS package share (works after colcon install).
        try:
            from ament_index_python.packages import get_package_share_directory
            model_sdf = os.path.join(
                get_package_share_directory("chess_robot_description"),
                "models", "pawn", "model.sdf",
            )
        except Exception:
            # Fallback: source-tree layout <pkg>/models/pawn/model.sdf
            pkg_root = os.path.dirname(os.path.abspath(__file__))  # .../scripts
            pkg_root = os.path.dirname(pkg_root)                   # .../chess_robot_description
            model_sdf = os.path.join(pkg_root, "models", "pawn", "model.sdf")

    if not os.path.isfile(model_sdf):
        print(f"ERROR: model.sdf not found at: {model_sdf}", file=sys.stderr)
        print("Pass --model-sdf /absolute/path/to/model.sdf if needed.", file=sys.stderr)
        return 2

    # Parse ranks
    try:
        ranks = [int(r.strip()) for r in args.ranks.split(",") if r.strip()]
    except ValueError:
        print("ERROR: --ranks must be comma-separated integers, e.g. 1,2,7,8", file=sys.stderr)
        return 2

    for r in ranks:
        if r < 1 or r > 8:
            print(f"ERROR: rank out of range: {r}", file=sys.stderr)
            return 2

    board_center_xy = (args.board_x, args.board_y)

    # Spawn pieces for each file on specified ranks
    for rank in ranks:
        for file_char in FILES:
            square = f"{file_char}{rank}"
            x, y = square_to_xy(square, board_center_xy, square_size)
            name = f"{args.prefix}{square}"

            cmd = [
                "ros2", "run", "ros_gz_sim", "create",
                "-name", name,
                "-file", model_sdf,
                "-x", f"{x:.4f}",
                "-y", f"{y:.4f}",
                "-z", f"{args.board_z_top:.4f}",
            ]

            if args.dry_run:
                print(f"[DRY] spawn {name} at {square} -> x={x:.4f}, y={y:.4f}, z={args.board_z_top:.4f}")
            else:
                rc = run(cmd)
                if rc != 0:
                    print(f"ERROR: spawn failed for {name} (square {square})", file=sys.stderr)
                    return rc

    print("Done.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
