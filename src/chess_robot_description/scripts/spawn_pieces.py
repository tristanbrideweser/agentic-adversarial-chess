#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import signal
import yaml
from typing import Tuple


FILES = "abcdefgh"

# Standard chess back-rank order, indexed by file (a..h).
BACK_RANK = ("rook", "knight", "bishop", "queen", "king", "bishop", "knight", "rook")

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

def run_all(cmds: list) -> int:
    """Spawn in batches of 4 to avoid DDS participant exhaustion."""
    import time
    failed = 0
    batch_size = 4
    for i in range(0, len(cmds), batch_size):
        batch = cmds[i:i + batch_size]
        procs = [subprocess.Popen(cmd) for cmd in batch]
        for p in procs:
            rc = p.wait()
            if rc != 0:
                failed += 1
        time.sleep(0.3)  # brief pause between batches
    return failed

def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn the standard chess starting position (back rank + pawns on ranks 1, 2, 7, 8).")
    parser.add_argument("--model-sdf", default="", help="Override SDF used for every square (e.g. for testing). If empty, the correct piece type for each square is used.")
    parser.add_argument("--config", default="", help="Path to board_params.yaml. If provided, overrides --board-* defaults.")
    parser.add_argument("--board-x", type=float, default=0.0, help="Board center X in world")
    parser.add_argument("--board-y", type=float, default=0.0, help="Board center Y in world")
    parser.add_argument("--board-z-top", type=float, default=0.800, help="Approx Z where pieces should spawn above board (falls onto board)")
    parser.add_argument("--board-size", type=float, default=0.45, help="Board side length (meters)")
    parser.add_argument("--prefix", default="p_", help="Name prefix for spawned models")
    parser.add_argument("--ranks", default="1,2,7,8", help="Comma-separated ranks to fill (default makes 32 pieces: 1,2,7,8)")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be spawned, do not spawn.")
    args = parser.parse_args()

    if args.config:
        with open(args.config) as f:
            cfg = yaml.safe_load(f).get("board", {})
        if "center_xy" in cfg:
            args.board_x, args.board_y = cfg["center_xy"]
        if "spawn_z" in cfg:
            args.board_z_top = float(cfg["spawn_z"])
        elif "top_z" in cfg:
            args.board_z_top = float(cfg["top_z"]) + 0.020
        if "size" in cfg:
            args.board_size = float(cfg["size"])

    # Board square size
    square_size = args.board_size / 8.0

    # Determine models directory (or single override SDF).
    override_sdf = args.model_sdf.strip()
    if override_sdf:
        if not os.path.isfile(override_sdf):
            print(f"ERROR: model.sdf not found at: {override_sdf}", file=sys.stderr)
            return 2
        models_dir = None
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            models_dir = os.path.join(
                get_package_share_directory("chess_robot_description"), "models",
            )
        except Exception:
            pkg_root = os.path.dirname(os.path.abspath(__file__))  # .../scripts
            pkg_root = os.path.dirname(pkg_root)                   # .../chess_robot_description
            models_dir = os.path.join(pkg_root, "models")

    def piece_sdf_for(rank: int, file_idx: int) -> str:
        """Pick the model.sdf for (rank, file) per standard chess starting position."""
        if override_sdf:
            return override_sdf
        color = "white" if rank in (1, 2) else "black"
        if rank in (2, 7):
            piece = "pawn"
        elif rank in (1, 8):
            piece = BACK_RANK[file_idx]
        else:
            # Caller asked for a non-starting rank (e.g. --ranks 4) — fill with pawns.
            piece = "pawn"
        path = os.path.join(models_dir, f"{piece}_{color}", "model.sdf")
        if not os.path.isfile(path):
            print(f"ERROR: model.sdf not found at: {path}", file=sys.stderr)
            sys.exit(2)
        return path

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

    # collect commands
    all_cmds = []
    for rank in ranks:
        for file_idx, file_char in enumerate(FILES):
            square = f"{file_char}{rank}"
            x, y = square_to_xy(square, board_center_xy, square_size)
            name = f"{args.prefix}{square}"
            cmd = [
                "ros2", "run", "ros_gz_sim", "create",
                "-name", name,
                "-file", piece_sdf_for(rank, file_idx),
                "-x", f"{x:.4f}",
                "-y", f"{y:.4f}",
                "-z", f"{args.board_z_top:.4f}",
            ]
            if args.dry_run:
                model_label = os.path.basename(os.path.dirname(piece_sdf_for(rank, file_idx)))
                print(f"[DRY] spawn {name} ({model_label}) at {square} -> x={x:.4f}, y={y:.4f}, z={args.board_z_top:.4f}")
            else:
                all_cmds.append(cmd)

    if not args.dry_run:
        failed = run_all(all_cmds)
        if failed:
            print(f"ERROR: {failed} pieces failed to spawn", file=sys.stderr)
            return 1

    print("Done.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
