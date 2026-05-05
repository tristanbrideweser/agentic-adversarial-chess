"""
test_grasp_filter.py
====================
Unit tests for grasp_filter.py.  No ROS required.

Run with:
    pytest grasp_planner/test/test_grasp_filter.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from src.grasp_planner.grasp_planner.grasp_candidates import (
    GraspCandidate,
    PieceType,
    Quaternion,
    get_profile,
    BLOCK_GRASP_Z,
    BLOCK_FINGER_SEP,
    UNIFORM_BLOCK_PIECES,
)
from src.grasp_planner.grasp_planner.grasp_filter import (
    FilterConfig,
    GraspFilter,
    filter_gpd_candidates,
    make_config,
    PIECE_FILTER_OVERRIDES,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

BOARD_Z = 0.762

def _uniform_z() -> float:
    """The single grasp Z used for all block pieces."""
    return BLOCK_GRASP_Z

# Keep _pawn_z / _queen_z as aliases so test names stay readable,
# but both return the same uniform value in block mode
def _pawn_z() -> float:
    return BLOCK_GRASP_Z

def _queen_z() -> float:
    return BLOCK_GRASP_Z


def _make(
    angle_deg=0.0,
    score=0.8,
    finger_sep=None,
    z_offset=0.0,
    source="gpd",
) -> GraspCandidate:
    # Use the uniform block Z + optional offset; default sep = block sep
    sep = finger_sep if finger_sep is not None else BLOCK_FINGER_SEP
    return GraspCandidate(
        position=(0.025, -0.025, BLOCK_GRASP_Z + z_offset),
        orientation=Quaternion.top_down(0.0),
        score=score,
        approach_angle_deg=angle_deg,
        finger_separation=sep,
        source=source,
    )


# ---------------------------------------------------------------------------
# FilterConfig
# ---------------------------------------------------------------------------

class TestFilterConfig:
    def test_defaults(self):
        cfg = FilterConfig()
        assert cfg.top_down_tolerance_deg == 15.0
        assert cfg.min_score == 0.0
        assert cfg.max_finger_separation_m == 0.080

    def test_make_config_returns_config(self):
        """make_config always returns a valid FilterConfig."""
        cfg = make_config(PieceType.PAWN)
        assert isinstance(cfg, FilterConfig)
        assert cfg.height_tolerance_m > 0

    def test_make_config_uniform_pieces_same_tolerance(self):
        """In uniform block mode all pieces get identical filter params."""
        cfgs = [make_config(pt) for pt in PieceType]
        tols = [c.height_tolerance_m for c in cfgs]
        assert len(set(tols)) == 1, "Expected identical height tolerance for all block pieces"

    def test_make_config_override_propagates(self):
        cfg = make_config(PieceType.PAWN, min_score=0.5)
        assert cfg.min_score == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Angle filter
# ---------------------------------------------------------------------------

class TestAngleFilter:
    def test_accepts_top_down(self):
        f = GraspFilter()
        result = f.filter_and_rank([_make(angle_deg=0.0)], PieceType.PAWN, _pawn_z())
        assert len(result) == 1

    def test_accepts_within_tolerance(self):
        f = GraspFilter(top_down_tolerance_deg=15.0)
        result = f.filter_and_rank([_make(angle_deg=14.9)], PieceType.PAWN, _pawn_z())
        assert len(result) == 1

    def test_rejects_beyond_tolerance(self):
        f = GraspFilter(top_down_tolerance_deg=15.0)
        result = f.filter_and_rank([_make(angle_deg=15.1)], PieceType.PAWN, _pawn_z())
        assert len(result) == 0

    def test_rejected_has_reason(self):
        f = GraspFilter(top_down_tolerance_deg=15.0)
        c = _make(angle_deg=45.0)
        f.filter_and_rank([c], PieceType.PAWN, _pawn_z())
        assert c.rejection_reason is not None
        assert "approach_angle" in c.rejection_reason

    def test_uniform_pieces_same_angle_tolerance(self):
        """In uniform block mode all pieces use the same angle tolerance."""
        cfgs = [make_config(pt) for pt in PieceType]
        tols = [c.top_down_tolerance_deg for c in cfgs]
        assert len(set(tols)) == 1, "Expected identical angle tolerance for all block pieces"


# ---------------------------------------------------------------------------
# Height filter
# ---------------------------------------------------------------------------

class TestHeightFilter:
    def test_accepts_exact_z(self):
        f = GraspFilter()
        target_z = _pawn_z()
        result = f.filter_and_rank([_make(z_offset=0.0)], PieceType.PAWN, target_z)
        assert len(result) == 1

    def test_accepts_within_tolerance(self):
        f = GraspFilter()
        target_z = _pawn_z()
        tol = make_config(PieceType.PAWN).height_tolerance_m
        c = _make(z_offset=tol - 0.001)
        result = f.filter_and_rank([c], PieceType.PAWN, target_z)
        assert len(result) == 1

    def test_rejects_outside_tolerance(self):
        f = GraspFilter()
        target_z = _pawn_z()
        tol = make_config(PieceType.PAWN).height_tolerance_m
        c = _make(z_offset=tol + 0.002)
        result = f.filter_and_rank([c], PieceType.PAWN, target_z)
        assert len(result) == 0

    def test_rejection_reason_mentions_z(self):
        f = GraspFilter()
        c = _make(z_offset=0.05)  # way off
        f.filter_and_rank([c], PieceType.PAWN, _pawn_z())
        assert "Z" in c.rejection_reason or "z" in c.rejection_reason.lower()


# ---------------------------------------------------------------------------
# Score filter
# ---------------------------------------------------------------------------

class TestScoreFilter:
    def test_accepts_above_threshold(self):
        f = GraspFilter(min_score=0.5)
        result = f.filter_and_rank([_make(score=0.6)], PieceType.PAWN, _pawn_z())
        assert len(result) == 1

    def test_rejects_below_threshold(self):
        f = GraspFilter(min_score=0.5)
        result = f.filter_and_rank([_make(score=0.4)], PieceType.PAWN, _pawn_z())
        assert len(result) == 0

    def test_rejection_reason_mentions_score(self):
        f = GraspFilter(min_score=0.9)
        c = _make(score=0.1)
        f.filter_and_rank([c], PieceType.PAWN, _pawn_z())
        assert "score" in c.rejection_reason


# ---------------------------------------------------------------------------
# Finger separation filter
# ---------------------------------------------------------------------------

class TestFingerSeparationFilter:
    def test_accepts_valid_separation(self):
        f = GraspFilter()
        # Block: collision_radius=0.018, min_sep = 2*0.018+0.002 = 0.038
        result = f.filter_and_rank([_make(finger_sep=BLOCK_FINGER_SEP)], PieceType.PAWN, _pawn_z())
        assert len(result) == 1

    def test_rejects_too_narrow(self):
        f = GraspFilter()
        # Well below block min sep
        result = f.filter_and_rank([_make(finger_sep=0.010)], PieceType.PAWN, _pawn_z())
        assert len(result) == 0

    def test_rejects_too_wide(self):
        f = GraspFilter()
        result = f.filter_and_rank([_make(finger_sep=0.090)], PieceType.PAWN, _pawn_z())
        assert len(result) == 0

    def test_rejection_reason_mentions_finger_separation(self):
        f = GraspFilter()
        c = _make(finger_sep=0.090)
        f.filter_and_rank([c], PieceType.PAWN, _pawn_z())
        assert "finger_separation" in c.rejection_reason


# ---------------------------------------------------------------------------
# Ranking
# ---------------------------------------------------------------------------

class TestRanking:
    def test_single_survivor_rank_zero(self):
        f = GraspFilter()
        cs = [_make(score=0.8)]
        ranked = f.filter_and_rank(cs, PieceType.PAWN, _pawn_z())
        assert len(ranked) == 1
        assert ranked[0].rank == 0

    def test_higher_score_ranked_better(self):
        f = GraspFilter()
        low = _make(score=0.3)
        high = _make(score=0.9)
        ranked = f.filter_and_rank([low, high], PieceType.PAWN, _pawn_z())
        assert len(ranked) == 2
        assert ranked[0].score > ranked[1].score

    def test_better_angle_ranked_higher_when_scores_equal(self):
        f = GraspFilter(top_down_tolerance_deg=15.0)
        good_angle = _make(score=0.8, angle_deg=2.0)
        bad_angle  = _make(score=0.8, angle_deg=13.0)
        ranked = f.filter_and_rank([bad_angle, good_angle], PieceType.PAWN, _pawn_z())
        assert len(ranked) == 2
        assert ranked[0].approach_angle_deg < ranked[1].approach_angle_deg

    def test_ranks_are_sequential(self):
        f = GraspFilter()
        cs = [_make(score=s) for s in [0.3, 0.7, 0.9, 0.5]]
        ranked = f.filter_and_rank(cs, PieceType.PAWN, _pawn_z())
        for i, c in enumerate(ranked):
            assert c.rank == i

    def test_empty_input_returns_empty(self):
        f = GraspFilter()
        assert f.filter_and_rank([], PieceType.QUEEN, _queen_z()) == []

    def test_all_rejected_returns_empty(self):
        f = GraspFilter(top_down_tolerance_deg=5.0)
        cs = [_make(angle_deg=90.0), _make(angle_deg=45.0)]
        ranked = f.filter_and_rank(cs, PieceType.PAWN, _pawn_z())
        assert ranked == []


# ---------------------------------------------------------------------------
# Per-piece-type filter differences
# ---------------------------------------------------------------------------

class TestPerPieceFilters:
    def _count_survivors(self, piece_type: PieceType, **candidate_kwargs):
        f = GraspFilter()
        profile = get_profile(piece_type)
        c = GraspCandidate(
            position=(0.0, 0.0, profile.grasp_z),
            orientation=Quaternion.top_down(0.0),
            finger_separation=profile.finger_separation_m,
            **{k: v for k, v in candidate_kwargs.items()},
        )
        ranked = f.filter_and_rank([c], piece_type, profile.grasp_z)
        return len(ranked)

    def test_all_six_accept_ideal_grasp(self):
        for pt in PieceType:
            assert self._count_survivors(pt, score=0.9, approach_angle_deg=0.0) == 1, (
                f"Piece type {pt} rejected an ideal grasp"
            )

    def test_all_pieces_same_angle_tolerance(self):
        """In uniform block mode all pieces have identical angle tolerance."""
        f = GraspFilter(top_down_tolerance_deg=15.0)
        profile = get_profile(PieceType.KING)
        # 14° should pass for every piece type (within default 15° tolerance)
        for pt in PieceType:
            c = GraspCandidate(
                position=(0.0, 0.0, profile.grasp_z),
                orientation=Quaternion.top_down(0.0),
                score=0.8,
                approach_angle_deg=14.0,
                finger_separation=profile.finger_separation_m,
            )
            ranked = f.filter_and_rank([c], pt, profile.grasp_z)
            assert len(ranked) == 1, f"{pt} should accept 14° approach angle"


# ---------------------------------------------------------------------------
# filter_gpd_candidates convenience function
# ---------------------------------------------------------------------------

class TestFilterGpdCandidates:
    def test_basic_filter(self):
        cs = [_make(score=0.9), _make(angle_deg=90.0)]
        result = filter_gpd_candidates(cs, "P", _pawn_z())
        assert len(result) == 1

    def test_empty_list(self):
        assert filter_gpd_candidates([], "Q") == []

    def test_invalid_fen_raises(self):
        with pytest.raises(ValueError):
            filter_gpd_candidates([_make()], "X")