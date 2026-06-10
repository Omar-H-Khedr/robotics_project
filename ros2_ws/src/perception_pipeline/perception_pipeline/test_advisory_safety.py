#!/usr/bin/env python3
"""Unit tests for v2_14 guarded advisory safety invariants.

Tests:
  1. Low-confidence fallback
  2. INSERT always defers
  3. DONE prediction alone cannot terminate task
  4. Safety gate blocks ML override
  5. Invalid context shape
  6. Missing model path / model loading failure
  7. Smoke test for runtime advisory interface
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import numpy as np
import pytest

from perception_pipeline.v2_14_safety_gated_action import (
    SafetyGatedActionInterface,
    ActionCommand,
    PhaseID,
    CONFIDENCE_THRESHOLD,
    SAFETY_CRITICAL_PHASES,
    CONTEXT_DIM,
    NUM_CLASSES,
)
from perception_pipeline.v2_14_advisory_node import (
    GuardedAdvisoryInterface,
    AdvisoryDecision,
    DONE_PRECISION_KNOWN_ISSUE,
    RETREAT_UNCERTAINTY_NOTE,
)


# ── Test 1: Low-confidence fallback ──────────────────────────────────────────

class TestLowConfidenceFallback:
    def test_low_confidence_rejected(self):
        adv = GuardedAdvisoryInterface(confidence_threshold=0.85)
        decision = adv.decide(
            pred_phase=PhaseID.APPROACH,
            pred_phase_name="APPROACH",
            confidence=0.50,
            margin=0.4,
            det_phase="MOVING_TO_START",
            det_phase_int=1,
            logits=[0.1, 0.5, 0.2, 0.05, 0.05, 0.05, 0.05],
        )
        assert decision.rejected is True
        assert decision.fallback is True
        assert "low_confidence" in decision.fallback_reason
        assert decision.accepted is False
        assert decision.proposed_action == "none"

    def test_high_confidence_not_rejected_for_confidence(self):
        adv = GuardedAdvisoryInterface(confidence_threshold=0.85)
        decision = adv.decide(
            pred_phase=PhaseID.APPROACH,
            pred_phase_name="APPROACH",
            confidence=0.92,
            margin=0.6,
            det_phase="MOVING_TO_START",
            det_phase_int=1,
            logits=[0.02, 0.92, 0.02, 0.01, 0.01, 0.01, 0.01],
        )
        assert decision.accepted is True
        assert decision.fallback is False

    def test_boundary_confidence_rejected(self):
        adv = GuardedAdvisoryInterface(confidence_threshold=0.85)
        decision = adv.decide(
            pred_phase=PhaseID.APPROACH,
            pred_phase_name="APPROACH",
            confidence=0.84,
            margin=0.5,
            det_phase="MOVING_TO_START",
            det_phase_int=1,
            logits=[0.05, 0.84, 0.05, 0.02, 0.02, 0.01, 0.01],
        )
        assert decision.rejected is True
        assert decision.fallback is True


# ── Test 2: INSERT always defers ─────────────────────────────────────────────

class TestInsertAlwaysDefers:
    def test_insert_always_rejected(self):
        adv = GuardedAdvisoryInterface()
        for conf in [0.5, 0.85, 0.99, 1.0]:
            decision = adv.decide(
                pred_phase=PhaseID.INSERT,
                pred_phase_name="INSERT",
                confidence=conf,
                margin=0.9,
                det_phase="APPROACH",
                det_phase_int=2,
                logits=[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            )
            assert decision.rejected is True, f"INSERT rejected at conf={conf}"
            assert decision.fallback is True
            assert "insert_defers" in decision.fallback_reason
            assert decision.safety_gate_status == "INSERT_DEFERRED"
            assert decision.proposed_action == "none"

    def test_insert_unsafe_if_executed_would_be_true_with_ml(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.INSERT,
            pred_phase_name="INSERT",
            confidence=0.99,
            margin=0.8,
            det_phase="SEARCH",
            det_phase_int=3,
            logits=[0.0, 0.0, 0.0, 0.0, 0.99, 0.01, 0.0],
        )
        assert decision.rejected is True
        assert decision.safety_gate_status == "INSERT_DEFERRED"


# ── Test 3: DONE prediction alone cannot terminate task ───────────────────────

class TestDoneNeverTrusted:
    def test_done_always_rejected(self):
        adv = GuardedAdvisoryInterface()
        for conf in [0.5, 0.85, 0.99, 1.0]:
            decision = adv.decide(
                pred_phase=PhaseID.DONE,
                pred_phase_name="DONE",
                confidence=conf,
                margin=0.9,
                det_phase="RETREAT",
                det_phase_int=6,
                logits=[0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.99],
            )
            assert decision.rejected is True, f"DONE rejected at conf={conf}"
            assert decision.fallback is True
            assert decision.fallback_reason == "done_never_trusted_from_ml"
            assert decision.done_false_positive_flag is True
            assert decision.unsafe_if_executed is True
            assert decision.safety_gate_status == "DONE_BLOCKED"

    def test_done_rejected_even_with_perfect_confidence(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.DONE,
            pred_phase_name="DONE",
            confidence=1.0,
            margin=1.0,
            det_phase="RETREAT",
            det_phase_int=6,
            logits=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        assert decision.rejected is True
        assert decision.unsafe_if_executed is True

    def test_done_documented_known_issue(self):
        assert "3.4%" in DONE_PRECISION_KNOWN_ISSUE
        assert "NEVER" in DONE_PRECISION_KNOWN_ISSUE


# ── Test 4: Safety gate blocks ML override ───────────────────────────────────

class TestSafetyGateBlocksOverride:
    def test_insert_safety_gate_status(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.INSERT,
            pred_phase_name="INSERT",
            confidence=0.99,
            margin=0.8,
            det_phase="APPROACH",
            det_phase_int=2,
            logits=[0.0, 0.0, 0.0, 0.0, 0.99, 0.01, 0.0],
        )
        assert decision.safety_gate_status == "INSERT_DEFERRED"

    def test_done_safety_gate_status(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.DONE,
            pred_phase_name="DONE",
            confidence=0.99,
            margin=0.8,
            det_phase="RETREAT",
            det_phase_int=6,
            logits=[0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.99],
        )
        assert decision.safety_gate_status == "DONE_BLOCKED"

    def test_search_low_margin_rejected(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.SEARCH,
            pred_phase_name="SEARCH",
            confidence=0.92,
            margin=0.2,
            det_phase="APPROACH",
            det_phase_int=2,
            logits=[0.0, 0.0, 0.0, 0.6, 0.0, 0.4, 0.0],
        )
        assert decision.rejected is True
        assert decision.fallback is True
        assert "search_low_margin" in decision.fallback_reason


# ── Test 5: Invalid context shape ────────────────────────────────────────────

class TestInvalidContextShape:
    def test_wrong_shape_raises(self):
        interface = SafetyGatedActionInterface(model_path=None)
        with pytest.raises(ValueError, match="must be"):
            interface.predict(np.zeros(74))

    def test_empty_array_raises(self):
        interface = SafetyGatedActionInterface(model_path=None)
        with pytest.raises(ValueError, match="must be"):
            interface.predict(np.zeros(10))

    def test_correct_shape_with_no_model_returns_fallback(self):
        interface = SafetyGatedActionInterface(model_path=None)
        cmd = interface.predict(np.zeros(CONTEXT_DIM))
        assert cmd.use_fallback is True
        assert cmd.fallback_reason == "no_model_loaded"
        assert cmd.confidence == 0.0


# ── Test 6: Missing model path / model loading failure ───────────────────────

class TestModelLoadingFailure:
    def test_no_model_returns_fallback(self):
        interface = SafetyGatedActionInterface(model_path=None)
        assert interface._model is None
        cmd = interface.predict(np.zeros(CONTEXT_DIM))
        assert cmd.use_fallback is True
        assert cmd.fallback_reason == "no_model_loaded"

    def test_nonexistent_model_path_no_crash(self):
        interface = SafetyGatedActionInterface(
            model_path="/nonexistent/path/model.pt"
        )
        assert interface._model is None
        cmd = interface.predict(np.zeros(CONTEXT_DIM))
        assert cmd.use_fallback is True

    def test_invalid_checkpoint_no_crash(self):
        with tempfile.NamedTemporaryFile(suffix=".pt", delete=False) as f:
            f.write(b"not a valid checkpoint")
            f.flush()
            interface = SafetyGatedActionInterface(model_path=f.name)
            assert interface._model is None
            cmd = interface.predict(np.zeros(CONTEXT_DIM))
            assert cmd.use_fallback is True


# ── Test 7: Smoke test for runtime advisory interface ────────────────────────

class TestAdvisoryInterfaceSmoke:
    def test_moving_to_start_accepted(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.MOVING_TO_START,
            pred_phase_name="MOVING_TO_START",
            confidence=0.92,
            margin=0.8,
            det_phase="MOVING_TO_START",
            det_phase_int=1,
            logits=[0.0, 0.92, 0.02, 0.01, 0.02, 0.01, 0.02],
        )
        assert decision.accepted is True
        assert decision.proposed_action == "suggest_moving_to_start"
        assert decision.fallback is False

    def test_approach_accepted(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.APPROACH,
            pred_phase_name="APPROACH",
            confidence=0.90,
            margin=0.7,
            det_phase="MOVING_TO_START",
            det_phase_int=1,
            logits=[0.0, 0.05, 0.90, 0.01, 0.02, 0.01, 0.01],
        )
        assert decision.accepted is True
        assert decision.proposed_action == "suggest_approach"

    def test_retreat_high_confidence_accepted(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.RETREAT,
            pred_phase_name="RETREAT",
            confidence=0.97,
            margin=0.7,
            det_phase="INSERT",
            det_phase_int=5,
            logits=[0.0, 0.0, 0.0, 0.0, 0.02, 0.97, 0.01],
        )
        assert decision.accepted is True
        assert decision.proposed_action == "suggest_retreat"
        assert decision.retreat_uncertainty_flag is True

    def test_retreat_low_confidence_rejected(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.RETREAT,
            pred_phase_name="RETREAT",
            confidence=0.88,
            margin=0.4,
            det_phase="INSERT",
            det_phase_int=5,
            logits=[0.0, 0.0, 0.0, 0.0, 0.12, 0.88, 0.0],
        )
        assert decision.rejected is True
        assert decision.retreat_uncertainty_flag is True
        assert "retreat_low_certainty" in decision.fallback_reason

    def test_retreat_low_margin_rejected(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=PhaseID.RETREAT,
            pred_phase_name="RETREAT",
            confidence=0.96,
            margin=0.4,
            det_phase="INSERT",
            det_phase_int=5,
            logits=[0.0, 0.0, 0.0, 0.0, 0.28, 0.68, 0.04],
        )
        assert decision.rejected is True
        assert decision.retreat_uncertainty_flag is True

    def test_unknown_phase_rejected(self):
        adv = GuardedAdvisoryInterface()
        decision = adv.decide(
            pred_phase=0,
            pred_phase_name="UNKNOWN",
            confidence=0.90,
            margin=0.6,
            det_phase="APPROACH",
            det_phase_int=2,
            logits=[0.90, 0.02, 0.02, 0.01, 0.02, 0.01, 0.02],
        )
        assert decision.rejected is True
        assert "unrecognized_phase" in decision.fallback_reason

    def test_advisory_to_row_format(self):
        decision = AdvisoryDecision(
            stamp_s=1234567890.123,
            tick_index=42,
            det_phase="APPROACH",
            det_phase_int=2,
            det_safety_level="OK",
            pred_phase_int=2,
            pred_phase_name="APPROACH",
            confidence=0.92,
            margin=0.8,
            proposed_action="suggest_approach",
            accepted=True,
            rejected=False,
            fallback=False,
            fallback_reason="",
            safety_gate_status="ADVISORY_ACCEPTED",
            unsafe_if_executed=False,
            done_false_positive_flag=False,
            retreat_uncertainty_flag=False,
        )
        row = decision.to_row()
        assert len(row) == 18
        assert row[9] == "suggest_approach"
        assert row[10] == True
        assert row[11] == False


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
