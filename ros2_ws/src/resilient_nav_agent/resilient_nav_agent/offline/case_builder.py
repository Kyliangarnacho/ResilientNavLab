"""Build physically separate Agent Input and Benchmark Truth channels."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from resilient_nav_agent.benchmark.schemas import BenchmarkTruth
from resilient_nav_agent.evidence import build_health_evidence
from resilient_nav_agent.incidents import build_offline_incident
from resilient_nav_agent.offline.schemas import OfflineAgentInput, OfflineRobotCase
from resilient_nav_agent.sanitizer import AgentInputSanitizer
from resilient_nav_agent.schemas import EvidenceItem, HealthState


class OfflineCaseBuildError(ValueError):
    """Safe failure for ambiguous or inconsistent offline raw data."""


class OfflineCaseBuilder:
    """Create two channels from observation data and a separate truth mapping."""

    def __init__(self, sanitizer: AgentInputSanitizer | None = None):
        self._sanitizer = sanitizer or AgentInputSanitizer()

    def build(
        self,
        *,
        case_id: str,
        raw_health: Sequence[Mapping[str, Any]],
        truth_mapping: Mapping[str, Any],
        trigger_component: str | None = None,
        supplemental_evidence: Sequence[EvidenceItem] = (),
    ) -> OfflineRobotCase:
        """Build the runtime channel before independently validating truth."""
        if not raw_health:
            raise OfflineCaseBuildError('at least one health observation is required')
        observations = [
            self._sanitizer.sanitize_health(item) for item in raw_health
        ]
        if len({item.component for item in observations}) != len(observations):
            raise OfflineCaseBuildError('health components must be unique per case')

        evidence = [
            build_health_evidence(
                observation,
                f'E-{case_id}-{index:02d}',
            )
            for index, observation in enumerate(observations, start=1)
        ]
        evidence.extend(supplemental_evidence)
        diagnosable = [
            item for item in observations
            if item.state in (HealthState.DEGRADED, HealthState.FAULT)
        ]
        incident = None
        if diagnosable:
            if trigger_component is None and len(diagnosable) != 1:
                raise OfflineCaseBuildError(
                    'multiple abnormal components require an explicit trigger'
                )
            selected = [
                item for item in diagnosable
                if item.component == (trigger_component or diagnosable[0].component)
            ]
            if len(selected) != 1:
                raise OfflineCaseBuildError('trigger component is not diagnosable')
            trigger = selected[0]
            incident = build_offline_incident(
                trigger,
                [item for item in observations if item != trigger],
                [item.evidence_id for item in evidence],
                incident_id=f'INC-{case_id}',
            )

        agent_input = OfflineAgentInput(
            case_id=case_id,
            incident=incident,
            evidence=evidence,
            health_observations=observations,
        )
        self._sanitizer.assert_safe_payload(
            agent_input.model_dump(mode='json')
        )

        truth_payload = dict(truth_mapping)
        if 'case_id' in truth_payload and truth_payload['case_id'] != case_id:
            raise OfflineCaseBuildError('truth case ID does not match builder input')
        truth_payload['case_id'] = case_id
        benchmark_truth = BenchmarkTruth.model_validate(truth_payload)
        return OfflineRobotCase(
            case_id=case_id,
            agent_input=agent_input,
            benchmark_truth=benchmark_truth,
        )
