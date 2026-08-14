"""Explicit real-model entry point for the RA-1A offline benchmark."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from enum import Enum

from agent_core.model import CompatibleModelClient, ModelConfig
from pydantic import Field, model_validator

from resilient_nav_agent.benchmark.runner import BatchBenchmarkRunner
from resilient_nav_agent.benchmark.schemas import BenchmarkKind, BenchmarkReport
from resilient_nav_agent.offline.fixtures import reference_robot_cases
from resilient_nav_agent.offline.schemas import OfflineAgentInput, OfflineRobotCase
from resilient_nav_agent.schemas import DomainModel, Identifier


REAL_SMOKE_CASE_IDS = ('CASE-001', 'CASE-002', 'CASE-006', 'CASE-008')
"""Representative IMU, wheel, camera, and healthy real-model smoke cases."""


class RealModelBenchmarkStatus(str, Enum):
    """Execution state separate from deterministic benchmark scoring."""

    COMPLETED = 'completed'
    BLOCKED = 'blocked'


class RealModelBenchmarkOutcome(DomainModel):
    """A safe report envelope which never stores endpoint configuration."""

    benchmark_kind: BenchmarkKind = BenchmarkKind.REAL_MODEL
    status: RealModelBenchmarkStatus
    selected_case_ids: tuple[Identifier, ...] = Field(min_length=1, max_length=8)
    report: BenchmarkReport | None = None
    blocker: str | None = None

    @model_validator(mode='after')
    def validate_outcome(self):
        """Keep blocked configuration and scored benchmark states distinct."""
        if self.status == RealModelBenchmarkStatus.COMPLETED:
            if self.report is None or self.blocker is not None:
                raise ValueError('completed real benchmark requires only a report')
            if self.report.benchmark_kind != BenchmarkKind.REAL_MODEL:
                raise ValueError('real benchmark report has an invalid label')
            if tuple(item.case_id for item in self.report.case_results) != (
                self.selected_case_ids
            ):
                raise ValueError('report cases do not match selected real cases')
        elif self.report is not None or not self.blocker:
            raise ValueError('blocked real benchmark requires only a blocker')
        return self


ModelClientFactory = Callable[[OfflineAgentInput], CompatibleModelClient]


def select_reference_cases(
    case_ids: Sequence[str] = REAL_SMOKE_CASE_IDS,
) -> list[OfflineRobotCase]:
    """Return selected fixture envelopes without exposing them to the client."""
    requested = tuple(case_ids)
    if not requested:
        raise ValueError('at least one benchmark case is required')
    if len(requested) != len(set(requested)):
        raise ValueError('benchmark case IDs must be unique')
    available = {case.case_id: case for case in reference_robot_cases()}
    unknown = sorted(set(requested).difference(available))
    if unknown:
        raise ValueError('unknown benchmark case IDs: ' + ', '.join(unknown))
    return [available[case_id] for case_id in requested]


def run_real_model_benchmark(
    *,
    case_ids: Sequence[str] = REAL_SMOKE_CASE_IDS,
    client_factory: ModelClientFactory | None = None,
) -> RealModelBenchmarkOutcome:
    """
    Run selected cases through a configured compatible client and Scorer.

    The factory receives only ``OfflineAgentInput``.  ``BenchmarkTruth`` stays
    inside ``BatchBenchmarkRunner`` until after each model-backed diagnosis.
    """
    cases = select_reference_cases(case_ids)
    selected_case_ids = tuple(case.case_id for case in cases)
    if client_factory is None:
        try:
            config = ModelConfig.from_env()
        except (TypeError, ValueError):
            return RealModelBenchmarkOutcome(
                status=RealModelBenchmarkStatus.BLOCKED,
                selected_case_ids=selected_case_ids,
                blocker=(
                    'Real API execution blocked by missing environment '
                    'configuration.'
                ),
            )

        def client_factory(_agent_input: OfflineAgentInput) -> CompatibleModelClient:
            return CompatibleModelClient(config)

    report = BatchBenchmarkRunner().run(
        cases,
        lambda agent_input: client_factory(agent_input).complete,
        benchmark_kind=BenchmarkKind.REAL_MODEL,
    )
    return RealModelBenchmarkOutcome(
        status=RealModelBenchmarkStatus.COMPLETED,
        selected_case_ids=selected_case_ids,
        report=report,
    )


def main(argv: Sequence[str] | None = None) -> None:
    """Print the real-model smoke or explicit full benchmark report as JSON."""
    parser = argparse.ArgumentParser(
        description='Run the RA-1A REAL MODEL BENCHMARK against a compatible API.',
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='run all CASE-001 through CASE-008 instead of the four-case smoke',
    )
    args = parser.parse_args(argv)
    cases = (
        tuple(case.case_id for case in reference_robot_cases())
        if args.all
        else REAL_SMOKE_CASE_IDS
    )
    print(run_real_model_benchmark(case_ids=cases).model_dump_json(indent=2))


__all__ = [
    'REAL_SMOKE_CASE_IDS',
    'RealModelBenchmarkOutcome',
    'RealModelBenchmarkStatus',
    'run_real_model_benchmark',
    'select_reference_cases',
]
