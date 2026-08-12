"""Robot diagnostics DomainExtension consumed by independent agent-core."""

from __future__ import annotations

from enum import Enum

from agent_core import DomainExtension, RouteDecision
from agent_core.context import ContextBundle
from pydantic import BaseModel, ConfigDict, Field, StrictBool

from resilient_nav_agent.offline.schemas import OfflineAgentInput
from resilient_nav_agent.prompts import ANALYZER_INSTRUCTION, SYSTEM_PROMPT
from resilient_nav_agent.sanitizer import AgentInputSanitizer
from resilient_nav_agent.schemas import ComponentIdentifier, ShortText


class IncidentCategory(str, Enum):
    """Categories understood by the RA-1A diagnosis Analyzer."""

    SENSOR_HEALTH = 'sensor_health'
    UNKNOWN = 'unknown'
    UNSUPPORTED = 'unsupported'


class RobotAnalysis(BaseModel):
    """Route-only analysis that intentionally cannot contain hypotheses."""

    model_config = ConfigDict(extra='forbid', frozen=True)

    incident_category: IncidentCategory
    primary_component: ComponentIdentifier | None = None
    evidence_sufficient: StrictBool
    needs_tools: StrictBool
    needs_more_evidence: StrictBool
    candidate_checks: list[ShortText] = Field(default_factory=list, max_length=16)
    short_reason: ShortText


class RobotDomainExtension(DomainExtension):
    """Offline Robot diagnosis policy consumed by independent agent-core."""

    def __init__(self, agent_input: OfflineAgentInput | None = None):
        self._agent_input = agent_input
        if agent_input is not None:
            AgentInputSanitizer().assert_safe_payload(
                agent_input.model_dump(mode='json')
            )

    @property
    def extension_id(self) -> str:
        """Return the stable Robot diagnostics extension identifier."""
        return 'resilient-nav.robot-diagnostics'

    @property
    def system_prompt(self) -> str:
        """Return the minimal offline diagnostic system boundary."""
        return SYSTEM_PROMPT

    @property
    def analyzer_instruction(self) -> str:
        """Return the structured bootstrap analyzer instruction."""
        return ANALYZER_INSTRUCTION

    @property
    def analysis_schema(self) -> type[BaseModel]:
        """Return the Robot-owned Pydantic analysis schema."""
        return RobotAnalysis

    def fallback_analysis(self, query: str) -> RobotAnalysis:
        """Return a safe needs-more-evidence fallback without parsing query."""
        del query
        return RobotAnalysis(
            incident_category=IncidentCategory.UNKNOWN,
            primary_component=None,
            evidence_sufficient=False,
            needs_tools=False,
            needs_more_evidence=True,
            candidate_checks=['Inspect additional sanitized health evidence.'],
            short_reason='Structured analysis was unavailable.',
        )

    def route(self, analysis: BaseModel, bundle: ContextBundle) -> RouteDecision:
        """Route to diagnose, needs-more-evidence, or blocked."""
        del bundle
        parsed = RobotAnalysis.model_validate(analysis)
        if parsed.needs_tools:
            return RouteDecision(
                route='blocked',
                should_answer=False,
                response_message='No registered read-only Tool is available.',
            )
        if parsed.incident_category == IncidentCategory.UNSUPPORTED:
            return RouteDecision(
                route='blocked',
                should_answer=False,
                response_message='The incident category is unsupported.',
            )
        if parsed.needs_more_evidence or not parsed.evidence_sufficient:
            return RouteDecision(
                route='needs_more_evidence',
                should_answer=True,
                additional_instructions=[
                    'Return a DiagnosisResult with status insufficient_evidence '
                    'and state which read-only evidence is missing.'
                ],
            )
        if parsed.primary_component is None:
            return RouteDecision(
                route='blocked',
                should_answer=False,
                response_message='A primary component is required.',
            )
        return RouteDecision(
            route='diagnose',
            should_answer=True,
            additional_instructions=[
                'Return only strict DiagnosisResult JSON. Diagnose using only '
                'the sanitized evidence and cite evidence_id values.'
            ],
        )

    def domain_state_context(self, conversation_id, query):
        """Expose only a deep-copied, already sanitized OfflineAgentInput."""
        del conversation_id, query
        if self._agent_input is None:
            return {}
        return self._agent_input.model_dump(mode='json')
