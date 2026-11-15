# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2026 Foad S. Farimani
# SPDX-FileNotice: Part of the DFM addon.

from typing import Any, Optional

from ...core.base.base_check import BaseCheck
from ...core.models import CheckResult, Severity
from ...core.processes.process import RuleFeedback, RuleLimit
from ...core.registries import register_check
from ...core.rules import Rulebook


@register_check(Rulebook.MIN_TOOL_CORNER_RADIUS)
class ToolCornerRadiusCheck(BaseCheck):
    """Inside corners along the tool axis tighter than the end mill: "At least" is the radius of the smallest
    tool the shop will use for these walls, "Aim for" the one that cuts them without slowing down."""

    @property
    def name(self) -> str:
        return "Tool Corner Radius Check"

    @property
    def progress_text(self) -> str:
        return "Checking inside corners against the tool radius"

    @property
    def required_analyzer_id(self) -> str:
        return "TOOL_CORNER_ANALYZER"

    def run_check(
        self,
        analysis_data_map: dict[Any, tuple[float, str, int]],
        rule_config: RuleLimit,
        rule: Rulebook,
        feedback: Optional[RuleFeedback] = None,
        **kwargs,
    ) -> list[CheckResult]:
        fb = feedback or RuleFeedback()
        target = self.safe_float(rule_config.target)
        limit = self.safe_float(rule_config.limit)
        if target is None and limit is None:
            return []
        corners: dict[tuple[str, int], float] = {}         # a fillet is seen from both its edges: once
        for value in analysis_data_map.values():
            radius, kind, index = value
            corners[(kind, index)] = radius
        results: list[CheckResult] = []
        for key in sorted(corners):
            measured = corners[key]
            if limit is not None and measured < limit:
                severity, template = Severity.ERROR, fb.error_msg
            elif target is not None and measured < target:
                severity, template = Severity.WARNING, fb.warning_msg
            else:
                continue
            effective_limit = limit if limit is not None else 0.0
            effective_target = target if target is not None else 0.0
            threshold = effective_limit if severity == Severity.ERROR else effective_target
            what = "sharp corner" if key[0] == "Edge" else f"R{measured:.2f}"
            results.append(
                CheckResult(
                    rule_id=rule,
                    overview=f"{what} < R{threshold:.2f}{rule.unit}",
                    message=self.format_feedback(template, measured, effective_target, effective_limit, rule.unit),
                    severity=severity,
                    failing_geometry=[key],
                    ignore=False,
                    value=float(measured),
                    limit=effective_limit,
                    comparison="<",
                    unit=rule.unit,
                )
            )
        return results
