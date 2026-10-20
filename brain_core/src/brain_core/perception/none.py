"""The detector that is not there: `perception_model: "none"`.

It is never ready, so both searches are refused with "no perception
source: perception_model is 'none'", what the contract asks of a brain
that cannot see. Every other member keeps working.
"""

from __future__ import annotations

from typing import Sequence

from ..ports import Box, Coverage, Deadline


class NoneDetector:
    name = "none"

    def why_unavailable(self) -> str:
        return "perception_model is 'none'"

    async def detect(self, image, phrases: Sequence[str], deadline: Deadline) -> list[Box]:
        return []

    def scan_coverage(self) -> Coverage:
        return Coverage()
