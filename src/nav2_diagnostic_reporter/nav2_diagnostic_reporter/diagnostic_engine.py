"""Small pure diagnostic result objects shared by the ROS proxy and tests."""

# Section: Standard-library result-object imports.
from dataclasses import dataclass
from typing import Dict

# Section: Stable machine-error imports.
from .error_codes import ErrorCode


# Section: Result returned by each diagnostic stage.
@dataclass(frozen=True)
class DiagnosticFinding:
    """Represent one deterministic diagnostic conclusion and technical context."""

    code: ErrorCode
    detail: str = ''
    values: Dict[str, str] = None

    @property
    def ok(self) -> bool:
        """Return whether this finding represents a successful check."""

        return self.code == ErrorCode.NONE


# Section: Successful preflight result constant factory.
def success_finding(detail: str = 'preflight passed') -> DiagnosticFinding:
    """Create a standard success finding without shared mutable state."""

    return DiagnosticFinding(ErrorCode.NONE, detail, {})
