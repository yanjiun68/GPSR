"""Contract tests for negative visible-attribute S_err handling."""  # Section: State the focused test purpose.

from __future__ import annotations  # Section: Keep annotations compatible with Python 3.10.

import sys  # Section: Add sibling source packages without requiring an installed overlay.
import unittest  # Section: Use the standard library test runner for portability.
from pathlib import Path  # Section: Resolve workspace-relative source files reliably.


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]  # Section: Locate the workspace root from this test file.
SCHEDULER_SOURCE = REPOSITORY_ROOT / "src" / "GPSR" / "gpsr_task_scheduler"  # Section: Locate parser sources.
TASKS_SOURCE = REPOSITORY_ROOT / "src" / "GPSR" / "robot_yasmin_tasks"  # Section: Locate shared GPSR action vocabulary.
SERVER_SOURCE = TASKS_SOURCE / "src" / "nodes" / "skill_servers" / "find_person_by_attribute_skill_server.cpp"  # Section: Locate the compiled S_err implementation.
sys.path[:0] = [str(SCHEDULER_SOURCE), str(TASKS_SOURCE)]  # Section: Prefer the edited source tree over installed packages.

from gpsr_task_scheduler.config import default_config  # Section: Use production-default parser vocabulary.
from gpsr_task_scheduler.parser import parse_command  # Section: Exercise the real command parser.


class NegativeAttributeFailureRouteTest(unittest.TestCase):  # Section: Verify parser-to-S_err behavior contracts.
    """Validate every required element of the negative-attribute failure route."""  # Special function: Group the S_err contract checks.

    def parse_selector(self, instruction: str) -> dict[str, object]:  # Special function: Parse one voice command and return its normalized selector.
        request, issues = parse_command("negative_attribute_test", instruction, default_config())  # Section: Invoke production parsing with a stable command ID.
        self.assertEqual([], issues)  # Section: Negative visible attributes must be accepted, not rejected at admission.
        self.assertIsNotNone(request)  # Section: Parsing must produce an executable request.
        return request.person_selector  # Section: Return the selector supplied to FIND_PERSON_BY_ATTRIBUTE.

    def test_without_glasses_routes_to_s_err(self) -> None:  # Special function: Cover the common negative wearable phrasing.
        self.assertEqual(  # Section: Verify the exact negative predicate accepted by the C++ server.
            {"all": [{"type": "glasses", "operator": "not", "value": "yes"}]},
            self.parse_selector("find the person without glasses"),
        )

    def test_negative_shirt_color_routes_to_s_err(self) -> None:  # Special function: Cover a negative visible clothing attribute.
        self.assertEqual(  # Section: Verify the color remains available to the server's negative predicate detector.
            {"all": [{"type": "shirt_color", "operator": "not", "value": "red"}]},
            self.parse_selector("find the person not wearing a red shirt"),
        )

    def test_positive_attribute_keeps_normal_search_path(self) -> None:  # Special function: Guard against treating positive requests as failure states.
        self.assertEqual(  # Section: Verify the normal affirmative attribute contract is unchanged.
            {"all": [{"type": "glasses", "operator": "equals", "value": "yes"}]},
            self.parse_selector("find the person with glasses"),
        )

    def test_server_contains_complete_s_err_contract(self) -> None:  # Special function: Verify the compiled server owns all required failure actions.
        source = SERVER_SOURCE.read_text(encoding="utf-8")  # Section: Inspect the exact C++ source compiled by colcon.
        self.assertIn("execute_negative_attribute_failure", source)  # Section: Require a dedicated S_err state handler.
        self.assertIn("the person with described attribute is not detected", source)  # Section: Require the mandated spoken report.
        self.assertIn("S_err_RETURNING_TO_INSTRUCTION_POINT", source)  # Section: Require an observable return-state feedback label.
        self.assertIn("navigation_goal.target_name = get_parameter(\"instruction_point\").as_string();", source)  # Section: Require a configured named return target.
        self.assertIn('result->outcome = "not_found";', source)  # Section: Require a failure outcome after report and return.


if __name__ == "__main__":  # Section: Permit direct execution without pytest or ROS test tooling.
    unittest.main(verbosity=2)  # Special function: Run every S_err contract test with visible results.
