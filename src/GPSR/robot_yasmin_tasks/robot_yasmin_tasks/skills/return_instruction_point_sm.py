"""Section: RETURN_TO_INSTRUCTION_POINT skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.nav2_client import Nav2Client
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_return(blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate a configured action or blackboard Instruction Point."""
    target = action.target_location or action.parameters.get("target_location")
    return (bool(target or bb_get(blackboard, bb.INSTRUCTION_POINT, None)), "Instruction point missing.")


class ReturnInstructionPointClient:
    """Special function: Navigate to stored instruction point."""
    def __init__(self, nav: Nav2Client | None = None) -> None:
        """Special function: Store nav client."""
        self.nav = nav or Nav2Client()
    def go(self, blackboard, action: ActionStepData):
        """Special function: Copy instruction point to target pose."""
        target = action.target_location or action.parameters.get("target_location")
        blackboard[bb.INSTRUCTION_POINT] = target or bb_get(blackboard, bb.INSTRUCTION_POINT, None)
        blackboard[bb.TARGET_POSE] = blackboard[bb.INSTRUCTION_POINT]
        return self.nav.navigate_to_pose(blackboard, action)


class ReturnInstructionPointSM(SimpleSkillSM):
    """Special function: Return to instruction point."""
    def __init__(self, client: ReturnInstructionPointClient | None = None) -> None:
        """Special function: Build RETURN_TO_INSTRUCTION_POINT state machine."""
        super().__init__("RETURN_INSTRUCTION_POINT_SM", (client or ReturnInstructionPointClient()).go, validate_return)
