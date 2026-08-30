from enum import Enum, auto
import re


class DialogueAct(Enum):
    QUESTION = auto()
    TASK_COMMAND = auto()
    CONFIRM_YES = auto()
    CONFIRM_NO = auto()
    CANCEL = auto()
    UNKNOWN = auto()


QUESTION_PREFIXES = (
    "what ",
    "what's ",
    "who ",
    "where ",
    "when ",
    "why ",
    "how ",
    "which ",
    "tell me ",
)

TASK_PREFIXES = (
    "go ",
    "navigate ",
    "find ",
    "locate ",
    "look for ",
    "bring ",
    "give ",
    "deliver ",
    "take ",
    "get ",
    "grasp ",
    "fetch ",
    "guide ",
    "escort ",
    "lead ",
    "follow ",
    "count ",
    "greet ",
    "salute ",
    "say hello ",
)


def normalize(text: str) -> str:
    text = str(text).lower().strip()
    return re.sub(r"\s+", " ", text)


def classify_dialogue_act(text: str) -> DialogueAct:
    text = normalize(text)

    if text in {"yes", "yeah", "correct", "that's right", "that is right"}:
        return DialogueAct.CONFIRM_YES

    if text in {"no", "nope", "incorrect", "that's wrong", "that is wrong"}:
        return DialogueAct.CONFIRM_NO

    if text in {"cancel", "stop", "never mind", "nevermind"}:
        return DialogueAct.CANCEL

    if text.startswith(QUESTION_PREFIXES):
        return DialogueAct.QUESTION

    if text.startswith(TASK_PREFIXES):
        return DialogueAct.TASK_COMMAND

    return DialogueAct.UNKNOWN
