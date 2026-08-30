from gpsr_dialogue_manager.dialogue_router import (
    DialogueAct,
    classify_dialogue_act,
)

def test_question():
    assert classify_dialogue_act("What is your team name?") == DialogueAct.QUESTION

def test_task_command():
    assert classify_dialogue_act("Find the apple in the kitchen") == DialogueAct.TASK_COMMAND

def test_cancel():
    assert classify_dialogue_act("stop") == DialogueAct.CANCEL

def test_unknown():
    assert classify_dialogue_act("hello robot") == DialogueAct.UNKNOWN
