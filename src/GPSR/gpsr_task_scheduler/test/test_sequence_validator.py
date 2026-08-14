"""Section: Exhaustive and seeded-random GPSR executable-plan validation tests."""

from gpsr_task_scheduler.sequence_validator import iter_exhaustive_commands, run_validation


def test_every_supported_task_template_has_all_phrase_permutations() -> None:
    """Special function: Ensure exhaustive validation spans every supported task family."""
    # Section: Four templates with four phrases each produce 4 × 4! command permutations.
    assert len(iter_exhaustive_commands()) == 96


def test_exhaustive_and_seeded_random_sequences_produce_executable_plans() -> None:
    """Special function: Verify task ordering survives exhaustive and random phrase sequences."""
    # Section: A fixed seed makes an eventual failure directly reproducible from CI output.
    report = run_validation(seed=17, random_cases=200)
    assert report.valid, report.failures
    assert report.exhaustive_cases == 96
    assert report.passed_cases == 296
