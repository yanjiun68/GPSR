// Section: Compile-time and pure-runtime regression checks for the deterministic GPSR core.
#include <gtest/gtest.h>

#include "robot_yasmin_tasks/runtime.hpp"

namespace robot_yasmin_tasks {

// Special function: Verify the semantic NOT_FOUND classification remains recoverable.
TEST(SkillFactoryContracts, NotFoundRemainsRecoverable) {
  EXPECT_TRUE(is_recoverable(NOT_FOUND));
  EXPECT_FALSE(is_fatal(NOT_FOUND));
}

// Special function: Verify infrastructure and cancellation outcomes remain distinct.
TEST(SkillFactoryContracts, TerminalOutcomeClassificationIsStable) {
  EXPECT_TRUE(is_fatal(CANCELED));
  EXPECT_TRUE(is_fatal(SKILL_UNAVAILABLE));
  EXPECT_FALSE(is_recoverable(CANCELED));
  EXPECT_TRUE(is_recoverable(TIMEOUT));
}

// Special function: Verify dependency skips never enter recovery or fatal routing.
TEST(SkillFactoryContracts, DependencySkipClassificationIsStable) {
  EXPECT_FALSE(is_recoverable(SKIPPED_DEPENDENCY_FAILED));
  EXPECT_FALSE(is_fatal(SKIPPED_DEPENDENCY_FAILED));
}

// Special function: Verify a completely successful normal traversal stays successful.
TEST(SkillFactoryContracts, AggregateCompleteSuccess) {
  const std::vector<SkillResultData> results{
      make_skill_result("A", SUCCESS, "done"),
      make_skill_result("B", SUCCESS, "done"),
      make_skill_result("C", SUCCESS, "done")};
  EXPECT_EQ(aggregate_queue_outcome(results), SUCCESS);
}

// Special function: Verify useful successes mixed with failure and skip aggregate partially.
TEST(SkillFactoryContracts, AggregatePartialSuccess) {
  const std::vector<SkillResultData> results{
      make_skill_result("A", SUCCESS, "done"),
      make_skill_result("B", NOT_FOUND, "missing"),
      make_skill_result("C", SKIPPED_DEPENDENCY_FAILED, "dependency failed")};
  EXPECT_EQ(aggregate_queue_outcome(results), PARTIAL_SUCCESS);
}

// Special function: Verify a normal traversal with no successful operation reports failure.
TEST(SkillFactoryContracts, AggregateNoSuccessfulOperation) {
  const std::vector<SkillResultData> results{
      make_skill_result("A", TIMEOUT, "timed out"),
      make_skill_result("B", SKIPPED_DEPENDENCY_FAILED, "dependency failed")};
  EXPECT_EQ(aggregate_queue_outcome(results), FAILED);
}

}  // namespace robot_yasmin_tasks
