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

}  // namespace robot_yasmin_tasks
