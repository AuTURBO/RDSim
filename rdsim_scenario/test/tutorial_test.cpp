#include <gtest/gtest.h>

TEST(rdsim_behavior_tree, a_first_test)
{
  ASSERT_EQ(5, 2 + 2);
}

TEST(rdsim_behavior_tree, a_second_test)
{
  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}