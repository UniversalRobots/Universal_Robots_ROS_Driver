// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-07-09
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>

#include <ur_robot_driver/rtde/rtde_client.h>

using namespace ur_driver;

const std::string ROBOT_IP = "192.168.56.101";

TEST(UrRobotDriver, rtde_handshake)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/rtde_output_recipe.txt";
  std::string input_recipe = "resources/rtde_input_recipe.txt";
  rtde_interface::RTDEClient client(ROBOT_IP, notifier, output_recipe, input_recipe);

  EXPECT_TRUE(client.init());
}

TEST(UrRobotDriver, rtde_handshake_wrong_ip)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/rtde_output_recipe.txt";
  std::string input_recipe = "resources/rtde_input_recipe.txt";
  rtde_interface::RTDEClient client("1.2.3.4", notifier, output_recipe, input_recipe);

  EXPECT_THROW(client.init(), UrException);
}

TEST(UrRobotDriver, rtde_handshake_illegal_ip)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/rtde_output_recipe.txt";
  std::string input_recipe = "resources/rtde_input_recipe.txt";
  rtde_interface::RTDEClient client("abcd", notifier, output_recipe, input_recipe);

  EXPECT_THROW(client.init(), UrException);
}

TEST(UrRobotDriver, no_recipe)
{
  comm::INotifier notifier;
  std::string output_recipe = "";
  std::string input_recipe = "";
  EXPECT_THROW(rtde_interface::RTDEClient client(ROBOT_IP, notifier, output_recipe, input_recipe), UrException);
}

TEST(UrRobotDriver, empty_recipe)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/empty.txt";
  std::string input_recipe = "resources/empty.txt";
  rtde_interface::RTDEClient client(ROBOT_IP, notifier, output_recipe, input_recipe);

  EXPECT_THROW(client.init(), UrException);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
