// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include <ur_rtde_driver/ur/ur_driver.h>

using namespace ur_driver;

int main(int argc, char* argv[])
{
  std::string robot_ip = "192.168.56.101";
  std::string script_filename = "urprog.urscript";
  std::string recipe_filename = "rtde_recipe.txt";

  if (argc > 2)
  {
    robot_ip = argv[1];
  }

  UrDriver driver(robot_ip, script_filename, recipe_filename);

  while (true)
  {
    sleep(1);
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver.getDataPackage();
    if (data_pkg)
    {
      data_pkg->toString();
    }
  }
  return 0;
}
