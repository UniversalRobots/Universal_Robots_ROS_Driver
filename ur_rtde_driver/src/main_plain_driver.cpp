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
  std::string ROBOT_IP = "192.168.56.101";

  if (argc > 1)
  {
    ROBOT_IP = argv[1];
  }

  UrDriver driver(ROBOT_IP);

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
