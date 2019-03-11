// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-01-10
 *
 */
//----------------------------------------------------------------------

#include <ur_rtde_driver/calibration.h>

int main(int argc, char* argv[])
{
  DHRobot my_robot;
  my_robot.segments_.push_back(DHSegment(0.1273, 0, 0, M_PI / 2));
  my_robot.segments_.push_back(DHSegment(0, -0.612, 0, 0));
  my_robot.segments_.push_back(DHSegment(0, -0.5723, 0, 0.0));
  my_robot.segments_.push_back(DHSegment(0.163841, 0, 0.0, M_PI / 2));
  my_robot.segments_.push_back(DHSegment(0.1157, 0, 0, -M_PI / 2));
  my_robot.segments_.push_back(DHSegment(0.0922, 0, 0, 0));
  DHRobot my_robot_calibration;
  // clang-format off
  //                                                 d,                      a,                       theta,                  alpha
  my_robot_calibration.segments_.push_back(DHSegment(0.00065609212979853    ,4.6311376834935676e-05  ,-7.290070070824746e-05 ,0.000211987863869334  ));
  my_robot_calibration.segments_.push_back(DHSegment(1.4442162376284788     ,-0.00012568315331862312 ,-0.01713897289704999   ,-0.0072553625957652995));
  my_robot_calibration.segments_.push_back(DHSegment(0.854147723854608      ,0.00186216581161458     ,-0.03707159413492756   ,-0.013483226769541364 ));
  my_robot_calibration.segments_.push_back(DHSegment(-2.2989425877563705    ,9.918593870679266e-05   ,0.054279462160583214   ,0.0013495820227329425 ));
  my_robot_calibration.segments_.push_back(DHSegment(-1.573498686836816e-05 ,4.215462720453189e-06   ,1.488984257025741e-07  ,-0.001263136163679901 ));
  my_robot_calibration.segments_.push_back(DHSegment(1.9072435590711256e-05 ,0                       ,1.551499479707493e-05  ,0                     ));
  // clang-format on

  Calibration calibration(my_robot + my_robot_calibration);
  KDL::Chain corrected_chain = calibration.correctChain();
  calibration.debugChain(corrected_chain);

  return 0;
}
