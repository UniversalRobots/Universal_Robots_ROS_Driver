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
  Calibration calibration;
  KDL::Chain corrected_chain = calibration.correctChain();
  calibration.debugChain(corrected_chain);

  return 0;
}
