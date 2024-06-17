// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-03-11
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>
#include <ur_calibration/calibration.h>

using ur_calibration::Calibration;
using ur_calibration::DHRobot;
using ur_calibration::DHSegment;

using CalibrationMat = Eigen::Matrix<double, 6, 4>;

/*
bool isApproximately(const double val1, const double val2, const double precision)
{
  return std::abs(val1 - val2) < precision;
}
*/
template <class Scalar_, int dim_>
void doubleEqVec(const Eigen::Matrix<Scalar_, dim_, 1> vec1, const Eigen::Matrix<Scalar_, dim_, 1> vec2,
                 const double precision)
{
  for (size_t i = 0; i < dim_; ++i)
  {
    EXPECT_NEAR(vec1[i], vec2[i], precision);
  }
}

DHRobot model_from_dh(std::array<double, 6> d, std::array<double, 6> a, std::array<double, 6> theta,
                      std::array<double, 6> alpha)
{
  DHRobot robot;
  for (size_t i = 0; i < 6; ++i)
  {
    robot.segments_.emplace_back(DHSegment(d[i], a[i], theta[i], alpha[i]));
  }
  return robot;
}

class CalibrationTest : public ::testing::TestWithParam<DHRobot>
{
public:
  void SetUp()
  {
    robot_models_.insert({ "ur10_ideal", model_from_dh({ 0.1273, 0, 0, 0.163941, 0.1157, 0.0922 },  // d
                                                       { 0, -0.612, -0.5723, 0, 0, 0 },             // a
                                                       { 0, 0, 0, 0, 0, 0 },                        // theta
                                                       { pi / 2, 0, 0, pi / 2, -pi / 2, 0 }         // alpha
                                                       ) });
    robot_models_.insert({ "ur10", model_from_dh({ 0.1273, 0, 0, 0.163941, 0.1157, 0.0922 },  // d
                                                 { 0, -0.612, -0.5723, 0, 0, 0 },             // a
                                                 { 0, 0, 0, 0, 0, 0 },                        // theta
                                                 { pi_2, 0, 0, pi_2, -pi_2, 0 }               // alpha
                                                 ) });
    robot_models_.insert({ "ur10e", model_from_dh({ 0.1807, 0, 0, 0.17415, 0.11985, 0.11655 },  // d
                                                  { 0, -0.6127, -0.57155, 0, 0, 0 },            // a
                                                  { 0, 0, 0, 0, 0, 0 },                         // theta
                                                  { pi_2, 0, 0, pi_2, -pi_2, 0 }                // alpha
                                                  ) });
  }
  void TearDown()
  {
    Test::TearDown();
  }

protected:
  const double pi = std::atan(1) * 4;
  const double pi_2 = 1.570796327;  // This is what the simulated robot reports as pi/2
  std::map<std::string, DHRobot> robot_models_;
};

TEST_F(CalibrationTest, ur10_fw_kinematics_ideal)
{
  const auto& my_robot = robot_models_["ur10_ideal"];
  Calibration calibration(my_robot);

  Eigen::Matrix<double, 6, 1> jointvalues;
  Eigen::Vector3d expected_position;
  Eigen::Quaterniond expected_orientation;
  {
    jointvalues << 0, 0, 0, 0, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
    EXPECT_DOUBLE_EQ(fk(0, 3), my_robot.segments_[1].a_ + my_robot.segments_[2].a_);
    EXPECT_DOUBLE_EQ(fk(1, 3), -1 * (my_robot.segments_[3].d_ + my_robot.segments_[5].d_));
    EXPECT_DOUBLE_EQ(fk(2, 3), my_robot.segments_[0].d_ - my_robot.segments_[4].d_);
  }

  {
    jointvalues << M_PI_2, -M_PI_4, M_PI_2, -M_PI_4, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);

    expected_position << my_robot.segments_[3].d_ + my_robot.segments_[5].d_,
        my_robot.segments_[1].a_ / std::sqrt(2) + my_robot.segments_[2].a_ / std::sqrt(2),
        my_robot.segments_[0].d_ - my_robot.segments_[1].a_ / std::sqrt(2) + my_robot.segments_[2].a_ / std::sqrt(2) -
            my_robot.segments_[4].d_;
    doubleEqVec<double, 3>(expected_position, fk.topRightCorner(3, 1), 1e-8);
  }
}

TEST_F(CalibrationTest, ur10_fw_kinematics_real)
{
  // This test compares a corrected ideal model against positions taken from a simulated robot.
  const auto& my_robot = robot_models_["ur10"];
  Calibration calibration(my_robot);

  Eigen::Matrix<double, 6, 1> jointvalues;
  Eigen::Vector3d expected_position;
  Eigen::Quaterniond expected_orientation;
  {
    jointvalues << -1.6007002035724084976209269370884, -1.7271001974688928726209269370884,
        -2.2029998938189905288709269370884, -0.80799991289247685699592693708837, 1.59510004520416259765625,
        -0.03099996248354131012092693708837;
    expected_position << -0.179925914147547, -0.606869755162764, 0.230789102067257;

    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);

    doubleEqVec<double, 3>(expected_position, fk.topRightCorner(3, 1), 1e-15);
  }
  {
    jointvalues << 1.32645022869110107421875, 2.426007747650146484375, 5.951572895050048828125,
        1.27409040927886962890625, -0.54105216661562138824592693708837, 0.122173048555850982666015625;
    expected_position << 0.39922988003280424074148413637886, 0.59688365069340565405298093537567,
        -0.6677819040276375961440180617501;

    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);

    doubleEqVec<double, 3>(expected_position, fk.topRightCorner(3, 1), 1e-15);
  }

  TearDown();
}

TEST_P(CalibrationTest, calibration)
{
  // This test compares the forward kinematics of the model constructed from uncorrected
  // parameters with the one from the corrected parameters. They are tested against random
  // joint values and should be equal (in a numeric sense).

  auto my_robot = robot_models_["ur10"];
  Calibration calibration(my_robot);

  auto robot_calibration = GetParam();

  Eigen::Matrix<double, 6, 1> jointvalues;
  jointvalues << 0, 0, 0, 0, 0, 0;

  for (size_t i = 0; i < 1000; ++i)
  {
    Calibration calibration(my_robot + robot_calibration);
    jointvalues = 2 * pi * Eigen::Matrix<double, 6, 1>::Random();
    Eigen::Matrix4d fk_orig = calibration.calcForwardKinematics(jointvalues);
    Eigen::Matrix3d rot_mat_orig = fk_orig.topLeftCorner(3, 3);
    Eigen::Quaterniond rot_orig(rot_mat_orig);
    calibration.correctChain();
    Eigen::Matrix4d fk_corrected = calibration.calcForwardKinematics(jointvalues);
    Eigen::Matrix3d rot_mat_corrected = fk_corrected.topLeftCorner(3, 3);
    Eigen::Quaterniond rot_corrected(rot_mat_corrected);
    double angle_error = std::abs(rot_orig.angularDistance(rot_corrected));
    EXPECT_NEAR(fk_orig(0, 3), fk_corrected(0, 3), 1e-12);
    EXPECT_NEAR(fk_orig(1, 3), fk_corrected(1, 3), 1e-12);
    EXPECT_NEAR(fk_orig(2, 3), fk_corrected(2, 3), 1e-12);
    EXPECT_NEAR(angle_error, 0.0, 1e-12);
  }
}

// Tests are parametrized by both, the robot model (e.g. "ur10e") and the DH diff as they are stored on the robot
// controller's calibration file.
// The test will then assemble the DH parameters using the ones from the base model and the calibration.
INSTANTIATE_TEST_SUITE_P(
    CalibrationTests, CalibrationTest,
    ::testing::Values(
        model_from_dh({ 0.00065609212979853, 1.4442162376284788, 0.854147723854608, -2.2989425877563705,
                        -1.573498686836816e-05, 1.9072435590711256e-05 },  // d
                      { 4.6311376834935676e-05, -0.00012568315331862312, 0.00186216581161458, 9.918593870679266e-05,
                        4.215462720453189e-06, 0 },  // a
                      { -7.290070070824746e-05, -0.01713897289704999, -0.03707159413492756, 0.054279462160583214,
                        1.488984257025741e-07, 1.551499479707493e-05 },  // theta
                      { 0.000211987863869334, -0.0072553625957652995, -0.013483226769541364, 0.0013495820227329425,
                        -0.001263136163679901, 0 }  // alpha
                      ),

        model_from_dh({ -0.000144894975118076141, 303.469135666158195, -309.88394307789747, 6.41459904397394975,
                        -4.48232900190081995e-05, -0.00087071402790364627 },  // d
                      { 0.000108651068627930392, 0.240324175250532346, 0.00180628180213493472, 2.63076149165684402e-05,
                        3.96638632500012715e-06, 0 },  // a
                      { 1.59613525316931737e-07, -0.917209621528830232, 7.12936131346499469, 0.0710299029424392298,
                        -1.64258976526054923e-06, 9.17286101034808787e-08 },  // theta
                      { -0.000444781952841921679, -0.00160215112531214153, 0.00631917793331091861,
                        -0.00165055247340828437, 0.000763682515545038854, 0 }  // alpha
                      ),
        model_from_dh({ -0.000160188285741325043, 439.140974079900957, -446.027059806332147, 6.88618689642360149,
                        -3.86588496858186748e-05, -0.00087908274257374186 },  // d
                      { 2.12234865571205891e-05, 0.632017132627700651, 0.00229833638891230319, -4.61409023720933908e-05,
                        -6.39280053471801522e-05, 0 },  // a
                      { -2.41478068943590252e-07, 1.60233952386694556, -1.68607190752171299, 0.0837331147700118711,
                        -1.01260355871157781e-07, 3.91986209186123702e-08 },  // theta
                      { -0.000650246557584166496, 0.00139416666825590402, 0.00693818880325995126,
                        -0.000811641562390219562, 0.000411120504570705592, 0 }  // alpha
                      )

            ));

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
