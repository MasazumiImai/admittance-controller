// Copyright (c) 2026 Masazumi Imai
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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "admittance_control/admittance_control.hpp"

namespace
{

using admittance_control::AdmittanceControl;
using admittance_control::Params;

Params params()
{
  Params params;
  params.mass.setConstant(2.0);
  params.damping.setZero();
  params.stiffness.setZero();
  params.deadband = 0.0;
  params.max_offset = 0.01;
  params.max_velocity = 1.0;
  return params;
}

TEST(Admittance, YieldsToTheInputWithinItsBounds)
{
  AdmittanceControl admittance(params());
  ASSERT_TRUE(admittance.update(Eigen::Vector3d::Zero(), 0.01));
  EXPECT_TRUE(admittance.offset().isZero());

  ASSERT_TRUE(admittance.update(100.0 * Eigen::Vector3d::UnitX(), 0.01));
  EXPECT_GT(admittance.velocity().x(), 0.0);
  for (int iteration = 1; iteration < 100; ++iteration) {
    ASSERT_TRUE(admittance.update(100.0 * Eigen::Vector3d::UnitX(), 0.01));
  }
  EXPECT_GT(admittance.offset().x(), 0.0);
  EXPECT_LE(admittance.offset().norm(), params().max_offset + 1.0e-12);
  EXPECT_LE(admittance.velocity().norm(), params().max_velocity + 1.0e-12);

  admittance.reset();
  EXPECT_TRUE(admittance.offset().isZero());
  EXPECT_TRUE(admittance.velocity().isZero());
  admittance.reset(Eigen::Vector3d(0.0, 0.005, 0.0));
  EXPECT_EQ(admittance.offset(), Eigen::Vector3d(0.0, 0.005, 0.0));
  EXPECT_TRUE(admittance.velocity().isZero());
}

TEST(AdmittanceControl, SpringSettlesAtInputOverStiffnessAndDeadbandIgnoresSmallInputs)
{
  Params p = params();
  p.damping.setConstant(20.0);
  p.stiffness.setConstant(100.0);
  p.deadband = 1.0;
  p.max_offset = std::numeric_limits<double>::infinity();
  p.max_velocity = std::numeric_limits<double>::infinity();
  AdmittanceControl admittance(p);
  ASSERT_TRUE(admittance.update(Eigen::Vector3d(0.5, 0.0, 0.0), 0.01));
  EXPECT_TRUE(admittance.offset().isZero());

  const Eigen::Vector3d input(10.0, -5.0, 2.0);
  for (int iteration = 0; iteration < 1000; ++iteration) {
    ASSERT_TRUE(admittance.update(input, 0.01));
  }
  EXPECT_TRUE(admittance.offset().isApprox(input / 100.0, 1.0e-6));
}

TEST(AdmittanceControl, RejectsInvalidInputsAndParams)
{
  AdmittanceControl admittance(params());
  ASSERT_TRUE(admittance.update(Eigen::Vector3d::UnitX(), 0.01));
  const Eigen::Vector3d offset = admittance.offset();
  const Eigen::Vector3d velocity = admittance.velocity();
  EXPECT_FALSE(admittance.update(Eigen::Vector3d::Constant(std::nan("")), 0.01));
  EXPECT_FALSE(admittance.update(Eigen::Vector3d::UnitX(), 0.0));
  EXPECT_FALSE(admittance.update(Eigen::Vector3d::UnitX(), std::nan("")));
  EXPECT_EQ(admittance.offset(), offset);
  EXPECT_EQ(admittance.velocity(), velocity);

  EXPECT_THROW(AdmittanceControl{Params{}}, std::invalid_argument);
  Params massless = params();
  massless.mass.x() = 0.0;
  EXPECT_THROW(AdmittanceControl{massless}, std::invalid_argument);
  Params nan_bound = params();
  nan_bound.max_offset = std::nan("");
  EXPECT_THROW(AdmittanceControl{nan_bound}, std::invalid_argument);
}

}  // namespace
