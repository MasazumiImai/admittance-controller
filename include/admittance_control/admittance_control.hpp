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

#ifndef ADMITTANCE_CONTROL__ADMITTANCE_CONTROL_HPP_
#define ADMITTANCE_CONTROL__ADMITTANCE_CONTROL_HPP_

#include <Eigen/Core>
#include <limits>

namespace admittance_control
{

// Diagonal gains and bounds. NaN = unset: isValid() rejects it, so every field must be configured.
struct Params
{
  static constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d mass = Eigen::Vector3d::Constant(kUnset);
  Eigen::Vector3d damping = Eigen::Vector3d::Constant(kUnset);
  Eigen::Vector3d stiffness = Eigen::Vector3d::Constant(kUnset);
  double deadband = kUnset;  // an input with a smaller norm counts as zero
  double max_offset = kUnset;  // bound on the offset norm (+inf: unbounded)
  double max_velocity = kUnset;  // bound on the offset rate norm (+inf: unbounded)

  [[nodiscard]] bool isValid() const noexcept;
};

// M x'' + D x' + K x = u for an offset x from the caller's reference, in axes the caller picks.
// u is a force (x in m) or a moment (x a rotation vector in rad).
class AdmittanceControl
{
public:
  explicit AdmittanceControl(const Params & params);  // throws std::invalid_argument if !isValid()

  // A non-finite input or dt <= 0 leaves the state unchanged and returns false.
  [[nodiscard]] bool update(const Eigen::Vector3d & input, double dt) noexcept;
  // Restarts at rest at the given offset.
  void reset(const Eigen::Vector3d & offset = Eigen::Vector3d::Zero()) noexcept;

  [[nodiscard]] const Eigen::Vector3d & offset() const noexcept { return offset_; }
  [[nodiscard]] const Eigen::Vector3d & velocity() const noexcept { return velocity_; }

private:
  Params params_;
  Eigen::Vector3d offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
};

}  // namespace admittance_control

#endif  // ADMITTANCE_CONTROL__ADMITTANCE_CONTROL_HPP_
