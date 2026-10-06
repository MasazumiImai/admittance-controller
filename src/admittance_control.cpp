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

#include "admittance_control/admittance_control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace admittance_control
{
namespace
{
constexpr double kMaxIntegrationDt = 0.02;  // [s] cap on the integration horizon
constexpr double kMaxIntegrationStep = 0.002;  // [s] sub-step for stable integration
}  // namespace

bool Params::isValid() const noexcept
{
  return mass.allFinite() && (mass.array() > 0.0).all() && damping.allFinite() &&
    (damping.array() >= 0.0).all() && stiffness.allFinite() && (stiffness.array() >= 0.0).all() &&
    std::isfinite(deadband) && deadband >= 0.0 && max_offset >= 0.0 && max_velocity >= 0.0;
}

AdmittanceControl::AdmittanceControl(const Params & params) : params_(params)
{
  if (!params_.isValid()) {
    throw std::invalid_argument("Invalid admittance parameters");
  }
}

bool AdmittanceControl::update(const Eigen::Vector3d & input, double dt) noexcept
{
  if (!input.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
    return false;
  }
  const Eigen::Vector3d u = input.norm() < params_.deadband ? Eigen::Vector3d::Zero() : input;

  const double dt_clamped = std::min(dt, kMaxIntegrationDt);
  const int steps = std::max(1, static_cast<int>(std::ceil(dt_clamped / kMaxIntegrationStep)));
  const double h = dt_clamped / static_cast<double>(steps);

  for (int step = 0; step < steps; ++step) {
    const Eigen::Vector3d acceleration = params_.mass.cwiseInverse().cwiseProduct(
      u - params_.damping.cwiseProduct(velocity_) - params_.stiffness.cwiseProduct(offset_));
    velocity_ += acceleration * h;
    const double velocity_norm = velocity_.norm();
    if (velocity_norm > params_.max_velocity) {
      velocity_ *= params_.max_velocity / velocity_norm;
    }
    offset_ += velocity_ * h;

    const double offset_norm = offset_.norm();
    if (offset_norm > params_.max_offset) {
      const Eigen::Vector3d limit_direction = offset_ / offset_norm;
      offset_ = params_.max_offset * limit_direction;
      const double outward_velocity = velocity_.dot(limit_direction);
      if (outward_velocity > 0.0) {
        velocity_ -= outward_velocity * limit_direction;
      }
    }
  }
  return true;
}

void AdmittanceControl::reset(const Eigen::Vector3d & offset) noexcept
{
  offset_ = offset;
  velocity_.setZero();
}

}  // namespace admittance_control
