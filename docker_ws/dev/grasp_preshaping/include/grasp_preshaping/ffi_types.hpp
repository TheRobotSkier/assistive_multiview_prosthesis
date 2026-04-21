// Copyright 2024 Daniel
// MIT License
//
// FFI type definitions for the grasp_preshaping Rust↔C++ interface.
//
// These structs MUST be kept in sync with the #[repr(C)] definitions in
// grasp_preshaping/src/c_api.rs.  Any layout change here requires a matching
// change in the Rust source and a rebuild of the shared library.

#ifndef GRASP_PRESHAPING_FFI_TYPES_HPP
#define GRASP_PRESHAPING_FFI_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace grasp_preshaping
{

struct GraspPoseFFI
{
  double px;
  double py;
  double pz;
  double qx;
  double qy;
  double qz;
  double qw;
};

struct GraspTwistFFI
{
  double lx;
  double ly;
  double lz;
  double ax;
  double ay;
  double az;
  std::array<double, 36> covariance;
};

struct PointCloudViewFFI
{
  std::size_t width;
  std::size_t height;
  std::size_t point_step;
  std::size_t x_off;
  std::size_t y_off;
  std::size_t z_off;
  const std::uint8_t * data_ptr;
  std::size_t data_len;
};

struct GraspComputeRequestFFI
{
  GraspPoseFFI pose;
  GraspTwistFFI twist;
  PointCloudViewFFI cloud;
};

struct GraspComputeResponseFFI
{
  std::uint8_t success;
  double closure_amount;
  double combined_score;
  std::int32_t grasp_type;
};

// Function-pointer types for the two exported Rust entry points.
using GraspComputeFn = int (*) (
  const GraspComputeRequestFFI *,
  GraspComputeResponseFFI *,
  char *,
  std::size_t);

using GraspApiVersionFn = std::uint32_t (*) ();

// Return codes from grasp_preshaping_compute().
constexpr int kGraspComputeOk          = 0;
constexpr int kGraspComputeInvalidArgs = 1;
constexpr int kGraspComputePanic       = 2;

// Grasp type identifiers returned in GraspComputeResponseFFI::grasp_type.
constexpr std::int32_t kGraspTypeUnknown     = 0;
constexpr std::int32_t kGraspTypeCylindrical = 1;
constexpr std::int32_t kGraspTypePinch       = 2;
constexpr std::int32_t kGraspTypeLateral     = 3;

}  // namespace grasp_preshaping

#endif  // GRASP_PRESHAPING_FFI_TYPES_HPP
