#pragma once

#include <px4ctrl/sampled_trajectory.h>

namespace px4ctrl
{
struct ExternalModel
{
  double gravity{9.805}, mass{0.811}, heading{0.0};
  Eigen::Vector3d drag{Eigen::Vector3d::Zero()};
  double lift{0.0};
};

// Shared by planner, independent audit and controller. Includes the controller's
// nominal aerodynamic correction exactly once; geometric heading supports roll.
ReferencePoint externalReference(ReferencePoint flat, const ExternalModel &model);

// Quintic Hermite reconstruction is exact for MINCO S3 when all original piece
// boundaries are included. Samples carry p/v/a, never independently lerped q/u.
class ExternalTrajectory final : public Trajectory
{
public:
  ExternalTrajectory(TimedReferences points, ExternalModel model);
  ReferencePoint evaluate(double t) const override;
  double duration() const override {return times_.back();}
  std::vector<double> boundaries() const override {return times_;}
private:
  using Polynomial = Eigen::Matrix<double, 3, 6>;
  std::vector<double> times_;
  std::vector<Polynomial, Eigen::aligned_allocator<Polynomial>> coefficients_;
  ExternalModel model_;
  ReferencePoint start_, end_;
};
}
