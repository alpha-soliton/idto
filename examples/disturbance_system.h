#include <random>

#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/math/spatial_force.h>
#include <drake/multibody/tree/multibody_element.h>
#include "drake/multibody/tree/rigid_body.h"
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/event.h>

namespace idto {
namespace examples {

using Eigen::VectorXd;
using Eigen::Vector3;

using drake::math::RigidTransform;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialForce;
using drake::multibody::RigidBody;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;
using drake::systems::VectorBase;
using drake::systems::PublishEvent;
using drake::systems::TriggerType;

class DisturbanceGenerator : public LeafSystem<double> {
 public:
  explicit DisturbanceGenerator(const MultibodyPlant<double>* plant,
      const double force_mag, const double period);

  double generate_random_force() const {
    return dis(gen);
  }

  mutable double last_force_update{0.0};

 private:
  void CalcDisturbance(const Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const;

  void OutputTargetTransform(const Context<double>& context,
      RigidTransform<double>* output) const;

  EventStatus PerStep(const Context<double>& context,
      DiscreteValues<double>* discrete_state) const;
  const MultibodyPlant<double>* plant_{nullptr};
  const double force_mag_;
  const double period_{0.0};
  mutable std::mt19937 gen;
  mutable std::uniform_real_distribution<> dis;
  drake::multibody::BodyIndex box_body_index_{};

  // 入力ポートのインデックスを保持
  drake::systems::InputPortIndex body_poses_port_index_;
};

}
}

