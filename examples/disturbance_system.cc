#include <random>

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

constexpr double kDisturbancePeriod = 1.0;

class DisturbanceGenerator : public LeafSystem<double> {
 public:
  explicit DisturbanceGenerator(const MultibodyPlant<double>* plant,
      const double force_mag, const double period) : plant_(plant),
      force_mag_(force_mag), period_(period),
      gen(std::random_device{}()),
      dis(-force_mag, force_mag) {
    this->DeclareAbstractOutputPort(
        "spatial_forces",
        &DisturbanceGenerator::CalcDisturbance);
    this->DeclarePeriodicDiscreteUpdateEvent(0.0, 0.0,
        &DisturbanceGenerator::PerStep);
    this->DeclareDiscreteState(1);
  }

  double generate_random_force() const {
    return dis(gen);
  }

 private:
  void CalcDisturbance(const Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
    output->resize(1);
    const RigidBody<double>* box_body = &plant_->GetBodyByName("box_ghost_body_x");
    (*output)[0].body_index = box_body->index();
    (*output)[0].p_BoBq_B = box_body->default_com();
    auto random_fy = context.get_discrete_state(0).value()[0];
    const Vector3<double> tau(0.0, 0.0, 0.0);
    const Vector3<double> force(0.0, random_fy, 0.0);
    auto random_force = SpatialForce<double>(tau, force);
    (*output)[0].F_Bq_W = random_force;
    random_fy = 0.0;
  }
  EventStatus PerStep(const Context<double>& context,
      DiscreteValues<double>* discrete_state) const {
    const Vector3<double> tau(0.0, 0.0, 0.0);
    auto random_fy = discrete_state->get_mutable_value(0);
    double random_fy_value = generate_random_force();
    random_fy(0) = random_fy_value;
    //const Vector3<double> force(0.0, random_fy(0), 0.0);
    //random_force = SpatialForce<double>(tau, force);

    return EventStatus::Succeeded();
  };
  const MultibodyPlant<double>* plant_{nullptr};
  const double force_mag_;
  const double period_{0.0};
  mutable std::mt19937 gen;
  mutable std::uniform_real_distribution<> dis;
  //SpatialForce<double> random_force;
};

}
}

