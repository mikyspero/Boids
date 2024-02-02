#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boid.hpp"

namespace dynamics {
// A struct containing parameters necessary for the dynamics of the simulation
// It specifies the behavior of the boids in the flock
struct running_parameters {
  int boids_number{120};
  double s{0.5};            // Separation parameter
  double a{0.6};            // Alignment parameter
  double c{0.02};           // Cohesion parameter
  double d_s{1.};           // Distance at which separation gets activated
  double d{9.};             // Distance to define the neighborhood between boids
  double left_bound{0.};    // Left bound of the simulation space
  double right_bound{176.}; // Right bound of the simulation space
  double upper_bound{99.};  // Upper bound of the simulation space
  double bottom_bound{0.};  // Bottom bound of the simulation space
  double maximum_velocity{80.}; // Maximum velocity of the boids
  double minimum_velocity{20.}; // Minimum velocity of the boids
};

// Functions that calculate the components of the boid acceleration
// Calculate separation component of the boid acceleration
math::R2 calculate_separation(Boid const &fixed_boid,
                              std::vector<Boid> const &flock, double const s,
                              double const d_s);

// Calculate alignment component of the boid acceleration
math::R2 calculate_alignment(Boid const &fixed_boid,
                             std::vector<Boid> const &flock, double const a);

// Calculate cohesion component of the boid acceleration
math::R2 calculate_cohesion(Boid const &fixed_boid,
                            std::vector<Boid> const &flock, double const c);

// Teleport a point toroidally within the simulation space
math::R2 teleport_toroidally(math::R2 &r, running_parameters const &parameters);

// Limit the speed of a boid based on the simulation parameters
math::R2 limit_speed(math::R2 &to_be_checked,
                     running_parameters const &parameters);

// Apply boid evolution to every boid in the vector
void evolve_flock(std::vector<Boid> &flock, double const delta_t,
                  running_parameters const &parameters);

// Evolve a single boid based on its neighbors and parameters
Boid evolve_boid(std::vector<Boid> const &flock, Boid &fixed_boid,
                 double const delta_t, running_parameters const &parameters);

// function to generate a random vector of boids following the given parameters
std::vector<dynamics::Boid>
create_flock(dynamics::running_parameters const &parameters);
} // namespace dynamics

#endif // namespace dynamics