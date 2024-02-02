#include "../include/flock.hpp"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <random>

namespace dynamics {

// Teleport a point toroidally within the simulation space

math::R2 teleport_toroidally(math::R2 &r,
                             running_parameters const &parameters) {
  // Check and adjust y-coordinate
  if (r.y > parameters.upper_bound) {
    r.y = (parameters.bottom_bound + std::abs(r.y) - parameters.upper_bound);
  }
  if (r.y < parameters.bottom_bound) {
    r.y = (parameters.upper_bound - std::abs(r.y - parameters.bottom_bound));
  }
  // Check and adjust x-coordinate
  if (r.x > parameters.right_bound) {
    r.x = (parameters.left_bound + std::abs(r.x - parameters.right_bound));
  }
  if (r.x < parameters.left_bound) {
    r.x = (parameters.right_bound - std::abs(r.x - parameters.left_bound));
  }
  return r;
}

// Limit the speed of a boid based on the simulation parameters
// the recursion covers the edge cases where the halving or doubling is not
// enough to bring the velocity under the limit
math::R2 limit_speed(math::R2 &to_be_checked,
                     running_parameters const &parameters) {
  // If velocity magnitude exceeds maximum, halve it
  if (math::calculate_norm(to_be_checked) > parameters.maximum_velocity) {
    to_be_checked *= 0.5;
    limit_speed(to_be_checked, parameters);
  }
  // If velocity magnitude is below minimum, double it
  if (math::calculate_norm(to_be_checked) < parameters.minimum_velocity) {
    to_be_checked *= 2.;
    limit_speed(to_be_checked, parameters);
  }
  return to_be_checked;
}

// Calculate separation component of the boid acceleration
math::R2 calculate_separation(Boid const &boid_to_evolve,
                              std::vector<Boid> const &flock, double const s,
                              double const d_s) {
  math::R2 separation_sum;
  // Iterate through all boids in the flock
  std::for_each(flock.begin(), flock.end(), [&](Boid const &current_boid) {
    // If the distance is less than d_s, add the separation vector
    if (calculate_distance(boid_to_evolve, current_boid) < d_s) {
      separation_sum += (current_boid.r() - boid_to_evolve.r());
    }
  });
  // Return the negation of the sum multiplied by the separation parameter
  return -separation_sum * s;
}

// Calculate alignment component of the boid acceleration
math::R2 calculate_alignment(Boid const &boid_to_evolve,
                             std::vector<Boid> const &flock, double const a) {
  double const n = flock.size();
  assert(n > 1);
  // Calculate the sum of velocity vectors of all boids in the flock
  auto velocity_sum =
      std::accumulate(flock.begin(), flock.end(), math::R2{},
                      [](math::R2 velocity_sum, Boid const &current_boid) {
                        velocity_sum += current_boid.v();
                        return velocity_sum;
                      });

  // Calculate the mean velocity vector of the flock (excluding the
  // boid_to_evolve)
  math::R2 mean_velocity =
      (velocity_sum - boid_to_evolve.v()) * (1. / (n - 1.));

  // Return the alignment vector
  return a * (mean_velocity - boid_to_evolve.v());
}

// Calculate cohesion component of the boid acceleration
math::R2 calculate_cohesion(Boid const &boid_to_evolve,
                            std::vector<Boid> const &flock, double const c) {
  // Return the cohesion vector
  return c * (calculate_CDM(flock, boid_to_evolve) - boid_to_evolve.r());
}

// Evolve a single boid based on its neighbors and parameters
Boid evolve_boid(std::vector<Boid> const &flock, Boid &boid_to_evolve,
                 double const delta_t, running_parameters const &parameters) {

  // Calculate new position first assigning it the value of the current position
  math::R2 new_r = boid_to_evolve.r() + boid_to_evolve.v() * delta_t;
  // initialize new velocity as previous velocity
  math::R2 new_v = boid_to_evolve.v();
  int const n = flock.size();
  // If there is more than one boid, calculate acceleration components and add
  // them
  if (n > 1) {
    // Initialize acceleration components
    math::R2 separation_velocity = calculate_separation(
        boid_to_evolve, flock, parameters.s, parameters.d_s);
    math::R2 alignment_velocity =
        calculate_alignment(boid_to_evolve, flock, parameters.a);
    math::R2 cohesion_velocity =
        calculate_cohesion(boid_to_evolve, flock, parameters.c);
    // add them
    new_v += separation_velocity + alignment_velocity + cohesion_velocity;
  }

  // Apply toroidal teleportation to the new position if the boid is beyond
  // borders
  boid_to_evolve.r(teleport_toroidally(new_r, parameters));

  // Limit the speed of the boid
  boid_to_evolve.v(limit_speed(new_v, parameters));

  return boid_to_evolve;
}

// Evolve the entire flock of boids based on the given parameters and a time
// step
void evolve_flock(std::vector<Boid> &flock, double const delta_t,
                  running_parameters const &parameters) {
  // since it was decided to use the std algorithm transform, we prepare
  // new vector to hold the new state
  std::vector<Boid> evolved_flock;
  evolved_flock.reserve(flock.size());

  // Iterate through each boid in the flock and evolve it
  std::transform(flock.begin(), flock.end(), std::back_inserter(evolved_flock),
                 [&](Boid boid_to_evolve) {
                   // return the evolved boid to the new satate
                   return evolve_boid(
                       get_neighborhood(flock, boid_to_evolve, parameters.d),
                       boid_to_evolve, delta_t, parameters);
                 });
  // Update the flock to the evolved state, this operation is the reason the
  // flock parameter is not const
  flock = evolved_flock;
}
// Function to create a flock of boids with uniformly distributed random
// positions and velocities
std::vector<dynamics::Boid>
create_flock(dynamics::running_parameters const &parameters) {
  std::vector<dynamics::Boid> flock;
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> dist_width(parameters.left_bound,
                                                    parameters.right_bound);
  std::uniform_real_distribution<double> dist_height(parameters.bottom_bound,
                                                     parameters.upper_bound);
  std::uniform_real_distribution<double> dist_speed(
      -parameters.minimum_velocity * 2, parameters.maximum_velocity / 2);
  flock.reserve(parameters.boids_number);
  for (int i{}; i != parameters.boids_number; ++i) {
    flock.emplace_back(
        dist_width(rd), dist_height(rd), dist_speed(rd),
        dist_speed(rd)); // it's better than push back since it build the object
                         // directly inside the vector
  }
  return flock;
}
} // namespace dynamics
