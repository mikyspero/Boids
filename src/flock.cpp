#include "../include/flock.hpp"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <random>

namespace dynamics {
  // Function to create a flock of boids with uniformly distributed random positions and velocities
std::vector<dynamics::Boid> create_flock(dynamics::running_parameters const& parameters){
  std::vector<dynamics::Boid> flock;
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> dist_width(parameters.left_bound,parameters.right_bound);
  std::uniform_real_distribution<double> dist_height(parameters.bottom_bound,parameters.upper_bound);
  std::uniform_real_distribution<double> dist_speed(-parameters.minimum_velocity*2,parameters.maximum_velocity/2);
  flock.reserve(parameters.boids_number);
  for (int i{}; i != parameters.boids_number; ++i) {
    flock.emplace_back(dist_width(rd), dist_height(rd),dist_speed(rd), dist_speed(rd));//it's better than push back since it build the object directly inside the vector
  }
  return flock;
}

  // Teleport a point toroidally within the simulation space
  Math::R2 teleport_toroidally(Math::R2 &r, running_parameters const &parameters) {
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
  Math::R2 limit_speed(Math::R2& to_be_checked, running_parameters const& parameters) {
    // If velocity magnitude exceeds maximum, halve it
    if (Math::calculate_norm(to_be_checked) > parameters.maximum_velocity) {
      to_be_checked /= 2.;
      return to_be_checked;
    }
    // If velocity magnitude is below minimum, double it
    if (Math::calculate_norm(to_be_checked) < parameters.minimum_velocity) {
      to_be_checked *= 2.;
      return to_be_checked;
    }
    return to_be_checked;
  }

  // Calculate separation component of the boid acceleration
  Math::R2 calculate_separation(Boid const &fixed_boid, std::vector<Boid> const &flock, double const s, double const d_s) {
    if (flock.empty()) {
      return {0, 0};
    }
    Math::R2 velocity_sum;
    // Iterate through all boids in the flock
    std::for_each(flock.begin(), flock.end(),
      [&](Boid const &current_boid) {
        // If the distance is less than d_s, add the separation vector
        if (calculate_distance(fixed_boid, current_boid) < d_s) {
          velocity_sum += (current_boid.r() - fixed_boid.r());
        }
      });
    // Return the negation of the sum multiplied by the separation parameter
    return -velocity_sum * s;
  }

  // Calculate alignment component of the boid acceleration
  Math::R2 calculate_alignment(Boid const &fixed_boid, std::vector<Boid> const &flock, double const a) {
    double const n = flock.size();
    if (flock.empty()) {
      return {0, 0};
    }

    assert(n > 1);
    // Calculate the sum of velocity vectors of all boids in the flock
    auto velocity_sum = std::accumulate(flock.begin(), flock.end(), Math::R2{},
      [](Math::R2 velocity_sum, Boid const &current_boid) {
        velocity_sum += current_boid.v();
        return velocity_sum;
      });

    // Calculate the mean velocity vector of the flock (excluding the fixed_boid)
    Math::R2 mean_velocity = (velocity_sum - fixed_boid.v()) * (1. / (n - 1.));

    // Return the alignment vector
    return (mean_velocity - fixed_boid.v()) * a;
  }

  // Calculate cohesion component of the boid acceleration
  Math::R2 calculate_cohesion(Boid const &fixed_boid, std::vector<Boid> const &flock, double const c) {
    if (flock.empty()) {
      return {0, 0};
    }
    // Return the cohesion vector
    return (calculate_CDM(flock, fixed_boid) - fixed_boid.r()) * c;
  }

  // Evolve a single boid based on its neighbors and parameters
  Boid evolve_boid(std::vector<Boid> const &flock, Boid &fixed_boid, double const delta_t, running_parameters const &parameters) {
    // Initialize acceleration components
    Math::R2 separation_velocity;
    Math::R2 alignment_velocity;
    Math::R2 cohesion_velocity;

    int const n = flock.size();

    // Calculate new position first assigning it the value of the current position
    Math::R2 new_r = fixed_boid.r() + fixed_boid.v() * delta_t;

    // If there is more than one boid, calculate acceleration components
    if (n > 1) {
      separation_velocity = calculate_separation(fixed_boid, flock, parameters.s, parameters.d_s);
      alignment_velocity = calculate_alignment(fixed_boid, flock, parameters.a);
      cohesion_velocity = calculate_cohesion(fixed_boid, flock, parameters.c);
    }

    // Calculate new velocity based on acceleration components
    Math::R2 new_v = fixed_boid.v() + separation_velocity + alignment_velocity + cohesion_velocity;

    // Apply toroidal teleportation to the new position
    fixed_boid.r(teleport_toroidally(new_r, parameters));

    // Limit the speed of the boid
    fixed_boid.v(limit_speed(new_v, parameters));

    return fixed_boid;
  }

  // Evolve the entire flock of boids based on the given parameters and time step
  void evolve_flock(std::vector<Boid> &flock, double const delta_t, running_parameters const &parameters) {
    // A vector evolved_flock is created to maintain the initial state of the
    // flock from which every Boid is evolved
    std::vector<Boid> evolved_flock;
    evolved_flock.reserve(flock.size());

    // Iterate through each boid in the flock and evolve it
    std::transform(flock.begin(), flock.end(), std::back_inserter(evolved_flock),
      [&](Boid fixed_boid) {
        // Get the neighborhood of the current boid
        return evolve_boid(get_neighborhood(flock, fixed_boid, parameters.d), fixed_boid, delta_t, parameters);
      });

    // Assert that the size of the original and evolved flocks are the same
    assert(flock.size() == evolved_flock.size()); // should never fail

    // Update the flock to the evolved state
    flock = evolved_flock;
  }

}  // namespace dynamics
