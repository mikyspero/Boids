#include "../include/boid.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace dynamics {
Boid::Boid(math::R2 r, math::R2 v) : r_{r}, v_{v} {}
Boid::Boid(double r_x, double r_y, double v_x, double v_y)
    : Boid({r_x, r_y}, {v_x, v_y}) {}
// straighforward getters and setters for the data members
math::R2 Boid::r() const { return r_; }
void Boid::r(math::R2 const &new_r) { r_ = new_r; }
math::R2 Boid::v() const { return v_; }
void Boid::v(math::R2 const &new_v) { v_ = new_v; }

double calculate_distance(Boid const &b1, Boid const &b2) {
  return math::calculate_distance(b1.r(), b2.r());
}

math::R2 calculate_CDM(std::vector<Boid> const &flock, Boid const &fixed_boid) {
  // we first assert if there is more than 1 Boind in the flock vector, this
  // calculation with less than 2 boids would be meaningless
  double const n = flock.size();
  // we make sure that in case an empty or a flock containing only the fixed
  // boid(the first case should not be possible) the position of the fixed boid
  // is returned
  assert(n > 1); // should never fail
  math::R2 mass_sum =
      std::accumulate(flock.begin(), flock.end(), math::R2{},
                      [](math::R2 &cumulative_mass, Boid const &current_boid) {
                        cumulative_mass += current_boid.r();
                        return cumulative_mass;
                      });
  return (mass_sum - fixed_boid.r()) * (1. / (n - 1.));
}

std::vector<Boid> get_neighborhood(std::vector<Boid> const &flock,
                                   Boid const &fixed_boid, double const d) {
  std::vector<Boid> neighborhood;
  std::for_each(flock.begin(), flock.end(), [&](Boid const &current_boid) {
    if (calculate_distance(fixed_boid, current_boid) < d) {
      neighborhood.push_back(current_boid);
    }
  });
  return neighborhood;
}
} // namespace dynamics
namespace view {

// Calculate mean distance of a flock of boids
double calculate_mean_distance(std::vector<dynamics::Boid> const &flock) {
  // Ensure there are at least three boids in the flock for meaningful
  // calculations of a srandard deviation
  double const n = flock.size();
  assert(n > 2); // should never fail
  double total_sum{0};
  int next_boid_index = 0;

  // Iterate through each boid as a potential fixed_boid
  std::for_each(
      flock.begin(), flock.end(), [&](dynamics::Boid const &fixed_boid) {
        auto next = std::next((flock.begin() + next_boid_index));

        // Calculate the sum of distances from the fixed_boid to all other boids
        double partial_sum = std::accumulate(
            next, flock.end(), 0.,
            [&](double sum_partial, dynamics::Boid const &current_boid) {
              sum_partial += calculate_distance(fixed_boid, current_boid);
              return sum_partial;
            });

        // Move to the next fixed_boid
        ++next_boid_index;

        // Accumulate the total sum of distances for later mean calculation
        total_sum += partial_sum;

        // Reset partial_sum for the next fixed_boid
        partial_sum = 0.;
      });

  // Number of distances equals the combinations of n boid, taken at groups of
  // two
  double mean_distance = total_sum / (n * (n - 1.) / 2.);
  return mean_distance;
}

// Calculate standard deviation of distances of a flock of boids; mean distance
// is passed for efficiency
double
calculate_standard_deviation_distance(std::vector<dynamics::Boid> const &flock,
                                      double mean_distance) {
  double const n = flock.size();
  assert(n > 1);

  int next_boid_index = 0;
  double total_sum{0.};
  double sum_squared_distances{0.};

  // Iterate through each boid as a potential fixed_boid
  std::for_each(
      flock.begin(), flock.end(), [&](dynamics::Boid const &fixed_boid) {
        auto next = std::next((flock.begin() + next_boid_index));

        // Calculate the sum of squared distances from the fixed_boid to all
        // other boids
        sum_squared_distances = std::accumulate(
            next, flock.end(), 0.,
            [&](double squared_distance, dynamics::Boid const &current_boid) {
              double distance = calculate_distance(fixed_boid, current_boid);
              squared_distance += distance * distance;
              return squared_distance;
            });
        // Move to the next fixed_boid
        ++next_boid_index;

        // Accumulate the total sum of squared distances for later standard
        // deviation calculation
        total_sum += sum_squared_distances;
        // Reset sum_squared_distances for the next fixed_boid
        sum_squared_distances = 0.;
      });

  // Calculate standard deviation using the formula
  double sigma = sqrt((total_sum / ((n * (n - 1.) / 2.) - 1.)) -
                      n * mean_distance * mean_distance / (n - 1));
  return sigma;
}

// Calculate mean of magnitudes of velocity vectors of a flock of boids
double calculate_mean_velocity(std::vector<dynamics::Boid> const &flock) {
  double const n = flock.size();
  assert(n > 1); // should never fail

  // Calculate the sum of velocity magnitudes for all boids
  double velocity_sum = std::accumulate(
      flock.begin(), flock.end(), 0.,
      [](double velocity_sum, dynamics::Boid const &current_boid) {
        velocity_sum += math::calculate_norm(current_boid.v());
        return velocity_sum;
      });

  // Calculate the mean velocity magnitude
  double mean_velocity = velocity_sum * (1. / n);
  return mean_velocity;
}

// Calculate standard deviation of velocities of a flock of boids; mean velocity
// is passed for efficiency
double
calculate_standard_deviation_velocity(std::vector<dynamics::Boid> const &flock,
                                      double mean_velocity) {
  double const n = flock.size();
  assert(n > 2);

  double sum_squared_velocities{};

  // Calculate the sum of squared velocity magnitudes for all boids
  sum_squared_velocities = std::accumulate(
      flock.begin(), flock.end(), 0.,
      [&](double velocity_sum, dynamics::Boid const &current_boid) {
        double vel_magnitude = math::calculate_norm(current_boid.v());
        velocity_sum += vel_magnitude * vel_magnitude;
        return velocity_sum;
      });

  // Calculate standard deviation using the formula
  double sigma = sqrt((sum_squared_velocities / (n - 1.)) -
                      n * mean_velocity * mean_velocity / (n - 1));
  return sigma;
}
} // namespace view