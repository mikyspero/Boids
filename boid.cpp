#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <numeric>

#include "boid.hpp"

namespace flocking{
  Boid::Boid(Math::R2  r, Math::R2 v):r_{r},v_{v}{}
  Boid::Boid(double r_x, double r_y, double v_x, double v_y):Boid({r_x,r_y},{v_x,v_y}){}
  //straighforward getters and setters for the data members
  Math::R2 Boid::r() const {
    return r_;
  }
  void Boid::r(Math::R2 const& new_r){
    r_= new_r;
  }
  Math::R2 Boid::v() const {
    return v_;
  }   
  void Boid::v(Math::R2 const& new_v){
    v_= new_v;
  }
    //
  double calculate_distance(Boid const &b1, Boid const &b2) {
    return Math::calculate_distance(b1.r(),b2.r());
  }

  Math::R2 calculate_CDM(std::vector<Boid> const &flock, Boid const &fixed_boid) {
    //we first assert if there is more than 1 Boind in the flock vector, this calculation with less than 2 boids would be meaningless
    double const n = flock.size();
    //we make sure that in case an empty or a flock containing only the fixed boid(the first case should not be possible)
    //the position of the fixed boid is returned
    if(n<2){
      return fixed_boid.r();
    }
    assert(n > 1);
    Math::R2 mass_sum = std::accumulate(flock.begin(), flock.end(), Math::R2{},
      [](Math::R2 &cumulative_mass, Boid const &current_boid) {
        cumulative_mass += current_boid.r();
        return cumulative_mass;
      });
    return (mass_sum - fixed_boid.r()) * (1. / (n - 1.));
  }

  std::vector<Boid> get_neighborhood(std::vector<Boid> const &flock, Boid const &fixed_boid,double const d) {
    std::vector<Boid> neighborhood;
    std::for_each(flock.begin(), flock.end(), 
      [&](Boid const &current_boid) {
          if (calculate_distance(fixed_boid, current_boid) < d) { // && &fixed_boid!=&current_boid) { //
            neighborhood.push_back(current_boid);
          }
      });
    return neighborhood;
  }
}
namespace post {

  // Calculate mean distance of a flock of boids
  double calculate_mean_distance(std::vector<flocking::Boid> const &flock) {
    // Ensure there are at least two boids in the flock for meaningful calculations
    double const n = flock.size();
    assert(n > 1);

    double total_sum{};
    int i = 0;

    // Iterate through each boid as a potential fixed_boid
    std::for_each(flock.begin(), flock.end(),
      [&](flocking::Boid const &fixed_boid) {
        auto next = std::next((flock.begin() + i));

        // Calculate the sum of distances from the fixed_boid to all other boids
        double partial_sum = std::accumulate(next, flock.end(), 0.,
          [&](double sum_partial, flocking::Boid const &current_boid) {
              sum_partial += calculate_distance(fixed_boid, current_boid);
              return sum_partial;
          });

        // Accumulate the total sum of distances for later mean calculation
        total_sum += partial_sum;

        // Reset partial_sum for the next fixed_boid
        partial_sum = 0.;

        // Move to the next fixed_boid
        ++i;
      });

    // Number of distances equals the combinations of n boid, taken at groups of two
    double mean_distance = total_sum / (n * (n - 1.) / 2.);
    return mean_distance;
  }

  // Calculate standard deviation of distances of a flock of boids; mean distance is passed for efficiency
  double calculate_standard_deviation_distance(std::vector<flocking::Boid> const &flock, double mean_distance) {
    double const n = flock.size();
    assert(n > 1);

    int i = 0;
    double total_sum{};
    double sum_squared_distances{};

    // Iterate through each boid as a potential fixed_boid
    std::for_each(flock.begin(), flock.end(),
      [&](flocking::Boid const &fixed_boid) {
        auto next = std::next((flock.begin() + i));

        // Calculate the sum of squared distances from the fixed_boid to all other boids
        sum_squared_distances = std::accumulate(next, flock.end(), 0.,
          [&](double squared_distance, flocking::Boid const &current_boid) {
            double distance = calculate_distance(fixed_boid, current_boid);
            squared_distance += distance * distance;
            return squared_distance;
          });

        // Accumulate the total sum of squared distances for later standard deviation calculation
        total_sum += sum_squared_distances;

        // Reset sum_squared_distances for the next fixed_boid
        sum_squared_distances = 0.;

        // Move to the next fixed_boid
        ++i;
      });

        // Number of distances equals the combinations of n boid, taken at groups of two
    double const c_n_2 = (n * (n - 1.) / 2.);
    double sigma;

    // Calculate standard deviation using the formula
    if (n != 2) {
        sigma = sqrt((total_sum / (c_n_2 - 1.)) - n * mean_distance * mean_distance / (n - 1));
      } else {
        // Standard deviation is undefined with only two boids
        sigma = 0;
      }

      return sigma;
  }

    // Calculate mean of magnitudes of velocity vectors of a flock of boids
  double calculate_mean_velocity(std::vector<flocking::Boid> const &flock) {
    double const n = flock.size();
    assert(n > 1);

    // Calculate the sum of velocity magnitudes for all boids
    double velocity_sum = std::accumulate(flock.begin(), flock.end(), 0.,
      [](double velocity_sum, flocking::Boid const &current_boid) {
        velocity_sum += Math::calculate_norm(current_boid.v());
        return velocity_sum;
      });

    // Calculate the mean velocity magnitude
    double mean_velocity = velocity_sum * (1. / n);
    return mean_velocity;
  }

  // Calculate standard deviation of velocities of a flock of boids; mean velocity is passed for efficiency
  double calculate_standard_deviation_velocity(std::vector<flocking::Boid> const &flock, double mean_velocity) {
    double const n = flock.size();
    assert(n > 1);

    double total_sum{};
    double sum_squared_velocities{};

    // Calculate the sum of squared velocity magnitudes for all boids
    sum_squared_velocities = std::accumulate(flock.begin(), flock.end(), 0.,
      [&](double velocity_sum, flocking::Boid const &current_boid) {
        double vel_magnitude = Math::calculate_norm(current_boid.v());
        velocity_sum += vel_magnitude * vel_magnitude;
        return velocity_sum;
      });

    // Accumulate the total sum of squared velocity magnitudes for later standard deviation calculation
    total_sum += sum_squared_velocities;

    // Calculate standard deviation using the formula
    double sigma = sqrt((sum_squared_velocities / (n - 1.)) - n * mean_velocity * mean_velocity / (n - 1));
    return sigma;
  }
}