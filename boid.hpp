#ifndef BOID_HPP
#define BOID_HPP

#include<vector>

#include "R2.hpp"

namespace dynamics {
  // Boid class represents an individual boid in the simulation
  class Boid {
  private:
    Math::R2 r_; // Position vector
    Math::R2 v_; // Velocity vector

  public:
    // Constructors
    Boid(Math::R2 r, Math::R2 v);
    Boid(double r_x, double r_y, double v_x, double v_y);

    // Getters and setters
    Math::R2 r() const;
    void r(Math::R2 const &new_r);
    Math::R2 v() const;
    void v(Math::R2 const &new_v);
  };

  // Calculate distance between two boids
  double calculate_distance(Boid const &b1, Boid const &b2);
  // calculates Center of mass of a group of boids minus a fixed_boid
  Math::R2 calculate_CDM(std::vector<Boid> const &flock, Boid const &fixed_boid);
  //we get the vector containing the boids whose distance from the fixed boid is less than d
  std::vector<Boid> get_neighborhood(std::vector<Boid> const &flock, Boid const &fixed_boid,double const d);
}

namespace view{
  // Mean distance of a flock of boids
  double calculate_mean_distance(std::vector<dynamics::Boid> const &flock);

// Standard deviation of distances of a flock of boids,the mean distance is passed to simplify the functuion
  double calculate_standard_deviation_distance(std::vector<dynamics::Boid> const &flock,double  mean_distance);

// Mean of magnitudes of velocity vectors of a flock of boids
  double calculate_mean_velocity(std::vector<dynamics::Boid> const &flock);

//  Standard deviation of distances of a flock of boids, the mean velocity is passed to simplify the functuion
  double calculate_standard_deviation_velocity(std::vector<dynamics::Boid> const &flock,double  mean_velocity);
}
#endif