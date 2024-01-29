#ifndef RENDER_HPP
#define RENDER_HPP


#include <iostream>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "flock.hpp"

namespace view {

  struct data{
  double mean_distance;
  double sigma_mean_distance;
  double mean_velocity;
  double sigma_mean_velocity;
  };

  data build_data(std::vector<dynamics::Boid> const& flock);
  std::string print_data_to_string(data const& to_be_printed,dynamics::running_parameters const& parameters);
  void render_boids(std::vector<dynamics::Boid> const& flock, dynamics::running_parameters const& parameters,sf::RenderWindow& simulation_window);
  void run_simulation(std::vector<dynamics::Boid>& flock, dynamics::running_parameters const& parameters);
  dynamics::running_parameters create_parameters();

}

#endif