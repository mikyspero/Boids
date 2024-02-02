#ifndef RENDER_HPP
#define RENDER_HPP

#include "flock.hpp" 
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>

namespace view {

// Data structure to hold statistical information about the flock
struct data {
  double mean_distance;
  double sigma_mean_distance;
  double mean_velocity;
  double sigma_mean_velocity;
};

// Function to build statistical data from the flock
data build_data(std::vector<dynamics::Boid> const &flock);

// Function to convert statistical data to a string for printing
std::string
print_data_to_string(data const &to_be_printed,
                     dynamics::running_parameters const &parameters);

// Function to render the boids in the simulation window
void render_boids(std::vector<dynamics::Boid> const &flock,
                  dynamics::running_parameters const &parameters,
                  sf::RenderWindow &simulation_window);

// Function to run the simulation with the given flock and parameters
void run_simulation(std::vector<dynamics::Boid> &flock,
                    dynamics::running_parameters const &parameters);

// Function to create default running parameters for the simulation
dynamics::running_parameters create_parameters();

} // namespace view

#endif