#include "../include/render.hpp"

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include <iostream>

namespace view {
    // Function to create parameters for the simulation (custom or default) needs heavy refactoring
  dynamics::running_parameters create_parameters(){
  
    std::string input;
    std::cout << " Good day, this is a boids program would you like to use the default settings or custom ones?\n";
    std::cout << " to use the default ones press enter a lowercase d if not enter anything else\n";
    std::cin>>input;
    if(input=="d"){
      return dynamics::running_parameters{};
    }
  
    int boid_number;
    std::cout << " Good day, to create a flock of boids please first insert their number, it must be an integer greater than 2, it's default value is 500\n";
    std::cin>>boid_number;
    while(std::cin.fail() || boid_number <2 ) {
      std::cin.clear();
      std::cin.ignore();
      std::cout << "Please insert a valid boid number, it must be an integer greater than 2\n";
      std::cin>>boid_number;
    }


    double separation;
    std::cout << "Now please insert the value of the separation parameter, it's a positive real number, its default value is 0.5\n";
    std::cin>>separation;
    while(std::cin.fail() || separation <= 0 ) {
      std::cin.clear();
      std::cin.ignore();
      std::cout << "Please insert a valid value, it must be aa positive real number, its default value is 0.5\n";;
      std::cin>>separation;
    }


    double alignement;
    std::cout << "Now please insert the value of the alignement parameter, it's a positive real number, its default value is 0.6\n";
    std::cin>>alignement;
    while(std::cin.fail() || alignement <= 0 ) {
      std::cin.clear();
      std::cin.ignore();
      std::cout << "Please insert a valid value, it must be aa positive real number, its default value is 0.6\n";;
      std::cin>>alignement;
    }


    double cohesion;
    std::cout << "Now please insert the value of the cohesion parameter, it's a positive real number, its default value is 0.15\n";
    std::cin>>cohesion;
    while(std::cin.fail() || cohesion <= 0 ) {
      std::cin.clear();
      std::cin.ignore();
      std::cout << "Please insert a valid value, it must be aa positive real number, its default value is 0.15\n";;
      std::cin>>cohesion;
    }

    return dynamics::running_parameters{boid_number,separation,alignement,cohesion};
  }
  
  void render_boids(std::vector<dynamics::Boid> const& flock, dynamics::running_parameters const& parameters,sf::RenderWindow& simulation_window){
  //calculation of the scales necessary to convert from the proprietary 2d vectors to those in ASML
  auto const x_scale = (.75 * sf::VideoMode::getDesktopMode().width) / (parameters.right_bound - parameters.left_bound);
  auto const y_scale = (.75 * sf::VideoMode::getDesktopMode().height) / (parameters.upper_bound - parameters.bottom_bound);

  sf::CircleShape boid_shape{5.0f, 12};
  boid_shape.setFillColor(sf::Color(sf::Color::White));
  // The simulation_window with the objects of the previous frame is cleared and filled with our
  simulation_window.clear(sf::Color(sf::Color::Black));
  // Every dynamics::Boid inside flock gets assigned a boids sf::CircleShape that are
  // drawn at the same position of the dynamics::Boid
  std::for_each(flock.begin(), flock.end(),
      [&](auto &b_i) {
        boid_shape.setPosition(b_i.r().x * x_scale, b_i.r().y* y_scale);
        simulation_window.draw(boid_shape);
      });
  // The simulation_window with every object is displayed
  simulation_window.display();
}

// Function to build data structure from the flock
data build_data(std::vector<dynamics::Boid> const& flock){
  double mean_distance{view::calculate_mean_distance(flock)};
  double sigma_mean_distance{view::calculate_standard_deviation_distance(flock,mean_distance)};
  double mean_velocity{view::calculate_mean_velocity(flock)};
  double sigma_mean_velocity{view::calculate_standard_deviation_velocity(flock,mean_velocity)};
  data built{mean_distance,sigma_mean_distance,mean_velocity,sigma_mean_velocity};
  return built;
}

// Function to convert data to a formatted string
  std::string print_data_to_string(data const& to_be_printed,dynamics::running_parameters const& parameters) {
    std::string to_be_returned{""};
    to_be_returned+="Mean Distance:  ";
    to_be_returned+=std::to_string(to_be_printed.mean_distance);
    std::string plus_minus{"  +/-  "};
    to_be_returned+=plus_minus;
    to_be_returned+=std::to_string(to_be_printed.sigma_mean_distance);
    to_be_returned+="\nMean Velocity:   ";
    to_be_returned+=std::to_string(to_be_printed.mean_velocity);
    to_be_returned+=plus_minus;
    to_be_returned+=std::to_string(to_be_printed.sigma_mean_velocity);
    to_be_returned+="\nBoids Number:   ";
    to_be_returned+=std::to_string(parameters.boids_number);
    to_be_returned+="\nSeparation Parameter:   ";
    to_be_returned+=std::to_string(parameters.s);
    to_be_returned+="\nAlignement Parameter:   ";
    to_be_returned+=std::to_string(parameters.a);
    to_be_returned+="\nCohesion Parameter:   ";
    to_be_returned+=std::to_string(parameters.c);
    return to_be_returned;
  }




  void render_data(std::vector<dynamics::Boid> const& flock,sf::RenderWindow& data_window,sf::Font const& font,dynamics::running_parameters const& parameters){
    sf::Text text;
    text.setFont(font);
    text.setString(print_data_to_string(build_data(flock),parameters));
    text.setCharacterSize(24);
    text.setFillColor(sf::Color::White);
    text.setPosition(0,0);
    data_window.clear(sf::Color::Black);
    data_window.draw(text);
    data_window.display();
  }



  void run_simulation(std::vector<dynamics::Boid> & flock, dynamics::running_parameters const& parameters) {
    //let's build a simulation_window 3/4 of our desktop
    unsigned const display_width = .75 * sf::VideoMode::getDesktopMode().width;
    unsigned const display_height = .75 * sf::VideoMode::getDesktopMode().height;
    sf::RenderWindow simulation_window(sf::VideoMode(display_width, display_height),"Boids Simulation");
    sf::RenderWindow data_window(sf::VideoMode(.33 * display_width, 0.22*display_height),"Data Display");
    simulation_window.setPosition({0, 50});
    data_window.setPosition({static_cast<int>(display_width),50});
    // This method limits the number of frames displayed to the refresh rate of 60Hz of the monitor
    simulation_window.setVerticalSyncEnabled(true);
    // Starting the frame_clock needed to evolve the simulation and update data
    sf::Clock frame_clock;
    sf::Clock data_clock;
    sf::Font font;
    if (!font.loadFromFile("utils/arial.ttf")){
      throw std::runtime_error("ERROR: Failed to load  files");
    }
    // Game loop, while the loopp runs (or when the simulation_window is not closed) each operation is going to bbe done
    render_boids(flock,parameters,simulation_window);
    while (simulation_window.isOpen() || data_window.isOpen()) {
      sf::Event event;
        while (simulation_window.pollEvent(event)) {
          if (event.type == sf::Event::Closed) {
            simulation_window.close();
            data_window.close();
          }
        }
        while (data_window.pollEvent(event)) {
          if (event.type == sf::Event::Closed) {
          data_window.close();       
          simulation_window.close();
          }
        } 
    
      sf::Time frame_time = frame_clock.restart();
      double frame_time_double = static_cast<double>(frame_time.asSeconds());
      dynamics::evolve_flock(flock, frame_time_double, parameters);
      render_boids(flock,parameters,simulation_window);
    
      sf::Time data_time = data_clock.getElapsedTime();
      int integer_data_time = static_cast<int>(data_time.asSeconds());
      if (integer_data_time % 2 == 0) {
        render_data(flock,data_window,font,parameters);
      }
    }
  }
}