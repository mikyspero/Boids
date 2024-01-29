#include "include/render.hpp"

#include <iostream>

int main(){
  
  dynamics::running_parameters parameters=view::create_parameters();
  std::vector<dynamics::Boid> flock=dynamics::create_flock(parameters);
  std::cout << "\nThe simulation will start now\n";
  std::cout << "\nBoids Number:"<<parameters.boids_number<<" s:"<<parameters.c<<" a:"<<parameters.a<<" c:"<<parameters.c<<" \n";
  try{
    view::run_simulation(flock, parameters);
  }
  catch(const std::exception& error){
    std::cerr << error.what() << '\n';
    std::cout<<"Simulation Aborted"<<"\n";
  }
  
  
}