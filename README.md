# Boids
This is a Boids 2D numerical simulation created for the undergraduate course "Programming for Physics" at Bologna University. It is written in C++ and uses the SFML (Simple and Fast Multimedia Library) for rendering and animation, requiring SFML's installation for the program to function correctly. This simulation models flocking behavior, mimicking how birds, fish, or other animals move in coordinated groups.

### Build Instructions 
To build an executable use Cmake and in the Boids directory run first the command

$ cmake -S . -B build

to build the building directory build and then

$ make -C build

to create the two executables boids and boids.test executables which are going to be activated with the following commands

$ executables/./boids

and

$ executables/./boids.test

the program has been tested in Ubuntu 22.04 using gcc and g++ .


Happy Boid Watching!


