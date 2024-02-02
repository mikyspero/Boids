# Boids
A Small 2d Boids simulation for the undergraduate course of "Programming for Physics" at   Bologna University,
written in C++ using the SFML library whose installation is required for a working program.

To build an executable use Cmake and in the Boids directory run first the command

$ cmake -S . -B build

to build the building directory build and then

$ make -C build

to create the two executables boids and boids.test executables which are going to be activated with the following commands

$ ./boids

and

$ ./boids.test

the program has been tested in Ubuntu 22.04 using the following compilers:
gcc 11.3.0, gcc 11.4.0 and g++11.4.0


Happy Boid Watching!


