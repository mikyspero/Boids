cmake_minimum_required(VERSION 3.15)
project(boids CXX)  # Explicitly specify C++ language

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the compilation flags for both compiling and linking
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fsanitize=address,undefined")

#check for dependencies
find_package(SFML 2.5.1 COMPONENTS graphics REQUIRED)
find_package(SFML 2.5.1 COMPONENTS window REQUIRED)
find_package(SFML 2.5.1 COMPONENTS system REQUIRED)

# Add your source files for the boids executable
set(SOURCES
    main.cpp
    src/boid.cpp
    src/r2.cpp
    src/flock.cpp
    src/render.cpp
)

# Add source files for the boids.test executable
set(SOURCES_TEST
    test/test.cpp
    src/boid.cpp
    src/r2.cpp
    src/flock.cpp
)


# Add an executable for the main program
add_executable(boids ${SOURCES})

# Link libraries and set additional flags for the main program
target_link_libraries(boids
    -fsanitize=address,undefined
    -lsfml-window
    -lsfml-system
    -lsfml-graphics
    -lm     #necessary for gcc conmpatibility
    -lstdc++#necessary for gcc conmpatibility
)

# Add an executable for the tests 
add_executable(boids.test ${SOURCES_TEST})

# Link libraries and set additional flags for the test program
target_link_libraries(boids.test
    -fsanitize=address,undefined
    -lm     #necessary for gcc conmpatibility
    -lstdc++#necessary for gcc conmpatibility
)


# Set the output directory for both executables
set_target_properties(boids boids.test
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)
