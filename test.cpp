
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "include/doctest.h"
#include "include/flock.hpp"

TEST_CASE("Class R2 tests") {
    SUBCASE("Vector addition") {
        Math::R2 v1(1.0, 2.0);
        Math::R2 v2(-3.0, -4.0);
        Math::R2 v3(0,0);
        Math::R2 v4(1.5,-1.);

        Math::R2 result = v1 + v2;
        CHECK(result.x == -2.);
        CHECK(result.y == -2.);

        result = v1 + v3;
        CHECK(result.x == 1.);
        CHECK(result.y == 2.);

        result = v1 + v4;
        CHECK(result.x == 2.5);
        CHECK(result.y == 1.);

    }

    SUBCASE("Vector negation") {
        Math::R2 v1(2.0, 3.0);
        Math::R2 result = -v1;
        CHECK(result.x == -2.0);
        CHECK(result.y == -3.0);

        Math::R2 v2(-3.0, -4.0);
        result=-v2;
        CHECK(result.x == 3.0);
        CHECK(result.y == 4.);
        
        Math::R2 v3(0,0);
        result=-v3;
        CHECK(result.x == 0.);
        CHECK(result.y == 0.);
    }

    SUBCASE("Vector subtraction") {
        Math::R2 v1(1.0, 2.0);
        Math::R2 v2(-3.0, -4.0);
        Math::R2 v3(0,0);
        Math::R2 v4(1.5,-1.);

        Math::R2 result = v1 - v2;
        CHECK(result.x == 4.);
        CHECK(result.y == 6.);

        result = v1 - v3;
        CHECK(result.x == 1.);
        CHECK(result.y == 2.);

        result = v1 - v4;
        CHECK(result.x == -0.5);
        CHECK(result.y == 3.);
    }

    SUBCASE("Vector scalar multiplication") {
        Math::R2 v(2.0, 3.0);
        double k = 1.5;
        Math::R2 result = v * k;
        Math::R2 result2 = k*v;

        CHECK(result.x == doctest::Approx(3.0));
        CHECK(result.y == doctest::Approx(4.5));
        CHECK(result == result2);

        
        k = -1.5;
        result = k*v;
        result2 = k*v;
        CHECK(result.x == doctest::Approx(-3.0));
        CHECK(result.y == doctest::Approx(-4.5));
        CHECK(result == result2);


        k = 0.;
        result = v * k;
        CHECK(result.x == doctest::Approx(0.));
        CHECK(result.y == doctest::Approx(0.));
    }

    SUBCASE("Vector equality and inequality") {
        Math::R2 v1(2.0, 3.0);
        Math::R2 v2(2.0, 3.0);
        Math::R2 v3(4.0, 5.0);

        CHECK(v1 == v2);
        CHECK(v1 != v3);
    }

    SUBCASE("Inner product") {
        Math::R2 v1(2.0, 3.0);
        Math::R2 v2(4.0, 5.0);
        double result = v1 * v2;
        CHECK(result == doctest::Approx(23.0));

        Math::R2 v3(1., 0.);
        Math::R2 v4(0., 1.);
        result = v3 * v4;
        CHECK(result == doctest::Approx(0.));

        Math::R2 v5(2.0, -3.0);
        Math::R2 v6(-4.0, 5.0);
        result = v5 * v6;
        CHECK(result == doctest::Approx(-23.0));
    }

    SUBCASE("Norm calculation") {
        Math::R2 v1(3.0, 4.0);
        Math::R2 v2(-3.0, -4.0);
        Math::R2 v3(0., 0.);

        double result = Math::calculate_norm(v1);
        CHECK(result == doctest::Approx(5.0));

        result = Math::calculate_norm(v2);
        CHECK(result == doctest::Approx(5.0));

        result = Math::calculate_norm(v3);
        CHECK(result == doctest::Approx(0.));
    }

    SUBCASE("Distance calculation") {
        Math::R2 v1(1.0, 2.0);
        Math::R2 v2(4.0, 6.0);
        double result = Math::calculate_distance(v1, v2);
        CHECK(result == doctest::Approx(5.0));

        result = Math::calculate_distance(v1, v1);
        CHECK(result == doctest::Approx(0.));

    }

       SUBCASE("Compound assignment operators") {
        Math::R2 v1(2.0, 3.0);
        Math::R2 v2(1.0, 2.0);

        v1 += v2;
        CHECK(v1.x == 3.0);
        CHECK(v1.y == 5.0);

        v1 -= v2;
        CHECK(v1.x == 2.0);
        CHECK(v1.y == 3.0);

        v1 *= 1.5;
        CHECK(v1.x == doctest::Approx(3.0));
        CHECK(v1.y == doctest::Approx(4.5));
    }
}


TEST_CASE("Boid Class") {
    Math::R2 initial_position(1.0, 3.0);
    Math::R2 initial_velocity(2.0, 4.0);

    dynamics::Boid boid(initial_position, initial_velocity);

    SUBCASE("Getters") {
        CHECK(boid.r() == initial_position);
        CHECK(boid.v() == initial_velocity);
    }

    SUBCASE("Setters") {
        Math::R2 new_position(5.4, 5.6);
        Math::R2 new_velocity(0.7, 8.0);

        boid.r(new_position);
        boid.v(new_velocity);

        CHECK(boid.r() == new_position);
        CHECK(boid.v() == new_velocity);
    }
}


TEST_CASE("dynamics functions"){

  SUBCASE("Distance between boids") {
    dynamics::Boid b1{{1., 3.}, {3., 9.}};
    dynamics::Boid b2{{2., 4.}, {-2., 0.}};
    dynamics::Boid b3{{0., 0.}, {0., 0.}};
    dynamics::Boid b4{{-5., -3.}, {6., 2.}};
    dynamics::Boid b5{{1., 3.}, {3., 9.}};
    CHECK(calculate_distance(b1, b2) == doctest::Approx(sqrt(2.)));
    CHECK(calculate_distance(b1, b5) == doctest::Approx(0.));
    CHECK(calculate_distance(b1, b3) == doctest::Approx(sqrt(10.)));
    CHECK(calculate_distance(b1, b4) == doctest::Approx(6 * sqrt(2.)));
  }

  SUBCASE("Two points mass displacement center") {
    dynamics::Boid b1{{1., 3.}, {3., 9.}};
    dynamics::Boid b2{{2., 4.}, {-2., -1.}};
    std::vector<dynamics::Boid> flock{b1,b2};

    CHECK(calculate_CDM(flock, b1).x == doctest::Approx(2.));
    CHECK(calculate_CDM(flock, b1).y == doctest::Approx(4.));
    CHECK(calculate_CDM(flock, b2).x == doctest::Approx(1.));
    CHECK(calculate_CDM(flock, b2).y == doctest::Approx(3.));
  }

  SUBCASE("Six points mass displacement center") {
    dynamics::Boid b1{{1., 3.}, {3., 9.}};
    dynamics::Boid b2{{2., 4.}, {-2., -1.}};
    dynamics::Boid b3{{0., 0.}, {0., 0.}};
    dynamics::Boid b4{{-5., -3.}, {6., 2.}};
    dynamics::Boid b5{{4., 5.}, {-1., 5.}};
    dynamics::Boid b6{{-9., 2.}, {7., -3.}};
    std::vector<dynamics::Boid> flock{b1,b2,b3,b4,b5,b6};


    CHECK(calculate_CDM(flock, b1).x == doctest::Approx(-1.6));
    CHECK(calculate_CDM(flock, b1).y == doctest::Approx(1.6));
    CHECK(calculate_CDM(flock, b2).x == doctest::Approx(-1.8));
    CHECK(calculate_CDM(flock, b2).y == doctest::Approx(1.4));
    CHECK(calculate_CDM(flock, b3).x == doctest::Approx(-1.4));
    CHECK(calculate_CDM(flock, b3).y == doctest::Approx(2.2));
    CHECK(calculate_CDM(flock, b4).x == doctest::Approx(-0.4));
    CHECK(calculate_CDM(flock, b4).y == doctest::Approx(2.8));
    CHECK(calculate_CDM(flock, b5).x == doctest::Approx(-2.2));
    CHECK(calculate_CDM(flock, b5).y == doctest::Approx(1.2));
    CHECK(calculate_CDM(flock, b6).x == doctest::Approx(0.4));
    CHECK(calculate_CDM(flock, b6).y == doctest::Approx(1.8));
  }

  SUBCASE("Testing calculate_separation_velocity") {
    std::vector<dynamics::Boid> flock;
    dynamics::Boid b1{{1., 3.}, {3., 9.}};
    dynamics::Boid b2{{2., 4.}, {-2., -1.}};
    dynamics::Boid b3{{0., 0.}, {0., 0.}};
    dynamics::Boid b4{{-1., -0.5}, {6., 2.}};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    CHECK(calculate_separation(b1,flock,1., 7.).x == doctest::Approx(2.));
    CHECK(calculate_separation(b1,flock,1., 7.).y == doctest::Approx(5.5));
    CHECK(calculate_separation(b2,flock,1., 7.).x == doctest::Approx(6.));
    CHECK(calculate_separation(b2,flock,1., 7.).y == doctest::Approx(9.5));
}

SUBCASE("Testing calculate_alignement_velocity") {
  std::vector<dynamics::Boid> flock;
  dynamics::Boid b1{{2., 2.}, {1., 1.}};
  dynamics::Boid b2{{4., 3.}, {4., 1.}};
  dynamics::Boid b3{{7., 6.}, {2., 3.}};
  dynamics::Boid b4{{1., 1.}, {-1., -2.}};

 
  flock.push_back(b1);
  flock.push_back(b2);
  flock.push_back(b3);
  flock.push_back(b4);

  CHECK(calculate_alignment( b1,flock,1.).x == doctest::Approx(0.67).epsilon(0.01));
  CHECK(calculate_alignment( b1,flock,1.).y == doctest::Approx(-0.33).epsilon(0.01));
  CHECK(calculate_alignment( b2,flock,1.).x == doctest::Approx(-3.33).epsilon(0.01));
  CHECK(calculate_alignment( b2,flock,1.).y == doctest::Approx(-0.33).epsilon(0.01));
  
}

SUBCASE("Testing calculate_cohesion_velocity") {
  std::vector<dynamics::Boid> flock;
  dynamics::Boid b1{{2., 2.}, {1., 1.}};
  dynamics::Boid b2{{4., 3.}, {4., 1.}};
  dynamics::Boid b3{{7., 6.}, {2., 3.}};
  dynamics::Boid b4{{1., 1.}, {-1., -2.}};

  flock.push_back(b1);
  flock.push_back(b2);
  flock.push_back(b3);
  flock.push_back(b4);

  CHECK(calculate_cohesion(b1, flock, 1.).x == doctest::Approx(2.));
  CHECK(calculate_cohesion(b1, flock, 1.).y == doctest::Approx(1.33).epsilon(0.01));
  CHECK(calculate_cohesion(b2, flock, 1.).x == doctest::Approx(-0.67).epsilon(0.01));
  CHECK(calculate_cohesion(b2, flock, 1.).y == doctest::Approx(0.));
}

SUBCASE("Testing teleport_toroidally") {
  dynamics::running_parameters const p{0, 0., 0., 0., 0., 0., 0., 10., 10., 0., 0., 0.};
  
  Math::R2 v1 {-1., 3.};
  Math::R2 v2 {1., -3.};
  Math::R2 v3 {-3., -4.};
  Math::R2 v4 {17., 16.};
  
  v1 = teleport_toroidally(v1, p);
  v2 = teleport_toroidally(v2, p);
  v3 = teleport_toroidally(v3, p);
  v4 = teleport_toroidally(v4, p);


  CHECK(v1.x == doctest::Approx(9.));
  CHECK(v1.y == doctest::Approx(3.));
  CHECK(v2.x == doctest::Approx(1.));
  CHECK(v2.y == doctest::Approx(7.));
  CHECK(v3.x == doctest::Approx(7.));
  CHECK(v3.y == doctest::Approx(6.));
  CHECK(v4.x == doctest::Approx(7.));
  CHECK(v4.y == doctest::Approx(6.));
}


SUBCASE("Limit Speed Function") {
    // Define a test running_parameters instance
    dynamics::running_parameters test_parameters{0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 5., 1.};
    test_parameters.maximum_velocity = 5.0;  // Set your desired maximum velocity
    test_parameters.minimum_velocity = 1.0;  // Set your desired minimum velocity

    SUBCASE("Limit speed - Above Maximum Velocity") {
        Math::R2 velocity_above_max(3.0, 4.0);  // Velocity magnitude exceeds maximum
        Math::R2 result = dynamics::limit_speed(velocity_above_max, test_parameters);
        CHECK(Math::calculate_norm(result) == doctest::Approx(Math::calculate_norm(velocity_above_max)));
        CHECK(Math::calculate_norm(result) <= test_parameters.maximum_velocity);
    }

    SUBCASE("Limit speed - Below Minimum Velocity") {
        Math::R2 velocity_below_min(0.5, 0.6);  // Velocity magnitude is below minimum
        Math::R2 result = dynamics::limit_speed(velocity_below_min, test_parameters);
        CHECK(Math::calculate_norm(result) == doctest::Approx(Math::calculate_norm(velocity_below_min)));
        CHECK(Math::calculate_norm(result) >= test_parameters.minimum_velocity);
    }

    SUBCASE("Limit speed - Within Velocity Limits") {
        Math::R2 velocity_within_limits(2.0, 2.0);  // Velocity magnitude within limits
        Math::R2 result = dynamics::limit_speed(velocity_within_limits, test_parameters);
        CHECK(result == velocity_within_limits);
    }

    SUBCASE("Limit speed - Maximum Velocity") {
        Math::R2 velocity_at_max(5.0, 0.0);  // Velocity at maximum allowed velocity
        Math::R2 result = limit_speed(velocity_at_max, test_parameters);
        CHECK(result == velocity_at_max);
    }

    SUBCASE("Limit speed - Minimum Velocity") {
        Math::R2 velocity_at_min(1.0, 0.0);  // Velocity at minimum allowed velocity
        Math::R2 result = dynamics::limit_speed(velocity_at_min, test_parameters);
        CHECK(result == velocity_at_min);
    }
  }
}


