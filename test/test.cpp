#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../include/doctest.h"
#include "../include/flock.hpp"

TEST_CASE("Class R2 and operators tests") {
  SUBCASE("Vector addition") {
    math::R2 v1(1.0, 2.0);
    math::R2 v2(-3.0, -4.0);
    math::R2 v3(0, 0);
    math::R2 v4(1.5, -1.);

    math::R2 result = v1 + v2;
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
    math::R2 v1(2.0, 3.0);
    math::R2 result = -v1;
    CHECK(result.x == -2.0);
    CHECK(result.y == -3.0);

    math::R2 v2(-3.0, -4.0);
    result = -v2;
    CHECK(result.x == 3.0);
    CHECK(result.y == 4.);

    math::R2 v3(0, 0);
    result = -v3;
    CHECK(result.x == 0.);
    CHECK(result.y == 0.);
  }

  SUBCASE("Vector subtraction") {
    math::R2 v1(1.0, 2.0);
    math::R2 v2(-3.0, -4.0);
    math::R2 v3(0, 0);
    math::R2 v4(1.5, -1.);

    math::R2 result = v1 - v2;
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
    math::R2 v(2.0, 3.0);
    double k = 1.5;
    math::R2 result = v * k;
    math::R2 result2 = k * v;

    CHECK(result.x == doctest::Approx(3.0));
    CHECK(result.y == doctest::Approx(4.5));
    CHECK(result == result2);

    k = -1.5;
    result = k * v;
    result2 = k * v;
    CHECK(result.x == doctest::Approx(-3.0));
    CHECK(result.y == doctest::Approx(-4.5));
    CHECK(result == result2);

    k = 0.;
    result = v * k;
    CHECK(result.x == doctest::Approx(0.));
    CHECK(result.y == doctest::Approx(0.));
  }

  SUBCASE("Vector equality and inequality") {
    math::R2 v1(2.0, 3.0);
    math::R2 v2(2.0, 3.0);
    math::R2 v3(4.0, 5.0);

    CHECK(v1 == v2);
    CHECK(v1 != v3);
  }

  SUBCASE("Inner product") {
    math::R2 v1(2.0, 3.0);
    math::R2 v2(4.0, 5.0);
    double result = v1 * v2;
    CHECK(result == doctest::Approx(23.0));

    math::R2 v3(1., 0.);
    math::R2 v4(0., 1.);
    result = v3 * v4;
    CHECK(result == doctest::Approx(0.));

    math::R2 v5(2.0, -3.0);
    math::R2 v6(-4.0, 5.0);
    result = v5 * v6;
    CHECK(result == doctest::Approx(-23.0));
  }

  SUBCASE("Norm calculation") {
    math::R2 v1(3.0, 4.0);
    math::R2 v2(-3.0, -4.0);
    math::R2 v3(0., 0.);

    double result = math::calculate_norm(v1);
    CHECK(result == doctest::Approx(5.0));

    result = math::calculate_norm(v2);
    CHECK(result == doctest::Approx(5.0));

    result = math::calculate_norm(v3);
    CHECK(result == doctest::Approx(0.));
  }

  SUBCASE("Distance calculation") {
    math::R2 v1(1.0, 2.0);
    math::R2 v2(4.0, 6.0);
    double result = math::calculate_distance(v1, v2);
    CHECK(result == doctest::Approx(5.0));

    result = math::calculate_distance(v1, v1);
    CHECK(result == doctest::Approx(0.));
  }

  SUBCASE("Compound assignment operators") {
    math::R2 v1(2.0, 3.0);
    math::R2 v2(1.0, 2.0);

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
  math::R2 initial_position(1.0, 3.0);
  math::R2 initial_velocity(2.0, 4.0);

  dynamics::Boid boid(initial_position, initial_velocity);

  SUBCASE("Getters") {
    CHECK(boid.r() == initial_position);
    CHECK(boid.v() == initial_velocity);
  }

  SUBCASE("Setters") {
    math::R2 new_position(5.4, 5.6);
    math::R2 new_velocity(0.7, 8.0);

    boid.r(new_position);
    boid.v(new_velocity);

    CHECK(boid.r() == new_position);
    CHECK(boid.v() == new_velocity);
  }
}

TEST_CASE("Distance between boids") {
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

TEST_CASE("Testing mass displacement center relative to fixed boid") {
  SUBCASE("four points center of mass displacement") {
    dynamics::Boid b1{{0., 7.}, {1., 1.}};
    dynamics::Boid b2{{6., 5.}, {1., 1.}};
    dynamics::Boid b3{{1., 9.}, {1., 1.}};
    dynamics::Boid b4{{2., 2.}, {-1., -1.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4};

    CHECK(calculate_CDM(flock, b1).x == doctest::Approx(3.));
    CHECK(calculate_CDM(flock, b1).y ==
          doctest::Approx(5.3333).epsilon(0.0001));
    CHECK(calculate_CDM(flock, b2).x == doctest::Approx(1.));
    CHECK(calculate_CDM(flock, b2).y == doctest::Approx(6.));
    CHECK(calculate_CDM(flock, b3).x ==
          doctest::Approx(2.6667).epsilon(0.0001));
    CHECK(calculate_CDM(flock, b3).y ==
          doctest::Approx(4.66667).epsilon(0.0001));
    CHECK(calculate_CDM(flock, b4).x ==
          doctest::Approx(2.3333).epsilon(0.0001));
    CHECK(calculate_CDM(flock, b4).y == doctest::Approx(7.));
  }

  SUBCASE("five points center of mass displacement") {
    dynamics::Boid b1{{1., 2.}, {1., 1.}};
    dynamics::Boid b2{{3., 2.}, {1., 1.}};
    dynamics::Boid b3{{3., 5.}, {1., 1.}};
    dynamics::Boid b4{{4., 2.}, {-1., -1.}};
    dynamics::Boid b5{{0., 1.}, {-1., -1.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};

    CHECK(calculate_CDM(flock, b1).x == doctest::Approx(2.5));
    CHECK(calculate_CDM(flock, b1).y == doctest::Approx(2.5));
    CHECK(calculate_CDM(flock, b2).x == doctest::Approx(2.));
    CHECK(calculate_CDM(flock, b2).y == doctest::Approx(2.5));
    CHECK(calculate_CDM(flock, b3).x == doctest::Approx(2.));
    CHECK(calculate_CDM(flock, b3).y == doctest::Approx(1.75));
    CHECK(calculate_CDM(flock, b4).x == doctest::Approx(1.75));
    CHECK(calculate_CDM(flock, b4).y == doctest::Approx(2.5));
    CHECK(calculate_CDM(flock, b5).x == doctest::Approx(2.75));
    CHECK(calculate_CDM(flock, b5).y == doctest::Approx(2.75));
  }
}

TEST_CASE("Testing get_neighborhood") {

  dynamics::Boid b1{{1., 2.}, {1., 1.}};
  dynamics::Boid b2{{3., 2.}, {1., 1.}};
  dynamics::Boid b3{{3., 5.}, {1., 1.}};
  dynamics::Boid b4{{4., 2.}, {-1., -1.}};
  dynamics::Boid b5{{0., 1.}, {-1., -1.}};
  std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};

  CHECK(get_neighborhood(flock, b1, 3.5).size() == 4);
  CHECK(get_neighborhood(flock, b2, 4.).size() == 5);
  CHECK(get_neighborhood(flock, b3, 2.).size() == 1);
  CHECK(get_neighborhood(flock, b5, 0.1).size() == 1);
}

TEST_CASE("Testing calculate_separation_velocity") {
  dynamics::Boid b1{{1., 2.}, {1., 1.}};
  dynamics::Boid b2{{3., 2.}, {1., 1.}};
  dynamics::Boid b3{{3., 5.}, {1., 1.}};
  dynamics::Boid b4{{4., 2.}, {-1., -1.}};
  dynamics::Boid b5{{0., 1.}, {-1., -1.}};
  std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};
  CHECK(calculate_separation(b1, flock, 1., 7.).x == doctest::Approx(-6));
  CHECK(calculate_separation(b1, flock, 1., 7.).y == doctest::Approx(-2));
  CHECK(calculate_separation(b2, flock, 1., 7.).x == doctest::Approx(4));
  CHECK(calculate_separation(b2, flock, 1., 7.).y == doctest::Approx(-2));
}

TEST_CASE("Testing calculate_alignement_velocity") {
  dynamics::Boid b1{{1., 2.}, {1., 1.}};
  dynamics::Boid b2{{3., 2.}, {1., 1.}};
  dynamics::Boid b3{{3., 5.}, {1., 1.}};
  dynamics::Boid b4{{4., 2.}, {-1., -1.}};
  dynamics::Boid b5{{0., 1.}, {-1., -1.}};
  std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};

  CHECK(calculate_alignment(b1, flock, 1.).x ==
        doctest::Approx(-1).epsilon(0.01));
  CHECK(calculate_alignment(b1, flock, 1.).y ==
        doctest::Approx(-1).epsilon(0.01));
  CHECK(calculate_alignment(b2, flock, 1.).x ==
        doctest::Approx(-1).epsilon(0.01));
  CHECK(calculate_alignment(b2, flock, 1.).y ==
        doctest::Approx(-1).epsilon(0.01));
}

TEST_CASE("Testing calculate_cohesion_velocity") {
  dynamics::Boid b1{{1., 2.}, {1., 1.}};
  dynamics::Boid b2{{3., 2.}, {1., 1.}};
  dynamics::Boid b3{{3., 5.}, {1., 1.}};
  dynamics::Boid b4{{4., 2.}, {-1., -1.}};
  dynamics::Boid b5{{0., 1.}, {-1., -1.}};
  std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};

  CHECK(calculate_cohesion(b1, flock, 1.).x == doctest::Approx(1.5));
  CHECK(calculate_cohesion(b1, flock, 1.).y ==
        doctest::Approx(0.5).epsilon(0.01));
  CHECK(calculate_cohesion(b2, flock, 1.).x ==
        doctest::Approx(-1).epsilon(0.01));
  CHECK(calculate_cohesion(b2, flock, 1.).y == doctest::Approx(0.5));
}

TEST_CASE("Testing teleport_toroidally") {
  dynamics::running_parameters const p{0,  0.,  0.,  0., 0., 0.,
                                       0., 10., 10., 0., 0., 0.};

  math::R2 v1{-1., 3.};
  math::R2 v2{1., -3.};
  math::R2 v3{-3., -4.};
  math::R2 v4{17., 16.};

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

TEST_CASE("Limit Speed Function") {
  // Define a test running_parameters instance
  dynamics::running_parameters test_parameters{0,  0., 0., 0., 0.,  0.,
                                               0., 0., 0., 0., 5., 1.};

  SUBCASE("Limit speed - Above Maximum Velocity") {
    math::R2 velocity_above_max(8.0, 8.0); // Velocity magnitude exceeds maximum
    math::R2 result =
        dynamics::limit_speed(velocity_above_max, test_parameters);
  
    CHECK(math::calculate_norm(result) <= test_parameters.maximum_velocity);
  }

  SUBCASE("Limit speed - Below Minimum Velocity") {
    math::R2 velocity_below_min(0.5, 0.5); // Velocity magnitude is below minimum
    math::R2 result =
        dynamics::limit_speed(velocity_below_min, test_parameters);

    CHECK(math::calculate_norm(result) >= test_parameters.minimum_velocity);
  }

  SUBCASE("Limit speed - Within Velocity Limits") {
    math::R2 velocity_within_limits(2.0,
                                    2.0); // Velocity magnitude within limits
    math::R2 result =
        dynamics::limit_speed(velocity_within_limits, test_parameters);
    CHECK(result == velocity_within_limits);
  }

  SUBCASE("Limit speed - Maximum Velocity") {
    math::R2 velocity_at_max(5.0, 0.0); // Velocity at maximum allowed velocity
    math::R2 result = limit_speed(velocity_at_max, test_parameters);
    CHECK(result == velocity_at_max);
  }

  SUBCASE("Limit speed - Minimum Velocity") {
    math::R2 velocity_at_min(1.0, 0.0); // Velocity at minimum allowed velocity
    math::R2 result = dynamics::limit_speed(velocity_at_min, test_parameters);
    CHECK(result == velocity_at_min);
  }
}

TEST_CASE("Testing evolve_boid") {
  SUBCASE("first test") {
    dynamics::running_parameters const p{5,  1.,  1.,  1., 5.,  1.,
                                         0., 10., 10., 0., 10., 2.};

    dynamics::Boid b1{{1., 2.}, {2, 1.}};
    dynamics::Boid b2{{3., 2.}, {3., 1.}};
    dynamics::Boid b3{{3., 5.}, {1., 2.}};
    dynamics::Boid b4{{4., 2.}, {-4., 0.}};
    dynamics::Boid b5{{0., 1.}, {-3., -1.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};
    auto delta_t = 0.2;

    evolve_boid(flock, b1, delta_t, p);

    CHECK(b1.r().x == doctest::Approx(1.4));
    CHECK(b1.r().y == doctest::Approx(2.2));

    CHECK(b1.v().x == doctest::Approx(-5.25));
    CHECK(b1.v().y == doctest::Approx(-1));
  }

  SUBCASE("second test") {
    dynamics::running_parameters const p{5,  1.,  1.,  1., 5.,  1.,
                                         0., 10., 10., 0., 10., 2.};

    dynamics::Boid b1{{6.5, -7.5}, {7., 7.}};
    dynamics::Boid b2{{1., -3.}, {5., 0.}};
    dynamics::Boid b3{{4., -6.}, {1., -1.}};
    dynamics::Boid b4{{4.2, 4.2}, {4., -3.}};
    dynamics::Boid b5{{12.2, 2.}, {2., -2.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};
    auto delta_t = 0.2;
    evolve_boid(flock, b1, delta_t, p);

    CHECK(b1.r().x == doctest::Approx(7.9));
    CHECK(b1.r().y == doctest::Approx(3.9));

    CHECK(b1.v().x == doctest::Approx(4.35));
    CHECK(b1.v().y == doctest::Approx(3.8));
  }
}

TEST_CASE("Testing evolve_flock") {
  SUBCASE("first case") {
    auto delta_t = 0.2;
    dynamics::Boid b1{{1., 2.}, {1., 1.}};
    dynamics::Boid b2{{3., 2.}, {1., 3.}};
    dynamics::Boid b3{{3., 5.}, {2., 1.}};
    dynamics::Boid b4{{4., 2.}, {-1., -1.}};
    dynamics::Boid b5{{0., 1.}, {0., 1.}};
    dynamics::Boid b6{{1., 7.}, {-1., -3.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5, b6};
    dynamics::running_parameters const p{5,  1.,  1.,  1., 5.,  1.,
                                         0., 10., 10., 0., 10., 2.};
    evolve_flock(flock, delta_t, p);

    CHECK(flock[0].r().x == doctest::Approx(1.2));
    CHECK(flock[0].r().y == doctest::Approx(2.2));
    CHECK(flock[1].r().x == doctest::Approx(3.2));
    CHECK(flock[1].r().y == doctest::Approx(2.6));
    CHECK(flock[2].r().x == doctest::Approx(3.4));
    CHECK(flock[2].r().y == doctest::Approx(5.2));
    CHECK(flock[3].r().x == doctest::Approx(3.8));
    CHECK(flock[3].r().y == doctest::Approx(1.8));
    CHECK(flock[4].r().x == doctest::Approx(0));
    CHECK(flock[4].r().y == doctest::Approx(1.2));
    CHECK(flock[5].r().x == doctest::Approx(0.8));
    CHECK(flock[5].r().y == doctest::Approx(6.4));

    CHECK(flock[0].v().x == doctest::Approx(2.));
    CHECK(flock[0].v().y == doctest::Approx(2.));
    CHECK(flock[1].v().x == doctest::Approx(1.));
    CHECK(flock[1].v().y == doctest::Approx(3.));
    CHECK(flock[2].v().x == doctest::Approx(2.));
    CHECK(flock[2].v().y == doctest::Approx(1.));
    CHECK(flock[3].v().x == doctest::Approx(-2.));
    CHECK(flock[3].v().y == doctest::Approx(-2.));
    CHECK(flock[4].v().x == doctest::Approx(0.));
    CHECK(flock[4].v().y == doctest::Approx(2));
    CHECK(flock[5].v().x == doctest::Approx(-1.));
    CHECK(flock[5].v().y == doctest::Approx(-3.));
  }
  SUBCASE("second case") {
    auto delta_t = 0.2;
    dynamics::Boid b1{{6.5, -7.5}, {7., 7.}};
    dynamics::Boid b2{{1., -3.}, {5., 0.}};
    dynamics::Boid b3{{4., -6.}, {1., -1.}};
    dynamics::Boid b4{{4.2, 4.2}, {4., -3.}};
    dynamics::Boid b5{{12.2, 2.}, {2., -2.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};
    dynamics::running_parameters const p{5,  1.,  1.,  1., 5.,  1.,
                                         0., 10., 10., 0., 10., 2.};
    evolve_flock(flock, delta_t, p);

    CHECK(flock[0].r().x == doctest::Approx(7.9));
    CHECK(flock[0].r().y == doctest::Approx(3.9));
    CHECK(flock[1].r().x == doctest::Approx(2.));
    CHECK(flock[1].r().y == doctest::Approx(7.));
    CHECK(flock[2].r().x == doctest::Approx(4.2));
    CHECK(flock[2].r().y == doctest::Approx(3.8));
    CHECK(flock[3].r().x == doctest::Approx(5.));
    CHECK(flock[3].r().y == doctest::Approx(3.6));
    CHECK(flock[4].r().x == doctest::Approx(2.6));
    CHECK(flock[4].r().y == doctest::Approx(1.6));

    CHECK(flock[0].v().x == doctest::Approx(7.));
    CHECK(flock[0].v().y == doctest::Approx(7.));
    CHECK(flock[1].v().x == doctest::Approx(5.));
    CHECK(flock[1].v().y == doctest::Approx(0.));
    CHECK(flock[2].v().x == doctest::Approx(2.));
    CHECK(flock[2].v().y == doctest::Approx(-2.));
    CHECK(flock[3].v().x == doctest::Approx(4.));
    CHECK(flock[3].v().y == doctest::Approx(-3.));
    CHECK(flock[4].v().x == doctest::Approx(2.));
    CHECK(flock[4].v().y == doctest::Approx(-2.));
  }
}

TEST_CASE("Testing mean distance and std_dev") {

  SUBCASE("Three boids") {
    dynamics::Boid b1{{6.5, -7.5}, {7., 7.}};
    dynamics::Boid b2{{1., -3.}, {5., 0.}};
    dynamics::Boid b3{{4., -6.}, {1., -1.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3};
    double mean = view::calculate_mean_distance(flock);

    CHECK(mean == doctest::Approx(4.7548).epsilon(0.0001));
    CHECK(view::calculate_standard_deviation_distance(flock, mean) ==
          doctest::Approx(2.1419).epsilon(0.0001));
  }

  SUBCASE("Five boids") {
    dynamics::Boid b1{{-1., 2.}, {0., 0.}};
    dynamics::Boid b2{{4., 0.}, {0., 0.}};
    dynamics::Boid b3{{5., 4.}, {0., 0.}};
    dynamics::Boid b4{{1., 2.}, {1., 1.}};
    dynamics::Boid b5{{2., 1.}, {1., 1.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};
    double mean = view::calculate_mean_distance(flock);

    CHECK(mean == doctest::Approx(3.6966));
    CHECK(view::calculate_standard_deviation_distance(flock, mean) ==
          doctest::Approx(0.6890).epsilon(0.0001));
  }
}

TEST_CASE("Testing mean velocity and std_dev") {

  SUBCASE("first case") {
    dynamics::Boid b1{{1., 3.}, {-4., 0.}};
    dynamics::Boid b2{{4., 5.}, {0., 0.}};
    dynamics::Boid b3{{0., 1.}, {2., -3.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3};
    double mean = view::calculate_mean_velocity(flock);

    CHECK(mean == doctest::Approx(2.5352).epsilon(0.0001));
    CHECK(view::calculate_standard_deviation_velocity(flock, mean) ==
          doctest::Approx(2.2044).epsilon(0.0001));
  }
  SUBCASE("second case") {
    dynamics::Boid b1{{6.5, -7.5}, {7., 7.}};
    dynamics::Boid b2{{1., -3.}, {5., 0.}};
    dynamics::Boid b3{{4., -6.}, {1., -1.}};
    dynamics::Boid b4{{4.2, 4.2}, {4., -3.}};
    dynamics::Boid b5{{12.2, 2.}, {2., -2.}};
    std::vector<dynamics::Boid> flock{b1, b2, b3, b4, b5};
    double mean = view::calculate_mean_velocity(flock);

    CHECK(mean == doctest::Approx(4.8284).epsilon(0.0001));
    CHECK(view::calculate_standard_deviation_velocity(flock, mean) ==
          doctest::Approx(3.2184).epsilon(0.0001));
  }
}
