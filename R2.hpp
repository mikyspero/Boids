#ifndef R2_HPP
#define R2_HPP
namespace Math{
  //The struct R2 represents a two-dimensional vector in the Euclidean space R^2,
  //it uses two doubles to describe a member of the R2 vector space
  //it's a struct since it's a simple composite type that mimics the behaviour of integrated type double
  // providing the mathematical underpinning of the boids simulation
  struct R2 {
    double x;
    double y;
    //constructors
    R2(double x, double y);
    R2();
    //the simmetric operator are defined in class to use the pointer *this in the redefinition of the operators
    R2& operator+=(R2 const& rhs);
    R2& operator-=(R2 const& rhs);
    R2& operator*=(double rhs);
    R2& operator/=(double rhs);
  };

// Symmetric operators operator@ are implemented in terms of member operators operator @=
//every operation possible between doubles is possible between R2
  R2 operator+(R2 const& lhs, R2 const& rhs);
  R2 operator-(R2 const& rhs);
  R2 operator-(R2 const& lhs, R2 const& rhs);
  //operators for product between vector and scalar
  R2 operator*(R2 const& lhs, double rhs);
  R2 operator*( double lhs, R2 const& rhs);
  R2 operator/(R2 const& lhs, R2 const& rhs);
  //operator for the inner product of R2
  double operator*(R2 const& lhs, R2 const& rhs);
 

  // Implemented comparison of vectors in order to use it for some tests
  bool operator==(R2 const& lhs, R2 const& rhs);
  bool operator!=(R2 const& lhs, R2 const& rhs);
  //following the implementation of the scalar product between vectors
  //I built functions to calculate norm and distance
  double calculate_norm(R2 const& v);
  double calculate_distance(R2 const& v1, R2 const& v2);
}
#endif

