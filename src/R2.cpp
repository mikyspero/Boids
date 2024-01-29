#include "../include/R2.hpp"

#include<cmath>
namespace Math{

  R2::R2(double x, double y) : x{x}, y{y} {}
  R2::R2():R2::R2(0.,0.){}

  //the unary operators are all implemented using the *this pointer
  R2& R2::operator+=(R2 const& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }

  R2& R2::operator-=(R2 const& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  R2& R2::operator*=(double rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  } 

  R2& R2::operator/=(double rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  } 
  //the simmetridc operators are implemented using h
  R2 operator+(R2 const& lhs, R2 const& rhs) {
    auto result{lhs};
    return result += rhs;
  }

  R2 operator-(R2 const& rhs){ 
    return {-rhs.x, -rhs.y};
  }

  R2 operator-(R2 const& lhs, R2 const& rhs) {
    auto result{lhs};
    return result -= rhs;
  }

  R2 operator*(R2 const& lhs, double rhs) {
    auto result{lhs};
    return result *= rhs;
  }

  R2 operator*(double lhs, R2 const& rhs) {
    return rhs*lhs;
  }

  double operator*(R2 const& lhs, R2 const& rhs){
    return lhs.x*rhs.x+lhs.y*rhs.y;
  } 

   R2 operator/(R2 const& lhs, double rhs) {
    auto result{lhs};
    return result /= rhs;
  }

  bool operator==(R2 const& lhs, R2 const& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }

  bool operator!=(R2 const& lhs, R2 const& rhs) { 
    return !(lhs == rhs); 
  }

  //the functions to calculate norm and distance are build from the inner product
  //mimicking how inner product inducts norm which inducts distance
  double calculate_norm(R2 const& v){
    return std::sqrt(v*v);
  }

  double calculate_distance(R2 const& v1, R2 const& v2){
    return calculate_norm(v2-v1);
  }
  
}