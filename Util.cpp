/*
 * Util.cpp
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 */

#include <cmath>
#include <cstdlib>
#include "Util.h"

// Square
double sq(double x) {
  return x*x;
}
    
// Minimum
double min(double a, double b) {
  return a<=b?a:b;
}

// Maximum
double max(double a, double b) {
  return a>=b?a:b;
}

// Asin safe
double asin_safe(double x) {
  return asin(max(-1,min(1,x)));
}

// Sqrt safe
double sqrt_safe(double x) {
  return sqrt(max(x,0));
}

// Atan2 safe
double atan2_safe(double y,double x) {
  if (y==0 && x==0)
    return 0;
  return atan2(y,x);
}

// Discriminant
double discr(double a, double b, double c){
  return sq(b) - 4*a*c;
}

// Quadratic equation
double root(double a, double b, double c,int eps) {
  if (a == 0) 
    return NaR; // THIS CASE SHOULD NEVER HAPPEN
  double d = discr(a,b,c);
  if (d < 0)
    return NaR; // THIS CASE SHOULD NEVER HAPPEN
  return (-b + eps*sqrt(d))/(2*a);
} 

// To range (-pi,pi]
double topi(double x) {
  while (x <= -Pi || Pi < x) {
    if (x <= -Pi)
      x+=2*Pi;
    else
      x-=2*Pi;
  }
  return x;
}

// To range [0,360)
double to360(double x) {
  while (x < 0 || 360 <= x) {
    if (x < 0)
      x+=360;
    else
      x-=360;
  }
  return x;
}

// Radians to degrees in [0,360)
double rad2deg(double x) {
  return to360(x*180/Pi);
}
    
// Degrees to radians in (-pi,pi]
double deg2rad(double x) {
  return topi(x*Pi/180.0);
}

// Degrees:minutes to radians
double degmin2rad(int d, double m) {
  return (d+sign(d)*m/60.0)*Pi/180.0;
}

// Meters to nautical miles
double m2nm(double x) {
  return x/1852.0;
}

// Nautical miles to meters
double nm2m(double x) {
  return x*1852;
}

// Knots to meters/seconds
double knots2msec(double x) {
  return nm2m(x)/3600.0;
}

// Meters/seconds to knots
double msec2knots(double x) {
  return m2nm(x)*3600;
}

// Feet to meters
double ft2m(double x) { 
  return 1.609*x/5.28;
}

// Meters to feet
double m2ft(double x) {
  return 5.28*x/1.609;
}

// Feet/minutes to meters/seconds
double ftmin2msec(double x) {
  return ft2m(x)/60.0;
}

// Meters/seconds to feet/minutes
double msec2ftmin(double x) {
  return m2ft(x)*60;
}

// Sign (0 is positive)
int sign(double x) {
  if (x >= 0)
    return 1;
  return -1;
}

// ground speed to vx (trk [rad] in true North-clockwise convention)
double gs2vx(double gs,double trk) {
  return gs*sin(trk);
}

// ground speed to vy (trk [rad] in true North-clockwise convention)
double gs2vy(double gs,double trk) {
  return gs*cos(trk);
}

