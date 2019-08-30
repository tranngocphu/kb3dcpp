/*
 * CD3D.cpp
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 * CD3D is an algorithm for 3-D conflict *detection*.
 *
 * Unit Convention
 * ---------------
 * - Units of distance are denoted [d]
 * - Units of time are denoted     [d]
 * - Units of speed are denoted    [d/t]
 *
 * REMARK: X points to East, Y points to North. 
 *
 * Naming Convention
 * -----------------
 *   The traffic aircraft is fixed at the origin of the coordinate system.
 * 
 *   D         : Horizontal separation [d]
 *   H         : Vertical separation [d]
 *   T         : Lookahead time [t]
 *   sx,sy,sz  : Relative position of the ownship [d,d,d]
 *   vx,vy,vz  : Relative velocity vector of the ownship [d/t,d/t,d/t]
 * 
 * Functions
 * ---------
 * - cd3d : Conflict detection with calculation of conflict interval 
 * 
 */

#include <cmath>
#include "Util.h"
#include "CD3D.h"

// Create a new CD3D object with protected zone D,H,T
CD3D::CD3D(double d, double h, double t) {
  D = d;
  H = h;
  T = t;
  time2los  = 0;
  time2lvs  = 0;
  time2lhs  = 0;
  t_in      = 0;
  t_out     = 0;
  duration  = 0;
  conflict  = false;
  filter    = 0;
}

double CD3D::get_filter() const {
  return filter;
}

void CD3D::set_filter(double t) {
  filter = t;
}

void CD3D::set_D(double d) {
  D = d;
}

double CD3D::get_D() const {
  return D;
}

void CD3D::set_H(double h) {
  H = h;
}

double CD3D::get_H() const {
  return H;
}

void CD3D::set_T(double t) {
  T = t;
}

double CD3D::get_T() const {
  return T;
}

// Aircraft are in violation at time 0
bool CD3D::violation(double sx, double sy, double sz) {
  return sq(sx)+sq(sy) < sq(D) && sq(sz) < sq(H);
}

/* cd3d: Conflict detection with conflict interval
 *
 * Let conflict = cd3d(sx,sy,sz,vx,vy,vz) in
 *   conflict ==> 
 *     time2los is the time to conflict
 *     duration is the duration of the conflict
 *
 *   violation ==>
 *     time2los = 0
 *     duration is the remaining time to exit the violation.
 *
 *   conflicts are reported only if duration > filter and time2los <= T
 *
 * OUTPUTS: time2los,duration,conflict
 *
 */

bool CD3D::cd3d(double sx, double sy, double sz, 
		double vx, double vy, double vz) { 
  
  double  d,a,b,t1,t2,theta1,theta2;
  
  conflict  = false;
  t_in      = 0;
  t_out     = 0;
  
  if (vx==0 && vy==0 && sq(sx)+sq(sy) < sq(D)) {
    // No horizontal movement
    conflict = sq(sz) < sq(H) || // Already in conflict
      (vz!=0 && vz*sz <= 0 && -H < sign(vz)*(T*vz + sz));
    if (conflict) {
      if (vz != 0) {
        t_in     = (sign(sz)*H - sz)/vz;
        t_out    = (-sign(sz)*H - sz)/vz;
        time2lvs = t_in; 
      } else {
        t_in = 0;
        t_out = T;
      }
    }
  } else {
    // Vertical conflict 
    // in the future
    d = 2*sx*vx*sy*vy + sq(D)*(sq(vx) + sq(vy)) -  (sq(sx)*sq(vy) + sq(sy)*sq(vx));
    if (d > 0) {
      a = sq(vx) + sq(vy);
      b = sx*vx + sy*vy;
      theta1 = (-b - sqrt(d))/a; //first intersection with D
      theta2 = (-b + sqrt(d))/a; //second intersection with D 
      time2lhs = theta1;
      // theta1 <= theta2
      if (vz == 0) {  // Horizontal movement only
	      t_in     = theta1;
        t_out    = theta2;
        conflict = sq(sz) < sq(H) ;
      } else {  // General case 
        t1 = (-sign(vz)*H-sz)/vz;
        t2 = (sign(vz)*H-sz)/vz;
        time2lvs = t1;
        // t1 < t2
        t_in      = max(theta1,t1);
        t_out     = min(theta2,t2);
        conflict  = theta1 < t2 && t1 < theta2;
      }  
    }
  }
  time2los = max(t_in,0);
  time2lhs = max(time2lhs,0);
  time2lvs = max(time2lvs,0);
  duration  = t_out - time2los;
  conflict &= t_in <= T && t_out > 0 && duration > filter;
  return conflict;
}