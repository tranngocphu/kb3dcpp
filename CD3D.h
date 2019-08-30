/*
 * CD3D.h
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 */

class CD3D {
 public: 
  // Outputs
  double  time2los;  // Time to loss of separation (from cd3d)
  double  time2lhs;  // Time to loss of horizontal separation
  double  time2lvs;  // Time to loss of vertical separation
  double  t_in;      // Time when relative trajectory enters the protected zone
  double  t_out;     // Time when realtive trajectory exits the protected zone
  double  duration;  // Duration of conflict (from cd3d)
  bool    conflict;  // Conflict detection? (from cd3d)
  
  CD3D(double d, double h, double t);
  double get_filter() const;
  void   set_filter(double t);
  void   set_D(double d);
  double get_D() const;
  void   set_H(double d);
  double get_H() const;
  void   set_T(double t);
  double get_T() const;
  bool   violation(double sx, double sy, double sz);
  bool   cd3d(double sx, double sy, double sz, 
	      double vx, double vy, double vz);
  
 private:
  
  // Horizontal (D) and vertical (H) separation
  double D,H;
  
  // Lookahead time
  double T;
  
  // Ignore conflicts that last less than filter units of time
  double filter; 
  
};
