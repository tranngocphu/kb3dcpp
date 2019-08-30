/*
 * Geodesic.h
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 */

const double WGS84 = 6378137;         // Major axis [m]
const double Flat  = 1/298.257223563; // Earth flattening

double gc_dist(double lat1, double lon1,  // [rad] [rad]
	       double lat2, double lon2); // [rad] [rad]

double true_course(double lat1, double lon1,  // [rad] [rad]
		   double lat2, double lon2); // [rad] [rad]

class Geodesic {
 public:
  
  // Outputs
  
  double sx,sy; // Relative position (wrt traffic)[m,m] (from geo2xy)
  double gcd;   // Great circle distance [rad] (from geo2xy)
  double tc;    // Initial true course [rad] (from geo2xy)
  
  Geodesic();
  
  void geo2xy(double latdeg_o,  // Ownship latitude   [deg]
	      double londeg_o,  // Ownship longitude  [deg]
	      double latdeg_i,  // Traffic latitude  [deg]
	      double londeg_i); // Traffic longitude [deg] 
  
};
