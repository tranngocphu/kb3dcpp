/*
 * Geodesic.cpp
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 * This file contains the transformations of coordinate systems that
 * are necessary to run KB3D.
 *
 * REFERENCE: [Will] Ed Williams, Aviation Formulary V1.42
 *            http://williams.best.vwh.net/avform.htm
 *            
 * DISCLAIMER :These approximations fail in the vicinity of either pole 
 *             and at large distances. The fractional errors are of order
 *             (distance/R)^2. See [Will] 
 *
 * REMARK : North latitudes are positive.
 *          East longitudes are positive.
 *          X points to East, Y points to North. 
 *
 * Unit Conventions
 * ----------------
 * - lat,lan,lon in radians [rad]
 * - x,y in meters [m]
 *
 * Functions
 * ---------
 *   gc_dist : Great circle distance
 *   tc      : Initial true course
 *   geo2xy  : Conversion from geodesic coordinates to Cartesian coordinates 
 * 
 */

#include <cmath>
#include "Util.h"
#include "Geodesic.h"

Geodesic::Geodesic() {
  sx = sy = gcd = tc = 0;
}

// Great circle distance [rad]
double gc_dist(double lat1, double lon1,   // [rad][rad]
	       double lat2, double lon2) { // [rad][rad]
  return 2*asin_safe(sqrt_safe(sq(sin((lat1-lat2)/2.0)) +
			       cos(lat1)*cos(lat2)*
			       sq(sin((lon1-lon2)/2.0))));
}

// Initial true course [rad]
double true_course(double lat1, double lon1,   // [rad][rad]
		   double lat2, double lon2) { // [rad][rad]
  return atan2_safe(sin(lon2-lon1)*cos(lat2),
		    cos(lat1)*sin(lat2)-
		    sin(lat1)*cos(lat2)*
		    cos(lon1-lon2));
}

/* geo2xy:  Converts from geodesic coordinates to cartesian coordinates 
 *          assuming flat earth
 * PRECONDITION: 
 *   latdeg_o,londeg_o: Ownship  position
 *    ([deg],[deg])
 *   latdeg_i,londeg_i: Traffic position
 *    ([deg],[deg])
 *
 * POSTCONDITION: 
 *   (sx,sy) is the relative position of the ownship wrt traffic [m,m]
 *    gcd : Great circle distance [rad]
 *    tc  : Initial true course [rad]
 *
 * OUTPUTS: sx,sy,gcd,tc
 */

void Geodesic::geo2xy(double latdeg_o,   // Ownship latitude   [deg]
		      double londeg_o,   // Ownship longitude  [deg]
		      double latdeg_i,   // Traffic latitude  [deg]
		      double londeg_i) { // Traffic longitude [deg] 
  
  double lat_o,lon_o; // [rad,rad]
  double lat_i,lon_i; // [rad,rad]
  double e2 = Flat*(2-Flat);
  
  lat_o = deg2rad(latdeg_o);
  lat_i = deg2rad(latdeg_i);
  lon_o = deg2rad(londeg_o);
  lon_i = deg2rad(londeg_i);
  double dlat = lat_o - lat_i;
  double dlon = lon_o - lon_i;
  double r32 = 1-e2*sq(sin(lat_i));
  double R1 = WGS84*(1-e2)/sqrt_safe(r32*r32*r32);
  double R2 = WGS84/sqrt_safe(1-e2*sq(sin(lat_i)));
  sy  = R1*dlat;
  sx  = R2*cos(lat_i)*dlon;
  gcd = gc_dist(lat_o,lon_o,lat_i,lon_i);
  tc  = true_course(lat_o,lon_o,lat_i,lon_i);
}

