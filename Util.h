/*
 * Util.h 
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 * 
 */

const double TauMin = 0.0001; // A minimum time for positive tau

const double NaR = 9999999;   // An arbitrary large number

const double Pi = 3.141592654; // Value of Pi 

double sq(double x);
double min(double a,double b);
double max(double a,double b);
double asin_safe(double x);
double sqrt_safe(double x);
double atan2_safe(double y,double x);
double discr(double a, double b, double c);
double root(double a, double b, double c,int eps);
double topi(double x);
double to360(double x);
double rad2deg(double x);
double deg2rad(double x);
double degmin2rad(int d, double m);
double m2nm(double x);
double nm2m(double x);
double knots2msec(double x);
double msec2knots(double x);
double ft2m(double x);
double m2ft(double x);
double ftmin2msec(double x);
double msec2ftmin(double x);
int    sign(double x);
double gs2vx(double vg,double trk);
double gs2vy(double vg,double trk);
