/*
 * KB3D.cpp
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 * KB3D is an algorithm for 3-D conflict *resolution* that provides
 * avoidance maneuvers for the ownship 
 *
 * Unit Convention
 * ---------------
 * - Units of distance are denoted [d]
 * - Units of time are denoted     [t]
 * - Units of speed are denoted    [d/t]
 * - Units of turn are radians     [rad] 
 *
 * REMARK: X points to East, Y points to North. Angles are in 
 *         True North/clockwise convention.
 *
 * Naming Convention
 * -----------------
 * The traffic aircraft is supposed to be fixed at the origin of the coordinate
 * system. 
 * D           : Horizontal separation [d]
 * H           : Vertical separation [d]
 * T           : Lookahead time [t]
 * vox,voy,voz : Ownship velocity vector  [d/t,d/t,d/t]
 * vix,viy,viz : Traffic velocity vector [d/t,d/t,d/t]
 *
 * Functions
 * ---------
 * - ground_speed, vertical, track : Independent resolutions
 * - kb3d_vertical   : Coordinated vertical maneuver (vertical speed)
 * - kb3d_horizontal : Coordinated horizontal maneuvers (ground speed, track)
 *
 * Output class variables
 * ----------------------
 * trk: Track [rad]
 * gs : Ground speed   [d/t]
 * vs : Vertical speed [d/t]
 *
 */

#include <cmath>
#include "Util.h"
#include "CD3D.h"
#include "KB3D.h"

// Create a new KB3D object with protected zone D,H,T
KB3D::KB3D(double d, double h, double t) {
  D = d;
  H = h;
  T = t;
  cd3d = new CD3D(D,H,T);
  vs = gs = trk = opt_gs = opt_trk = NaR;
  vx = vy = kvx = kvy = ovx = ovy = 0;
}

KB3D::~KB3D() {
  delete cd3d;
}

void KB3D::set_D(double d) {
  cd3d->set_D(d);
  D = d;
}

double KB3D::get_D() const {
  return D;
}

void KB3D::set_H(double h) {
  cd3d->set_H(h);
  H = h;
}

double KB3D::get_H() const {
  return H;
}

void KB3D::set_T(double t) {
  cd3d->set_T(t);
  T = t;
}

double KB3D::get_T() const {
  return T;
}

/* General Precondition for KB3D
 *
 * - Aircraft are in predicted conflict
 * - Aircraft are not in violation
 * - Aircraft have a positive ground speed
 *
 */

bool KB3D::precondition(double sx, double sy, double sz,
			double vox, double voy, double voz,
			double vix, double viy, double viz) {
  return 
    !cd3d->violation(sx,sy,sz) &&
    cd3d->cd3d(sx,sy,sz,vox-vix,voy-viy,voz-viz) &&
    sq(vox) + sq(voy) > 0 && 
    sq(vix) + sq(viy) > 0;
}

/* Coordination strategies */

int KB3D::vertical_coordination(double sx, double sy, double sz, double pz) {
  int epsilon=1;
  if (pz == 0 && sz == 0)
    epsilon=break_symm(sx,sy,sz);
  else if (pz == 0) 
    epsilon=sign(sz);
  else
    epsilon=sign(pz);
  return epsilon;
}

int KB3D::horizontal_coordination(double sx, double sy, 
				  double vx, double vy) { 
  return sign(sy*vx - sx*vy);
}

/* Support functions */

// Time of closest approach (horizontal plane)
double tau(double sx, double sy, 
	   double vx, double vy) { 
  double a = sq(vx)+sq(vy);
  if (a!=0)
    return -(sx*vx + sy*vy)/a;
  return NaR;
}

// Is tau positive ?
bool tau_pos(double sx, double sy, 
	     double vx, double vy) {
  return sx*vx + sy*vy <= -TauMin;
}

// Epsilon 
int eps_line(double sx, double sy, 
	     double vx, double vy) {
  return sign(sx*vx+sy*vy)/sign(sx*vy-sy*vx);
}

int break_symm(double sx, double sy, double sz) {
  if (sx < 0 || (sx == 0 && sy < 0)) 
    return 1;
  return -1;
}

double KB3D::delta(double sx,double sy, double vx, double vy) {
  return sq(D) * (sq(vx) + sq(vy)) - 
    sq(sx * vy - sy * vx);
}

double KB3D::theta(double sx,double sy,double vx,double vy,int eps) {
  double v = sq(vx)+sq(vy);
  if (v == 0)
    return 0; // THIS CASE SHOULD NEVER HAPPEN
  double d = delta(sx,sy,vx,vy);
  return (-sx*vx - sy*vy + eps * sqrt(d))/v;
}

double KB3D::vertical_theta1(double sx, double sy, double sz,
			     double vx, double vy, double viz,
			     int eps) {
  double t = theta(sx,sy,vx,vy,-1);
  if (t==0)
    return NaR; // THIS CASE SHOULD NEVER HAPPEN
  return viz + (eps*H - sz)/t;
}

double KB3D::vertical_theta2(double sx, double sy, double sz,
			     double vx, double vy, double viz){

  double t = theta(sx,sy,vx,vy,1);
  if (t==0)
    return NaR; // THIS CASE SHOULD NEVER HAPPEN
  return
    viz + (sign(sz)*H - sz)/t;
}

/* vertical: Independent vertical speed only maneuver
 *
 * Let vs = vertical(...) in
 * vs != NaR ==>
 *   (vox,voy,vs) is an independent ground speed only solution for the ownship.
 *
 */

double KB3D::vertical(double sx, double sy, double sz,
		      double vox, double voy, double voz,
		      double vix, double viy, double viz,
		      int epsilon) {
  double vs = NaR;
  double vx = vox - vix;
  double vy = voy - viy;
  
  if (sq(vx) + sq(vy) == 0)
    vs = viz;
  else if (epsilon*sz < H && sq(sx) + sq(sy) > sq(D))
    vs = vertical_theta1(sx,sy,sz,vx,vy,viz,epsilon);
  else if (epsilon*sz >= H) 
    vs =  vertical_theta2(sx,sy,sz,vx,vy,viz);
  return vs;
}

double KB3D::ground_speed_k(double sx, double sy, 
			    double vox, double voy,
			    double vix, double viy, int epsilon) {
  
  double k = 0;
  double a = sq(D)*(sq(vox)+sq(voy))- 
    sq(sx*voy - sy*vox);
  double b = 2*((sx*voy - sy*vox)*(sx*viy - sy*vix) - 
		sq(D)*(vox*vix + voy*viy));
  double c = sq(D)*(sq(vix)+ sq(viy))-
    sq(sx*viy - sy*vix);
  if (a == 0  && (c*b >= 0 || 
		  !tau_pos(sx,sy,-c/b*vox-vix,-c/b*voy-viy))) 
    k = 0;
  else if (a==0)
    k = -c/b;
  else if (discr(a,b,c) >= 0) {
    k = root(a,b,c,epsilon);
    if (k <= 0 || !tau_pos(sx,sy,k*vox-vix,k*voy-viy))
      k = 0;
  }
  return k;
}

/* ground_speed: Independent ground speed only maneuver
 *
 * Let gs = ground_speed(...) in
 * gs != NaR ==>
 *   gs = || kvx,kvy ||, where 
 *   kvx = k*vox, kvy = k*voy
 *   (kvx,kvy,voz) is an independent ground speed only solution 
 *   for the ownship.
 *
 * OUTPUTS: kvx,kvy 
 */

double KB3D::ground_speed(double sx, double sy, 
			  double vox, double voy,
			  double vix, double viy, int epsilon) {
  
  double k = ground_speed_k(sx,sy,vox,voy,vix,viy,epsilon);
  kvx = kvy = 0;
  if (k > 0 && eps_line(sx,sy,k*vox-vix,k*voy-viy) == epsilon) {
    kvx = k*vox;
    kvy = k*voy;
  } else {
    k = ground_speed_k(sx,sy,vox,voy,vix,viy,-epsilon);
    if (k > 0 && eps_line(sx,sy,k*vox-vix,k*voy-viy) == epsilon) {
      kvx = k*vox;
      kvy = k*voy;
    }
  }
  if (kvx != 0 || kvy != 0) 
    return sqrt(sq(kvx)+sq(kvy));
  return NaR;
}

void KB3D::track_vx_vy(double sx, double sy,
		       double vox, double voy,
		       double vix, double viy,
		       int epsilon) {
  vx = vy = 0;
  double  s2   = sq(sx)+sq(sy);
  double  v2   = sq(vox)+sq(voy);
  double  R    = D/sqrt(s2 - sq(D));
  double  sxy  = sx - epsilon*R*sy;
  double  syx  = sy + epsilon*R*sx;
  double  viyx = viy*sxy - vix*syx;
  double  a    = sq(s2)/(s2-sq(D));
  double  b    = 2*syx*viyx;
  double  c    = sq(viyx) - sq(sxy)*v2;
  if (sxy == 0 && (syx == 0 || v2 < sq(vix)))
    return;
  if (sxy == 0) {
    double  vy = sign(voy)*sqrt(v2-sq(vix));
    if (tau_pos(sx,sy,0,vy-viy)) {
      vx = vix;
      vy = vy;
    } else if (tau_pos(sx,sy,0,-vy-viy)) {
      vx = vix;
      vy = -vy;
    } else 
      return;
  } else if (discr(a,b,c) >= 0) {
    double  vx1 = root(a,b,c,1);
    double  vy1 = (viyx + syx*vx1)/sxy;
    double  vx2 = root(a,b,c,-1);
    double  vy2 = (viyx + syx*vx2)/sxy;
    bool    tp1 = tau_pos(sx,sy,vx1-vix,vy1-viy);
    bool    tp2 = tau_pos(sx,sy,vx2-vix,vy2-viy);
    if (tp1 && (! tp2 || 
		vx1*vox+vy1*voy > vx2*vox+vy2*voy)) {
      vx = vx1;
      vy = vy1; 
    } else if (tp2) {
      vx = vx2;
      vy = vy2; 
    } else
      return;
  }
}

/* track: Independent track only maneuver
 *
 * Let trk = heanding(...) in
 * trk != NaR ==>
 *   trk = atan2(vx,vy) and 
 *   (vx,vy,voz) is an independent track only solution for the ownship.
 *
 * REMARK: trk is in True North/clockwise convention.
 *
 * OUTPUTS: vx,vy
 */

double KB3D::track(double sx, double sy,
		   double vox, double voy,
		   double vix, double viy,
		   int epsilon) {
  track_vx_vy(sx,sy,vox,voy,vix,viy,epsilon);
  if (vx != 0 || vy != 0) 
    return atan2_safe(vx,vy);
  return NaR;
}

double  KB3D::alpha(double sx,double sy) {
  if (sx != 0 || sy != 0)
    return sq(D)/(sq(sx)+sq(sy));
  return 0;
}

double  KB3D::beta(double sx,double sy) {
  if (sx != 0 || sy != 0)
    return D*sqrt_safe(sq(sx)+sq(sy)-sq(D))/(sq(sx)+sq(sy));
  return 0;
}

double KB3D::Q(double sx,double sy, int epsilon) {
  return alpha(sx,sy)*sx+epsilon*beta(sx,sy)*sy;
}

double contact_time(double sx,double sy,double qx,double qy,
		    double vx,double vy) {
  double d = vx*(qx-sx) + vy*(qy-sy);
  if (d != 0)
    return (sq(qx-sx) + sq(qy-sy)) / d;
  if (qx == sx && qy == sy) 
    return 0;
  return -1;
}

void KB3D::optimal_vx_vy(double sx,double sy,
			 double vox,double voy,
			 double vix,double viy,
			 int epsilon) {
  ovx = ovy = 0;
  double vx  = vox-vix;    
  double vy  = voy-viy;
  double qpx = Q(sx,sy,epsilon);
  double qpy = Q(sy,sx,-epsilon);
  double tpq = contact_time(sx,sy,qpx,qpy,vx,vy);
  if (tpq > 0) {
    ovx = (qpx-sx)/tpq+vix;
    ovy = (qpy-sy)/tpq+viy;
  } else if (tpq == 0) {
    ovx = -sy*(sx*vy-vx*sy)+vix;
    ovy =  sx*(sx*vy-vx*sy)+viy;
  } 
}

/* optimal: Independent optimal track and ground speed only maneuver
 *
 * optimal(...) computes opt_trk and opt_gs, where
 * opt_trk != NaR and opt_gs != NaR ==>
 *   opt_trk = atan2(ovx,ovy) and opt_gs = || ovx,ovy || and
 *   (ovx,ovy,voz) is an optimal combined track and ground speed resolution
 *   for the the ownship.
 *
 * REMARK: opt_trk is in True North/clockwise convention.
 *
 * OUTPUTS: opt_trk,opt_gs,ovx,ovy
 */

void KB3D::optimal(double sx, double sy,
		   double vox, double voy,
		   double vix, double viy,
		   int epsilon) {
  opt_trk = opt_gs = NaR;
  optimal_vx_vy(sx,sy,vox,voy,vix,viy,epsilon);
  if ((ovx != 0 || ovy != 0) && 
      eps_line(sx,sy,ovx-vix,ovy-viy) == epsilon) {
    opt_trk = atan2_safe(ovx,ovy);
    opt_gs  = sqrt(sq(ovx)+sq(ovy));
    return;
  }
  optimal_vx_vy(sx,sy,vox,voy,vix,viy,-epsilon);
  if ((ovx != 0 || ovy != 0) && 
      eps_line(sx,sy,ovx-vix,ovy-viy) == epsilon) {
    opt_trk = atan2_safe(ovx,ovy);
    opt_gs  = sqrt(sq(ovx)+sq(ovy));
    return;
  }
  ovx = ovy = 0;
}

/* kb3d_vertical: Coordinated vertical maneuver.
 *
 * PRECONDITION:
 *   precondition(...) 
 *
 * POSTCONDITION:
 *   vs != NaR ==>
 *     (vox,voy,vs) is a coordinated vertical only resolution 
 *     for the ownship.
 *
 * OUTPUTS: vs
 *
 */

void KB3D::kb3d_vertical(double sx, double sy, double sz,
			 double vox, double voy, double voz,
			 double vix, double viy, double viz) {
  vs = NaR;
  if (precondition(sx,sy,sz,vox,voy,voz,vix,viy,viz)) {
    double pz = sz+cd3d->time2los*(voz-viz);
    vs = vertical(sx,sy,sz,vox,voy,voz,vix,viy,viz,
		  vertical_coordination(sx,sy,sz,pz));
  }
}
    
/* kb3d_horizontal: Coordinated horizontal maneuvers.
 *
 * PRECONDITION:
 *   precondition(...) &&
 *   sq(sx)+sq(sy) > sq(D)
 *
 * POSTCONDITION:
 *   gs != NaR ==>
 *     gs yields a coordinated ground speed only solution for the ownship.
 *   trk != NaR ==>
 *     trk yields a coordinated track only solution for the ownship.
 *   opt_trk != NaR and opt_gs != NaR ==>
 *     opt_trk and opt_gs yield a coordinated optimal combined track and ground
 *     speed solution for the ownship.
 *
 * REMARK: trk and opt_tr are in True North/clockwise convention.
 *
 * OUTPUTS: trk,gs,opt_gs,opt_trk
 *
 */

void KB3D::kb3d_horizontal(double sx, double sy, double sz,
			   double vox, double voy, double voz,
			   double vix, double viy, double viz) {
  
  gs = trk = opt_trk= opt_gs = NaR;
  
  if (precondition(sx,sy,sz,vox,voy,voz,vix,viy,viz) &&
      sq(sx)+sq(sy) > sq(D)) {
    int epsilon = horizontal_coordination(sx,sy,vox-vix,voy-viy);
    gs  = ground_speed(sx,sy,vox,voy,vix,viy,epsilon);
    trk = track(sx,sy,vox,voy,vix,viy,epsilon);
    optimal(sx,sy,vox,voy,vix,viy,epsilon);
  }
}

/* kb3d: Coordinated horizontal and vertical maneuvers 
 *
 * PRECONDITION:
 *   precondition(...) &&
 *   sq(sx)+sq(sy) > sq(D)
 *
 * POSTCONDITION:
 *   vs != NaR ==>
 *     (vox,voy,vs) is a coordinated vertical only resolution 
 *     for the ownship.
 *   gs != NaR ==>
 *     gs yields a coordinated ground speed only solution for the ownship.
 *   trk != NaR ==>
 *     trk yields a coordinated track only solution for the ownship.
 *   opt_trk != NaR and opt_gs != NaR ==>
 *     opt_trk and opt_gs yield a coordinated optimal combined track and ground
 *     speed solution for the ownship.
 *
 * REMARK: trk and opt_tr are in True North/clockwise convention.
 *
 * OUTPUTS: trk,gs,opt_trk,opt_gs,vs
 *
 */

void KB3D::kb3d(double sx, double sy, double sz,
		double vox, double voy, double voz,
		double vix, double viy, double viz) {
  kb3d_vertical(sx,sy,sz,vox,voy,voz,vix,viy,viz);
  kb3d_horizontal(sx,sy,sz,vox,voy,voz,vix,viy,viz);
}
