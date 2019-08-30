/*
 * CDR.cpp
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 * Simple interface to CD3D and KB3D. State information for each aircraft is
 * given as:
 * lat [deg], lon [deg], alt [feet], trk [deg], gs [knots], vs [feet/min]
 *
 * D [nm]   : Horizontal separation  
 * H [feet] : Vertical separation
 * T [sec]  : Lookahead time
 *
 * REMARK: East longitudes are positive.
 *         Degrees are in True North/clockwise convention.
 *
 * BASIC USAGE:
 *   // Create a CDR object
 *   CDR* cdr = new CDR(D,H,T,
 *                      lat_o,lon_o,alt_o,trk_o,gs_o,vs_o,
 *                      lat_i,lon_i,alt_i,trk_i,gs_i,vs_i);
 *
 *  // Check if there is a conflict
 *  if (cdr->detection()) {
 *    // Check if there is a violation
 *     if (!cdr->violation()) {
 *       // Call resolution algorithm
 *       cdr->resolution();
 *       ...
 *     }
 *  ...
 * }
 */

#include <cmath>
#include <string>
#include "Util.h"
#include "CD3D.h"
#include "KB3D.h"
#include "LoS.h"
#include "CDR.h"
#include "Geodesic.h"

CDR::CDR(double D,       // [nm]
	 double H,       // [feet]
	 double T,       // [sec]
	 double x_o,     // [deg or nm]
         double y_o,     // [deg or nm]
         double alt_o,   // [feet]
         double trk_o,   // [deg] True North/clockwise
	 double gs_o,    // [knots]
	 double vs_o,    // [feet/min]
         double x_i,     // [deg or nm]
         double y_i,     // [deg or nm]
         double alt_i,   // [feet]
         double trk_i,   // [deg] True North/clockwise
	 double gs_i,    // [knots]
	 double vs_i,    // [feet/min]
	 bool gxy) {     // true: Geodesic, false: Rectangular

  // Converts all distance units to meters, all time units to seconds, 
  // all angles to radians, and all speeds to meters per second

  d = nm2m(D);
  h = ft2m(H);
  t = T;

  cd = new CD3D(d,h,t);
  cr = new KB3D(d,h,t);

  if (gxy) {
    Geodesic* geo = new Geodesic(); 
    geo->geo2xy(x_o,y_o,x_i,y_i);
    sx = geo->sx;
    sy = geo->sy;
    distance = rad2deg(geo->gcd)*60.0;
    course   = rad2deg(geo->tc);
    delete geo;
  } else {
    sx = nm2m(x_o - x_i);
    sy = nm2m(y_o - y_i);
    distance = sqrt_safe(sq(x_i - x_o) + sq(y_i - y_o));
    course = rad2deg(atan2_safe(x_i - x_o,y_i - y_o));
  }

  sz = ft2m(alt_o-alt_i);
  double gs   = knots2msec(gs_o);
  double trk  = deg2rad(trk_o);
  vox = gs2vx(gs,trk);
  voy = gs2vy(gs,trk);
  voz = ftmin2msec(vs_o);
  
  gs  = knots2msec(gs_i);
  trk = deg2rad(trk_i);
  vix = gs2vx(gs,trk);
  viy = gs2vy(gs,trk);
  viz = ftmin2msec(vs_i);

  time2los = 0;
  t_in = 0;
  t_out = 0;
  duration = 0;
  filter = 1;
  recovery = 0;
  newtrk = newgs = newvs = opttrk = optgs = NaR;
}

CDR::~CDR() {
  delete cd;
  delete cr;
}

bool CDR::violation() const {
  cd->set_D(d);
  cd->set_H(h);
  cd->set_T(t);
  return cd->violation(sx,sy,sz);
}

bool CDR::detection() {
  cd->set_D(d);
  cd->set_H(h);
  cd->set_T(t);
  cd->set_filter(filter);
  cd->cd3d(sx,sy,sz,vox-vix,voy-viy,voz-viz);
  time2los = cd->time2los;
  time2lhs = cd->time2lhs;
  time2lvs = cd->time2lvs;
  duration = cd->duration;
  t_in = cd->t_in;
  t_out = cd->t_out;
  return cd->conflict;
}

void CDR::resolution() {
  CD3D* cdnow = new CD3D(d,h,t);
  cr->set_D(d);
  cr->set_H(h);
  cr->set_T(t);
  recovery = 0;

  if (cdnow->cd3d(sx,sy,sz,vox-vix,voy-viy,voz-viz)) {
    if (!cdnow->violation(sx,sy,sz)) { // aircraft are separated
      cr->kb3d(sx,sy,sz,vox,voy,voz,vix,viy,viz);
      if (cr->trk != NaR) { 
	recovery = 1;
	newtrk = rad2deg(cr->trk);
      }
      if (cr->gs != NaR) {
	recovery = 1;
	newgs = msec2knots(cr->gs);
      }
      if (cr->opt_trk != NaR) {
	recovery = 1;
	opttrk = rad2deg(cr->opt_trk);
	optgs = msec2knots(cr->opt_gs);
      }
      if (cr->vs != NaR) {
	recovery = 1;
	newvs = msec2ftmin(cr->vs);
      }
    } else { // aircraft are in violation
      recovery = -1;
      newvs = msec2ftmin(vertical_recovery(sx,sy,sz,voz,viz,h,fabs(cdnow->t_in)));
    }
  }
}

void CDR::set_DHT(double D,   // [nm] 
		  double H,   // [ft]
		  double T) { // [sec]
  d = nm2m(D);
  h = ft2m(H);
  t = T;
}

double CDR::get_D() const { // [nm]
  return m2nm(d);
}

double CDR::get_H() const { // [feet]
  return m2ft(h);
}

double CDR::get_T() const { // [sec]
  return t;
}

void CDR::set_detection_filter(double f) { 
  filter = f;
}

double CDR::get_detection_filter() const {
  return filter;
}

