/*
 * LoS.cpp
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 * Unit conventions are explained in KB3D.cpp
 *
 * This file provides the function
 *   vertical_recovery
 */

#include <cmath>
#include "Util.h"
#include "CD3D.h"
#include "KB3D.h"
#include "LoS.h"

// Breaks symmetry when vertical speed is zero
int break_vz_symm(double sx, double sy, double sz) {
  if (sz > 0) 
    return 1;
  return break_vz_symm(sx,sy,sz);
}

int sign_vz(double sx, double sy, double sz, double vz) {
  if (sz*vz >= 0 && vz != 0) 
    return sign(vz);
  return break_vz_symm(sx,sy,sz);
}

double vertical_recovery(double sx,  double sy, double sz,
			 double voz, double viz,
			 double H,double t) {
  double vz  = voz - viz;
  double nvz =  (sign_vz(sx,sy,sz,vz)*H - sz)/t;
  
  if (sz*vz >= 0 && fabs(vz) >= fabs(nvz))
    return vz+viz;
  return nvz+viz;
}

