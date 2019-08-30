/*
 * main.cc
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 */

#include <iostream>
#include "Util.h"
#include "CD3D.h"
#include "KB3D.h"
#include "CDR.h"

using namespace std;

string  kb3d_version = "KB3D++-3.a (03/28/08)";

int main(int argc, char* argv[]) {
  
  double D = 5;    // 5 nautical miles
  double H = 1000; // 1000 ft
  double T = 300;  // 5 minutes

  cout << kb3d_version << "\n\n"; 
  
 // Create a CDR object
  CDR* cdr = 
    new CDR(D,H,T,
	    // ownship : lon [deg] lat[deg] alt[feet] trk[deg] gs[knots] vs [ft/min]
	                     10,    -0.383,    11500,   96,      253,      -500,
	    // traffic: lon [deg] lat[deg] alt[feet] trk[deg] gs[knots] vs [ft/min]
	                     10,      0,       10000,  287,     316,       200);
  cout << "Distance: " 
       << cdr -> distance << " [nm]\n"
       << "Course: " 
       << cdr -> course << " [deg]\n";
  // Check if there is a conflict
  if (cdr->detection()) {
    // Check if there is a violation
    if (cdr->violation()) {
      cout << "Aircraft are in violation\n";
      cout << "Time of entry: " << cdr->t_in << " [sec]\n";
      cout << "Time of exit: " << cdr->t_out << " [sec]\n";
    }
    else {  
        cout << "Time to loss of separation: " << cdr->time2los << " [sec]\n";
	cout << "Time to loss of horizontal separation: "
	     << cdr->time2lhs << " [sec]\n";
	cout << "Time to loss of vertical separation: " 
	     << cdr->time2lvs << " [sec]\n";
	cout << "Duration of conflict: " << cdr->duration << " [sec]\n";
    }
    // Call resolution algorithm
    cdr->resolution();
    if (cdr->newtrk != NaR) 
      cout << "Track Only = " << cdr->newtrk << " [deg]\n";
    if (cdr->newgs != NaR) 
      cout << "Ground Speed Only = " << cdr->newgs << " [knots]\n";
    if (cdr->opttrk != NaR) 
      cout << "Optimal Track = " << cdr->opttrk << " [deg]\n";
    if (cdr->optgs != NaR) 
      cout << "Optimal Ground Speed = " << cdr->optgs << " [knots]\n";
    if (cdr->newvs != NaR) 
      cout << "Vertical Speed Only = " << cdr->newvs << " [ft/min]\n";
  } else 
    cout << "No predicted conflict in " << T << " [sec] (filter: "
	 << cdr->get_detection_filter() << " [sec])\n";
  delete cdr;
}
