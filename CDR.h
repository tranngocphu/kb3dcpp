/*
 * CDR.h 
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 */

class CDR {
  public:

  // CDR outputs
  double distance;  // Horizontal distance between aircraft [nm]
  double course;    // Course from ownship to traffic aircraft [deg]
  int    recovery;  // Type of recovery: -1 : violation, 0: none, 1: conflict

  // Conflict Detection Outputs
  double  time2los;  // Time to loss of separation [sec]
  double  time2lhs;  // Time to loss of horizontal separation [sec]
  double  time2lvs;  // Time to loss of vertical separation [sec]
  double  duration;  // Duration of loss of separation [sec]
  double  t_in;      // Time of entry (protected zone) [sec]
  double  t_out;     // Time of exit (protected zone) [sec]

  // Conflict Resolution Outputs
  double  newtrk;       // Independent track [deg]
  double  newgs;        // Independent ground speed [knots]
  double  newvs;        // Independent vertical speed [feet/min]
  double  opttrk,optgs; // Combined optimal track [deg] and 
                        // ground speed [knots]

  CDR(double D,        // [nm]
      double H,        // [feet]
      double T,        // [sec]  
      double x_o,      // [deg or nm]
      double y_o,      // [deg or nm]
      double alt_o,    // [feet]
      double trk_o,    // [deg] True North/clockwise
      double gs_o,     // [knots]
      double vs_o,     // [feet/min]
      double x_i,      // [deg or nm]
      double y_i,      // [deg or nm]
      double alt_i,    // [feet]
      double trk_i,    // [deg] True North/clockwise
      double gs_i,     // [knots]
      double vs_i,     // [feet/min]
      bool   gxy=true);// true: Geodesic, false: Rectangular
  ~CDR();

  bool violation() const;
  
  bool detection();

  void resolution();
  
  void   set_DHT(double D,  // [nm]
		 double H,  // [feet]
		 double T); // [sec]
  
  double get_D() const;        // [nm]
  double get_H() const;        // [feet]
  double get_T() const;        // [sec
  
  void   set_detection_filter(double f); // [sec] 
  double get_detection_filter() const;   // [sec]

  // private:   Should be private. Left public for debugging
  CD3D *cd;
  KB3D *cr;
  double d,h,t,sx,sy,sz,vox,voy,voz,vix,viy,viz;
  double filter;

};
