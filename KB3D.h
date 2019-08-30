/*
 * KB3D.h 
 * Release: KB3D++-3.a (03/28/08)
 *
 * Contact: Cesar Munoz (munoz@nianet.org)
 * National Institute of Aerospace
 * http://research.nianet.org/fm-at-nia/KB3D
 *
 */

class KB3D {
 public: 

  // Outputs
  double vs;      // Vertical speed (computed by kb3d and kb3d_vertical)
  double gs,trk;  // Ground speed and track (computed by kb3d and 
                  // kb3d_horizontal)
  double opt_gs,
         opt_trk; // Optimal ground speed and track (computed by kb3d and 
                  // kb3d_horizontal)

  KB3D(double d, double h, double t);
  ~KB3D();

  void   set_D(double d);
  double get_D() const;
  void   set_H(double d);
  double get_H() const;
  void   set_T(double t);
  double get_T() const;
  bool   precondition(double sx, double sy, double sz,
		      double vox, double voy, double voz,
		      double vix, double viy, double viz);
  int    vertical_coordination(double sx, double sy, double sz, double pz);
  int    horizontal_coordination(double sx, double sy, 
                                 double vx, double vy);
  double vertical(double sx, double sy, double sz,
		  double vox, double voy, double voz,
		  double vix, double viy, double viz,
		  int epsilon);  
  double ground_speed(double sx, double sy, 
		      double vox, double voy,
		      double vix, double viy, int epsilon); 
  double track(double sx, double sy,
	       double vox, double voy,
	       double vix, double viy,
	       int epsilon);
  void optimal(double sx, double sy,
	       double vox, double voy,
	       double vix, double viy,
	       int epsilon);
  void   kb3d_vertical(double sx, double sy, double sz,
		       double vox, double voy, double voz,
		       double vix, double viy, double viz);
  void   kb3d_horizontal(double sx, double sy, double sz,
			 double vox, double voy, double voz,
			 double vix, double viy, double viz);
  void   kb3d(double sx, double sy, double sz,
	      double vox, double voy, double voz,
	      double vix, double viy, double viz);

  double kvx,kvy; // From ground speed
  double vx,vy;   // From track
  double ovx,ovy; // From optimal ground speed and track
    
 private:
  
  // Horizontal (H) and vertical (H) separation
  double D,H;
  
  // Lookahead time
  double T;
  
  // To call conflict detection algorithm in precondition
  CD3D* cd3d;
  
  double delta(double sx,double sy, double vx, double vy);
  double theta(double sx,double sy,double vx,double vy,int eps);
  double vertical_theta1(double sx, double sy, double sz,
			 double vx, double vy, double viz,
			 int eps);
  double vertical_theta2(double sx, double sy, double sz,
			 double vx, double vy, double vz);
  double ground_speed_k(double sx, double sy, 
			double vox, double voy,
			double vix, double viy, int epsilon);
  double alpha(double sx, double xy);
  double beta(double sx, double xy);
  double Q(double sx,double sy, int epsilon);
  void optimal_vx_vy(double sx,double sy,
		     double vox,double voy,
		     double vix,double viy,
		     int epsilon);
  void   track_vx_vy(double sx, double sy,
		       double vox, double voy,
		       double vix, double viy,
		       int epsilon);
};

int     break_symm(double sx, double sy, double sz);
double  tau(double sx, double sy, double vx, double vy); 
bool    tau_pos(double sx, double sy, double vx, double vy);
int     eps_line(double sx, double sy, double vx, double vy);
