/**
 * @brief ROS node for Distributed LP class.
 * @file dlp_node.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 */
/*
 * Copyright 2017 Mohamed Abdelkader.
 *
 * This file is part of the DLP package and subject to the license terms
 * in the LICENSE file of the DLP repository.
 * https://github.com/mzahana/DLP.git
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "dlp.h"

#include "dlp/DefendersState.h"
#include "dlp/DlpState.h"
#include "dlp/EnemyState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/NavSatFix.h"

#define PI 3.14159265
#define M_DEG_TO_RAD 0.01745329251994
#define CONSTANTS_RADIUS_OF_EARTH 6371000.0
//#define DBL_EPSILON 2.2204460492503131E-16



/**
 * TODO
 * add a ROS paramter that is used checks if local state estimate (of defenders) is required
 * WHY origin shifts are hardcoded to zero in line 507 ??????
 */

/**
* Callbacks class
*/
class CallBacks
{
public:
	dlp::DefendersState		d_loc_msg;
	dlp::EnemyState			e_loc_msg;
	geometry_msgs::PoseStamped 	local_enu_msg;
	sensor_msgs::NavSatFix 		gps_msg;
	float 				current_lat;
	float 				current_long;

	 CallBacks() { }
	~CallBacks() { }

	// Defenders locations callback
	void d_loc_cb(const dlp::DefendersState::ConstPtr& msg)
	{
		d_loc_msg.header			= msg->header;
		d_loc_msg.defenders_count		= msg->defenders_count;
		d_loc_msg.defenders_sectors		= msg->defenders_sectors;
		d_loc_msg.defenders_position		= msg->defenders_position;
	}

	// Enemy locations callback
	void e_loc_cb(const dlp::EnemyState::ConstPtr& msg)
	{
		e_loc_msg.header	= msg->header;
		e_loc_msg.enemy_count	= msg->enemy_count;
		e_loc_msg.enemy_sectors	= msg->enemy_sectors;
		e_loc_msg.enemy_position = msg->enemy_position;
		e_loc_msg.is_captured	= msg->is_captured;
	}

	// my local ENU position, from mavros.

	void local_enu_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

		local_enu_msg.header		= msg->header;
		local_enu_msg.pose.position	= msg->pose.position;
		/*
		cout << "cb3 \n";
		cout << "local_pos: " << local_enu_msg.pose.position.x << " "
								<< local_enu_msg.pose.position.y << " "
								<< local_enu_msg.pose.position.z << "\n";
		*/

		double v_x = msg->pose.position.x;
		double v_y = msg->pose.position.y;
		double v_z = msg->pose.position.z;
	}
	
	// current gps coordinates
	void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
		gps_msg.header		= msg->header;
		gps_msg.latitude	= msg->latitude;
		gps_msg.longitude	= msg->longitude;
		gps_msg.altitude	= msg->altitude;

		current_lat		= gps_msg.latitude;
		current_long		= gps_msg.longitude;
	}


};

/*
***************
* Helpers Class
***************
*/
class Helpers{
public:
	float lat0, lon0, lat, lon;
	float dx, dy;
	// local coords
	float dx_L, dy_L;

	// lat/long of east corner w.r.t lat0,lon0
	float PE_lat, PE_long;
	// needed rotation to project on local grid, [rad]
	float local_rot;
	// grid side length
	float grid_side_length;

	 Helpers() { }
	~Helpers() { }

	void compute_local_rot(){
		// get dx/dy in global ENU
		LLA2ENU(lat0, lon0, PE_lat, PE_long);
		// get rotation
		local_rot = atan2(dy,dx);
	}

	/**
	* computes the length of the grid side. Assuming square grid
	* it uses the East point PE_lat/PE_long, and the origin lat0/lon0 to compute that
	*/
	void compute_grid_side_length(){
		LLA2ENU(lat0, lon0, PE_lat, PE_long);
		grid_side_length = sqrt(dx*dx + dy*dy);
	}

	/**
	* converts the GPS coordinates of target lat/lon w.r.t to lat0/lon0
	* to global ENU.
	* Then,to local ENU frame, it uses local_rot
	* assumes local_rot is computed using compute_local_rot()
	* assumes that target GPS is stored in lat,lon
	*/
	void global2local_ENU(){
		compute_local_rot();
		LLA2ENU(lat0, lon0, lat, lon);
		dx_L = cos(local_rot)*dx + sin(local_rot)*dy;
		dy_L = -1*sin(local_rot)*dx + cos(local_rot)*dy;

		dx = dx_L; dy = dy_L;
	}

	/**
	* converts local ENU coordinates dx_L, dy_L
	* to global ENU.
	* Then converts it to GPS w.r.t lat0/lon0
	*/
	void local2global_GPS(){
		compute_local_rot();
		dx = cos(local_rot)*dx_L - sin(local_rot)*dy_L;
		dy = sin(local_rot)*dx_L + cos(local_rot)*dy_L;

		// x/y swtiched as they are in ENU not NED (what the function expects)
		NED2LLA(lat0, lon0, dy, dx);
	}

	void LLA2ENU(float lat_0, float lon_0, float lat, float lon){


		float curr_lat_rad = lat_0 * M_DEG_TO_RAD;
		float curr_lon_rad = lon_0 * M_DEG_TO_RAD;
		float curr_sin_lat = sin(curr_lat_rad);
		float curr_cos_lat = cos(curr_lat_rad);

		float lat_rad = lat * M_DEG_TO_RAD;
		float lon_rad = lon * M_DEG_TO_RAD;

		float sin_lat = sin(lat_rad);
		float cos_lat = cos(lat_rad);

		float cos_d_lon = cos(lon_rad - curr_lon_rad);

		float arg = curr_sin_lat * sin_lat + curr_cos_lat * cos_lat * cos_d_lon;

		if (arg > 1.0)
			arg = 1.0;
		else if (arg < -1.0)
			arg = -1.0;

		float c = acos(arg);
		float k;
		if(abs(c) < DBL_EPSILON)
			k=1;
		else
			k=c/sin(c);

		dx = k * cos_lat * sin(lon_rad - curr_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
		dy = k * (curr_cos_lat * sin_lat - curr_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	}
	// NED -> Lat/Long
	void NED2LLA(float lat_0, float lon_0, float delta_x, float delta_y){
		float curr_lat_rad = lat_0 * M_DEG_TO_RAD;
		float curr_lon_rad = lon_0 * M_DEG_TO_RAD;
		float curr_sin_lat = sin(curr_lat_rad);
		float curr_cos_lat = cos(curr_lat_rad);

		float x_rad = delta_x / CONSTANTS_RADIUS_OF_EARTH;
		float y_rad = delta_y / CONSTANTS_RADIUS_OF_EARTH;
		float c = sqrt(x_rad * x_rad + y_rad * y_rad);
		float sin_c = sin(c);
		float cos_c = cos(c);

		float lat_rad, lon_rad;

		if (abs(c) > DBL_EPSILON){
			lat_rad = asin(cos_c * curr_sin_lat + (x_rad * sin_c * curr_cos_lat) / c);
			lon_rad = (curr_lon_rad + atan2(y_rad * sin_c, c * curr_cos_lat * cos_c - x_rad * curr_sin_lat * sin_c));
		}

		else{
			float lat_rad = curr_lat_rad;
			float lon_rad = curr_lon_rad;
		}

		lat = lat_rad * 180.0 / PI;
		lon = lon_rad * 180.0 / PI;
	}
	
}; 

int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "dlp_node");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	*/
	ros::NodeHandle nh;


	/**
	* Retrieve game paramters from ROS paramters server.
	* If a requested paramter is not set, it will be assigned default value.
	*/
	bool bDebug;
	nh.param("debug", bDebug, false);
	int myID, rows, cols;
	nh.param("myID", myID, 0);

	/* Centralized vz distributed approaches*/
	bool bGlobalComms;
	nh.param("global_comms", bGlobalComms, false);

	/* Global vs. local enemy sensing */
	bool bLocalSensing;
	nh.param("local_sensing", bLocalSensing, false);

	/* using local estimates of all defenders states */
	bool bUseLocalEstimate;
	nh.param("use_local_estimates", bUseLocalEstimate, true);

	std::vector<int> grid_size, default_grid_size;
	// default to 7x7 grid
	default_grid_size.push_back(7); default_grid_size.push_back(7);
	nh.param< vector<int> >("grid_size", grid_size, default_grid_size);

	int Nd, Ne;
	nh.param("N_defenders", Nd, 3);
	nh.param("N_attackers", Ne, 2);

	int nBase, nBaseRef;
	nh.param("nBase", nBase, 1);
	nh.param("nBaseRef", nBaseRef, 3);

	std::vector<int> base, baseRef, default_base, default_baseRef;
	default_base.push_back(1);
	default_baseRef.push_back(2); default_baseRef.push_back(8); default_baseRef.push_back(9);
	nh.param< vector<int> >("Base", base, default_base);
	nh.param< vector<int> >("BaseRef", baseRef, default_baseRef);

	/**	
	* loading static obstacles ROS parameters
	*/
	int N_static_obs;
	nh.param("Nobs_sectors", N_static_obs, 3);
	std::vector<int> obstacles_set, obstacles_set_default;
	obstacles_set_default.push_back(1); obstacles_set_default.push_back(2); obstacles_set_default.push_back(3);
	nh.param< vector<int> >("obs_sectors", obstacles_set, obstacles_set_default);

	std::vector<float> d_velocity, default_d_velocity;
	default_d_velocity.push_back(1.0); default_d_velocity.push_back(1.0); default_d_velocity.push_back(1.0);
	nh.param< vector<float> >("d_velocity", d_velocity, default_d_velocity);

	int Tp;
	nh.param("Tp", Tp, 2);

	int Nr;
	nh.param("neighbor_radius", Nr, 1);

	// set strategy weights, alpha, beta
	float alpha, beta;
	nh.param<float>("alpha", alpha, -0.99);
	nh.param<float>("beta", beta, -0.01);

	std::vector<float> origin_shifts, default_shifts;
	default_shifts.push_back(0.0); default_shifts.push_back(0.0);
	nh.param< vector<float> >("origin_shifts", origin_shifts, default_shifts);

	std::vector<float> sector_size, default_sector_size;
	default_sector_size.push_back(1.0); default_sector_size.push_back(1.0);
	nh.param< vector<float> >("sector_size", sector_size, default_sector_size);

	float altitude_setpoint;
	nh.param<float>("altitude_setpoint", altitude_setpoint, 1.0);

	float update_freq;
	nh.param<float>("update_freq", update_freq, 50.0);

	bool use_gps;
	nh.param("use_gps", use_gps, false);

	bool use_sim;
	nh.param("use_sim", use_sim, false);

	float p_lat0;
	nh.param<float>("lat0", p_lat0, 47.397742);

	float p_long0;
	nh.param<float>("long0", p_long0, 8.5455933);

	/**
	* get grid corners parameters
	*/
	bool use_grid_corners;
	nh.param("use_grid_corners", use_grid_corners, false);
	std::vector<float> Po, PE, PN, PNE, corner_default;
	corner_default.push_back(10.0); corner_default.push_back(20.0);// default lattitude/longitude
	nh.param< vector<float> >("grid_corner_Po", Po, corner_default);
	nh.param< vector<float> >("grid_corner_PE", PE, corner_default);
	nh.param< vector<float> >("grid_corner_PN", PN, corner_default);
	nh.param< vector<float> >("grid_corner_PNE", PNE, corner_default);

	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	ros::Publisher dlp_state_pub = nh.advertise<dlp::DlpState>("dlp_state", 1);

	/*
	* Objects instantiations
	*/
	// object of Helpers calss
	Helpers hp;
	CallBacks cb;
	
	hp.lat0 = p_lat0; hp.lon0 = p_long0;

	/**
	* For outdoor use, we assume square grid.
	* We assume the origin and East GPS points are provided
	* a square grid is created based on this.
	* The number of sectors in each dimension is the same
	* grid_size paramter should contain the same number for rows and columns
	* only the first element is used (number of rows)
	*/

	if (use_gps && use_grid_corners){
		hp.lat0 = Po[0]; hp.lon0 = Po[1];
		//East corner
		hp.PE_lat = PE[0]; hp.PE_long = PE[1];

		// compute required rotation to project on local grid defined by origin and an East point
		hp.compute_local_rot();
		// assume square grid, use origin point and east pont
		hp.compute_grid_side_length();
		// update sector size. It's the same for rows and columns in outdoor test
		grid_size[1] = grid_size[0];
		sector_size[0] = hp.grid_side_length / grid_size[0];
		sector_size[1] = sector_size[0];
		origin_shifts[0] = 0.0; origin_shifts[1] = 0.0;
	}


	/**
	* Subscribers
	*/

	ros::Subscriber d_loc_sub = nh.subscribe("/defenders_locations", 1, &CallBacks::d_loc_cb, &cb);
	ros::Subscriber e_loc_sub = nh.subscribe("/enemy_locations", 1, &CallBacks::e_loc_cb, &cb);
	ros::Subscriber local_enu_sub = nh.subscribe("mavros/local_position/pose", 1, &CallBacks::local_enu_cb, &cb);
	ros::Subscriber gps_sub = nh.subscribe("mavros/global_position/global", 1, &CallBacks::gps_cb, &cb);

	ros::Rate loop_rate(update_freq);

	/**
	* Create DLP problem and set parameters.
	*/
	DLP problem;
	problem.DEBUG = bDebug;
	problem.set_myID(myID);
	
	/* set local vs global enemy sensing*/
	problem.set_local_attacker_sensing(bLocalSensing);

	problem.set_nRows(grid_size[0]);
	problem.set_nCols(grid_size[1]);
	problem.set_grid_resolution(sector_size[0], sector_size[1]);
	problem.set_origin_shifts(origin_shifts[0], origin_shifts[1]);

	problem.set_Tp(Tp);
	problem.set_Nd(Nd);
	problem.set_Ne(Ne);

	/* update time stamp */
	problem.set_dt(1.0/update_freq);

	MatrixXf d_velocity_M = MatrixXf::Constant(Nd,1,1.0);
	for (int i=0; i<Nd; i++)
		d_velocity_M(i,0) = d_velocity[i];

	problem.set_d_velocity(d_velocity_M);

	MatrixXf Base(nBase,1);
	for (int i=0; i<nBase; i++){
		Base(i,0) = base[i];// base filled from ros params
	}
	MatrixXf BaseRef(nBaseRef,1);
	for (int i=0; i<nBaseRef; i++){
		BaseRef(i,0) = baseRef[i];// baseRef filled from ros params
	}

	problem.set_BaseRef(nBaseRef, BaseRef);
	problem.set_Base(nBase, Base);

	// set strategy weights
	problem.set_weights(alpha, beta);

	MatrixXf eloc(Ne,1);
	MatrixXf temp_eloc(Ne,1);
	MatrixXf e_predicted_loc(Ne,1);
	MatrixXf dloc(Nd,1);

	MatrixXf d_current_position(3,Nd);

	MatrixXf sensedN;
	MatrixXf neighbors_next_loc;


	MatrixXf next_loc; /* next optimal locations */

	MatrixXf enu(3,1);
	int sector_from_enu;


	clock_t start, end;
	

	// Initialize defenders/enemy messages
	for (int i=0; i<Nd; i++){
		dloc(i,0) = i+1;
	}

	for (int i=0; i<Ne; i++){
		eloc(i,0) = i+Nd+1;
	}
	
	cb.d_loc_msg.defenders_count = Nd;
	cb.e_loc_msg.enemy_count = Ne;
	cb.d_loc_msg.defenders_sectors.resize(Nd);
	cb.d_loc_msg.defenders_position.resize(Nd);
	for (int i=0; i<Nd; i++){
		cb.d_loc_msg.defenders_sectors[i] = (int)dloc(i,0);
		enu = problem.get_ENU_from_sector(dloc(i,0));
		cb.d_loc_msg.defenders_position[i].x = enu(0,0);
		cb.d_loc_msg.defenders_position[i].y = enu(1,0);
		cb.d_loc_msg.defenders_position[i].z = altitude_setpoint;
	}

	cb.e_loc_msg.enemy_position.resize(Ne);
	cb.e_loc_msg.enemy_sectors.resize(Ne);
	cb.e_loc_msg.is_captured.resize(Ne);
	for (int i=0; i<Ne; i++){
		cb.e_loc_msg.enemy_sectors[i] = (int)eloc(i,0);
		enu = problem.get_ENU_from_sector(eloc(i,0));
		cb.e_loc_msg.enemy_position[i].x = enu(0,0);
		cb.e_loc_msg.enemy_position[i].y = enu(1,0);
		cb.e_loc_msg.enemy_position[i].z = altitude_setpoint;
		cb.e_loc_msg.is_captured[i] = false;
	}
	enu = problem.get_ENU_from_sector(dloc(myID,0));
	/*
	cb.local_enu_msg.header.stamp = ros::Time::now();
	cb.local_enu_msg.pose.position.x = (double)enu(0,0);
	cb.local_enu_msg.pose.position.y = (double)enu(1,0);
	cb.local_enu_msg.pose.position.z = (double)enu(2,0);
	*/

	problem.set_d_current_locations(dloc);
	problem.set_e_current_locations(eloc);
	problem.set_my_current_location(dloc(myID,0));

	/* set static obstacles sectors */
	MatrixXf obstacles_sectors(N_static_obs,1); 
	for (int i=0; i< N_static_obs; i++)
		obstacles_sectors(i,0) = obstacles_set[i];
	
	problem.set_static_obstacles(N_static_obs, obstacles_sectors);

	problem.setup_problem();


	/* Main loop. */
	while (ros::ok())
	{

		//ROS_INFO("%s", msg.data.c_str());

		/**
		* The publish() function is how you send messages. The parameter
		* is the message object. The type of this object must agree with the type
		* given as a template parameter to the advertise<>() call, as was done
		* in the constructor above.
		*/
		//chatter_pub.publish(msg);

		start = clock();


		// get defenders locations
		dloc = MatrixXf::Constant(Nd,1,0.0);
		for (int i=0; i< cb.d_loc_msg.defenders_count; i++){
			enu(0,0) = cb.d_loc_msg.defenders_position[i].x;
			enu(1,0) = cb.d_loc_msg.defenders_position[i].y;
			enu(2,0) = cb.d_loc_msg.defenders_position[i].z;
			d_current_position(0,i) = enu(0,0); /* x */
			d_current_position(1,i) = enu(1,0); /* y */
			d_current_position(2,i) = enu(2,0); /* z */
			dloc(i,0) =problem.get_sector_from_ENU(enu);
		}
		problem.set_d_current_position(d_current_position);
		if (problem.DEBUG)
			cout << "[dlp_node] [loop] d_loc:  " << dloc.transpose() << "\n";

		problem.set_d_current_locations(dloc);

		// get enemy locations
		eloc = MatrixXf::Constant(Ne,1,0.0);
		for (int i=0; i< cb.e_loc_msg.enemy_count; i++){
			
			if (cb.e_loc_msg.is_captured[i]){
				eloc(i,0) = 0.0;
			}
			else{
				enu(0,0) = cb.e_loc_msg.enemy_position[i].x;
				enu(1,0) = cb.e_loc_msg.enemy_position[i].y;
				enu(2,0) = cb.e_loc_msg.enemy_position[i].z;
				eloc(i,0) =problem.get_sector_from_ENU(enu);
			}
		}
		if (problem.DEBUG)
			cout << "[dlp_node] [loop] e_loc:  " << eloc.transpose() << "\n";
		problem.set_e_current_locations(eloc);
		
		// set my current location
		if (use_gps){
			if(use_grid_corners){
				// update sector size. It's the same for rows and columns in outdoor test
				grid_size[1] = grid_size[0];
				hp.compute_grid_side_length();
				sector_size[0] = hp.grid_side_length / grid_size[0];
				sector_size[1] = sector_size[0];
			}
			// TODO
			// convert my gps coords to ENU; ignore height (not used)
			// result stored in hp.dx, hp.dy
			if (use_grid_corners)
				hp.global2local_ENU();
			else
				hp.LLA2ENU(hp.lat0, hp.lon0, cb.current_lat, cb.current_long);

			// bound the result
			if (hp.dx > (grid_size[0]*sector_size[0]) ){
				ROS_WARN("current x= %f is out of grid bounds. Theresholded.", hp.dx);
				hp.dx = grid_size[0]*sector_size[0] - (sector_size[0]/2.0);
			}
			if (hp.dx < 0 ){
				ROS_WARN("current x= %f is out of grid bounds. Theresholded.", hp.dx);
				hp.dx = (sector_size[0]/2.0);
			}

			if (hp.dy > (grid_size[1]*sector_size[1]) ){
				ROS_WARN("current y= %f is out of grid bounds. Theresholded.", hp.dy);
				hp.dy = grid_size[1]*sector_size[1] - (sector_size[1]/2.0);
			}
			if (hp.dy < 0 ){
				ROS_WARN("current y= %f is out of grid bounds. Theresholded.", hp.dy);
				hp.dy = (sector_size[1]/2.0);
			}
			// get/set my current sector number from my current ENU
			enu(0,0)= hp.dx;
			enu(1,0)= hp.dy;
			enu(2,0)= cb.local_enu_msg.pose.position.z;
			//problem.set_origin_shifts(0.0, 0.0); /* TODO WHY???? */
			if (problem.DEBUG)
				cout << "[dlp_node] [loop] use_gps=True  " << "\n";
		}
		else{// probably indoors
			enu(0,0)= cb.local_enu_msg.pose.position.x;
			enu(1,0)= cb.local_enu_msg.pose.position.y;
			enu(2,0)= cb.local_enu_msg.pose.position.z;
			problem.set_origin_shifts(origin_shifts[0], origin_shifts[1]);
			if (problem.DEBUG)
				cout << "[dlp_node] [loop] use_gps=False  " << "\n";
		}
		if (use_sim){
			problem.set_my_current_location(dloc(myID,0));
			if (problem.DEBUG)
				cout << "[dlp_node] [loop] use_sim=True  " << "\n";
		}
		else{
			problem.set_my_current_location(problem.get_sector_from_ENU(enu));
			if (problem.DEBUG)
				cout << "[dlp_node] [loop] use_sim=False  " << "\n";
		}
		if (problem.DEBUG)
			cout << "[dlp_node] [loop] my current location:  " << problem.get_my_current_location() << "\n";

		// test conversion from sector to ENU and vise versa
		//enu = problem.get_ENU_from_sector(dloc(myID,0));
		//sector_from_enu = problem.get_sector_from_ENU(enu);

		if (bGlobalComms)
			problem.update_LP();
		else{
			if (bUseLocalEstimate)
				problem.update_LP_with_local_estimate();
			else
				problem.update_LP_dist();
		}

		problem.solve_simplex();// faster than interior point

		//problem.solve_intp();
		if (bGlobalComms)
			problem.extract_centralized_solution();
		else{
			if (bUseLocalEstimate)
				problem.extract_local_solution_estimate();
			else
				problem.extract_local_solution();
		}

		//problem.get_d_next_locations(next_loc);
		neighbors_next_loc = problem.get_neighbor_next_locations();

		sensedN = problem.get_sensed_neighbors();


		end = clock();

		// fill ROS message: DlpState
		dlp::DlpState my_state;
		my_state.header.stamp = ros::Time::now();
		my_state.my_id = myID;
		my_state.my_current_sector = problem.get_my_current_location();
		my_state.my_next_sector = problem.get_my_next_location();

		// local_position by mavros
		my_state.my_current_local_position.x = (float)cb.local_enu_msg.pose.position.x;
		my_state.my_current_local_position.y = (float)cb.local_enu_msg.pose.position.y;
		my_state.my_current_local_position.z = (float)cb.local_enu_msg.pose.position.z;

		// enu location of next target sector, considering shifts in the origin!!!
		enu = problem.get_ENU_from_sector_noShift(problem.get_my_next_location());
		
		if (use_gps){// where each vehicle's local fixed frame can be different
			// TODO
			// set current grid position
			my_state.my_current_position.x = (float)hp.dx;
			my_state.my_current_position.y = (float)hp.dy;
			my_state.my_current_position.z = (float)cb.local_enu_msg.pose.position.z;

	
			// set next local grid position
			my_state.my_next_position.x = enu(0,0);
			my_state.my_next_position.y = enu(1,0);
			my_state.my_next_position.z = altitude_setpoint;

			// get next mavros local_position
			// from enu in grid position to absolute LLA
			if (use_grid_corners){
				hp.dx_L = enu(0,0); hp.dy_L = enu(1,0);
				hp.local2global_GPS();
			}
			else
				hp.NED2LLA(hp.lat0, hp.lon0, enu(1,0), enu(0,0));
			// from abolute LLA to mavros local_position

			hp.LLA2ENU(cb.current_lat, cb.current_long, hp.lat, hp.lon);
			enu(0,0) = hp.dx;
			enu(1,0) = hp.dy;
			my_state.my_next_local_position.x = my_state.my_current_local_position.x + enu(0,0);
			my_state.my_next_local_position.y = my_state.my_current_local_position.y+ enu(1,0);
			my_state.my_next_local_position.z = altitude_setpoint;
		}
		else{// most probably using mocap, and the vehicles local fixed frame is the same
			// current grid position equal to mavros local_position
			my_state.my_current_position.x = (float)cb.local_enu_msg.pose.position.x;
			my_state.my_current_position.y = (float)cb.local_enu_msg.pose.position.y;
			my_state.my_current_position.z = (float)cb.local_enu_msg.pose.position.z;
			// current mavros local_position
			my_state.my_current_local_position.x = (float)cb.local_enu_msg.pose.position.x;
			my_state.my_current_local_position.y = (float)cb.local_enu_msg.pose.position.y;
			my_state.my_current_local_position.z = (float)cb.local_enu_msg.pose.position.z;
			
			// next grid position
			my_state.my_next_position.x = enu(0,0);
			my_state.my_next_position.y = enu(1,0);
			my_state.my_next_position.z = altitude_setpoint;
			// next mavros  local_position
			my_state.my_next_local_position.x = enu(0,0);
			my_state.my_next_local_position.y = enu(1,0);
			my_state.my_next_local_position.z = altitude_setpoint;
		}

		if (problem.get_N_sensed_neighbors() > 0){
			my_state.sensed_neighbors.resize(problem.get_N_sensed_neighbors());
			for (int i=0; i< problem.get_N_sensed_neighbors(); i++){
				my_state.sensed_neighbors[i] = sensedN(i,0);
			}
		}
		else
			my_state.sensed_neighbors.clear();

		// set current sensed (and predicted) attackers locations
		e_predicted_loc = problem.get_predicted_attackers_sectors();
		my_state.sensed_attackers_locaitons.resize(Ne);
		my_state.predicted_attackers_locations.resize(Ne);
		for (int i=0; i<Ne; i++){
			if (eloc(i,0)>0){
				my_state.sensed_attackers_locaitons[i] = (int) eloc(i,0);
			}	my_state.predicted_attackers_locations[i] = (int) e_predicted_loc(i,0);
		}

		my_state.execution_time_ms = (float)( (end-start)/( (clock_t)1000) );

		// publish my_state msg
		dlp_state_pub.publish(my_state);

		if (problem.DEBUG){
			cout << "###################################################### \n" ;
			cout << "[dlp_node] I am agent: " << problem.get_myID() << "\n";
			cout << "[dlp_node] Problem solved in Total time= " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
			cout << "[dlp_node] attacker current locations: " << eloc.transpose()<< endl;
			cout << "[dlp_node] Defenders locations: " << dloc.transpose() << endl;
			cout << "[dlp_node] local planned transition: "<< problem.get_my_current_location()
				<< " ---> "
				<< problem.get_my_next_location()
				<< "\n";
			if (problem.get_N_sensed_neighbors() > 0)
				cout << "[dlp_node] Sensed Neighbors: " << sensedN.transpose() << "\n";
			else
				cout << "[dlp_node] Sensed Neighbors: []" <<"\n";
			cout << "###################################################### \n" ;
		}

		
		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}
