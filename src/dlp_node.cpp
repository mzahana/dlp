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

/**
 * This is a ROS skelton code for publisher subscriber.
 * TODO: Need to to adapt to DLP.
 */

/**
* Callbacks class
*/
class CallBacks
{
public:
	dlp::DefendersState	d_loc_msg;
	dlp::EnemyState		e_loc_msg;
	geometry_msgs::PoseStamped local_enu_msg;

	 CallBacks() { }
	 //v_pos.push_back(0);v_pos.push_back(0);v_pos.push_back(0);}
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
		cout << "cb3 \n";
		cout << "local_pos: " << local_enu_msg.pose.position.x << " "
								<< local_enu_msg.pose.position.y << " "
								<< local_enu_msg.pose.position.z << "\n";

		double v_x = msg->pose.position.x;
		double v_y = msg->pose.position.y;
		double v_z = msg->pose.position.z;
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
	int myID, rows, cols;
	nh.param("myID", myID, 0);

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

	int Tp;
	nh.param("Tp", Tp, 2);

	int Nr;
	nh.param("neighbor_radius", Nr, 1);

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

	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	ros::Publisher dlp_state_pub = nh.advertise<dlp::DlpState>("dlp_state", 10);

	/**
	* Subscribers
	*/
	CallBacks cb;
	ros::Subscriber d_loc_sub = nh.subscribe("/defenders_locations", 10, &CallBacks::d_loc_cb, &cb);
	ros::Subscriber e_loc_sub = nh.subscribe("/enemy_locations", 10, &CallBacks::e_loc_cb, &cb);
	ros::Subscriber local_enu_sub = nh.subscribe("mavros/local_position/pose", 10, &CallBacks::local_enu_cb, &cb);

	ros::Rate loop_rate(update_freq);

	/**
	* Create DLP problem and set parameters.
	*/
	DLP problem;
	problem.DEBUG = true;
	problem.set_myID(myID);

	problem.set_nRows(grid_size[0]);
	problem.set_nCols(grid_size[1]);
	problem.set_grid_resolution(sector_size[0], sector_size[1]);
	problem.set_origin_shifts(origin_shifts[0], origin_shifts[1]);

	problem.set_Tp(Tp);
	problem.set_Nd(Nd);
	problem.set_Ne(Ne);

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

	MatrixXf eloc(Ne,1);
	MatrixXf dloc(Nd,1);

	MatrixXf sensedN;
	MatrixXf neighbors_next_loc;


	MatrixXf next_loc; /* next optimal locations */

	MatrixXf enu(3,1);
	int sector_from_enu;


	clock_t start, end;

	// Initialize defenders/enemy messages
	dloc(0,0)=15.0; dloc(1,0)=17.0; dloc(2,0)=19.0;
	eloc(0,0)=28.0;eloc(1,0)=26.0;
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
	for (int i=0; i<Ne; i++){
		cb.e_loc_msg.enemy_sectors[i] = (int)eloc(i,0);
		enu = problem.get_ENU_from_sector(eloc(i,0));
		cb.e_loc_msg.enemy_position[i].x = enu(0,0);
		cb.e_loc_msg.enemy_position[i].y = enu(1,0);
		cb.e_loc_msg.enemy_position[i].z = altitude_setpoint;
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
			dloc(i,0) =problem.get_sector_from_ENU(enu);
		}
		problem.set_d_current_locations(dloc);


		// get enemy locations
		eloc = MatrixXf::Constant(Ne,1,0.0);
		for (int i=0; i< cb.e_loc_msg.enemy_count; i++){
			enu(0,0) = cb.e_loc_msg.enemy_position[i].x;
			enu(1,0) = cb.e_loc_msg.enemy_position[i].y;
			enu(2,0) = cb.e_loc_msg.enemy_position[i].z;
			eloc(i,0) =problem.get_sector_from_ENU(enu);
		}
		problem.set_e_current_locations(eloc);


		// set my current location
		enu(0,0)= cb.local_enu_msg.pose.position.x;
		enu(1,0)= cb.local_enu_msg.pose.position.y;
		enu(2,0)= cb.local_enu_msg.pose.position.z;
		problem.set_my_current_location(problem.get_sector_from_ENU(enu));

		// test conversion from sector to ENU and vise versa
		//enu = problem.get_ENU_from_sector(dloc(myID,0));
		//sector_from_enu = problem.get_sector_from_ENU(enu);

		//problem.update_LP();
		problem.update_LP_dist();

		problem.solve_simplex();// faster than interior point

		//problem.solve_intp();
		//problem.extract_centralized_solution();
		problem.extract_local_solution();

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

		my_state.my_current_position.x = (float)cb.local_enu_msg.pose.position.x;
		my_state.my_current_position.y = (float)cb.local_enu_msg.pose.position.y;
		my_state.my_current_position.z = (float)cb.local_enu_msg.pose.position.z;

		enu = problem.get_ENU_from_sector(problem.get_my_next_location());

		my_state.my_next_position.x = enu(0,0);
		my_state.my_next_position.y = enu(1,0);
		my_state.my_next_position.x = altitude_setpoint;
		my_state.sensed_neighbors.resize(problem.get_N_sensed_neighbors());
		for (int i=0; i< problem.get_N_sensed_neighbors(); i++){
			my_state.sensed_neighbors[i] = sensedN(i,0);
		}

		my_state.execution_time = (float)( (end-start)/( (clock_t)1000000) );

		// publish my_state msg
		dlp_state_pub.publish(my_state);

		if (problem.DEBUG){
			cout << "###################################################### \n" ;
			cout << "I am agent: " << problem.get_myID() << "\n";
			cout << "Problem solved in Total time= " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
			cout << "attacker current locations: " << eloc.transpose()<< endl;
			cout << "Defenders locations: " << dloc.transpose() << endl;
			cout << "local planned transition: "<< problem.get_my_current_location()
				<< " ---> "
				<< problem.get_my_next_location()
				<< "\n";
			cout << "Sensed Neighbors: " << sensedN.transpose() << "\n";
			cout << "###################################################### \n" ;
		}

		
		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}
