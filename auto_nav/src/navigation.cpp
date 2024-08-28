#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
using std::placeholders::_1;

#include "rclcpp/qos.hpp"

//QP Solver
#include "QuadProg++.hh"


#include "librealsense2/rs.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "opencv2/opencv.hpp"

//xtensor
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>

//Math Libraries
#include <stdio.h>
#include <math.h> //cosf
#include <cmath> //M_PI, round
#include <sstream>
#include <algorithm>
#include <iostream>

//Markers
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"


//Ground Filter Includes
#include <random>
#include <set>
#include <librealsense2/rsutil.h>


//DDS Tuning
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "rclcpp/publisher.hpp"
// #include "rclcpp/subscriber.hpp"




class GapBarrier : public rclcpp::Node {
    private:
    
    //topics
	std::string depth_image_topic, depth_info_topic, cv_ranges_topic,
	depth_index_topic, depth_points_topic, lidarscan_topic, drive_topic, odom_topic,
	mux_topic, imu_topic;


	//subscriptions

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;


	//publications
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr driver_pub;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr cv_ranges_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;



	//time
	double current_time = this->get_clock()->now().seconds();
	double prev_time = current_time;
	double time_ref = 0.0;
	double heading_beam_angle;

 	//Walls
	double tau;
	std::vector<double> wl0; std::vector<double> wr0;
	int optim_mode;

	//lidar-preprocessing
	int scan_beams; double right_beam_angle, left_beam_angle;
	int ls_str, ls_end, ls_len_mod, ls_len_mod2; double ls_fov, angle_cen, ls_ang_inc;
	double max_lidar_range, safe_distance;
	
	//obstacle point detection
	std::string drive_state;
	double angle_bl, angle_al, angle_br, angle_ar;
	int n_pts_l, n_pts_r; double max_lidar_range_opt;

	//odom
	double yaw;


	//steering and stop
	double vel;
	double CenterOffset, wheelbase;
	double stop_distance, stop_distance_decay;
	double k_p, k_d;
	double max_steering_angle;
	double vehicle_velocity; double velocity_zero;

	double stopped_time;
	double stop_time1, stop_time2;

	double yaw0, dtheta; double turn_angle;
	double turn_velocity;


	//nav
	int nav_active;

	//camera and cv

	int use_camera;
	double min_cv_range, max_cv_range, cv_distance_to_lidar;
	int num_cv_sample_rows, num_cv_sample_cols;

	double cv_ground_angle, cv_lidar_range_max_diff, camera_height;
	double cv_real_to_theo_ground_range_ratio;
	double cv_real_to_theo_ground_range_ratio_near_horizon;
	double cv_ground_range_decay_row;
	double cv_pitch_angle_hardcoded;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_confidence;

	rs2_intrinsics intrinsics;
	bool intrinsics_defined = false;
	sensor_msgs::msg::Image cv_image_data;
	bool cv_image_data_defined = false;

	//more cv

	sensor_msgs::msg::LaserScan cv_ranges_msg;
	int cv_rows, cv_cols;
	xt::xarray<int> cv_sample_rows_raw;
	xt::xarray<int> cv_sample_cols_raw;

	visualization_msgs::msg::Marker marker;


	//Ground Points Filter
	std::set<std::pair<int,int>> best_inliers;
	float a_prev=0; float b_prev=-1; float c_prev=0; float d_prev=0; 
	int num_points_best=0;
	float threshold = 0.02; float plane_length_best; bool first_time = false;
	bool skip = false; float tol = 0.1; float min_dist = 0.03;
	float nx,ny,nz; 
	float a_best, b_best, c_best, d_best;

	
	
	float ny_prev = 0.;


	std::vector<std::vector<float>> best_valids;




    public:

    GapBarrier() : Node("navigation"){
		
		//declaring depth topics
		this->declare_parameter("depth_image_topic", " ");
		this->declare_parameter("depth_info_topic", " ");
		this->declare_parameter("cv_ranges_topic", " ");
		this->declare_parameter("depth_index_topic", " ");
		this->declare_parameter("depth_points_topic", " ");


		//getting depth topic

		depth_image_topic = this->get_parameter("depth_image_topic").as_string();
		depth_info_topic = this->get_parameter("depth_info_topic").as_string();
		cv_ranges_topic = this->get_parameter("cv_ranges_topic").as_string();
		depth_index_topic = this->get_parameter("depth_index_topic").as_string();
		depth_points_topic = this->get_parameter("depth_points_topic").as_string();


		//declaring lidar parameters
		this->declare_parameter("scan_beams", -1);
		this->declare_parameter("right_beam_angle", 0.0);
		this->declare_parameter("left_beam_angle", 0.0);
		this->declare_parameter("scan_range", 0.0);
		this->declare_parameter("safe_distance",0.0);

		this->declare_parameter("stop_distance", 0.0);
		this->declare_parameter("stop_distance_decay", 0.0);
		this->declare_parameter("velocity_zero", 0.0);



		//lidar params
		scan_beams = this->get_parameter("scan_beams").as_int();
		right_beam_angle = this->get_parameter("right_beam_angle").as_double();
		left_beam_angle = this->get_parameter("left_beam_angle").as_double();
		max_lidar_range = this->get_parameter("scan_range").as_double();
		safe_distance = this->get_parameter("safe_distance").as_double();

		stop_distance = this->get_parameter("stop_distance").as_double();
		stop_distance_decay = this->get_parameter("stop_distance_decay").as_double();

		velocity_zero = this->get_parameter("velocity_zero").as_double();





		//obstacle point detection topics
		this->declare_parameter("heading_beam_angle", 0.0);
		this->declare_parameter("angle_bl", 0.0);
		this->declare_parameter("angle_al", 0.0);
		this->declare_parameter("angle_br", 0.0);
		this->declare_parameter("angle_ar", 0.0);
		this->declare_parameter("n_pts_l", -1);
		this->declare_parameter("n_pts_r", -1);
		this->declare_parameter("max_lidar_range_opt", 0.0);


		//obstacle point detection
		heading_beam_angle = this->get_parameter("heading_beam_angle").as_double();
		angle_bl = this->get_parameter("angle_bl").as_double();
		angle_al = this->get_parameter("angle_al").as_double();
		angle_br = this->get_parameter("angle_br").as_double();
		angle_ar = this->get_parameter("angle_ar").as_double();
        n_pts_l = this->get_parameter("n_pts_l").as_int();
        n_pts_r = this->get_parameter("n_pts_r").as_int();
        max_lidar_range_opt = this->get_parameter("max_lidar_range_opt").as_double();


		//walls
		this->declare_parameter("tau", 0.0);
		this->declare_parameter("optim_mode", -1);


		tau = this->get_parameter("tau").as_double();
		optim_mode = this->get_parameter("optim_mode").as_int();
		wl0 = {0.0, -1.0}; wr0 = {0.0, 1.0};


		//lidar init
		drive_state = "normal";
		ls_ang_inc = 2*M_PI/scan_beams;
		ls_str = int(round(scan_beams*(right_beam_angle))/(2*M_PI));
		ls_end = int(round(scan_beams*(left_beam_angle))/(2*M_PI));
		ls_len_mod = ls_end-ls_str+1;
		ls_fov = ls_len_mod*ls_ang_inc;
		angle_cen = ls_fov/2;
		ls_len_mod2 = 0;


		//QoS Settings
		// auto qos_profile = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		// auto rmw = qos_profile.get_rmw_qos_profile();

		// rmw.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		// std::string reliability = (rmw.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) ? "Reliable" : "Best Effort";

		// auto msg = std_msgs::msg::String();
		// std::stringstream ss;
		// ss << reliability;
		// msg.data = ss.str();

		// RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());


		//odom init
		vel = 0.0; 

		//declaring steering params
		this->declare_parameter("CenterOffset", 0.0);
		this->declare_parameter("wheelbase",0.0);
		this->declare_parameter("k_p", 0.0);
		this->declare_parameter("k_d", 0.0);
		this->declare_parameter("max_steering_angle",0.0);
		this->declare_parameter("vehicle_velocity",0.0);
		this->declare_parameter("turn_velocity",0.0);

		//reading steering params
		CenterOffset = this->get_parameter("CenterOffset").as_double();
		wheelbase = this->get_parameter("wheelbase").as_double();
		k_p = this->get_parameter("k_p").as_double();
		k_d = this->get_parameter("k_d").as_double();
		max_steering_angle = this->get_parameter("max_steering_angle").as_double();
		vehicle_velocity = this->get_parameter("vehicle_velocity").as_double();
		turn_velocity = this->get_parameter("turn_velocity").as_double();

		//declaring timing
		this->declare_parameter("stop_time1", 0.0);
		this->declare_parameter("stop_time2", 0.0);

		//timing
		stopped_time = 0.0;
		stop_time1 = this->get_parameter("stop_time1").as_double();
		stop_time2 = this->get_parameter("stop_time2").as_double();

		//camera
		this->declare_parameter("use_camera", 0);

		use_camera = this->get_parameter("use_camera").as_int();



		//cv
		this->declare_parameter("min_cv_range", 0.0);
		this->declare_parameter("max_cv_range", 0.0);
		this->declare_parameter("cv_distance_to_lidar",0.0);
		this->declare_parameter("num_cv_sample_rows", 0);
		this->declare_parameter("num_cv_sample_cols", 0);


		this->declare_parameter("camera_height", 0.0);

		this->declare_parameter("cv_ground_angle", 0.0);
		this->declare_parameter("cv_real_to_theo_ground_range_ratio", 0.0);
		this->declare_parameter("cv_real_to_theo_ground_range_ratio_near_horizon", 0.0);
		this->declare_parameter("cv_ground_range_decay_row", 0.0);
		this->declare_parameter("cv_pitch_angle_hardcoded", 0.0);

		min_cv_range = this->get_parameter("min_cv_range").as_double();
		max_cv_range = this->get_parameter("max_cv_range").as_double();
		cv_distance_to_lidar = this->get_parameter("cv_distance_to_lidar").as_double();
		num_cv_sample_rows = this->get_parameter("num_cv_sample_rows").as_int();
		num_cv_sample_cols = this->get_parameter("num_cv_sample_cols").as_int();

		
		camera_height = this->get_parameter("camera_height").as_double();
		cv_ground_angle = this->get_parameter("cv_ground_angle").as_double();
		cv_real_to_theo_ground_range_ratio = this->get_parameter("cv_real_to_theo_ground_range_ratio").as_double();
		cv_real_to_theo_ground_range_ratio_near_horizon = this->get_parameter("cv_real_to_theo_ground_range_ratio_near_horizon").as_double();
		cv_pitch_angle_hardcoded = this->get_parameter("cv_pitch_angle_hardcoded").as_double();



		// RCLCPP_INFO(this->get_logger(), "%d", optim_mode);

		//Subscription
		lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
								     1, std::bind(&GapBarrier::lidar_callback, this, _1));

		odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&GapBarrier::odom_callback, this, _1));

		joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&GapBarrier::joy_callback, this, _1));


		//Publication

		driver_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
		marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("wall_markers",2);

		//testing QoS change
		// pub = this->create_publisher<std_msgs::msg::String>("chatter", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
		// std_msgs::msg::String msg;
		// msg.data = "hello";
		// pub->publish(msg);

		if(use_camera){

			//subscriptions

			depth_img = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(), std::bind(&GapBarrier::imageDepth_callback, this, _1));
			depth_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(depth_info_topic, 1, std::bind(&GapBarrier::imageDepthInfo_callback, this, _1));
			depth_img_confidence = this->create_subscription<sensor_msgs::msg::Image>("/camera/confidence/image_rect_raw",1, std::bind(&GapBarrier::confidenceCallback, this, _1));


			RCLCPP_INFO(this->get_logger(), "USE_CAMERA = 1");

			cv_ranges_msg = sensor_msgs::msg::LaserScan();
			cv_ranges_msg.header.frame_id = "laser";
			cv_ranges_msg.angle_increment = ls_ang_inc;
			cv_ranges_msg.time_increment = 0;
			cv_ranges_msg.range_min  =0;
			cv_ranges_msg.range_max = max_lidar_range;
			cv_ranges_msg.angle_min = 0;
			cv_ranges_msg.angle_max = 2*M_PI;


			cv_ranges_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(cv_ranges_topic, 1);


		
		}


    }

	/// ---------------------- GENERAL HELPER FUNCTIONS ----------------------
		int equiv_sign(double qt){
			if(qt < 0) return -1;
			else if (qt == 0 ) return 0;
			else return 1;
		}


		int arg_max(std::vector<float> ranges){

			int idx = 0;

			for(int i =1; i < int(ranges.size()); ++i){
				if(ranges[idx] < ranges[i]) idx = i;
			}

			return idx;


		}

		float dist(float x1, float y1, float z1, float x2, float y2, float z2){
			return sqrtf(powf((x1-x2), 2) + powf((y1-y2),2) + powf((z1-z2),2));
		}

	/// ---------------------- MAIN FUNCTIONS ----------------------



	void confidenceCallback(const sensor_msgs::msg::Image::SharedPtr data){ return; }


	void imageDepthInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo){

		// RCLCPP_INFO(this->get_logger(), "INSTRINSICS DEFINED: %d", intrinsics_defined);


		if(intrinsics_defined){ return; }
		else{

			intrinsics.width = cameraInfo->width;
			intrinsics.height = cameraInfo->height;
			intrinsics.ppx = cameraInfo->k[2];
			intrinsics.ppy = cameraInfo->k[5];
			intrinsics.fx = cameraInfo->k[0];
			intrinsics.fy = cameraInfo->k[4];


			if (cameraInfo->distortion_model == "plumb_bob") {
				intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;   
			}
			else if (cameraInfo->distortion_model == "equidistant") {
				intrinsics.model = RS2_DISTORTION_KANNALA_BRANDT4;
			}

			for(int i =0; i < 5; ++i){
				intrinsics.coeffs[i] = cameraInfo->d[i];
			}

			intrinsics_defined = true;

			cv_rows = intrinsics.height;
			cv_cols = intrinsics.width;

			// RCLCPP_INFO(this->get_logger(),"DEFINED");

			cv_sample_rows_raw= xt::linspace<int>(0, cv_rows-1, num_cv_sample_rows);
			cv_sample_cols_raw= xt::linspace<int>(0, cv_cols-1, num_cv_sample_cols);



		}
	}

	void imageDepth_callback(const sensor_msgs::msg::Image::SharedPtr img){

		// RCLCPP_INFO(this->get_logger(), "DEFINED");


		if(intrinsics_defined){
			cv_image_data.header = img->header;
			cv_image_data.height = img->height;
			cv_image_data.width = img->width;
			cv_image_data.encoding = img->encoding;
			cv_image_data.is_bigendian = img->is_bigendian;
			cv_image_data.step = img->step;
			cv_image_data.data = img->data;
			cv_image_data_defined = true;
			// RCLCPP_INFO(this->get_logger(), "%s", cv_image_data.encoding);
		}
		else{
			cv_image_data_defined = false;
		}
		
	}

	void findGroundPlane(cv::Mat cv_image){

			int max_iterations = 5;

			//finding three random points
			int amt = 3;
			std::random_device rd;
			std::mt19937_64 generator(rd());

			float a,b,c,d; int num_points_cur=0; int row_size = (int) cv_sample_rows_raw.size(); int col_size = (int) cv_sample_cols_raw.size();

			std::uniform_int_distribution<int> dist_rows{0.6*row_size -1, row_size-1};
			std::uniform_int_distribution<int> dist_cols{0, col_size-1};
			int num_gen  = 0; 

			num_points_best = 0; int tries = 0;


			std::vector<float> cv_pt1(3); std::vector<float> cv_pt2(3); std::vector<float> cv_pt3(3);
			float *cv_pts[] = {&cv_pt1[0], &cv_pt2[0], &cv_pt3[0]};

			float alpha = 0.8;
			bool skip = false; bool tried = false;



			while((max_iterations--) && num_points_best < 3000){

				++num_gen;
				skip = false;
				std::vector<std::vector<float>> valids;

				std::set<std::pair<int,int>> indices;


				if(max_iterations <= 2 && abs(ny_prev + 1) < 0.1 && !tried){ 
					skip = true; 
					tried = true;
					a = a_prev;
					b = b_prev;
					c = c_prev;
					d = d_prev;
				
				
				}


				int row,col; float depth;

				while(indices.size() != amt){
					row = cv_sample_rows_raw[dist_rows(generator)]; col = cv_sample_cols_raw[dist_cols(generator)];
					depth = cv_image.ptr<uint16_t>(row)[col]/1000.;

					if(depth > min_cv_range && depth < max_cv_range) { 
						indices.insert(std::make_pair(row, col));
						
					}

					if(tries > 10) {					
						return;
					}
					tries++;
				}



				if(!skip){

					//TEST RANDOM

					auto it = indices.begin();

					for(int i =0; i < amt; ++i){
						row = it->first; col = it->second;
						float pixel[2] = {(float) col, (float) row};
						depth = cv_image.ptr<uint16_t>(row)[col]/1000.;
						rs2_deproject_pixel_to_point(cv_pts[i], &intrinsics, pixel, depth);
						++it;
					}

					// if(dist(cv_pt1[0],cv_pt1[1],cv_pt1[2],cv_pt2[0],cv_pt2[1],cv_pt2[2]) < min_dist || dist(cv_pt1[0],cv_pt1[1],cv_pt1[2],cv_pt3[0],cv_pt3[1],cv_pt3[2]) < min_dist
					// || dist(cv_pt2[0],cv_pt2[1],cv_pt2[2],cv_pt3[0],cv_pt3[1],cv_pt3[2]) < min_dist) {++max_iterations; continue;}


					a = (cv_pt2[1] - cv_pt1[1])*(cv_pt3[2] - cv_pt1[2]) - (cv_pt2[2] - cv_pt1[2])*(cv_pt3[1] - cv_pt2[1]);

					b = (cv_pt2[2] - cv_pt1[2])*(cv_pt3[0] - cv_pt1[0]) - (cv_pt2[0] - cv_pt1[0])*(cv_pt3[2] - cv_pt2[2]);

					c = (cv_pt2[0] - cv_pt1[0])*(cv_pt3[1] - cv_pt1[1]) - (cv_pt2[1] - cv_pt1[1])*(cv_pt3[0] - cv_pt2[0]);


					d = -(a*cv_pt1[0] + b*cv_pt1[1] + c*cv_pt1[2]);

					if(b > 0){b = -b;}

					//smoothing
					a = alpha*a + (1-alpha)*a_prev;
					b = alpha*b + (1-alpha)*b_prev;
					c = alpha*c + (1-alpha)*c_prev;
					d = alpha*d + (1-alpha)*d_prev;

				}

				float plane_length = sqrtf(a*a+b*b+c*c+d*d);

				num_points_cur = 0; std::pair<int,int> rc;






				for(int i = 0; i < (int)cv_sample_rows_raw.size(); ++i){

					row = cv_sample_rows_raw[i];

					for(int j=0; j < (int)cv_sample_cols_raw.size(); ++j){

							col = cv_sample_cols_raw[j];

							// rc = std::make_pair(row, col);


							// if(indices.count(rc)) continue;


							float pixel_app[2] = {(float) col, (float) row};
							depth = cv_image.ptr<uint16_t>(row)[col]/1000.;
							if(depth > max_cv_range || depth < min_cv_range)
							{
								continue;
							}

							std::vector<float> cv_point(3);
							float pixel[2] = {static_cast<float> (col), static_cast<float> (row)};
							rs2_deproject_pixel_to_point(cv_point.data(), &intrinsics, pixel, depth);

							float distance = fabs(a*cv_point[0] + b*cv_point[1] + c*cv_point[2] + d)/plane_length;

							float fused_y = 0.2*(0.15-distance) + cv_point[1]*0.8;
							// std::cout << fused_y << std::endl;


							// if (cv_point[1] < -1.5*camera_height || cv_point[1] >= 0.6*camera_height){ continue; }
							// else if(distance <= threshold && cv_point[1] >= 0.12){
							// 	++num_points_cur; 	
							// }
							// else {
							// 	valids.push_back(cv_point);
							// }


							// if(cv_point[1] < -1.5*camera_height || cv_point[1] >= 0.6*camera_height || cv_point[0] >= 1.5) {continue;}
							// else if(fused_y >= 0.11 && distance <= threshold){
							// 	++num_points_cur;
							// }
							// else{
							// 	// std::cout << fused_y << std::endl;
							// 	valids.push_back(cv_point);
							// }

							// if(fused_y < -1.5*camera_height || cv_point[1] >= 0.5*camera_height || row <= cv_rows/2) {continue;}
							// else if(fused_y >= 0.11 || distance <= threshold){
							// 	++num_points_cur;
							// }
							// else{
							// 	valids.push_back(cv_point);
							// }
							if(cv_point[1] >= 0.12 && distance <= threshold){
								++num_points_cur;
							}
							else if(fused_y < 0.01 || row <= cv_rows*0.4 || cv_point[1] >= 0.6*camera_height) {continue;}
							else{
								valids.push_back(cv_point);
							}


	


					}

				}



				if(num_points_cur >= num_points_best){
					num_points_best = num_points_cur;
					plane_length_best = plane_length;
					best_valids = valids;
				}
			
			}

			a_prev = a;
			b_prev = b;
			c_prev = c;
			d_prev = d;
			// alpha = (float) num_points_best/(num_pts_prev)* alpha;
			// num_pts_prev = num_points_best;

			ny_prev = b/plane_length_best;

			// std::cout << ny_prev << std::endl;

			// std::cout << "ny_prev " << ny_prev << " num_pts " << num_points_best << std::endl;
			// std::cout << best_valids.size() << std::endl;
			

		}
	
	void augment_camera(std::vector<float> &lidar_ranges){

		// std::cout << cv_image_data_defined << std::endl;
		//type is cv_bridge::CvImage pointer, arrow operator will return
		//opencv Mat 2D array.

		cv::Mat cv_image=(cv_bridge::toCvCopy(cv_image_data,cv_image_data.encoding))->image;
		findGroundPlane(cv_image); 

		//use to debug
		// bool assert=( (cv_rows==cv_image.rows) && (cv_cols==cv_image.cols) );

		// print();

		//1. Obtain pixel and depth
		
		// for(int i=0 ; i < cv_sample_cols_raw.size(); i++) {

		// 	int col= cv_sample_cols_raw[i];

		// 	for(int j=0; j < cv_sample_rows_raw.size(); j++)
		// 	{
		// 		int row = cv_sample_rows_raw[j];


		// 		float depth = (cv_image.ptr<uint16_t>(row)[col])/(float)1000;

		// 		if(depth > max_cv_range or depth < min_cv_range)
		// 		{
		// 			continue;
		// 		}

		// 		//2 convert pixel to xyz coordinate in space using camera intrinsics, pixel coords, and depth info
		// 		std::vector<float> cv_point(3); 
		// 		float pixel[2] = {(float) col, (float) row};
		// 		rs2_deproject_pixel_to_point(cv_point.data(), &intrinsics, pixel, depth);

		// 		//xyz points in 3D space, process and combine with lidar data
		// 		float cv_coordx=cv_point[0];
		// 		float cv_coordy=cv_point[1];
		// 		float cv_coordz=cv_point[2];


		// 		float d_temp = a_best*cv_coordx + b_best*cv_coordy + c_best*cv_coordz + d_best; 
		// 		float distance = fabs(d_temp)/plane_length_best;
		// 		// float distance_projected = -d_temp*ny/(a_best*nx+b_best*ny+c_best*nz);
		// 		// std::cout << distance_projected << std::endl;
		// 		// std::cout << nx << " " << ny << " " << nz << std::endl;


		// 		//FIX: WORKS WHEN USING cv_coordy > 0.5*camera_height

		// 		// if((distance <= threshold && cv_coordy >= 0.11) || cv_coordy > 0.5*camera_height || cv_coordy < -2.5*camera_height) continue; 
		// 		// if((distance <= threshold && cv_coordy >= 0.13) || cv_coordy > 0.6*camera_height || cv_coordy < -2.5*camera_height) continue; 


		// 		if(cv_coordy > 0.6*camera_height || cv_coordy < -2.5*camera_height) continue; 
		// 		// std::cout << "d_best: " << d_best << "cv_coordy " << cv_coordy  << "distance: " << distance << std::endl;

		// 		// imu_pitch=0;
		// 		// imu_roll=0;

		// 		// float cv_coordy_s = -1*cv_coordx*std::sin(imu_pitch) + cv_coordy*std::cos(imu_pitch)*std::cos(imu_roll) 
		// 		// + cv_coordz *std::cos(imu_pitch)*std::sin(imu_roll);


		// 		//3. Overwrite Lidar Points with Camera Points taking into account dif frames of ref

		// 		float lidar_coordx = -(cv_coordz+cv_distance_to_lidar);
		// 		float lidar_coordy = cv_coordx;
		// 		float cv_range_temp = sqrtf(std::pow(lidar_coordx,2)+ std::pow(lidar_coordy,2));

		// 		// int beam_index= (int) roundf(scan_beams*std::atan2(lidar_coordy, lidar_coordx)/(2*M_PI));
		// 		int beam_index= (int) std::floor(scan_beams*std::atan2(lidar_coordy, lidar_coordx)/(2*M_PI));
		// 		if(beam_index < 0) continue;


		// 		float lidar_range = lidar_ranges[beam_index];
		// 		lidar_ranges[beam_index] = std::min(lidar_range, cv_range_temp);
				
		// 	}
		// }


		for(auto &itr : best_valids){

			float cv_coordx=itr[0];
			float cv_coordy=itr[1];
			float cv_coordz=itr[2];


			float lidar_coordx = -(cv_coordz+cv_distance_to_lidar);
			float lidar_coordy = cv_coordx;
			float cv_range_temp = sqrtf(std::pow(lidar_coordx,2)+ std::pow(lidar_coordy,2));

			// int beam_index= (int) roundf(scan_beams*std::atan2(lidar_coordy, lidar_coordx)/(2*M_PI));
			int beam_index= (int) std::floor(scan_beams*std::atan2(lidar_coordy, lidar_coordx)/(2*M_PI));
			if(beam_index < 0) continue;


			float lidar_range = lidar_ranges[beam_index];
			lidar_ranges[beam_index] = std::min(lidar_range, cv_range_temp);


		}


		rclcpp::Time current_time = this->now();
		cv_ranges_msg.header.stamp=current_time;
		cv_ranges_msg.ranges=lidar_ranges;

		cv_ranges_pub->publish(cv_ranges_msg);

		best_valids.clear();
			

	}



	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
		// RCLCPP_INFO(this->get_logger(), "%d", msg->buttons[5]);
		nav_active = msg->buttons[5];
	}


	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
		vel = odom_msg->twist.twist.linear.x;
		yaw = 2*atan2(odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);


		// RCLCPP_INFO(this->get_logger(), "%f", vel);
	}



	void getWalls(std::vector<std::vector<double>> obstacle_points_l, std::vector<std::vector<double>> obstacle_points_r,
		std::vector<double> wl0, std::vector<double> wr0, double alpha, std::vector<double> &wr, std::vector<double> &wl){
			if(!optim_mode){
				//right
				quadprogpp::Matrix<double> Gr,CEr,CIr;
				quadprogpp::Vector<double> gr0,ce0r,ci0r,xr;
				int n,m,p; char ch;
				int n_obs_r = obstacle_points_r.size(); int n_obs_l = obstacle_points_l.size();


				//left
				n = 2; m = 0; p = n_obs_l;
				quadprogpp::Matrix<double> Gl, CEl, CIl;
				quadprogpp::Vector<double> gl0, ce0l, ci0l, xl;


				//left matrices
				Gl.resize(n,n);
				{
					std::istringstream is("1.0, 0.0,"
														"0.0, 1.0 ");

					for(int i=0; i < n; ++i)
						for(int j=0; j < n; ++j)
							is >> Gl[i][j] >> ch;
				}
				gl0.resize(n);
				{
					for(int i =0; i < int(wl0.size()); ++i) gl0[i] = wl0[i] * (alpha-1);
				}
				CEl.resize(n,m);
				{
					CEl[0][0] = 0.0;
					CEl[1][0] = 0.0;
				}
				ce0l.resize(m);

				CIl.resize(n,p);
				{
					for(int i=0; i < p; ++i){
						CIl[0][i] = -obstacle_points_l[i][0];
						CIl[1][i] = -obstacle_points_l[i][1]; 
					}
				}
				ci0l.resize(p);
				{
					for(int i =0; i < p; ++i) ci0l[i] = -1.0;
				}

				xl.resize(n);
				// std::stringstream ss;
				solve_quadprog(Gl, gl0, CEl, ce0l, CIl, ci0l, xl);
				wl[0] = xl[0]; wl[1] = xl[1];
				// ss << xl[0] << " " << xl[1];
				// for(int j =0; j < int(obstacle_points_l.size()); ++j){
				// 	ss << obstacle_points_l[j][0] << " " << obstacle_points_l[j][1];
				// }
				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();

				// ROS_INFO("%f, %f", wl[0], wl[1]);


				p = n_obs_r;
				//right matrices
				Gr.resize(n,n);
				{
					std::istringstream is("1.0, 0.0,"
													"0.0, 1.0 ");

					for(int i =0; i < n; ++i)
						for(int j=0; j < n; ++j)
							is >> Gr[i][j] >> ch;

				}

				gr0.resize(n);
				{
					for(int i = 0; i < int(wr0.size()); ++i) gr0[i] = wr0[i] * (alpha-1);
				}

				CEr.resize(n,m);
				{
					CEr[0][0] = 0.0;
					CEr[1][0] = 0.0;
				}
				ce0r.resize(m);

				
				CIr.resize(n,p);
				{
						for(int i =0; i < p; ++i){
							CIr[0][i] = -obstacle_points_r[i][0];
							CIr[1][i] = -obstacle_points_r[i][1];
						}
				}

				ci0r.resize(p);
				{
					for(int i =0; i < p; ++i) ci0r[i] = -1.0;
				}

				xr.resize(n);
				solve_quadprog(Gr, gr0, CEr, ce0r, CIr, ci0r, xr);
				// ss << xr[0] << " " << xr[1];
				wr[0] = xr[0]; wr[1] = xr[1]; 


			}
			else{
				quadprogpp::Matrix<double> G,CE,CI;
				quadprogpp::Vector<double> gi0, ce0, ci0, x;

				// char ch;
				int n_obs_l = obstacle_points_l.size(); int n_obs_r = obstacle_points_r.size();
				
				
				int n,m,p;
				n = 3; m = 0; p = n_obs_l + n_obs_r + 2;

				G.resize(n,n);
				{


					G[0][1] = G[0][2] = G[1][0] = G[1][2] = G[2][0] = G[2][1] = 0.0;
					G[0][0] = G[1][1] = 1.0;
					G[2][2] = 0.0001;

				}
				gi0.resize(n);
				{
					for(int i =0; i < n; ++i) gi0[i] = 0.0;
				}

				CE.resize(n,m);
				{
					CE[0][0] = 0.0;
					CE[1][0] = 0.0;
					CE[2][0] = 0.0;
				}
				ce0.resize(m);

				CI.resize(n,p);
				{
					for(int i =0; i < n_obs_r; ++i){
						CI[0][i] = obstacle_points_r[i][0];
						CI[1][i] = obstacle_points_r[i][1];
						CI[2][i] = 1.0;
					}

					for(int i = n_obs_r; i < n_obs_l + n_obs_r; ++i){
						CI[0][i] = -obstacle_points_l[i-n_obs_r][0];
						CI[1][i] = -obstacle_points_l[i-n_obs_r][1];
						CI[2][i] = -1.0;
					}

					CI[0][n_obs_l+n_obs_r] = 0.0; CI[1][n_obs_l+n_obs_r] = 0.0; CI[2][n_obs_l+n_obs_r] = 1.0;
					CI[0][n_obs_l+n_obs_r+1] = 0.0; CI[1][n_obs_l+n_obs_r+1] = 0.0; CI[2][n_obs_l+n_obs_r+1] = -1.0;

				}
				ci0.resize(p);
				{
					for(int i =0; i < n_obs_r+n_obs_l; ++i){
						ci0[i] = -1.0;
					}
					
					ci0[n_obs_r+n_obs_l] = 0.9; ci0[n_obs_r+n_obs_l+1] = 0.9;
				}
				x.resize(n);

				solve_quadprog(G, gi0, CE, ce0, CI, ci0, x);



				wr[0] = (x[0]/(x[2]-1)); wr[1] = (x[1]/(x[2]-1));

				wl[0] = (x[0]/(x[2]+1)); wl[1] = (x[1]/(x[2]+1));



				std::stringstream ss;

				// ss << wl[0] << " " << wl[1];


				// for(int i =0; i < n; ++i)
				// 	for(int j =0; j < n; ++j) ss << G[i][j] << " ";

				// for(int j =0; j < int(obstacle_points_l.size()); ++j){
				// 	ss << obstacle_points_l[j][0] << " " << obstacle_points_l[j][1];
				// }


				// ss << x[0]/(x[2]-1) << " " << x[1]/(x[2]-1);
				// ss << solve_quadprog(G, gi0, CE, ce0, CI, ci0, x);
				// std_msgs::msg::String msg; msg.data = ss.str();
				// RCLCPP_INFO(this->get_logger(),"%s", msg.data.c_str());
				// msg.data.clear();

			}

	}


    




    std::pair <std::vector<std::vector<float>>, std::vector<float>> preprocess_lidar(std::vector<float> ranges){

			std::vector<std::vector<float>> data(ls_len_mod,std::vector<float>(2));
			std::vector<float> data2(100);

			//sets distance to zero for obstacles in safe distance, and max_lidar_range for those that are far.
			for(int i =0; i < ls_len_mod; ++i){
				if(ranges[ls_str+i] <= safe_distance) {data[i][0] = 0; data[i][1] = i*ls_ang_inc-angle_cen;}
				else if(ranges[ls_str+i] <= max_lidar_range) {data[i][0] = ranges[ls_str+i]; data[i][1] = i*ls_ang_inc-angle_cen;}
				else {data[i][0] = max_lidar_range; data[i][1] = i*ls_ang_inc-angle_cen;}
			}


			int k1 = 100; int k2 = 40;
			float s_range = 0; int index1, index2;
			
			//moving window
			for(int i =0; i < k1; ++i){
				s_range = 0;

				for(int j =0; j < k2; ++j){
					index1 = int(i*ranges.size()/k1+j);
					if(index1 >= int(ranges.size())) index1 -= ranges.size();
					
					index2 = int(i*ranges.size()/k1-j);
					if(index2 < 0) index2 += ranges.size();

					s_range += std::min(ranges[index1], (float) max_lidar_range) + std::min(ranges[index2], (float)max_lidar_range);

				}
				data2[i] = s_range;
			}

			return std::make_pair(data,data2);




	}

    std::pair<int, int> find_max_gap(std::vector<std::vector<float>> proc_ranges){
			int j =0; int str_indx = 0; int end_indx = 0; 
			int str_indx2 = 0; int end_indx2 = 0;
			int range_sum = 0; int range_sum_new = 0;

			/*This just finds the start and end indices of gaps (non-safe distance lidar return)
			then does a comparison to find the largest such gap.*/
			for(int i =0; i < ls_len_mod; ++i){
				if(proc_ranges[i][0] != 0){
					if (j==0){
						str_indx = i;
						range_sum_new = 0;
						j = 1;
					}
					range_sum_new += proc_ranges[i][0];
					end_indx = i;
				}
				if(j==1 && (proc_ranges[i][0] == 0 || i == ls_len_mod - 1)){
					j = 0;

					if(range_sum_new > range_sum){
						end_indx2 = end_indx;
						str_indx2 = str_indx;
						range_sum = range_sum_new;
					}
				}
			}

			return std::make_pair(str_indx2, end_indx2);
	}

    float find_best_point(int start_i, int end_i, std::vector<std::vector<float>> proc_ranges){
			float range_sum = 0;
			float best_heading =0;


			for(int i = start_i; i <= end_i; ++i){
				range_sum += proc_ranges[i][0];
				best_heading += proc_ranges[i][0]*proc_ranges[i][1];

			}

			if(range_sum != 0){
				best_heading /= range_sum;
			}

			return best_heading; 
	}

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr data){

        // RCLCPP_INFO(this->get_logger(), "receiving data");

        ls_ang_inc = static_cast<double>(data->angle_increment);
        scan_beams = int(2*M_PI/data->angle_increment);
		ls_str = int(round(scan_beams*(right_beam_angle))/(2*M_PI));
		ls_end = int(round(scan_beams*(left_beam_angle))/(2*M_PI));

		// RCLCPP_INFO(this->get_logger(), "%d, %d, %d", scan_beams, ls_str, ls_end);

		std::vector<float> fused_ranges = data->ranges;

		// for(int i =0; i < fused_ranges.size() ;++i){
		// 	RCLCPP_INFO(this->get_logger(), "%f", fused_ranges[i]);
		// }

		if(use_camera){
			if(cv_image_data_defined) {
				augment_camera(fused_ranges);
			}
		}



		if(!nav_active){ 
			drive_state = "normal";
			return; 	
		}

		

        //INTEGRATE CAMERA HERE


        std::pair<std::vector<std::vector<float>>, std::vector<float>> lidar_preprocess = preprocess_lidar(fused_ranges);

        std::vector<std::vector<float>> proc_ranges = lidar_preprocess.first;
		std::vector<float> mod_ranges = lidar_preprocess.second;


		// for(int i =0; i < ls_len_mod ;++i){
		// 	for(int j=0; j < 2; ++j){
		// 		RCLCPP_INFO(this->get_logger(), "%f", proc_ranges[i][j]);
		// 	}
		// }

		rclcpp::Time t = this->now();
		current_time = t.seconds();
		double dt = current_time - prev_time;
		prev_time = current_time;



        int sec_len = int(heading_beam_angle/data->angle_increment);


		double min_distance, velocity_scale, delta_d, velocity;

		// RCLCPP_INFO(this->get_logger(), "%s", data->header.frame_id.c_str());

		// RCLCPP_INFO(this->get_logger(), "%f", angle_cen);



        if(drive_state == "normal"){
			int str_indx, end_indx, index_l, index_r, start_indx_l, start_indx_r; double heading_angle;
			std::pair<int,int> max_gap = find_max_gap(proc_ranges);
			str_indx = max_gap.first; end_indx = max_gap.second;

			// RCLCPP_INFO(this->get_logger(), "%d, %d", str_indx, end_indx);


			heading_angle = find_best_point(str_indx, end_indx, proc_ranges);

			// RCLCPP_INFO(this->get_logger(), "%f", heading_angle);

			index_l = int(round((angle_bl-angle_al)/(data->angle_increment*n_pts_l)));
			index_r = int(round((angle_ar-angle_br)/(data->angle_increment*n_pts_r)));


			double mod_angle_al = static_cast<double>(angle_al+heading_angle);
			
			double mod_angle_br = static_cast<double> (angle_br + heading_angle);

			if(mod_angle_br > 2*M_PI) mod_angle_br -= 2*M_PI;
			else if (mod_angle_br < 0) mod_angle_br += 2*M_PI;

			start_indx_l = int(round(mod_angle_al/data->angle_increment));
			start_indx_r = int(round(mod_angle_br/data->angle_increment));


			std::vector<std::vector<double>> obstacle_points_l;
			obstacle_points_l.push_back({0, max_lidar_range_opt});
			obstacle_points_l.push_back({1, max_lidar_range_opt});



			std::vector<std::vector<double>> obstacle_points_r;
			obstacle_points_r.push_back({0, -max_lidar_range_opt});
			obstacle_points_r.push_back({1, -max_lidar_range_opt});

			// // RCLCPP_INFO(get_logger(), "%f", data->angle_increment);



			int k_obs = 0; int obs_index;

			double x_obs, y_obs;

			for(int k = 0; k < n_pts_l; ++k){
				obs_index = (start_indx_l + k*index_l) % scan_beams;

				double obs_range = static_cast<double>(fused_ranges[obs_index]);
					

				if(obs_range <= max_lidar_range_opt){


					if(k_obs == 0){
						obstacle_points_l[0] = {-obs_range*cos(mod_angle_al+k*index_l*data->angle_increment), -obs_range*sin(mod_angle_al+k*index_l*data->angle_increment) };
					}
					else if (k_obs == 1){
						obstacle_points_l[1] = {-obs_range*cos(mod_angle_al+k*index_l*data->angle_increment), -obs_range*sin(mod_angle_al+k*index_l*data->angle_increment) };
					}
					else{
						x_obs = -obs_range*cos(mod_angle_al+k*index_l*data->angle_increment);
						y_obs = -obs_range*sin(mod_angle_al+k*index_l*data->angle_increment);

						std::vector<double> obstacles = {x_obs, y_obs};
						obstacle_points_l.push_back(obstacles);
					}

					k_obs+=1;
				}
			}


			k_obs = 0;


			for(int k = 0; k < n_pts_r; ++k){
				obs_index = (start_indx_r+k*index_r) % scan_beams;
				double obs_range = static_cast<double>(fused_ranges[obs_index]);

				if(obs_range <= max_lidar_range_opt) {
						
					if(k_obs == 0){
						obstacle_points_r[0] = {-obs_range*cos(mod_angle_br+k*index_r*data->angle_increment), -obs_range*sin(mod_angle_br+k*index_r*data->angle_increment)};
					}
					else if(k_obs == 1){
						obstacle_points_r[1] = {-obs_range*cos(mod_angle_br+k*index_r*data->angle_increment),-obs_range*sin(mod_angle_br+k*index_r*data->angle_increment)};
					}
					else{
						x_obs = -obs_range*cos(mod_angle_br+k*index_r*data->angle_increment);
						y_obs = -obs_range*sin(mod_angle_br+k*index_r*data->angle_increment);

						std::vector<double> obstacles = {x_obs, y_obs};
						obstacle_points_r.push_back(obstacles);
	
					}

						k_obs += 1;
				}
			}


            double alpha = 1-exp(-dt/tau);

            std::vector<double> wl = {0.0, 0.0};
            std::vector<double> wr = {0.0, 0.0};


            getWalls(obstacle_points_l, obstacle_points_r, wl0, wr0, alpha, wr, wl);

			// RCLCPP_INFO(this->get_logger(), "left wall: ( %f, %f )", wl[0], wl[1]);

			wl0[0] = wl[0]; wl0[1] = wl[1]; 
			wr0[0] = wr[0]; wr0[1] = wr[1];


			double dl,dr;
			double wr_dot, wl_dot;
			wr_dot = wl_dot = 0;

			for(int i =0; i< 2; ++i){
				wl_dot += wl[i]*wl[i];
				wr_dot += wr[i]*wr[i];
			}

			dl=1/sqrt(wl_dot); dr = 1/sqrt(wr_dot);

			std::vector<double> wr_h = {wr[0] * dr , wr[1] * dr};
			std::vector<double> wl_h = {wl[0] * dl , wl[1] * dl};



			//TODO: RVIZ2 MARKERS 

			// RCLCPP_INFO(this->get_logger(), "HELLO!");
			marker.header.frame_id = "base_link";
			marker.header.stamp = this->now();
			marker.type = visualization_msgs::msg::Marker::LINE_LIST;
			marker.id = 0;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.scale.x = 0.1;
			marker.color.a = 1.0;
			marker.color.r = 0.5;
			marker.color.g = 0.5;
			marker.color.b = 0.0;
			marker.pose.orientation.w = 1;

			marker.lifetime = rclcpp::Duration (0.1);


			int line_len = 1;
			geometry_msgs::msg::Point p;
			marker.points.clear();


			p.x = dl*(-wl_h[0]-line_len*wl_h[1]);	p.y = dl*(-wl_h[1]+line_len*wl_h[0]);	p.z = 0; 
			marker.points.push_back(p);

			p.x = dl*(-wl_h[0]+line_len*wl_h[1]);	p.y = dl*(-wl_h[1]-line_len*wl_h[0]);	p.z = 0;
			marker.points.push_back(p);

			p.x = dr*(-wr_h[0]-line_len*wr_h[1]);	p.y = dr*(-wr_h[1]+line_len*wr_h[0]);	p.z = 0;
			marker.points.push_back(p);

			p.x = dr*(-wr_h[0]+line_len*wr_h[1]);	p.y = dr*(-wr_h[1]-line_len*wr_h[0]);	p.z = 0;
			marker.points.push_back(p);

			p.x = 0; p.y = 0; p.z = 0;
			marker.points.push_back(p);

			p.x = line_len*cosf(heading_angle);	p.y = line_len*sinf(heading_angle);	p.z = 0;
			marker.points.push_back(p);

			marker_pub->publish(marker);








			//Ackermann Steering

			if(vel >= 0.01 || vel <= -0.01){
				double d_tilde, d_tilde_dot;
				d_tilde = dl - dr - CenterOffset;
				d_tilde_dot = vel*(wl_h[0] - wr_h[0]);
				delta_d = atan(wheelbase*(k_p*d_tilde + k_d*d_tilde_dot)/(std::pow(vel,2)*(-wl_h[1]+wr_h[1])));
			}
			else delta_d = 0.0;

			// RCLCPP_INFO(this->get_logger(), "%f", delta_d);

			if(delta_d >= max_steering_angle) delta_d = max_steering_angle;
			else if (delta_d <= -max_steering_angle) delta_d = -max_steering_angle;

			min_distance = max_lidar_range + 100; int idx1, idx2;
			idx1 = -sec_len+int(scan_beams/2); idx2 = sec_len + int(scan_beams/2);

			for(int i =idx1; i <= idx2; ++i){
				if(fused_ranges[i] < min_distance) min_distance = fused_ranges[i];
			}

			velocity_scale = 1-exp(-std::max(min_distance - stop_distance, 0.0)/stop_distance_decay);
			velocity = velocity_scale*vehicle_velocity;

			if(velocity <= velocity_zero){

				if(time_ref ==0.0){
					t = this->now();
					time_ref = t.seconds();
				}

				t = this->now();
				double stopped_time = t.seconds() - time_ref;

				if(stopped_time >= stop_time1){
					// RCLCPP_INFO(this->get_logger(), "HELLO!");
					drive_state = "backup";
					time_ref = 0.0;

					yaw0 = yaw;
					turn_angle = arg_max(mod_ranges) * (2.0 * M_PI/mod_ranges.size()) - M_PI;

				}

			}
			else time_ref = 0.0;
		}

		else if(drive_state == "backup"){

			dtheta = yaw - yaw0;

			if(abs(dtheta) > 1.0){
				if(equiv_sign(dtheta) != equiv_sign(turn_angle)) dtheta += 4*M_PI;
				else dtheta -= 4*M_PI;
			}

			min_distance = *std::min_element(fused_ranges.begin(), fused_ranges.begin()+sec_len);
			velocity_scale = 1-exp(-std::max(min_distance-stop_distance, 0.0)/stop_distance_decay);

			delta_d = - equiv_sign(turn_angle)*max_steering_angle;
			velocity = -velocity_scale * turn_velocity;

			if(abs(dtheta) >= abs(turn_angle/2.0)){
				drive_state = "turn";
				time_ref = 0.0;
			}
			else if(-velocity <= velocity_zero){
				if(time_ref == 0.0){
					t = this->now();
					time_ref = t.seconds();
				}
				else{
					t = this->now();
					stopped_time = t.seconds() - time_ref;
					
					if(stopped_time >= stop_time2){
						drive_state = "turn";
						time_ref = 0.0;
					}
				}

			}
			else time_ref = 0.0;




		}

		else{

			min_distance = *std::min_element(fused_ranges.begin()-sec_len + int(scan_beams/2), fused_ranges.begin()+sec_len+int(scan_beams/2));
			velocity_scale = 1-exp(-std::max(min_distance-stop_distance, 0.0)/stop_distance_decay);

			delta_d = equiv_sign(turn_angle)*max_steering_angle;
			velocity = velocity_scale*turn_velocity;

			dtheta = yaw-yaw0;


			if(abs(dtheta) > 1.0){
				if(equiv_sign(dtheta) != equiv_sign(turn_angle)){
					if(dtheta <0) dtheta += 4*M_PI;
					else dtheta -= 4*M_PI;
				}
			}


			if(abs(dtheta) >= abs(turn_angle)){
				delta_d = 0.0;
				velocity = 0.0;

				if(time_ref == 0.0){
					t = this->now();
					time_ref = t.seconds();
					stopped_time = 0.0;
				}
				else{
					t = this->now();
					stopped_time = t.seconds() - time_ref;
				}

				if(stopped_time >= stop_time2){
					drive_state = "normal";
					time_ref = 0.0;
					wl0[0] = 0.0; wl0[1] = -1.0;
					wr0[0] = 0.0; wr0[1] = 1.0;
				}
			}
			else if(velocity <= velocity_zero){
				if(time_ref == 0.0){
					t = this->now();
					time_ref = t.seconds();
				}

				else{
					t=this->now();
					stopped_time = t.seconds()-time_ref;


					if(stopped_time >= 1.0){
						drive_state = "backup";
						time_ref  = 0.0;
						yaw0 = yaw;
						turn_angle = arg_max(mod_ranges)*(2.0* M_PI/mod_ranges.size())-M_PI;
					}
				}

			}
			else time_ref =0.0;

		}


		ackermann_msgs::msg::AckermannDriveStamped drive_msg;
		drive_msg.header.stamp = this->now();
		drive_msg.header.frame_id = "base_link";
		drive_msg.drive.steering_angle = delta_d;
		drive_msg.drive.speed = velocity;

		driver_pub->publish(drive_msg);
		

		// RCLCPP_INFO(this->get_logger(), "state: %s, steering: %f", drive_state.c_str(), delta_d);
		// RCLCPP_INFO(this->get_logger(), "time: %f", this->now().seconds());


    }


};




int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GapBarrier>();

    while(rclcpp::ok()){rclcpp::spin_some(node);}



}
