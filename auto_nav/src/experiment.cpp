#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using std::placeholders::_1;



class Experiment : public rclcpp::Node {

    private:

        //lidar pub
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr erpm_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;




        //lidar sub
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

        //drive sub
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub;

        //joy sub
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

        //vesc sub
        rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub;


        // The car state and parameters
        double desired_speed=0;double desired_steer_ang=0; double last_servo=0; double last_rpm=0;


        //car parameters
        


        //vescs gains
        double speed_to_erpm_gain, speed_to_erpm_offset,
        steering_angle_to_servo_gain, steering_angle_to_servo_offset, wheelbase;


        double max_acceleration, max_servo_speed, driver_smoother_rate, max_delta_servo, max_delta_rpm;
        
        std_msgs::msg::Float64 last_servo_state; // Last recived servo state

                

        std_msgs::msg::Float64 erpm_msg;
        std_msgs::msg::Float64 servo_msg;
        std_msgs::msg::Float64 brake_msg;


        // std_msgs::msg::Float64 last_servo_state;
        vesc_msgs::msg::VescStateStamped last_vesc_state;


        rclcpp::TimerBase::SharedPtr update_command;

        int auto_nav;


        

        //topics
        std::string lidarscan_topic;



    public:
        Experiment() : Node("experiment"){

            lidarscan_topic = "/scan2";

            //pubs

            lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 1);

            // erpm_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed",1);
            // servo_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position",1);
            // brake_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/brake",1);

            // odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom",1);



            //subs

            lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan2", 
								     1, std::bind(&Experiment::lidar_callback, this, _1));


            // drive_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1, std::bind(&Experiment::drive_callback, this, _1));

            // joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/Joy",1, std::bind(&Experiment::joy_callback, this, _1));

            //vesc subs

            // vesc_sub = this->create_subscription<vesc_msgs::msg::VescStateStamped>("/sensors/core",1, std::bind(&Experiment::vesc_callback, this, _1));



            //gains
            this->declare_parameter("speed_to_erpm_gain", 0.0);
            this->declare_parameter("speed_to_erpm_offset", 0.0);
            this->declare_parameter("steering_angle_to_servo_gain", 0.0);
            this->declare_parameter("steering_angle_to_servo_offset", 0.0);

            speed_to_erpm_gain = this->get_parameter("speed_to_erpm_gain").as_double();
            speed_to_erpm_offset = this->get_parameter("speed_to_erpm_offset").as_double();
            steering_angle_to_servo_gain = this->get_parameter("steering_angle_to_servo_gain").as_double();
            steering_angle_to_servo_offset = this->get_parameter("steering_angle_to_servo_offset").as_double();



            //car params
            this->declare_parameter("wheelbase", 0.0);
            this->declare_parameter("max_accel", 0.0);
            this->declare_parameter("max_steering_vel", 0.0);
            this->declare_parameter("driver_smoother_rate", 0.0);


            wheelbase = this->get_parameter("wheelbase").as_double();
            max_acceleration = this->get_parameter("max_accel").as_double();
            max_servo_speed = this->get_parameter("max_steering_vel").as_double();
            driver_smoother_rate = this->get_parameter("driver_smoother_rate").as_double();

            

            desired_steer_ang=steering_angle_to_servo_offset;
            last_servo=steering_angle_to_servo_offset;
            last_servo_state.data=steering_angle_to_servo_offset;



            max_delta_servo = std::abs(steering_angle_to_servo_gain * max_servo_speed / driver_smoother_rate);

            // update_command = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration (1.0/driver_smoother_rate), std::bind(&Experiment::publish_driver_command, this));
            





        }

        void vesc_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr state){

            double current_speed = (state->state.speed - speed_to_erpm_offset)/(speed_to_erpm_gain);


            if(std::fabs(current_speed) < 0.15 || std::fabs(state->state.current_motor) < 0.01){
                current_speed = 0.0;
            }

            double current_steering_angle, current_angular_velocity;

            current_steering_angle = (last_servo_state.data - steering_angle_to_servo_offset)/ steering_angle_to_servo_gain;





        }

        // void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){

        //     auto_nav = msg->buttons[5];

        //     RCLCPP_INFO(this->get_logger(), "%d", auto_nav);

        // }


        void publish_driver_command(){
            double desired_delta = desired_steer_ang - last_servo;
            double clipped_delta = std::max(std::min(desired_delta, max_delta_servo), -max_delta_servo);

            double smoothed_servo = last_servo + clipped_delta;
            last_servo  = smoothed_servo;
            servo_msg.data = smoothed_servo;

            double desired_rpm = desired_speed - last_rpm;
            double clipped_rpm = std::max(std::min(desired_rpm, max_delta_rpm), -max_delta_rpm);
            double smoothed_rpm = last_rpm + clipped_rpm; 
            last_rpm = smoothed_rpm;
            erpm_msg.data = smoothed_rpm;


            servo_pub->publish(servo_msg);
            erpm_pub->publish(erpm_msg);


        }


        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr data){


            int n1 = data->ranges.size()/2;

            std::vector<float> scan_ranges = data->ranges;
            std::vector<float> scan_intensities = data->intensities;
            for(int i =0; i < n1; ++i){
                scan_ranges[i] = data->ranges[n1+i];
                scan_ranges[n1+i] = data->ranges[i];
                scan_intensities[i] = data->intensities[n1+i];
                scan_intensities[n1+i] = data->intensities[i];
            }

            sensor_msgs::msg::LaserScan scan_msg;
            scan_msg.header.stamp = data->header.stamp;
            scan_msg.header.frame_id = data->header.frame_id;
            scan_msg.angle_min = data->angle_min + 3.141592;
            scan_msg.angle_max = data->angle_max + 3.141592;
            scan_msg.angle_increment = data->angle_increment;
            scan_msg.range_max = data->range_max;
            scan_msg.ranges = scan_ranges;
            scan_msg.intensities = scan_intensities;
            
            
            lidar_pub->publish(scan_msg);

        }

        // void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){

        //     desired_speed = msg->drive.speed;
        //     desired_steer_ang = msg->drive.steering_angle;
        //     desired_speed = speed_to_erpm_gain * desired_speed + speed_to_erpm_offset;
        //     desired_steer_ang = steering_angle_to_servo_gain * desired_steer_ang + steering_angle_to_servo_offset;
        // }













};



int main(int argc, char ** argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Experiment>();

    while(rclcpp::ok()){rclcpp::spin_some(node);}



}