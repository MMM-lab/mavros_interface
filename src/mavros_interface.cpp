/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_broadcaster.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

float vel{0.0};
float angular_vel{0.0};
ros::Time last_msg_time{}; //TODO: include in main

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    last_msg_time = ros::Time::now();
    geometry_msgs::Twist cmd_vel;
    cmd_vel = *msg;
    vel = cmd_vel.linear.x;
    angular_vel = cmd_vel.angular.z;
}

void decode_sp(geometry_msgs::PoseStamped *attitude_sp, mavros_msgs::Thrust *thrust_sp, float thrust, float yaw){

    ros::Time now_time = ros::Time::now();
    attitude_sp->header.stamp = now_time;
    thrust_sp->header.stamp = now_time;

    tf::Quaternion quat=tf::createQuaternionFromRPY(0.0 , 0.0, yaw);
    geometry_msgs::Quaternion geometry_quat;
    tf::quaternionTFToMsg(quat, geometry_quat);
    attitude_sp->pose.orientation = geometry_quat;
    thrust_sp->thrust = vel;
}

float yaw_sp(float angular_vel){
    static ros::Time last_time{};
    ros::Time now_time = ros::Time::now();
    float since_last = (now_time - last_time).toSec();
    last_time = now_time;

    static float yaw = 0.0;

    if ( since_last < 1.0f && angular_vel < 1.0f  ) {
        yaw += angular_vel * since_last;
    }

    return yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("cmd_vel", 10, cmd_vel_cb);
    ros::Publisher attitude_sp_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher thrust_sp_pub = nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped attitude_sp;
    mavros_msgs::Thrust thrust_sp;

    decode_sp(&attitude_sp, &thrust_sp, 0.0f, 0.0f);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
	attitude_sp_pub.publish(attitude_sp);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
		ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if ( (ros::Time::now() - last_msg_time).toSec() > 0.5 ) {
            vel = 0.0;
            angular_vel = 0.0;
        }
        decode_sp(&attitude_sp, &thrust_sp, vel, yaw_sp(angular_vel));
	attitude_sp_pub.publish(attitude_sp);
	thrust_sp_pub.publish(thrust_sp);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

