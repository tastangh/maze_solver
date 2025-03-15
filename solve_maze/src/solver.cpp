#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/tf.h>

class Solver {
public:
    Solver() : nh("~"), moved_backwards(false), wall_found(false) {
        scan_sub = nh.subscribe("/scan", 1, &Solver::scanCallback, this);
        odom_sub = nh.subscribe("/odom", 1, &Solver::odomCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        nh.param<double>("wall_distance", d, 0.5);
        nh.param<double>("parallel_band_width", r, 0.05);
        nh.param<double>("straight_vel", straight_vel, 0.3);
        nh.param<double>("rotate_vel", rotate_vel, 0.5);
        nh.param<double>("corner_threshold", corner_threshold, 0.35);
        
        nh.param<double>("odom_goal_x", odom_goal_x, 2.0);
        nh.param<double>("odom_goal_y", odom_goal_y, 2.0);

        side = 1; // 1: Sağ duvar, -1: Sol duvar takibi
        odom_x = 0.0;
        odom_y = 0.0;
        odom_yaw = 0.0;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg_p) {
        scan_msg = *scan_msg_p;

        double front_distance = getDistanceAtAngle(0.0);
        double min_safe_distance = 0.4; 

        if (!moved_backwards && front_distance < min_safe_distance) {
            ROS_WARN("Too close to the wall! Moving backward...");
            moveBackward(0.2, 1.0);
            ros::Duration(1.0).sleep();
            moved_backwards = true;
        }

        if (!wall_found) {
            findNearestWall();
            return;
        }

        if (goalReached()) {
            stopRobot();
            ROS_INFO("Goal reached!");
            ros::shutdown();
            return;
        }

        followWall();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        odom_x = odom_msg->pose.pose.position.x;
        odom_y = odom_msg->pose.pose.position.y;

        tf::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_yaw = yaw;
    }

    void findNearestWall() {
        double min_distance = scan_msg.range_max;
        int min_index = -1;

        for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
            if (scan_msg.ranges[i] < min_distance && scan_msg.ranges[i] > scan_msg.range_min) {
                min_distance = scan_msg.ranges[i];
                min_index = i;
            }
        }

        if (min_index != -1) {
            double angle = scan_msg.angle_min + min_index * scan_msg.angle_increment;
            ROS_INFO("Nearest wall distance: %f, Angle: %f", min_distance, angle);
            turnToAngle(angle);
            ros::Duration(1.0).sleep();
            wall_found = true;
        } else {
            ROS_WARN("No wall found!");
            stopRobot();
        }
    }

    void followWall() {
        double front_distance = getDistanceAtAngle(0.0);
        double front_side_distance = getDistanceAtAngle(side * M_PI / 4.0); 
        double side_distance = getDistanceAtAngle(side * M_PI / 2.0); 
    
        ROS_INFO("Front: %.2f, Front-Side: %.2f, Side: %.2f", front_distance, front_side_distance, side_distance);
    
        if (front_distance < corner_threshold) {
            ROS_INFO("90-degree corner detected, turning inward");
            turnRobot(-side * M_PI / 2.0);
            ros::Duration(1.0).sleep();
            return;
        }
    
        if (side_distance > d + 0.6) {
            ROS_INFO("270-degree corner detected, turning outward");
            turnRobot(side * M_PI / 2.0);
            ros::Duration(1.0).sleep();
            return;
        }
    
        double alpha = atan2(front_side_distance * cos(M_PI/4) - side_distance, front_side_distance * sin(M_PI/4));
        double actual_side_distance = side_distance * cos(alpha);
    
        double error_distance = actual_side_distance - d;
        double error_angle = alpha;
    
        double angular_z = Kp_distance * error_distance + Kp_angle * error_angle;
        angular_z = std::max(std::min(angular_z, rotate_vel), -rotate_vel);
    
        cmd_vel_msg.linear.x = straight_vel;
        cmd_vel_msg.angular.z = angular_z;
        cmd_vel_pub.publish(cmd_vel_msg);
    }

    double getDistanceAtAngle(double angle) {
        int index = angleToIndex(angle);
        if (index >= 0 && index < scan_msg.ranges.size())
            return scan_msg.ranges[index];
        return scan_msg.range_max;
    }

    int angleToIndex(double angle) {
        int index = (int)((angle - scan_msg.angle_min) / scan_msg.angle_increment);
        return std::min(std::max(index, 0), (int)scan_msg.ranges.size() - 1);
    }

    void turnRobot(double angle) {
        double target_yaw = odom_yaw + angle;
        turnToAngle(target_yaw);
        ros::Duration(0.5).sleep();
    }

    void turnToAngle(double target_yaw) {
        target_yaw = atan2(sin(target_yaw), cos(target_yaw));
        double error = target_yaw - odom_yaw;
        error = atan2(sin(error), cos(error));

        ros::Rate rate(20);
        while (fabs(error) > 0.02) {
            double direction = error > 0 ? 1.0 : -1.0;
            cmd_vel_msg.angular.z = direction * rotate_vel;
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_pub.publish(cmd_vel_msg);
            ros::spinOnce();
            rate.sleep();

            error = atan2(sin(target_yaw - odom_yaw), cos(target_yaw - odom_yaw));
        }
        stopRobot();
    }

    bool goalReached() {
        return fabs(odom_x) <= 1.0 && fabs(odom_y) <= 1.0;
    }

    void moveBackward(double speed, double duration) {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(duration);
    
        cmd_vel_msg.linear.x = -speed;
        cmd_vel_msg.angular.z = - speed;

        ros::Rate rate(10);
        while (ros::Time::now() - start_time < timeout) {
            cmd_vel_pub.publish(cmd_vel_msg);
            rate.sleep();
        }
        stopRobot();
    }

    void stopRobot() {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub, odom_sub;
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;
    sensor_msgs::LaserScan scan_msg;

    double d, r, straight_vel, rotate_vel, corner_threshold, odom_goal_x, odom_goal_y;
    bool moved_backwards, wall_found;
    int side;
    double odom_x, odom_y, odom_yaw;
    const double Kp_distance = 1;
    const double Kp_angle = 1.5;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_following");
    Solver solver;
    ros::spin();
    return 0;
}
