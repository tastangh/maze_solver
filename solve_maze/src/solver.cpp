#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/tf.h>

class Solver {
public:
    Solver() : nh("~") {
        scan_sub = nh.subscribe("/scan", 1, &Solver::scanCallback, this);
        odom_sub = nh.subscribe("/odom", 1, &Solver::odomCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        nh.param<double>("wall_distance", d, 0.5);
        nh.param<double>("parallel_band_width", r, 0.05);
        nh.param<double>("straight_vel", straight_vel, 0.15);
        nh.param<double>("rotate_vel", rotate_vel, 0.3);
        nh.param<double>("corner_threshold", corner_threshold, 0.35);
        
        nh.param<double>("odom_goal_x", odom_goal_x, 2.0);
        nh.param<double>("odom_goal_y", odom_goal_y, 2.0);

        wall_found = false;
        side = 1; // 1 sağ duvar, -1 sol duvar takibi

        odom_x = 0.0;
        odom_y = 0.0;
        odom_yaw = 0.0;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg_p) {
        scan_msg = *scan_msg_p;

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
        double front_side_distance = getDistanceAtAngle(side * M_PI / 4.0);  // 45 derece yan-ön
        double side_distance = getDistanceAtAngle(side * M_PI / 2.0);        // 90 derece tam yan
    
        ROS_INFO("Front: %.2f, Front-Side: %.2f, Side: %.2f", front_distance, front_side_distance, side_distance);
    
        // 90 derece iç köşe kontrolü
        if (front_distance < corner_threshold) {
            ROS_INFO("90-degree corner detected, turning inward");
            turnRobot(-side * M_PI / 2.0);
            ros::Duration(1.0).sleep();
            return;
        }
    
        // 270 derece dış köşe kontrolü (yan mesafede ani büyüme)
        if (side_distance > d + 0.6) {
            ROS_INFO("270-degree corner detected, turning outward");
            turnRobot(side * M_PI / 2.0);
            ros::Duration(1.0).sleep();
            return;
        }
    
        // Robotun duvara açısını hesapla (0 olması paralel olduğunu gösterir)
        double alpha = atan2(front_side_distance * cos(M_PI/4) - side_distance, front_side_distance * sin(M_PI/4));
    
        // Robotun duvara göre mesafesi gerçek anlamda hesaplanır
        double actual_side_distance = side_distance * cos(alpha);
    
        // Hata: hem açı (alpha), hem de mesafe (actual_side_distance - d) olarak ele alınır
        double error_distance = actual_side_distance - d;
        double error_angle = alpha;
    
        // Orantısal kontrolcü (ikili hata)
        double angular_z = Kp_distance * error_distance + Kp_angle * error_angle;
    
        // Açısal hızı sınırla
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
    

    void stopRobot() {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;
    sensor_msgs::LaserScan scan_msg;

    double d, r, straight_vel, rotate_vel, corner_threshold, odom_goal_x, odom_goal_y;
    bool wall_found;
    int side;
    double odom_x, odom_y, odom_yaw;
    const double Kp_distance = 1.0;  // Mesafe hata katsayısı
    const double Kp_angle = 1.5;     // Açı hata katsay
    };

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_following");
    Solver solver;
    ros::spin();
    return 0;
}
