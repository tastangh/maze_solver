#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/transform_datatypes.h>

class Solver {
public:
    Solver() : nh("~") {
        scan_sub = nh.subscribe("/scan", 1, &Solver::scanCallback, this);
        odom_sub = nh.subscribe("/odom", 1, &Solver::odomCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        nh.param<double>("wall_distance", d, 0.5);
        nh.param<double>("parallel_band_width", r, 0.1);
        nh.param<double>("straight_vel", straight_vel, 0.2);
        nh.param<double>("rotate_vel", rotate_vel, 0.3);
        nh.param<double>("corner_threshold", corner_threshold, 0.6);

        odom_goal_x = 0.0;
        odom_goal_y = 0.0;

        wall_found = false;
        side = -1;  // Sağdan takip varsayılan

        odom_x = 0.0;
        odom_y = 0.0;
        odom_yaw = 0.0;

        // 🚀 İlk hareket: Geri giderek duvardan uzaklaş
        moveBackward(0.5);  // 0.5 saniye geri git
        ros::Duration(0.5).sleep();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg_p) {
        scan_msg = *scan_msg_p;

        if (!wall_found) {
            findNearestWall();
            return;
        }

        if (goalReached()) {
            stopRobot();
            ROS_INFO("🏁 Labirent başarıyla tamamlandı! 🏁");
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
            ROS_INFO("📏 En yakın duvar mesafesi: %f, Açı: %f", min_distance, angle);

            turnToAngle(angle);
            ros::Duration(1.0).sleep();

            wall_found = true;
        } else {
            ROS_WARN("⚠️ Çevrede duvar bulunamadı, bekleniyor...");
            stopRobot();
        }
    }

    void followWall() {
        double front_distance = getDistanceAtAngle(0.0);
        double side_distance = getDistanceAtAngle(side * M_PI / 2.0);

        ROS_INFO("⬆️ Ön mesafe: %f, ➡️ Yan mesafe: %f", front_distance, side_distance);

        if (front_distance < corner_threshold) {
            ROS_INFO("🔄 90° köşe algılandı, dönüş yapılıyor.");
            turnRobot(side * M_PI / 2.0);
            ros::Duration(1.0).sleep();
            return;
        }

        if (side_distance > d + r) {
            ROS_INFO("🔄 270° köşe algılandı, dışa doğru yöneliniyor.");
            turnRobot(side * -M_PI / 2.0);
            ros::Duration(1.0).sleep();
            return;
        }

        double error = side_distance - d;
        double angular_z = Kp * error;
        angular_z = std::max(std::min(angular_z, rotate_vel), -rotate_vel);

        cmd_vel_msg.linear.x = straight_vel;
        cmd_vel_msg.angular.z = angular_z;
        cmd_vel_pub.publish(cmd_vel_msg);
    }

    double getDistanceAtAngle(double angle) {
        int index = angleToIndex(angle);
        if (index >= 0 && index < scan_msg.ranges.size()) {
            return scan_msg.ranges[index];
        }
        return scan_msg.range_max;
    }

    int angleToIndex(double angle) {
        double relative_angle = angle;
        return (int)((relative_angle - scan_msg.angle_min) / scan_msg.angle_increment);
    }

    void turnRobot(double angle) {
        double target_yaw = odom_yaw + angle;
        turnToAngle(target_yaw);
    }

    void turnToAngle(double target_yaw) {
        target_yaw = atan2(sin(target_yaw), cos(target_yaw));
        double error = target_yaw - odom_yaw;
        error = atan2(sin(error), cos(error));

        while (fabs(error) > 0.05) {
            double direction = (error > 0) ? 1.0 : -1.0;
            cmd_vel_msg.angular.z = direction * rotate_vel;
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_pub.publish(cmd_vel_msg);

            ros::spinOnce();
            ros::Duration(0.05).sleep();

            error = target_yaw - odom_yaw;
            error = atan2(sin(error), cos(error));
        }

        stopRobot();
    }

    void moveBackward(double duration) {
        ROS_INFO("⬅️ Geri hareket: %f saniye", duration);
        cmd_vel_msg.linear.x = -0.1;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::Duration(duration).sleep();
        stopRobot();
    }

    bool goalReached() {
        return (fabs(odom_x) <= 1.0 && fabs(odom_y) <= 1.0);
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

    double d;
    double r;
    double straight_vel;
    double rotate_vel;
    double corner_threshold;
    double odom_goal_x, odom_goal_y;

    bool wall_found;
    int side;

    double odom_x;
    double odom_y;
    double odom_yaw;

    double Kp = 0.5;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_following");
    Solver solver;
    ros::spin();
    return 0;
}
