
#include "rover_desc_pkg/move_base_cmd.h"
#include <iostream>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;
#define MAX_STEER_WHEEL 0.8
#define MAX_SPEED_BODY 5

move_base_cmd::move_base_cmd() {

    // Subscribe to the "servo" topic.
    cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &move_base_cmd::cmdVelCallback, this);
    ackermann_cmd_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("rover/ackermann_cmd", 1);
}

/**
 * Convert cmd_vel to ackermann_msgs and publish it to ackermann_cmd.
 */
void move_base_cmd::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    speed = msg->linear.x;
    w = msg->angular.z;
    called = true;
    // Convert to ackermann steering.

}

void move_base_cmd::publish() {
    if (called) {
        convertTransRotToSteeringAngle(speed, w, WHEELBASE);

//     Check values.
        if (m_speed > 0 && m_speed > MAX_SPEED) {
            m_speed = MAX_SPEED;
        } else if (m_speed < 0 && m_speed < -1 * MAX_SPEED) {
            m_speed = -MAX_SPEED;
        }

        if (m_steering_angle > 0 && m_steering_angle > MAX_STEERING_ANGLE) {
            m_steering_angle = MAX_STEERING_ANGLE;
        } else if (m_steering_angle < 0 && m_steering_angle < -1 * MAX_STEERING_ANGLE) {
            m_steering_angle = -MAX_STEERING_ANGLE;
        }

        // Publish the data.

        if (ackermann.drive.steering_angle <= MAX_STEER_WHEEL && ackermann.drive.steering_angle >= -MAX_STEER_WHEEL) {
            ackermann.drive.steering_angle = m_steering_angle;

        }

        if (ackermann.drive.speed <= MAX_SPEED_BODY && ackermann.drive.speed >= -MAX_SPEED_BODY) {
            ackermann.drive.speed = m_speed;

        }
        if (ackermann.drive.speed > MAX_SPEED_BODY) {
            ackermann.drive.speed = MAX_SPEED_BODY;

        }

        if (ackermann.drive.speed < -MAX_SPEED_BODY) {
            ackermann.drive.speed = -MAX_SPEED_BODY;

        }


        if (ackermann.drive.steering_angle > MAX_STEER_WHEEL) {
            ackermann.drive.steering_angle = MAX_STEER_WHEEL - 0.1;
        }
        if (ackermann.drive.steering_angle < -MAX_STEER_WHEEL) {
            ackermann.drive.steering_angle = -MAX_STEER_WHEEL + 0.1;
        }
        ackermann.header.stamp = ros::Time::now();
        ackermann.header.frame_id = "chassis_footprint";

        if (ackermann_cmd_pub.getNumSubscribers() > 0) {
            cout << ackermann.drive.steering_angle << endl;
            cout << ackermann.drive.speed << endl;
            ackermann_cmd_pub.publish(ackermann);
        } else {

        }
    } else {
        ackermann1.header.stamp = ros::Time::now();
        ackermann1.header.frame_id = "chassis_footprint";
        ackermann1.drive.steering_angle = ackermann.drive.steering_angle;
        ackermann1.drive.speed = ackermann.drive.speed;
        ackermann_cmd_pub.publish(ackermann1);
    }
    called = false;

}

void move_base_cmd::convertTransRotToSteeringAngle(double speed, double w, double wheelbase) {
    if (speed == 0 && w == 0) {
        m_speed = 0.0;
        m_steering_angle = 0.0;

        return;
    }

    double radius = speed / w;

    // Prepare the speed for ackermann_cmd.
    m_speed = speed;
    m_steering_angle = atan(wheelbase / radius);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_base_cmd");
    move_base_cmd cmd_to_ackermann_m;
    ros::Rate loop_rate(50);
    while (ros::ok()) {

        cmd_to_ackermann_m.publish();


        ros::spinOnce();
    }
    return 0;
}
