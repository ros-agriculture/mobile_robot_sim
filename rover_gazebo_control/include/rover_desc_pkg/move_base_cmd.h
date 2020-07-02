#ifndef CMD_VEL_TO_ACKERMANN_H
#define CMD_VEL_TO_ACKERMANN_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>

#define WHEELBASE 0.806

#define MAX_SPEED 100
#define MAX_STEERING_ANGLE 1.57

/**
 * Class that convert cmd_vel to ackermann_cmd
 */
class move_base_cmd {
public:
    move_base_cmd();

    void publish();

private:
    ros::NodeHandle nh_;
    bool called{};
    // Speed.
    double speed{};
    double w{};
    double m_speed{};

    // Steering.
    double m_steering_angle{};

    // Messages.
    geometry_msgs::Twist cmd_vel;
    ackermann_msgs::AckermannDriveStamped ackermann;
    ackermann_msgs::AckermannDriveStamped ackermann1;

    /* Subscriber to the servo topic. */
    ros::Subscriber cmd_vel_sub;

    /* Callback for the servo topic. */
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

    /* Publisher to the ackermann_cmd topic. */
    ros::Publisher ackermann_cmd_pub;

    void convertTransRotToSteeringAngle(double speed, double w, double wheelbase);


};

#endif /* CMD_VEL_TO_ACKERMANN_H */
