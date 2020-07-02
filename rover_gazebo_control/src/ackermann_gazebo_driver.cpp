// general include statements
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
// rover_desc_pkg message include statements
#define WHEELBASE 0.806
#define WHEEL_WIDTH 0.356

#define MAX_SPEED 100
#define MAX_STEERING_ANGLE 1.309



// define messages as shorter and easier to read variables, one line per message
// type to be used by node


// namespaces
using namespace std;

// class that does all subscribing and publishing
class SubscribeAndPublish {
    std_msgs::Float64 left_wheel_msg;
    std_msgs::Float64 right_wheel_msg;
    std_msgs::Float64 vel_msg_left;
    std_msgs::Float64 vel_msg_right;
    std_msgs::Float64 vel_msg;

public:
    SubscribeAndPublish() {



        // set publisher topics and queue sizes for published messages
        left_steer_block_publisher = n.advertise<std_msgs::Float64>(
                "/rover/front_left_steering_ctrlr/command", 1000);
        right_steer_block_publisher = n.advertise<std_msgs::Float64>(
                "/rover/front_right_steering_ctrlr/command", 1000);
        left_front_wheel_publisher = n.advertise<std_msgs::Float64>(
                "/rover/front_left_wheel_ctrlr/command", 1000);
        right_front_wheel_publisher = n.advertise<std_msgs::Float64>(
                "/rover/front_right_wheel_ctrlr/command", 1000);
        left_rear_wheel_publisher = n.advertise<std_msgs::Float64>(
                "/rover/rear_left_wheel_ctrlr/command", 1000);
        right_rear_wheel_publisher = n.advertise<std_msgs::Float64>(
                "/rover/rear_right_wheel_ctrlr/command", 1000);

        //  set subscriber topic, queue size for subscribed messages, and callback
        //        function to be called whenever messages are detected on topic
//        sub_1 = n.subscribe("control_model", 10, &SubscribeAndPublish::callback,
//                            this);

        sub_2 = n.subscribe<ackermann_msgs::AckermannDriveStamped>("rover/ackermann_cmd", 10,
                                                                   &SubscribeAndPublish::commandCallback,
                                                                   this);

    }

//
//     callback function, takes in message type from subscribed topic, gets called
//     whenever new messages are detected on subscribed topic

    void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &input) {
        double previous_right = right_wheel_msg.data;
        double previous_left = left_wheel_msg.data;


        double steer_mid = input->drive.steering_angle;
        double vel_rear = input->drive.speed;
        double angular_velocity = vel_rear * tan(steer_mid) / WHEELBASE;

        std::vector<double> angles = GetAckAngles(steer_mid);
        std::vector<double> speeds = getDiffSpeeds(vel_rear, steer_mid);

        // fill message data fields

        vel_msg.data = vel_rear / 0.082;


        if (angles[1] > 0.8) {
            left_wheel_msg.data = 0.802;
            right_wheel_msg.data = 0.6175;
//            cout << left_wheel_msg.data << " left" <<endl;
//            cout << right_wheel_msg.data << " right" <<endl;
//            cout << angles[1] << endl;
        } else {
            left_wheel_msg.data = angles[0];
            right_wheel_msg.data = angles[1];
        }

        if (angles[1] < -0.8) {
            right_wheel_msg.data = -0.802;
            left_wheel_msg.data = -0.6175;
//            cout << left_wheel_msg.data << " left" <<endl;
//            cout << right_wheel_msg.data << "right" << endl;
//            cout << angles[0] << endl;
        } else {
            right_wheel_msg.data = angles[1];
            left_wheel_msg.data = angles[0];
        }


        vel_msg_left.data = speeds[0] / 0.082;
        vel_msg_right.data = speeds[1] / 0.082;


        // publish message object, type to publish must agree with declared publish
        // type of publish object

    }

    void pub() {
        left_steer_block_publisher.publish(left_wheel_msg);
        right_steer_block_publisher.publish(right_wheel_msg);
        left_rear_wheel_publisher.publish(vel_msg_left);
        right_rear_wheel_publisher.publish(vel_msg_right);
        left_front_wheel_publisher.publish(vel_msg);
        right_front_wheel_publisher.publish(vel_msg);
    }

    static std::vector<double> GetAckAngles(double phi) {
        std::vector<double> phi_angles;
        double numerator = 2.0 * WHEELBASE * sin(phi);
        phi_angles.assign(4, 0.0);

        phi_angles[0] = atan2(numerator,
                              (2.0 * WHEELBASE * cos(phi) - WHEEL_WIDTH * sin(phi)));
        phi_angles[1] = atan2(numerator,
                              (2.0 * WHEELBASE * cos(phi) + WHEEL_WIDTH * sin(phi)));
        return phi_angles;
    }

    static std::vector<double> getDiffSpeeds(double vel, double phi) {
        std::vector<double> wheel_speeds;
        wheel_speeds.assign(4, 0.0);
        wheel_speeds[0] = vel * (1.0 - (WHEEL_WIDTH * tan(phi)) /
                                       (2.0 * WHEELBASE));
        wheel_speeds[1] = vel * (1.0 + (WHEEL_WIDTH * tan(phi)) /
                                       (2.0 * WHEELBASE));
        return wheel_speeds;
    }

private:
    // create ROS object to access communication operations
    ros::NodeHandle n;

    // create Publisher object
    ros::Publisher pub_1;
    ros::Publisher left_steer_block_publisher;
    ros::Publisher right_steer_block_publisher;
    ros::Publisher left_rear_wheel_publisher;
    ros::Publisher right_rear_wheel_publisher;
    ros::Publisher left_front_wheel_publisher;
    ros::Publisher right_front_wheel_publisher;

    // create Subscriber object
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
}; // End of class SubscribeAndPublish

// main function that runs while node is active, NOT a while loop unless you
// include one
int main(int argc, char **argv) {
    // Initiate ROS and set node name, should generally be file name
    ros::init(argc, argv, "rover_gazebo_driver");

    // Create an object of class SubscribeAndPublish that will take care of
    // everything
    SubscribeAndPublish SAPObject;
    ros::Rate loop_rate(50);
    // trigger any callbacks
    //  ros::spin();

    while (ros::ok()) {
        SAPObject.pub();
        ros::spinOnce();
        loop_rate.sleep(); // Don't forget this! *
    }

    return 0;
}