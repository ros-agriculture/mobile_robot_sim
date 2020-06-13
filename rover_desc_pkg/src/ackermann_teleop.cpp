// general include statements
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <termios.h>
#include <unistd.h>
// rover_desc_pkg message include statements
#include <rover_desc_pkg/CheckpointMsg.h>
#include <rover_desc_pkg/ControlMsg.h>
#include <rover_desc_pkg/GoalMsg.h>
#include <rover_desc_pkg/ObjectMsg.h>
#include <rover_desc_pkg/ObjectStateMsg.h>
#include <rover_desc_pkg/PositionMsg.h>
#include <rover_desc_pkg/StateMsg.h>
#include <rover_desc_pkg/TransmissionMsg.h>

// define messages as shorter and easier to read variables, one line per message
// type to be used by node
typedef rover_desc_pkg::ControlMsg myControlMsg;

// namespaces
using namespace std;

/* Declarations */
// functions
char getUserInput();

// parameters
double steer_step;
double accel_step;

// variables
double accel_k_in;
double head_k_in;
char userInput;

// main function that runs while node is active, NOT a while loop unless you
// include one
int main(int argc, char **argv) {
    // intialize ROS and set node name, should generally be file name
    ros::init(argc, argv, "ackermann_teleop");

    // create ROS object to access communication operations
    ros::NodeHandle n;

    // create Publisher object and set topic to publish to and queue size for
    // published messages
    ros::Publisher pub_1 = n.advertise<myControlMsg>("control_teleop", 5);

    // set publish rate, in Hz
    ros::Rate loop_rate(5);

    // initialize parameters
    n.param<double>("steer_step", steer_step, 0.1);
    n.param<double>("accel_step", accel_step, 1.0);

    double count = 0; // counter for running through while loop, used for example
    // while loop that runs while node is active
    while (1) {
        // create message object to get filled with data and then published
        myControlMsg output;

        /* CODE GOES HERE */
        userInput = getUserInput();

        // reset control variables
        accel_k_in = 0.0;
        head_k_in = 0.0;

        if (userInput == 's') {
            accel_k_in = -1000000; // negative infinity is read as code for full brake
            head_k_in =
                    -1000000; // negative infinity is read as code for reset steering
        } else if (userInput == 'a') {
            head_k_in = steer_step;
        } else if (userInput == 'd') {
            head_k_in = -steer_step;
        } else if (userInput == 'w') {
            accel_k_in = accel_step;
        } else if (userInput == 'x') {
            accel_k_in = -accel_step;
        } else if (userInput == 'q') {
            break;
        } else {
            cout << "Please enter valid input" << endl;
        }

        // fill message data fields
        output.header.stamp = ros::Time::now(); // this line likely won't change
        output.vel_k_in = accel_k_in; // this line can be set to your variable
        output.head_k_in = head_k_in; // this line can be set to your variable

        // publish message object, type to publish must agree with declared publish
        // type of publish object
        pub_1.publish(output);

        // trigger any callbacks
        ros::spinOnce();

        // pause as long as needed to meet publish rate
        // loop_rate.sleep();

        ++count; // increment our example counter
    }

    return 0;
}

char getUserInput() {

    char buf = 0;

    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");

    return (buf);
}