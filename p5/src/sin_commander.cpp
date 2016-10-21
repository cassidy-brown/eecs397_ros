// Cassidy Brown cmb195
// sin_commander node for project 4
// Ppublishes to pos_cmd and pos_cmd2
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<math.h>

#define PI 3.14159265

//global variables for callback functions to populate for use in main program 
std_msgs::Float64 g_sin_vel_cmd;


int main(int argc, char **argv) {
    ros::init(argc, argv, "commander_p5"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "sin_commander" 
    ros::NodeHandle nh; // node handle 
    //publish a velocity command computed by this controller; 
    ros::Publisher pos_publisher = nh.advertise<std_msgs::Float64>("three_DOF_robot/joint1_position_controller/command", 1);
    ros::Publisher pos_publisher2 = nh.advertise<std_msgs::Float64>("three_DOF_robot/joint2_position_controller/command", 1);
    ros::Publisher pos_publisher3 = nh.advertise<std_msgs::Float64>("three_DOF_robot/joint3_position_controller/command", 1);

    double Kv = 2.0; // velocity feedback gain 
    double dt_controller = 0.10; // 10Hz controller sample rate
    double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 

    std_msgs::Float64 sin_pos_cmd;
    sin_pos_cmd.data = 0.0; // init velocity command to zero
    double radians = 0.0; // initialize at zero radians

    std_msgs::Float64 sin_pos_cmd2;
    sin_pos_cmd2.data = 0.0; // init velocity command to zero

    std_msgs::Float64 sin_pos_cmd3;
    sin_pos_cmd3.data = 0.0; // init velocity command to zero

    // enter the main loop: get velocity state and velocity commands 
    // compute command force to get system velocity to match velocity command 
    // publish this force for use by the complementary simulator 
    while (ros::ok()) {
        radians = radians + dt_controller * 3.14159265 / 2; // increment radian value by a magnitude of pi
							//radians will increase by Pi/2 every second
        //determine velocity to command with amplitude and frequency 
        sin_pos_cmd.data = sin(radians * 0.8) - .5;  
        sin_pos_cmd2.data = sin(radians * 1.2);   
        sin_pos_cmd3.data = sin(radians * 1.4) - .5;   

        pos_publisher.publish(sin_pos_cmd);
        pos_publisher2.publish(sin_pos_cmd2);
        pos_publisher3.publish(sin_pos_cmd3);
        ROS_INFO("position command = %f", sin_pos_cmd.data);
        ROS_INFO("position command 2 = %f", sin_pos_cmd2.data);
        ROS_INFO("position command 3 = %f", sin_pos_cmd3.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
} 
