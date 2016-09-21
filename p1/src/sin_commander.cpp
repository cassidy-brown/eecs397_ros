// sin_commander node: 
// based on minimal_controller
// subscribes to "amp_cmd" and "freq_cmd"
// publishes "vel_cmd" 
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<math.h>

//global variables for callback functions to populate for use in main program 
std_msgs::Float64 g_amp;
std_msgs::Float64 g_freq; 
std_msgs::Float64 g_sin_vel_cmd;

void myCallbackAmplitude(const std_msgs::Float64& message_holder) {
    // check for data on topic "amp_cmd" 
    ROS_INFO("received amplitude value is: %f", message_holder.data);
    g_amp.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

void myCallbackFrequency(const std_msgs::Float64& message_holder) {
    // check for data on topic "freq_cmd" 
    ROS_INFO("received frequency command value is: %f", message_holder.data);
    g_freq.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander_p1"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "sin_commander" 
    ros::NodeHandle nh; // node handle 
    //create 2 subscribers: one each for sensing amplitude and frequency 
    ros::Subscriber my_subscriber_object1 = nh.subscribe("amp_cmd_p1", 1, myCallbackAmplitude);
    ros::Subscriber my_subscriber_object2 = nh.subscribe("freq_cmd_p1", 1, myCallbackFrequency);
    //publish a velocity command computed by this controller; 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd_p1", 1);
    double Kv = 2.0; // velocity feedback gain 
    double dt_controller = 0.10; //specify 10Hz controller sample rate (pretty slow, but 
    //illustrative) 
    double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 

    g_amp.data = 1.0; //initialize amplitude to one
    g_freq.data = 1.0; // initialize frequency to one
    g_sin_vel_cmd.data = 0.0; // init velocity command to zero
    double radians = 0.0; // initialize at zero radians

    // enter the main loop: get velocity state and velocity commands 
    // compute command force to get system velocity to match velocity command 
    // publish this force for use by the complementary simulator 
    while (ros::ok()) {
        radians = radians + dt_controller * 3.14159265 / 2; // increment radian value by a magnitude of pi
							//radians will increase by Pi/2 every second
        //determine velocity to command with amplitude and frequency 
        g_sin_vel_cmd.data = g_amp.data * sin(radians * g_freq.data); 
        //publish to "vel_cmd" for minimal_controller to pick up
        my_publisher_object.publish(g_sin_vel_cmd); 
        ROS_INFO("velocity command = %f", g_sin_vel_cmd.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
} 
