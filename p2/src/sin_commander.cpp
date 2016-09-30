// sin_commander node: 
// based on minimal_controller
// advertises "sin_configuration" service which gets amplitude and frequency commands
// publishes "vel_cmd" 
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<math.h>
#include<p2/SinPrompt.h>

//global variables for callback functions to populate for use in main program 
std_msgs::Float64 g_amp;
std_msgs::Float64 g_freq; 
std_msgs::Float64 g_sin_vel_cmd;

// Callback for sin_configuration service which sets the global amplitude and frequency variables based on the passed in request
bool callback(p2::SinPromptRequest& request, p2::SinPromptResponse& response) {
    ROS_INFO("received amplitude value is: %f", request.amplitude);
    ROS_INFO("received frequency value is: %f", request.frequency);
    g_amp.data = request.amplitude; // post the received data in a global var for access by main prog. 
    g_freq.data = request.frequency;

    response.success = true; //probably don't need anything in the response, but here it is for good measure
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander_p2"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "sin_commander" 
    ros::NodeHandle nh; // node handle 

    // SERVICE
    //set up service called sin_configuration to listen for amplitude and frequency
    ros::ServiceServer service = nh.advertiseService("sin_configuration", callback);
    // everything else service-related happens in the callback and in the client

    // PUBLISHER
    // publish a velocity command computed by this controller; 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd_p2", 1);
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
