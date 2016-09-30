// sin_prompt
// client which prompts for a sin wave's amplitude and frequency
#include<iostream>
#include<ros/ros.h> 
#include<p2/SinPrompt.h>

int main(int argc, char **argv) {
    // Initialize sin_prompt node
    ros::init(argc, argv, "sin_prompt");
    // initialize node handle
    ros::NodeHandle n;
    // Attach client to sin_configuration service (advertised in sin_commander)
    ros::ServiceClient client = n.serviceClient<p2::SinPrompt>("sin_configuration");
    // Create a service object
    p2::SinPrompt srv;

    while (ros::ok()) {
        // Get amplitude and frequency from user
        // Assign values in service request
        std::cout<<"\n";
        std::cout << "enter desired amplitude: ";
        std::cin>> srv.request.amplitude;
        std::cout << "enter desired frequency: ";
        std::cin>> srv.request.frequency;

        // Call the client with the requested values
      	if (!client.call(srv)){
            ROS_INFO("Call made; success: %d", srv.response.success);
        } else {
    	    ROS_ERROR("Failed to call service sin_configuration");
            return 1;
        }
    }
    return 0; // should never get here, unless roscore dies 
} 
