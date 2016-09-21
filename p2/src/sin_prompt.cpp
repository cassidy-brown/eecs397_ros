// sin_commander node: 
// based on minimal_controller
// subscribes to "amp_cmd" and "freq_cmd"
// publishes "vel_cmd" 
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<p2/SinPrompt.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_prompt");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<p2::SinPrompt>("sin_prompt");
    p2::SinPrompt srv;
    double amplitude;
    double frequency;
    while (ros::ok()) {
        cout<<endl;
        cout << "enter desired amplitude: ";
        cin>>amplitude;
        cout<<endl;
        cout << "enter desired frequency: ";
        cin>>frequency;

	srv.data.amplitude = amplitude;
	srv.data.frequency = frequency;

  	if (client.call(srv)){
	    ROS_ERROR("Failed to call service lookup_by_name");
            return 1;
        }
    }
    return 0; // should never get here, unless roscore dies 
} 
