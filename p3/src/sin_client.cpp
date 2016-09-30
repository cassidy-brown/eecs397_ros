// sin_client
// ROS p3, adaptd from learning_ros example code
// Cassidy Brown cmb195

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <p3/sinConfigurationAction.h>


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const p3::sinConfigurationResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "sin_conmmander_client_node"); // name this node 
        // here is a "goal" object compatible with the server
        p3::sinConfigurationGoal goal; 
        
        // use the name of our server
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<p3::sinConfigurationAction> action_client("sin_commander", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        while(ros::ok()) {
            // Get amplitude, frequency, and cycles from user
            // Assign values in action goal
            std::cout<<"\n";
            std::cout << "enter desired amplitude: ";
            std::cin>> goal.amplitude;
            std::cout << "enter desired frequency: ";
            std::cin>> goal.frequency;
            std::cout << "enter desired number of cycles: ";
            std::cin>> goal.num_cycles;
            
            action_client.sendGoal(goal,&doneCb);
            
            //Each cycle should take 4 seconds(divided by frequency), as we go pi/2 radians per second (when f = 1), so we wait 5 just for good measure
            action_client.waitForResult(ros::Duration(5 * goal.num_cycles / goal.frequency));
        
        }

    return 0;
}

