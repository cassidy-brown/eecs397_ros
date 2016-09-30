// sin_commander as action server

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <p3/sinConfigurationAction.h>
#include <std_msgs/Float64.h> 

#define PI 3.14159265

class SinCommander {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<p3::sinConfigurationAction> as_;
    
    // here are some message types to communicate with our client(s)
    p3::sinConfigurationGoal goal_; // goal message, received from client
    p3::sinConfigurationResult result_; // put results here, to be sent back to the client when done w/ goal
    p3::sinConfigurationFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    // Publisher to send velocity command
    ros::Publisher publisher_;



public:
    SinCommander(); //define the body of the constructor outside of class definition

    // Destructor... basically, we don't do anything out of the ordinary when the class is destroyed
    ~SinCommander(void) {    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<p3::sinConfigurationAction>::GoalConstPtr& goal);
};

// Constructor implementation
SinCommander::SinCommander() :
   as_(nh_, "sin_commander", boost::bind(&SinCommander::executeCB, this, _1),false) 
// in the above initialization, we name the server "sin_commander"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of SinCommander...");
    // initialize publisher
    publisher_ = nh_.advertise<std_msgs::Float64>("vel_cmd_p3", 1);

    as_.start(); //start the server running
}

//executeCB implementation
void SinCommander::executeCB(const actionlib::SimpleActionServer<p3::sinConfigurationAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    ROS_INFO("goal amplitude is: %f", goal->amplitude);
    ROS_INFO("goal frequency is: %f", goal->frequency);
    ROS_INFO("goal num_cycles is: %f", goal->num_cycles);
    
    //PUBLISHER
    // publish a velocity command computed by this controller; 
    double Kv = 2.0; // velocity feedback gain 
    double dt_controller = 0.10; //specify 10Hz controller sample rate
    double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 

    std_msgs::Float64 vel_cmd; // declare velocity command
    double radians = 0.0; // initialize at zero radians
    double goalRad = goal->num_cycles * 2 * PI / goal->frequency;   // the number of radians the commands should cover based on the goal number of cycles

    // enter the main loop: run until we've reached the radian goal 
    while (radians < goalRad && ros::ok()) {
        radians = radians + dt_controller * PI / 2; // increment radian value by a magnitude of pi
                            //radians will increase by Pi/2 every second
        
        //determine velocity to command with amplitude and frequency from goal
        vel_cmd.data = goal->amplitude * sin(radians * goal->frequency); 
        //publish to "vel_cmd" for minimal_controller to pick up
        publisher_.publish(vel_cmd); 
        ROS_INFO("velocity command = %f", vel_cmd.data);
        naptime.sleep(); // wait for remainder of specified period; 
    }

    // Finishing the loop, we've completed the desired cycles
    // set the velocity command back to zero
    vel_cmd.data = 0.0;
    publisher_.publish(vel_cmd);

    if(radians >= goalRad){ // success
        result_.success = true;
        as_.setSucceeded(result_); 
    } else { //ROS died
        result_.success = false;
        as_.setAborted(result_);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sin_commander_server_node"); // name this node 

    ROS_INFO("instantiating the sin commander action server: ");

    SinCommander as_object; // create an instance of the class "SinCommander"
    
    ros::spin();

    return 0;
}

