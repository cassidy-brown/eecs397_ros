//Part_5/object_grabber/src
// example_object_grabber_action_client: 
// wsn, September, 2016
// illustrates use of object_grabber action server called "objectGrabberActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <object_grabber/object_grabber2Action.h>
#include <part_fetcher/PartFetcherAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <object_manipulation_properties/object_manipulation_properties.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <generic_gripper_services/genericGripperInterface.h>

int g_object_grabber_return_code;

class ObjectGrabberActionThing {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<part_fetcher::PartFetcherAction> as_;
    actionlib::SimpleActionClient<object_grabber::object_grabber2Action> object_grabber_ac;
    ros::NodeHandle nh;
    XformUtils xformUtils;
    
    // here are some message types to communicate with our client(s)
    part_fetcher::PartFetcherGoal goal_; // goal message, received from client
    part_fetcher::PartFetcherResult result_; // put results here, to be sent back to the client when done w/ goal
    part_fetcher::PartFetcherFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    ObjectGrabberActionThing(); //define the body of the constructor outside of class definition

    ~ObjectGrabberActionThing(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal);
};

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabber2ResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    g_object_grabber_return_code = result->return_code;
    ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
}

ObjectGrabberActionThing::ObjectGrabberActionThing() :
   as_(nh_, "object_grabber_position_receiver", boost::bind(&ObjectGrabberActionThing::executeCB, this, _1),false),
   object_grabber_ac("object_grabber_action_service", true)
{
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 
    ROS_INFO("in constructor of ObjectGrabberActionThing...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

// void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
//         const object_grabber::object_grabber2ResultConstPtr& result) {
//     ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
//     g_object_grabber_return_code = result->return_code;
//     ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
// }

void ObjectGrabberActionThing::executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& positionGoal) {

    geometry_msgs::PoseStamped object_pickup_poseStamped = positionGoal->object_frame;
    geometry_msgs::PoseStamped object_dropoff_poseStamped = positionGoal->desired_frame;

    object_grabber::object_grabber2Goal object_grabber_goal;
    bool finished_before_timeout;

    //stuff a goal message:  set action code to grab block and provide block's pose
    object_grabber_goal.action_code = object_grabber::object_grabber2Goal::GRAB_W_TOOL_Z_APPROACH; //specify the object to be grabbed
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; //from object_manipulation_properties.h
    object_grabber_goal.desired_frame = object_pickup_poseStamped;

    ROS_INFO("attempt to grab toy block at object pose: ");
    xformUtils.printStampedPose(object_pickup_poseStamped);
    ROS_INFO("sending goal: ");
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return;
    }
    //test return code:
    if (g_object_grabber_return_code!= object_grabber::object_grabber2Result::SUCCESS) {
        ROS_WARN("return code was not SUCCESS; giving up");
        return;       
    }
    //drop off the block at specified coordinates
    //stuff a goal message with action code and goal pose
    object_grabber_goal.action_code = object_grabber::object_grabber2Goal::DROPOFF_ALONG_TOOL_Z;
    object_grabber_goal.desired_frame = object_dropoff_poseStamped; //des pose
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
    
    ROS_INFO("attempting to place toy block at object pose: ");
    xformUtils.printStampedPose(object_dropoff_poseStamped);
    ROS_INFO("sending goal: ");
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_grabber_action_client");
    // ros::NodeHandle nh;
    // XformUtils xformUtils;
    // actionlib::SimpleActionClient<object_grabber::object_grabber2Action> object_grabber_ac("object_grabber_action_service", true);
    // geometry_msgs::PoseStamped toy_block_poseStamped;
    // geometry_msgs::PoseStamped toy_block_dropoff_poseStamped;
    // //hard code an object pose; later, this will come from perception
    // toy_block_poseStamped.header.frame_id = "torso"; //set approach pose for toy block
    // toy_block_poseStamped.pose.position.x = 0.5;
    // toy_block_poseStamped.pose.position.y = -0.35;
    // toy_block_poseStamped.pose.position.z = -0.125; //specify block frame w/rt torso frame
    // toy_block_poseStamped.pose.orientation.x = 0;
    // toy_block_poseStamped.pose.orientation.y = 0;
    // toy_block_poseStamped.pose.orientation.z = 0.842;
    // toy_block_poseStamped.pose.orientation.w = 0.54;
    // toy_block_poseStamped.header.stamp = ros::Time::now();

    // toy_block_dropoff_poseStamped = toy_block_poseStamped; //specify desired drop-off pose of block
    // toy_block_dropoff_poseStamped.pose.orientation.z = 1;
    // toy_block_dropoff_poseStamped.pose.orientation.w = 0;

    ROS_INFO("waiting for server: ");
    // Instantiate an object. It will do the rest.
    ObjectGrabberActionThing as_object;

    // after setting up server, just spin
    ros::spin();
    
    return 0;
}


// Set up action server that gets pickup position so we don't have

// Assignment 6: lidar point cloud -> calculate min and max (x,y) of block to determine if cube or prism
// Set up client in object_finder_action_client to send pose info to 
//      corresponding server in example_object_grabber_action_client2 instead of hardcoding positions in grabber client