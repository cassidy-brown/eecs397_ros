/* Cassidy Brown - cmb195
 * ROS P8
 * Code adapted from Part_5/baxter/baxter_fk_ik/baxter_reachability_from_above

 * The code runs for the UR10 robot, testing reachability for a full plane of x and y values 
 * at 6 z-positions of interest, taken from the NIST magic number document
 */

// reachability_from_above.cpp
// wsn, September 2016
// compute reachability, w/ z_tool_des pointing down;
// distance from gripper frame to flange frame is 0.1577
// cafe table is 0.79m above ground plane

// search for reachability for flange over x_range = [0.4,1] , y_range= [-1,1] at z_range =[0,0.1]


#include <ur_fk_ik/ur_kin.h> 
#include <fstream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ur10_reachability");
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;

    UR10FwdSolver fwd_solver;
    UR10IkSolver ik_solver;

    b_des<<0,0,-1; //tool flange pointing down
    n_des<<0,0,1; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    

    std::vector<Eigen::VectorXd> q_solns;

    Eigen::Affine3d a_tool_des; // expressed in DH frame  
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double x_des,y_des,z_des;
    double x_min = 0.4;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 1.0;
    double z_poi[6] = {0.0, 0.724275, 0.950316, 0.903960, 0.750201, 1.099893};  //z positions of interest
    double dx = 0.05;
    double dy = 0.05;
    Eigen::Vector3d p_des;
    int nsolns;
    std::vector<Eigen::Vector3d> reachable, approachable;

    // Loop through z positions of interest
    for(int i = 0; i < 6; i++){
        double z_des = z_poi[i];
    
        // Check full range of x and y positions at specified z
        for (double x_des = x_min;x_des<x_max;x_des+=dx) {
            for (double y_des = y_min; y_des<y_max; y_des+=dy) {
                p_des[0] = x_des;
                p_des[1] = y_des;
                p_des[2] = z_des;
                a_tool_des.translation() = p_des;

                nsolns = ik_solver.ik_solve(a_tool_des, q_solns);
                if (nsolns>0) { //test grasp pose:
                        ROS_INFO("soln at x,y = %f, %f, %f",p_des[0],p_des[1], z_des);
                        reachable.push_back(p_des);
                }
                
            }
        }
    }
    ROS_INFO("saving the results...");
    nsolns = reachable.size();
    ofstream outfile;
    outfile.open("ur10_reachable_x_y");
    
    for (int i=0;i<nsolns;i++) {
        p_des = reachable[i];
        outfile<<p_des[0]<<", "<<p_des[1] << ", " << p_des[2]<<endl;
    }
    outfile.close();
    
    nsolns = approachable.size();
    ofstream outfile2;
    outfile2.open("ur10_approachable_x_y");
    
    for (int i=0;i<nsolns;i++) {
        p_des = approachable[i];
        outfile2<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile2.close();    
 
    return 0;
}
