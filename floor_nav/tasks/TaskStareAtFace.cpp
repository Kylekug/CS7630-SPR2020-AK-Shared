#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
#include "sensor_msgs/RegionOfInterest.h"
#include <vector>
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskStareAtFace::initialise() 
{
    //ROS_INFO("Setting heading to %.2f deg", cfg.target*180./M_PI);
    //if (cfg.relative) {
        //const geometry_msgs::Pose2D & tpose = env->getPose2D();
        //initial_heading = tpose.theta;
    //} else {
        //initial_heading = 0.0;
    //}
    
    //if(env->getFaces()[0].x_offset > 50) {
		//cfg.target = remainder(initial_heading - M_PI/2, 2*M_PI); }
	//else {
		//cfg.target = remainder(initial_heading + M_PI/2, 2*M_PI); }
    
    printf("Now, I'm staring for 2 seconds!");
    //just put in a simple sleep to "stare" at the face for 2 sec,
    ros::Duration(2).sleep();
    
    stopSpinning = ros::Time::now() + ros::Duration(1);
    
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    //const geometry_msgs::Pose2D & tpose = env->getPose2D();
    //double alpha = remainder(initial_heading+cfg.target-tpose.theta,2*M_PI);
    //if (fabs(alpha) < cfg.angle_threshold) {
		//return TaskStatus::TASK_COMPLETED;
    //}
    //double rot = cfg.k_theta*alpha;
    //if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
    //if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
    //env->publishVelocity(0.0, rot);
	//return TaskStatus::TASK_RUNNING;
	env->publishVelocity(0.0,1.0);
	
	if(ros::Time::now() > stopSpinning) {
		return TaskStatus::TASK_COMPLETED;
	}
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
