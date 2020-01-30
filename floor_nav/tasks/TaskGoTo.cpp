#include <math.h>
#include "TaskGoTo.h"
#include "floor_nav/TaskGoToConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

//#define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoTo::initialise() 
{
    ROS_INFO("Going to %.2f %.2f %.2f",cfg.goal_x,cfg.goal_y,cfg.theta);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init=tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init=0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoTo::iterate()
{
const geometry_msgs::Pose2D & tpose = env->getPose2D();
double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
if(!cfg.smart_mode){
		if (r < cfg.dist_threshold) {
			//test
			double beta = remainder(theta_init+cfg.theta-tpose.theta,2*M_PI);
			if (fabs(beta)>cfg.ang_threshold){
				double rot = ((beta>0)?+1:-1)*cfg.max_angular_velocity;
				env->publishVelocity(0,rot);
				return TaskStatus::TASK_RUNNING;
			} else {
				return TaskStatus::TASK_COMPLETED;
				}
		
			}
    double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
   

		if (fabs(alpha) > M_PI/9) {
			double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;

			env->publishVelocity(0,rot);
		} else {
			double vel = std::min(cfg.k_v * r,cfg.max_velocity);
			double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
			env->publishVelocity(vel, rot);
		}
	return TaskStatus::TASK_RUNNING;
}else{
		double alpha = remainder(atan2(y_init+cfg.goal_y-tpose.y,x_init+cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
		double beta =-(alpha +tpose.theta-cfg.theta-theta_init);
		#ifdef DEBUG_GOTOPOSE
			printf("Cmd x %.2f y %.2f r %.2f a %.2f b %.2f\n",tpose.x,tpose.y,r,alpha,beta);
		#endif
		if (r < cfg.dist_threshold){
			if (beta < cfg.ang_threshold){
				return TaskStatus::TASK_COMPLETED;
		    }else{
				//double rot = std::max(std::min(cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
				double rot = ((beta>0)?+1:-1)*cfg.max_angular_velocity;
			    env->publishVelocity(0,rot);
			    }
		}else{
			double vel = cfg.k_v * r;	
			if (vel > cfg.max_velocity) vel = cfg.max_velocity;	
			double rot = std::max(std::min(cfg.k_alpha*alpha+ cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
			env->publishVelocity(vel,rot);
			}
	return TaskStatus::TASK_RUNNING;
	 }
}

TaskIndicator TaskGoTo::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoTo);
