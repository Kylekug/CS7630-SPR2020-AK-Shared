#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"
#include "sensor_msgs/RegionOfInterest.h"
#include <vector>
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::iterate()
{
    //const geometry_msgs::Pose2D & tpose = env->getPose2D();
    //double r = hypot(cfg.roi_y-tpose.y,cfg.roi_x-tpose.x);
    //if (r < cfg.roi_radius) {
        //ROS_INFO("Detected ROI at %.2f %.2f",tpose.x, tpose.y);
		//return TaskStatus::TASK_COMPLETED;
    //}
    //if(env->getFaces().size() > 0) {
		//printf("%i Faces!\n", env->getFaces().size());
	//} else {
		//printf("No face!\n");
	//}
	
	if(env->getFaces().size() > 0) {
		printf("TaskWaitForFace found a face!\n");
		return TaskStatus::TASK_COMPLETED;
	}
	
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);
