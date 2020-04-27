/*
Stolen from: http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class FrontierWrapper
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  move_base_msgs::MoveBaseResult result_;

  ros::Subscriber finished_sub;
  ros::Publisher goal_pub;
  
  bool goalFinished;

  tf::TransformListener listener_;

public:

  FrontierWrapper(std::string name) :
    as_(nh_, name, boost::bind(&FrontierWrapper::executeCB, this, _1), false),
    action_name_(name),
    goalFinished(false)
  {
    finished_sub = nh_.subscribe("wrapper_finished", 1, &FrontierWrapper::finishedCB, this);
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("wrapper_goal", 1);
    as_.start();
  }

  ~FrontierWrapper(void)
  {
  }

  void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
  {
    goalFinished = false;
    goal_pub.publish(goal->target_pose);
    
    ros::Rate r(10);
    
    while(!goalFinished) {
        
        if(as_.isPreemptRequested() || !ros::ok()) {
            as_.setPreempted();
            return;
        }
        
        r.sleep();
    }

    as_.setSucceeded(result_);
  }
  
  void finishedCB (const std_msgs::HeaderConstPtr &msg)
  {
    goalFinished = true;
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "FrontierWrapper");

  FrontierWrapper myLittleWrapper("move_base"); //to "simulate" the real move_base
  ros::spin();

  return 0;
}
