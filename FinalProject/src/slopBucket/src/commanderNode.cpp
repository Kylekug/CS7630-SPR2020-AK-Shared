#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

const float batteryThreshold = 60;

class commanderNode {
    
    protected:
    
        ros::NodeHandle nh_;
        
        ros::Subscriber wrapper_goal_sub;
        ros::Publisher wrapper_finished_pub;

        ros::Subscriber occ_finished_sub;
        ros::Publisher occ_goal_pub;
        
        ros::Subscriber battery_sub;
        
        float voltage;
        ros::Timer stateMachineTimer;
        enum state {explore, charging};
        state currentState;
        
        geometry_msgs::PoseStamped exploreGoal;
        
        bool newExploreGoal;
        
    public:
    
        commanderNode() : voltage(0), currentState(charging), newExploreGoal(false)
        {
            wrapper_goal_sub = nh_.subscribe("wrapper_goal", 1, &commanderNode::wrapper_goal_sub_CB, this);
            wrapper_finished_pub = nh_.advertise<std_msgs::Header>("wrapper_finished", 1);
            
            occ_finished_sub = nh_.subscribe("/path_follower/goal_reached", 1, &commanderNode::occ_finished_sub_CB, this);
            occ_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
            
            battery_sub = nh_.subscribe("/vrep/voltage", 1, &commanderNode::battery_sub_CB, this);
            
            stateMachineTimer = nh_.createTimer(ros::Duration(1.0/30.0), &commanderNode::stateMachineCB, this);
        }
        
        ~commanderNode(void)
        {
        }
        
        void wrapper_goal_sub_CB (const geometry_msgs::PoseStampedConstPtr &goal) {
            exploreGoal = *goal;
            newExploreGoal = true;
            //occ_goal_pub.publish(goal);
        }
        
        void occ_finished_sub_CB (const std_msgs::HeaderConstPtr &msg) {
            if (currentState == explore) {
                wrapper_finished_pub.publish(msg);
            }
        }
        
        void battery_sub_CB (const std_msgs::Float32ConstPtr &msg) {
            voltage = msg->data;
        }
        
        void stateMachineCB (const ros::TimerEvent &) {
            switch (currentState) {
                
                case explore:
                {
                    if(newExploreGoal) {
                        occ_goal_pub.publish(exploreGoal);
                        newExploreGoal = false;
                    }
                    
                    if(voltage <= batteryThreshold) {
                        currentState = charging;
                        geometry_msgs::PoseStamped chargingPose {};
                        chargingPose.header.stamp = ros::Time::now();
                        chargingPose.header.frame_id = "map"; //return to middle of /map
                        chargingPose.pose.orientation.w = 1.0;
                        occ_goal_pub.publish(chargingPose);
                    }
                    
                break;}
                
                case charging: 
                {
                    if (voltage >= 99.0) {
                        currentState = explore;
                        newExploreGoal = true;
                    }
                
                break;}
                
            }
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "commanderNode");
  commanderNode myLittleCommander; //to "simulate" the real move_base
  ros::spin();

  return 0;
}
