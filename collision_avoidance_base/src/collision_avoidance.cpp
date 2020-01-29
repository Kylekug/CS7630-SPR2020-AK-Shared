#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>

#define _max_velocity 1.0

/*
 * NO LONGER TRUE, VALUES OK:
 * NOTES: 	desired.x ranges from -2.0 to +2.0 (this should correspond to max_velocity)
 * 			radius should be no smaller than 0.10 (1.0 is ~ 1 meter?)
 */

class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber cloudSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;
        ros::NodeHandle nh;

        // This might be useful
        double radius;
		float closest = 10000;
		geometry_msgs::Twist currentTwist;
		
        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
        }

        void scan_callback(const sensor_msgs::LaserScanConstPtr msg) {
            // unsigned int n = msg->ranges.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float theta = msg->angle_min + i * msg->angle_increment;
            //     float x = msg->ranges[i]*cos(theta);
            //     float y = msg->ranges[i]*sin(theta);
            //     float z =0
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
            
            //ROS_INFO("scan_callback START");
            
            unsigned int n = msg->ranges.size();
            for (unsigned int i=0;i<n;i++) {
                 // Selecting points in the direction of motion and not too far aside
                 float theta = msg->angle_min + i * msg->angle_increment;
                 float x = msg->ranges[i]*cos(theta);
                 float y = msg->ranges[i]*sin(theta);
                 
                 if(y > -0.2 && y < 0.2 && x > 0){
                     // Pythagora theorem to compute the distance from the robot to a 
                     // to a given point
                     //float current = x*x + y*y;
                     closest=x*x + y*y;
                     //ROS_INFO("CURRENT: %.12f", current);
/*                     if(current < closest){
                         closest = current;
                         // ROS_INFO("%.3f %.3f",lastpc[i].x,lastpc[i].y);
                    }
*/                      
					ROS_INFO("CLOSEST: %.12f", closest);
                 }
				
				geometry_msgs::Twist filtered = findClosestAcceptableVelocity(currentTwist);
				
				velPub.publish(filtered);
				
				//ROS_INFO("VELOCITY: %.12f",filtered.linear.x);
				
             }
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            currentTwist = desired;
            
            // TODO: modify desired using the laser point cloud or laser scan

			if (desired.linear.x > 0.0) {
				res.linear.x = (desired.linear.x/2.0)*std::min(_max_velocity,_max_velocity*(closest - 0.1 * radius)/(0.9 * radius));
				if (res.linear.x < 0.0) {	res.linear.x = 0.0;	}
             }
             else if (desired.linear.x < 0.0) {
				 res.linear.x = (desired.linear.x/2.0)*std::max(_max_velocity, -1.0 * desired.linear.x);
				if (res.linear.x > 0.0) {	res.linear.x = 0.0;	}
			}
            return res;
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::scan_callback,this);
            cloudSub = nh.subscribe("clouds",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius",radius,1.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


