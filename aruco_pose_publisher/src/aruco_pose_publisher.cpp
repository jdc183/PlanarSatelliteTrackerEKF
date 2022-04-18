#include<ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

using namespace std;

ros::Publisher aruco_pub;
ros::Subscriber pose_sub;
geometry_msgs::PoseWithCovarianceStamped current_pose;
geometry_msgs::PoseStamped temp_pose;

double poscov = pow(0.00025,2);
double phicov = pow(0.1,2);
double scale_factor = 30.0/16.0;


void callback(const geometry_msgs::PoseStamped& poseReceived) {
	 
	temp_pose = poseReceived;
	current_pose.pose.pose.position.x = temp_pose.pose.position.x * scale_factor;
	current_pose.pose.pose.position.y = temp_pose.pose.position.y * scale_factor;
	current_pose.pose.pose.position.z = temp_pose.pose.position.z * scale_factor;
	current_pose.pose.pose.orientation = temp_pose.pose.orientation;
	current_pose.pose.covariance[0] = poscov * scale_factor;
	current_pose.pose.covariance[7] = poscov * scale_factor;
	current_pose.pose.covariance[14] = poscov * scale_factor;
	current_pose.pose.covariance[21] = phicov;
	current_pose.pose.covariance[28] = phicov;
	current_pose.pose.covariance[35] = phicov;
	current_pose.header.frame_id = "head_camera";
	current_pose.header.stamp = ros::Time::now();

	aruco_pub.publish(current_pose);
	ROS_INFO("Publishing");
	
}



int main(int argc, char **argv) {
    ros::init(argc,argv, "aruco_pub"); // name of the node
    ros::NodeHandle nh;

    aruco_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/aruco_stamped",1);

    pose_sub = nh.subscribe("/aruco_single/pose", 1, callback);

    ros::spin();
    return 0;
}
