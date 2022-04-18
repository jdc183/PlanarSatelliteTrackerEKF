#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10);
    while (nh.ok()){
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.695*30.0/16.0));
        transform.setRotation(tf::Quaternion(1,0,0,0));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"map","head_camera"));
        rate.sleep();
    }
    return 0;
}