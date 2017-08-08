#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "pioneer_vicon_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()) {
    transform.setOrigin( tf::Vector3(-0.25, 0.0, 0.58) );
    transform.setRotation( tf::Quaternion(0.055, 0.0, -M_PI) ); // Yaw, pitch, roll
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pioneer:robot", "pioneer_vicon"));
    rate.sleep();
  }
  return 0;
};
