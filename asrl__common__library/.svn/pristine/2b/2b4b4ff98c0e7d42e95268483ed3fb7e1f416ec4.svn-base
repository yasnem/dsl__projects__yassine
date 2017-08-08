#ifndef ASRL_ROSUTIL_TRANSFORMATION_UTILITIES_HPP
#define ASRL_ROSUTIL_TRANSFORMATION_UTILITIES_HPP

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <asrl/math/Transformation.hpp>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace asrl { namespace rosutil {


    void rosInfoTransformation(Eigen::Matrix4d &T);
    void rosInfoRotation(Eigen::Matrix3d &C);

    geometry_msgs::Vector3 toVector3Message(const Eigen::Vector3d & v);
    geometry_msgs::Point toPointMessage(const Eigen::Vector3d & v);

    void addCFrameToLineList(std::vector<geometry_msgs::Point> & points, std::vector<std_msgs::ColorRGBA> & colors, const Eigen::Matrix4d & T_0_k, double cframeSize);

    visualization_msgs::MarkerArray buildMaFromPoses(const std::string & baseFrameId, ros::Time timestamp, std::vector<geometry_msgs::Pose>& poses, double cframeSize);

    geometry_msgs::Quaternion toQuaternionMessage(const Eigen::Quaterniond & v);
    geometry_msgs::Quaternion toQuaternionMessage(const Eigen::Vector4d & v);

    geometry_msgs::TransformStamped toTransformStampedMessage(const std::string & base, const std::string & child, const asrl::math::Transformation & T_base_child, const ros::Time & stamp);

    geometry_msgs::TransformStamped toTransformStampedMessage(const std::string & base, const std::string & child, const Eigen::Matrix4d & T_base_child, const ros::Time & stamp);

    geometry_msgs::Transform toTransformMessage(const asrl::math::Transformation & T_base_child);
    geometry_msgs::Transform toTransformMessage(const Eigen::Matrix4d & T_base_child);

    std::vector<geometry_msgs::Transform> toTransformMessageVector(const std::vector<asrl::math::Transformation, Eigen::aligned_allocator<asrl::math::Transformation> >& T_base_child);
    std::vector<geometry_msgs::Transform> toTransformMessageVector(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > & T_base_child);

    geometry_msgs::Pose toPoseMessage(const Eigen::Matrix4d & T_base_pose);
    geometry_msgs::Pose toPoseMessage(const asrl::math::Transformation & T_base_pose);
    geometry_msgs::PoseWithCovariance toPoseWithCovarianceMessage(const asrl::math::Transformation & T_base_pose);
    asrl::math::Transformation fromPoseWithCovarianceMessage(const geometry_msgs::PoseWithCovariance & T_base_pose);
    geometry_msgs::PoseStamped toPoseStampedMessage(const std::string & base_frame_id, const Eigen::Matrix4d & T_base_pose, const ros::Time & stamp);
    geometry_msgs::PoseStamped toPoseStampedMessage(const std::string & base_frame_id, const asrl::math::Transformation & T_base_pose, const ros::Time & stamp);

    Eigen::Matrix4d fromPoseMessage(const geometry_msgs::Pose& pose);
    //Eigen::Matrix4d fromStampedTransformation(tf::StampedTransform const & T_base_child); //Why was there two identical declarations? -- JDG
    Eigen::Matrix4d fromStampedTransformation(tf::StampedTransform const & T_base_child);
    Eigen::Vector4d fromQuaternionMessage(const geometry_msgs::Quaternion& quaternion_msg);

    tf::StampedTransform fromPoseToStampedTransform(const geometry_msgs::Pose& pose);

    asrl::math::Transformation fromTransformMessage(const geometry_msgs::Transform & T_to_from);
    asrl::math::Transformation fromTransformMessage(const geometry_msgs::Transform & T_to_from, const boost::array<double, 36> & covariance);

    void fillCovarianceArray( const asrl::math::Transformation & T_to_from, boost::array<double, 36> & outCovariance);
    void fillCovarianceArray( const asrl::math::Transformation::covariance_t & U_to_from, boost::array<double, 36> & outCovariance);
    asrl::math::Transformation::covariance_t buildCovarianceMatrix(const boost::array<double, 36> & covariance);

    geometry_msgs::Vector3 quat2rpy(const tf::Quaternion & q);

  }} // namespace asrl::rosutil


#endif /* ASRL_ROSUTIL_TRANSFORMATION_UTILITIES_HPP */
