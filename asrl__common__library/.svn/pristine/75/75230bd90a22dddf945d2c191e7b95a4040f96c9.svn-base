#include <asrl/rosutil/transformation_utilities.hpp>
#include <tf/tf.h>
#include <Eigen/Dense>
//#include <asrl/math/quaternion_algebra.hpp>

namespace asrl { namespace rosutil {

    void rosInfoTransformation(Eigen::Matrix4d &T)
    {
        ROS_INFO("\n[%4.2f, %4.2f, %4.2f, %4.2f;\n%4.2f %4.2f %4.2f, %4.2f;\n%4.2f %4.2f %4.2f, %4.2f;\n%4.2f, %4.2f, %4.2f, %4.2f]", T(0,0), T(0,1), T(0,2), T(0,3), T(1,0), T(1,1), T(1,2), T(1,3), T(2,0), T(2,1), T(2,2), T(2,3), T(3,0), T(3,1), T(3,2), T(3,3) );
    }

    void rosInfoRotation(Eigen::Matrix3d &C)
    {
        ROS_INFO("\n[%4.2f, %4.2f, %4.2f;\n%4.2f %4.2f %4.2f;\n%4.2f %4.2f %4.2f]", C(0,0), C(0,1), C(0,2), C(1,0), C(1,1), C(1,2), C(2,0), C(2,1), C(2,2) );
    }

    geometry_msgs::Vector3 toVector3Message(const Eigen::Vector3d & v)
    {
      geometry_msgs::Vector3 v3;
      v3.x = v[0];
      v3.y = v[1];
      v3.z = v[2];

      return v3;
    }

    geometry_msgs::Point toPointMessage(const Eigen::Vector3d & v)
    {
      geometry_msgs::Point v3;
      v3.x = v[0];
      v3.y = v[1];
      v3.z = v[2];

      return v3;
    }

    void addCFrameToLineList(std::vector<geometry_msgs::Point> & points, std::vector<std_msgs::ColorRGBA> & colors, const Eigen::Matrix4d & T_0_k, double size)
    {
      std_msgs::ColorRGBA col[3];
      // red
      col[0].a = col[0].r = 1.0; col[0].g = col[0].b = 0.0;
      // green
      col[1].a = col[1].g = 1.0; col[1].r = col[1].b = 0.0;
      // blue
      col[2].a = col[2].b = 1.0; col[2].g = col[2].r = 0.0;


      // Draw a coordinate frame.
      Eigen::Vector3d p_0_k_0 = T_0_k.topRightCorner<3,1>();
      for(int i = 0; i < 3; i++)
	{
	  points.push_back(toPointMessage(p_0_k_0));
	  colors.push_back(col[i]);

	  points.push_back(toPointMessage(p_0_k_0 + (T_0_k.col(i).head<3>() * size)));
	  colors.push_back(col[i]);
	}
    }

    visualization_msgs::MarkerArray buildMaFromPoses(const std::string & baseFrameId, ros::Time timestamp, std::vector<geometry_msgs::Pose>& poses, double cframeSize)
    {
    	visualization_msgs::MarkerArray ma;
    	ma.markers.resize(2);
    	visualization_msgs::Marker & cframes = ma.markers[0];
    	cframes.header.frame_id = baseFrameId;
    	cframes.header.stamp = timestamp; // TODO: check on this...not sure this is right
    	cframes.ns = "/vertices";
    	cframes.id = 0;
    	cframes.type = visualization_msgs::Marker::LINE_LIST;
    	cframes.action = visualization_msgs::Marker::ADD;
    	cframes.lifetime = ros::Duration();
    	cframes.pose.position.x = cframes.pose.position.y = cframes.pose.position.z = 0.0;
    	cframes.pose.orientation.x = cframes.pose.orientation.y = cframes.pose.orientation.z = 0.0;
    	cframes.pose.orientation.w = 1.0;
    	cframes.scale.x = cframes.scale.y = cframes.scale.z = 0.01;
    	cframes.color.r = cframes.color.g = cframes.color.b = cframes.color.a = 1.f;
    	cframes.frame_locked = false;

    	std_msgs::ColorRGBA pathColor;
    	pathColor.r = 0.0; pathColor.a = pathColor.g = pathColor.b = 1.0;
    	visualization_msgs::Marker & path = ma.markers[1];
    	path.header.frame_id = baseFrameId;
    	path.header.stamp = timestamp; // TODO: again, I am just setting this equal to the time above. Not sure if this is right...
    	path.ns = "/path";
    	path.id = 1;
    	path.type = visualization_msgs::Marker::LINE_STRIP;
    	path.action = visualization_msgs::Marker::ADD;
    	path.lifetime = ros::Duration();
    	path.pose.position.x = path.pose.position.y = path.pose.position.z = 0.0;
    	path.pose.orientation.x = path.pose.orientation.y = path.pose.orientation.z = 0.0;
    	path.pose.orientation.w = 1.0;
    	path.scale.x = path.scale.y = path.scale.z = 0.01;
    	path.color = pathColor;
    	path.frame_locked = false;

    	Eigen::Matrix4d T_0_k = Eigen::Matrix4d::Identity();
    	for(unsigned i = 0; i < poses.size(); ++i)
    	{
    		// Create an Eigen 3 matrix from tf pose messages
    		//T_0_k = fromStampedTransformation(
    		//		fromPoseToStampedTransform(poses[i]) );
    		T_0_k = fromPoseMessage(poses[i]);

		/*
    		ROS_INFO("T_0_k = [%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f]",
    				T_0_k(0,0),T_0_k(0,1),T_0_k(0,2),T_0_k(0,3),
    				T_0_k(1,0),T_0_k(1,1),T_0_k(1,2),T_0_k(1,3),
    				T_0_k(2,0),T_0_k(2,1),T_0_k(2,2),T_0_k(2,3),
    				T_0_k(3,0),T_0_k(3,1),T_0_k(3,2),T_0_k(3,3));
		*/

    		// Add to line list
    		asrl::rosutil::addCFrameToLineList(cframes.points, cframes.colors, T_0_k, cframeSize);

    		// Draw a line from the previous to the next.
    		path.points.push_back(asrl::rosutil::toPointMessage(T_0_k.topRightCorner<3,1>()));
    	}

    	return ma;
    }

    Eigen::Vector4d fromQuaternionMessage(const geometry_msgs::Quaternion& quaternion_msg)
    {
        Eigen::Vector4d q;
        q[0] = quaternion_msg.x;
        q[1] = quaternion_msg.y;
        q[2] = quaternion_msg.z;
        q[3] = quaternion_msg.w;

        return q;
    }

    geometry_msgs::Quaternion toQuaternionMessage(const Eigen::Vector4d & v)
    {
        geometry_msgs::Quaternion q_msg;
        q_msg.x = v[0];
        q_msg.y = v[1];
        q_msg.z = v[2];
        q_msg.w = v[3];

      return q_msg;
    }

    geometry_msgs::Quaternion toQuaternionMessage(const Eigen::Quaterniond & v)
    {
      geometry_msgs::Quaternion q;
      q.x = v.x();
      q.y = v.y();
      q.z = v.z();
      q.w = v.w();

      return q;
    }

    geometry_msgs::Transform toTransformMessage(const asrl::math::Transformation & T_base_child)
    {
      return toTransformMessage(T_base_child.T().matrix());
    }

    geometry_msgs::Transform toTransformMessage(const Eigen::Matrix4d & T_base_child)
    {
      Eigen::Quaterniond q_bc(T_base_child.topLeftCorner<3,3>());
      Eigen::Vector3d p_b_c_b = T_base_child.topRightCorner<3,1>();

      geometry_msgs::Transform ts;
      ts.translation = toVector3Message(p_b_c_b);
      ts.rotation = toQuaternionMessage(q_bc);

      return ts;
    }

    std::vector<geometry_msgs::Transform> toTransformMessageVector(const std::vector<asrl::math::Transformation, Eigen::aligned_allocator<asrl::math::Transformation> >& T_base_child)
    {
      //The return value
      std::vector<geometry_msgs::Transform> msgVector;

      //Iterate over the vector. Sure wish I had typedef'ed
      for (std::vector<asrl::math::Transformation, Eigen::aligned_allocator<asrl::math::Transformation> >::const_iterator TIter = T_base_child.begin(); TIter != T_base_child.end(); ++TIter)
      {
        //Pushback onto the return value
        msgVector.push_back( toTransformMessage(*TIter) );
      }

      //Return
      return msgVector;
    }

    std::vector<geometry_msgs::Transform> toTransformMessageVector(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > & T_base_child)
    {
      //The return value
      std::vector<geometry_msgs::Transform> msgVector;

      //Iterate over the vector. Sure wish I had typedef'ed
      for (std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >::const_iterator eTIter = T_base_child.begin(); eTIter != T_base_child.end(); ++eTIter)
      {
        //Pushback onto the return value
        msgVector.push_back( toTransformMessage(*eTIter) );
      }

      //Return
      return msgVector;
    }

    geometry_msgs::TransformStamped toTransformStampedMessage(const std::string & base, const std::string & child, const asrl::math::Transformation & T_base_child, const ros::Time & stamp)
    {
      return toTransformStampedMessage(base, child, T_base_child.T().matrix(), stamp);
    }

    geometry_msgs::TransformStamped toTransformStampedMessage(const std::string & base, const std::string & child, const Eigen::Matrix4d & T_base_child, const ros::Time & stamp)
    {
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = base;
      ts.child_frame_id = child;
      ts.transform = toTransformMessage(T_base_child);

      return ts;
    }

    asrl::math::Transformation fromPoseWithCovarianceMessage(const geometry_msgs::PoseWithCovariance & T_base_pose)
    {
      return asrl::math::Transformation(fromPoseMessage(T_base_pose.pose),buildCovarianceMatrix(T_base_pose.covariance));
    }

    geometry_msgs::PoseWithCovariance toPoseWithCovarianceMessage(const asrl::math::Transformation & T_base_pose)
    {
      geometry_msgs::PoseWithCovariance pwc;
      pwc.pose = toPoseMessage(T_base_pose);
      fillCovarianceArray(T_base_pose.U(), pwc.covariance);
      return pwc;
    }

    geometry_msgs::PoseStamped toPoseStampedMessage(const std::string & frame_id, const asrl::math::Transformation & T_base_pose, const ros::Time & stamp)
    {
      return toPoseStampedMessage(frame_id, T_base_pose.T().matrix(), stamp);
    }

    geometry_msgs::PoseStamped toPoseStampedMessage(const std::string & frame_id, const Eigen::Matrix4d & T_base_pose, const ros::Time & stamp)
    {
      geometry_msgs::PoseStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = frame_id;
      ts.pose = toPoseMessage(T_base_pose);

      return ts;
    }

    geometry_msgs::Pose toPoseMessage(const Eigen::Matrix4d & T_base_pose)
    {
      Eigen::Quaterniond q_bp(T_base_pose.topLeftCorner<3,3>());
      Eigen::Vector3d p_b_p_b = T_base_pose.topRightCorner<3,1>();

      geometry_msgs::Pose ts;
      ts.position = toPointMessage(p_b_p_b);
      ts.orientation = toQuaternionMessage(q_bp);

      return ts;
    }

    geometry_msgs::Pose toPoseMessage(const asrl::math::Transformation & T_base_pose)
    {
      return toPoseMessage(T_base_pose.T().matrix());
    }


    Eigen::Matrix4d fromStampedTransformation(tf::StampedTransform const & T_base_child)
    {
      Eigen::Vector3d p_b_c_b(T_base_child.getOrigin().x(), T_base_child.getOrigin().y(), T_base_child.getOrigin().z());
      //Eigen::Quaterniond q_bc(T_base_child.getRotation().w(),T_base_child.getRotation().x(),T_base_child.getRotation().y(),T_base_child.getRotation().z());
      //btMatrix3x3 C_bc( T_base_child.getRotation() );
      tf::Matrix3x3 C_bc( T_base_child.getRotation() );

      Eigen::Matrix4d T_b_c = Eigen::Matrix4d::Identity();
      T_b_c.topRightCorner<3,1>() = p_b_c_b;

      //T_b_c.topLeftCorner<3,3>() = q_bc.matrix();

      T_b_c.topLeftCorner<3,3>() << C_bc[0][0], C_bc[0][1], C_bc[0][2],
	                          C_bc[1][0], C_bc[1][1], C_bc[1][2],
	                          C_bc[2][0], C_bc[2][1], C_bc[2][2];
      return T_b_c;

    }


    asrl::math::Transformation fromTransformMessage(const geometry_msgs::Transform & T)
    {

      Eigen::Vector3d p_b_c_b(T.translation.x, T.translation.y, T.translation.z);
      //Eigen::Quaterniond q_bc(T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w);
      //btMatrix3x3 C_bc( btQuaternion(T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w) );
      tf::Matrix3x3 C_bc( tf::Quaternion(T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w) );

      Eigen::Matrix4d Tmx = Eigen::Matrix4d::Identity();
      Tmx.topRightCorner<3,1>() = p_b_c_b;
      //Tmx.topLeftCorner<3,3>() = q_bc.matrix();

      Tmx.topLeftCorner<3,3>() << C_bc[0][0], C_bc[0][1], C_bc[0][2],
	                          C_bc[1][0], C_bc[1][1], C_bc[1][2],
	                          C_bc[2][0], C_bc[2][1], C_bc[2][2];

      return asrl::math::Transformation(Tmx);
    }


    asrl::math::Transformation fromTransformMessage(const geometry_msgs::Transform & mT, const boost::array<double, 36> & covariance)
    {
      asrl::math::Transformation T = fromTransformMessage(mT);
      T.U() = buildCovarianceMatrix(covariance);
      return T;
    }

    Eigen::Matrix4d fromPoseMessage(const geometry_msgs::Pose& pose)
    {
      Eigen::Vector3d p_b_c_b(pose.position.x, pose.position.y, pose.position.z);
      //Eigen::Quaterniond q_bc(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
      //btMatrix3x3 C_bc( btQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w) );
      tf::Matrix3x3 C_bc( tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w) );
      Eigen::Matrix4d T_b_c = Eigen::Matrix4d::Identity();
      T_b_c.topRightCorner<3,1>() = p_b_c_b;
      //T_b_c.topLeftCorner<3,3>() = q_bc.matrix();
      T_b_c.topLeftCorner<3,3>() << C_bc[0][0], C_bc[0][1], C_bc[0][2],
                            	      C_bc[1][0], C_bc[1][1], C_bc[1][2],
	                              C_bc[2][0], C_bc[2][1], C_bc[2][2];

    	return T_b_c;
    }

    void fillCovarianceArray( const asrl::math::Transformation & T_to_from, boost::array<double, 36> & outCovariance)
    {
      fillCovarianceArray(T_to_from.U(),outCovariance);
    }

    void fillCovarianceArray( const asrl::math::Transformation::covariance_t & U_to_from, boost::array<double, 36> & outCovariance)
    {
      int p = 0;
      for(int c = 0; c < U_to_from.cols(); c++)
	for(int r = 0; r < U_to_from.rows(); r++)
	  {
	    outCovariance[p++] = U_to_from(r,c);
	  }

    }

    asrl::math::Transformation::covariance_t buildCovarianceMatrix(const boost::array<double, 36> & covariance)
    {
      asrl::math::Transformation::covariance_t U;
      int p = 0;
      for(int c = 0; c < U.cols(); c++)
	for(int r = 0; r < U.rows(); r++)
	  {
	    U(r,c) = covariance[p++];
	  }
      return U;
    }




    tf::StampedTransform fromPoseToStampedTransform(const geometry_msgs::Pose& pose)
    {
    	tf::StampedTransform T;
    	T.setIdentity();
    	tf::Point p(0,0,0);
    	tf::Quaternion q(0,0,0,1);
    	/*
      p.setX((btScalar)pose.position.x);
    	p.setY((btScalar)pose.position.y);
    	p.setZ((btScalar)pose.position.z);
    	q.setX((btScalar)pose.orientation.x);
    	q.setY((btScalar)pose.orientation.y);
    	q.setZ((btScalar)pose.orientation.z);
    	q.setW((btScalar)pose.orientation.w);
      */

      p.setX((tfScalar)pose.position.x);
      p.setY((tfScalar)pose.position.y);
      p.setZ((tfScalar)pose.position.z);
      q.setX((tfScalar)pose.orientation.x);
      q.setY((tfScalar)pose.orientation.y);
      q.setZ((tfScalar)pose.orientation.z);
      q.setW((tfScalar)pose.orientation.w);
    	
      /*
      p.setX((double)pose.position.x);
      p.setY((double)pose.position.y);
      p.setZ((double)pose.position.z);
      q.setX((double)pose.orientation.x);
      q.setY((double)pose.orientation.y);
      q.setZ((double)pose.orientation.z);
      q.setW((double)pose.orientation.w);
      */

      T.setOrigin(p);
    	T.setRotation(q);
    	return T;
    }

    geometry_msgs::Vector3 quat2rpy(const tf::Quaternion & q)
    {
      geometry_msgs::Vector3 rpy;

      rpy.x = atan2( 2.0 * (q.getW()*q.getX() + q.getY()*q.getZ()), 1.0 - 2.0 * (q.getX()*q.getX() + q.getY()*q.getY()) );
      rpy.y = asin(  2.0 * (q.getW()*q.getY() - q.getZ()*q.getX()) );
      rpy.z = atan2( 2.0 * (q.getW()*q.getZ() + q.getX()*q.getY()), 1.0 - 2.0 * (q.getY()*q.getY() + q.getZ()*q.getZ()) );

      return rpy;
    }

  }} // namespace asrl::rosutil

