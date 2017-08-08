#include <asrl/math/quaternion_algebra.hpp>
#include <asrl/assert_macros.hpp>
#include <Eigen/Geometry>

namespace asrl { namespace math {

    // quaternion rotation.
    Eigen::Vector4d r2quat(Eigen::Matrix3d const & R){
      
      double c1 = R(0,0);
      double c2 = R(1,0);
      double c3 = R(2,0);
      double c4 = R(0,1);
      double c5 = R(1,1);
      double c6 = R(2,1);
      double c7 = R(0,2);
      double c8 = R(1,2);
      double c9 = R(2,2);

      Eigen::Vector4d aq;

      double one = 1.0;
      double half = 0.5;
      double quarter = 0.25;
      //	%Compute absolute values of the four quaternions from diags of Eqn 12-13
      aq(0)=half*sqrt(fabs(one+c1-c5-c9));
      aq(1)=half*sqrt(fabs(one-c1+c5-c9));
      aq(2)=half*sqrt(fabs(one-c1-c5+c9));
      aq(3)=half*sqrt(fabs(one+c1+c5+c9));

      unsigned maxq = 0;
      double maxqval = aq(0);

      for(unsigned i = 1; i < 4; i++) {
        if(aq(i) > maxqval) {
          maxq = i;
          maxqval = aq(i);
        }
      }

      double q1,q2,q3,q4,c;
      if(maxq == 0){
        q1=aq(0);
        c = quarter/aq(0);
        q2=c*(c4+c2);
        q3=c*(c7+c3);
        q4=c*(c8-c6);
      } else if(maxq == 1){
        q2=aq(1);
        c = quarter/aq(1);
        q1=c*(c4+c2);
        q3=c*(c6+c8);
        q4=c*(c3-c7);
      } else if(maxq == 2){
        q3=aq(2);
        c = quarter/aq(2);
        q1=c*(c3+c7);
        q2=c*(c6+c8);
        q4=c*(c4-c2);
      } else {
        q4=aq(3);
        c = quarter/aq(3);
        q1=c*(c8-c6);
        q2=c*(c3-c7);
        q3=c*(c4-c2);
      }
      Eigen::Vector4d q(q1,q2,q3,q4);
      if(q(3) < 0)
        q = -q;

      return q;
    }

    Eigen::Matrix3d quat2r(Eigen::Vector4d const & q){
    
      ASRL_ASSERT_EQ_TOL_DBG(std::runtime_error,q.norm(),1.f,1e-4, "The quaternion must be a unit vector to represent a rotation");
      //double n = q(3);
      //Eigen::Vector3d e = makeV3(q(0),q(1),q(2));
      //R = (n^2 - e'*e) * eye(3) + 2 * e * e' + 2 * n * crossMx(e);
      //Eigen::Matrix3d R = (n*n - e.dot(e))*Eigen::Matrix3d::Identity() + 2 * e * e.transpose() + 2 * n * crossMx(e);
    
      Eigen::Matrix3d R;

      // [ q0^2 - q1^2 - q2^2 + q3^2,           2*q0*q1 + 2*q2*q3,           2*q0*q2 - 2*q1*q3]
      // [         2*q0*q1 - 2*q2*q3, - q0^2 + q1^2 - q2^2 + q3^2,           2*q0*q3 + 2*q1*q2]
      // [         2*q0*q2 + 2*q1*q3,           2*q1*q2 - 2*q0*q3, - q0^2 - q1^2 + q2^2 + q3^2]
      R(0,0) = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
      R(0,1) = q[0]*q[1]*2.0+q[2]*q[3]*2.0;
      R(0,2) = q[0]*q[2]*2.0-q[1]*q[3]*2.0;
      R(1,0) = q[0]*q[1]*2.0-q[2]*q[3]*2.0;
      R(1,1) = -q[0]*q[0]+q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
      R(1,2) = q[0]*q[3]*2.0+q[1]*q[2]*2.0;
      R(2,0) = q[0]*q[2]*2.0+q[1]*q[3]*2.0;
      R(2,1) = q[0]*q[3]*(-2.0)+q[1]*q[2]*2.0;
      R(2,2) = -q[0]*q[0]-q[1]*q[1]+q[2]*q[2]+q[3]*q[3];

      return R;
    }


    Eigen::Matrix4d quatPlus(Eigen::Vector4d const & q)
    {
      // [  q3,  q2, -q1, q0]
      // [ -q2,  q3,  q0, q1]
      // [  q1, -q0,  q3, q2]
      // [ -q0, -q1, -q2, q3]
      Eigen::Matrix4d Q;
      Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
      Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
      Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
      Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];

      return Q;
    }

    Eigen::Matrix4d quatOPlus(Eigen::Vector4d const & q)
    {
      // [  q3, -q2,  q1, q0]
      // [  q2,  q3, -q0, q1]
      // [ -q1,  q0,  q3, q2]
      // [ -q0, -q1, -q2, q3]

      Eigen::Matrix4d Q;
      Q(0,0) =  q[3]; Q(0,1) = -q[2]; Q(0,2) =  q[1]; Q(0,3) =  q[0];
      Q(1,0) =  q[2]; Q(1,1) =  q[3]; Q(1,2) = -q[0]; Q(1,3) =  q[1];
      Q(2,0) = -q[1]; Q(2,1) =  q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
      Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];

      return Q;

    }

    Eigen::Vector4d qplus(Eigen::Vector4d const & q, Eigen::Vector4d const & p)
    {
      Eigen::Vector4d qplus_p;
      // p0*q3 + p1*q2 - p2*q1 + p3*q0
      qplus_p[0] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
      // p2*q0 - p0*q2 + p1*q3 + p3*q1
      qplus_p[1] = p[2]*q[0] - p[0]*q[2] + p[1]*q[3] + p[3]*q[1];
      // p0*q1 - p1*q0 + p2*q3 + p3*q2
      qplus_p[2] = p[0]*q[1] - p[1]*q[0] + p[2]*q[3] + p[3]*q[2];
      // p3*q3 - p1*q1 - p2*q2 - p0*q0
      qplus_p[3] = p[3]*q[3] - p[1]*q[1] - p[2]*q[2] - p[0]*q[0];

      return qplus_p;
    }

    Eigen::Vector4d qoplus(Eigen::Vector4d const & q, Eigen::Vector4d const & p)
    {
      Eigen::Vector4d qoplus_p;
      // p0*q3 - p1*q2 + p2*q1 + p3*q0
      qoplus_p[0] = p[0]*q[3] - p[1]*q[2] + p[2]*q[1] + p[3]*q[0];
      // p0*q2 - p2*q0 + p1*q3 + p3*q1
      qoplus_p[1] = p[0]*q[2] - p[2]*q[0] + p[1]*q[3] + p[3]*q[1];
      // p1*q0 - p0*q1 + p2*q3 + p3*q2
      qoplus_p[2] = p[1]*q[0] - p[0]*q[1] + p[2]*q[3] + p[3]*q[2];
      // p3*q3 - p1*q1 - p2*q2 - p0*q0
      qoplus_p[3] = p[3]*q[3] - p[1]*q[1] - p[2]*q[2] - p[0]*q[0];

      return qoplus_p;
    }

    Eigen::Vector4d quatInv(Eigen::Vector4d const & q)
    {
      Eigen::Vector4d qret = q;
      invertQuat(qret);
      return qret;
    }

    void invertQuat(Eigen::Vector4d & q)
    {
      q.head<3>() = -q.head<3>();
    }


    Eigen::Vector3d qeps(Eigen::Vector4d const & q)
    {
      return q.head<3>();
    }
  
    double qeta(Eigen::Vector4d const & q)
    {
      return q[3];
    }



    Eigen::Vector4d axisAngle2quat(Eigen::Vector3d const & a)
    {
      double theta = a.norm();
      if(theta < 1e-12)
        return Eigen::Vector4d(0,0,0,1);
    
      Eigen::Vector3d axis = a/theta;
      theta*=0.5;
      double st = sin(theta);
      double ct = cos(theta);
      axis *= st;
      return Eigen::Vector4d(axis[0],axis[1],axis[2],ct);

    }

    Eigen::Vector3d quat2AxisAngle(Eigen::Vector4d const & q)
    {
      double theta = 2*acos(qeta(q));
      if(theta < 1e-12)
        return Eigen::Vector3d::Zero();

      Eigen::Vector3d a = qeps(q);
      a /= a.norm();
      a *= theta;
      return a;
    }

    Eigen::Matrix<double,4,3> quatJacobian(Eigen::Vector4d const & p)
    {
      Eigen::Matrix<double,4,3> J;
      // [  p3, -p2,  p1]
      // [  p2,  p3, -p0]
      // [ -p1,  p0,  p3]
      // [ -p0, -p1, -p2]
    
      J(0,0) =  p[3];
      J(0,1) = -p[2];
      J(0,2) =  p[1];
      J(1,0) =  p[2];
      J(1,1) =  p[3];
      J(1,2) = -p[0];
      J(2,0) = -p[1];
      J(2,1) =  p[0];
      J(2,2) =  p[3];
      J(3,0) = -p[0];
      J(3,1) = -p[1];
      J(3,2) = -p[2];

      return J*0.5;
    }

    Eigen::Vector4d updateQuat(Eigen::Vector4d const & q, Eigen::Vector3d const & dq)
    {
      // the following code is an optimized version of:
      // Eigen::Vector4d dq4 = axisAngle2quat(dq);
      // Eigen::Vector4d retq = quatPlus(dq4)*q;
      // return retq;

      double angle = dq.norm();
      if(angle < 1e-12)
        return q;

      double angle_2 = angle*0.5;
      double sa = sin(angle_2);
      double recip_a = 1.0/angle;
      double ca = cos(angle_2);
      double sa_over_a = sa*recip_a;
      Eigen::Vector3d dq3 = dq * sa_over_a;
      Eigen::Vector4d retq;
      retq[0] = q[0]*ca+dq3[0]*q[3]-dq3[1]*q[2]+dq3[2]*q[1];
      retq[1] = q[1]*ca+dq3[0]*q[2]+dq3[1]*q[3]-dq3[2]*q[0];
      retq[2] = q[2]*ca-dq3[0]*q[1]+dq3[1]*q[0]+dq3[2]*q[3];
      retq[3] = q[3]*ca-dq3[0]*q[0]-dq3[1]*q[1]-dq3[2]*q[2];

      return retq;
    }

  }} // namespace asrl::math

