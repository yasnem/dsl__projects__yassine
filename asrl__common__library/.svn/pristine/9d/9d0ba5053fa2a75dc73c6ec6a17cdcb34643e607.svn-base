///
/// @file   SpecialOrthogonal3.hpp
/// @author Sean Anderson <sean.anderson@mail.utoronto.ca>
/// @date   Friday, 24 May 2013
///
/// @brief  Utility functions for the Special Orthogonal Group SO(3)
///         Should be agnostic of Transformation class. Only use Eigen.
///         Adapted from asrl/math/rotations.hpp
///

#ifndef ASRL_MATH2_SPECIALORTHOGONAL3_HPP
#define ASRL_MATH2_SPECIALORTHOGONAL3_HPP

// Define various pi functions

#define ASRL_1_PI   	  0.3183098861837907 // 1/pi
#define ASRL_2_PI 		  0.6366197723675814 // 2/pi
#define ASRL_2PI 		    6.283185307179586  // 2*pi
#define ASRL_PI 		    3.141592653589793  // pi
#define ASRL_PI_2 		  1.5707963267948966 // pi/2
#define ASRL_PI_4 		  0.7853981633974483 // pi/4
#define ASRL_DEG2RAD	  0.017453292519943  // pi/180
#define ASRL_RAD2DEG	  57.295779513082323 // 180/pi

#define ASRL_1_PI_F 	  0.3183098861837907f
#define ASRL_2_PI_F 	  0.6366197723675814f
#define ASRL_PI_F 		  3.141592653589793f
#define ASRL_PI_2_F 	  1.5707963267948966f
#define ASRL_PI_4_F		  0.7853981633974483f
#define ASRL_2PI_F 		  6.283185307179586f
#define ASRL_DEG2RAD_F	0.017453292519943f
#define ASRL_RAD2DEG_F	57.295779513082323f

#include <Eigen/Core>

/// *************************************************************
///
/// Foreword
///
/// UTIAS uses the "more standard" C matrix rotations. These are
/// opposite (negative angle / tranpose) to the ones normally shown 
/// in wikipedia or computer graphics.
///
/// If a counter-clockwise rotation (theta) about some axis is 
/// applied at frame A to get to frame B, then:
///
///       p_a = R(theta) * p_b     --> "R_ab"
///       p_b = C(theta) * p_a     --> "C_ba"
///
/// *************************************************************

namespace asrl { 
namespace math2 {

  /// *************************************************************
  ///
  /// Euler Angles Conversions (Roll Pitch Heading)
  ///
  /// *************************************************************

  /// 
  /// Single axis rotations
  /// 
  /// @param radians counter-clockwise around axis
  ///
  /// @return C_ba rotation matrix
  ///
  Eigen::Matrix3d CfromRotX(double radians);
  Eigen::Matrix3d CfromRotY(double radians);
  Eigen::Matrix3d CfromRotZ(double radians);
  
  /// 
  /// Roll-Pitch-Heading rotations
  /// 
  /// C = [ cos(z)*cos(y),  sin(z)*cos(x)+cos(z)*sin(y)*sin(x),  sin(z)*sin(x)-cos(z)*sin(y)*cos(x)]
  ///     [-sin(z)*cos(y),  cos(z)*cos(x)-sin(z)*sin(y)*sin(x),  cos(z)*sin(x)+sin(z)*sin(y)*cos(x)]
  ///     [        sin(y),                      -cos(y)*sin(x),                       cos(y)*cos(x)]
  ///
  /// @param counter-clockwise radians around x axis
  /// @param counter-clockwise radians around y axis
  /// @param counter-clockwise radians around z axis
  ///
  /// @return C_ba rotation matrix
  ///
  Eigen::Matrix3d CfromRPH(double x, double y, double z);
  Eigen::Matrix3d CfromRPH(const Eigen::Vector3d & x);
  
  /// 
  /// Inverse Operation to Recover Roll-Pitch-Heading rotations
  /// 
  /// @param C_ba rotation matrix
  ///
  /// @return counter-clockwise radians around x, y and z axis
  ///
  Eigen::Vector3d CtoRPH(const Eigen::Matrix3d & C);

  /// *************************************************************
  ///
  /// Axis-Angle/Rotation Conversions
  ///
  ///
  /// axis_angle = angle*unitv = a*[ax ay az], where sqrt(ax*ax + ay*ay + az*az) == 1.0
  ///
  /// C_ba = cos(a)*identity + (1 - cos(a))*unitv*unitv^T - sin(a)*cross(unitv)
  ///      = [    ca + (1-ca)*ax*ax, (1-ca)*ax*ay + sa*az, (1-ca)*ax*az-sa*ay]
  ///        [ (1-ca)*ax*ay - sa*az,    ca + (1-ca)*ay*ay, (1-ca)*ay*az+sa*ax]
  ///        [ (1-ca)*ax*az + sa*ay, (1-ca)*ay*az - sa*ax,  ca + (1-ca)*az*az]
  ///
  /// Naive implementation:
  ///    Eigen::Vector3d unit(ax, ay, az);
  ///    return ca*Eigen::Matrix3d::Identity() + (double(1) - ca)*unit*unit.transpose() - sa*cross(ax, ay, az);
  ///
  /// *************************************************************
  
  /// 
  /// see above for details on axis-angle
  /// 
  /// @param a radians
  /// @param ax x component of unit axis
  /// @param ay y component of unit axis
  /// @param az z component of unit axis
  ///
  /// @return C_ba rotation matrix
  ///
  Eigen::Matrix3d AxisAngleToC(double a, double ax, double ay, double az);
  Eigen::Matrix3d AxisAngleToC(double x, double y, double z);
  Eigen::Matrix3d AxisAngleToC(const Eigen::Vector3d & aaxis);
  
  /// 
  /// Recover Axis-Angle
  /// 
  /// @param C_ba rotation matrix
  ///
  /// @return axis angle representation
  ///
  Eigen::Vector3d CToAxisAngle(const Eigen::Matrix3d & C);

  /// *************************************************************
  ///
  /// SO(3)
  ///
  /// *************************************************************

  /// 
  /// Skew-symmetric, linear cross-product operator
  /// 
  /// @param x scalar, typically a small angle (in radians) around x axis
  /// @param y scalar, typically a small angle (in radians) around y axis
  /// @param z scalar, typically a small angle (in radians) around z axis
  ///
  /// @return skew-symmetric, linear cross-product matrix
  ///
  Eigen::Matrix3d cross(double x, double y, double z);
  Eigen::Matrix3d cross(const Eigen::Vector3d & aaxis);

  /// 
  /// Exponential Map for C_ba (rotation matrix)
  /// 
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 4
  ///
  /// @return C_ba rotation matrix
  ///
  Eigen::Matrix3d ExpMap3(const Eigen::Vector3d & aaxis, unsigned int numTerms = 0);

  /// 
  /// "S_ba" Matrix, relates the body centric translation (rho) to the transformation matrix translation (r)
  /// 
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 4
  ///
  /// @return S_ba translation relation matrix
  ///
  Eigen::Matrix3d AxisAngleToS(double a, double ax, double ay, double az);
  Eigen::Matrix3d AxisAngleToS(double x, double y, double z);
  Eigen::Matrix3d AxisAngleToS(const Eigen::Vector3d & aaxis, unsigned int numTerms = 0);

  /// 
  /// Get C_ba and S_ba simultaneously (more efficient)
  /// 
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param *C return C_ba by reference
  /// @param *S return S_ba by reference
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 4
  ///
  void AxisAngleToCandS(const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Matrix3d * S, unsigned int numTerms = 0);

  /// 
  /// Inverse "S_ba" Matrix, relates the body centric translation (rho) to the transformation matrix translation (r)
  /// 
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  ///
  /// @return inverse S_ba translation relation matrix
  ///
  Eigen::Matrix3d AxisAngleToSinv(double a, double ax, double ay, double az);
  Eigen::Matrix3d AxisAngleToSinv(double x, double y, double z);
  Eigen::Matrix3d AxisAngleToSinv(const Eigen::Vector3d & aaxis);

  /// *************************************************************
  ///
  /// Utility Functions
  ///
  /// *************************************************************
  
  /// 
  /// Moves a value in radians to within -pi, pi
  /// 
  double angleMod(double radians); 

  /// 
  /// Degree/Radian conversion
  /// 
  double deg2rad(double degrees);
  double rad2deg(double radians);

}} // namespace asrl::math2

#endif


