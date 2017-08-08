#include <asrl/math2/SpecialOrthogonal3.hpp>

#include <asrl/assert_macros.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

namespace asrl { 
namespace math2 {


  /// *************************************************************
  ///
  /// Euler Angles Conversions (Roll Pitch Heading)
  ///
  /// *************************************************************

  Eigen::Matrix3d CfromRotX(double radians)
  {
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) = 1.0; C(0,1) = 0.0; C(0,2) =  0;
    C(1,0) = 0.0; C(1,1) =  c;  C(1,2) =  s;
    C(2,0) = 0.0; C(2,1) = -s;  C(2,2) =  c;
    return C;
  }

  Eigen::Matrix3d CfromRotY(double radians)
  {
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) =  c;   C(0,1) = 0.0; C(0,2) = -s;
    C(1,0) =  0.0; C(1,1) = 1.0; C(1,2) = 0.0;
    C(2,0) =  s;   C(2,1) = 0.0; C(2,2) =  c;
    return C;
  }
  
  Eigen::Matrix3d CfromRotZ(double radians)
  {
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) =  c;  C(0,1) =  s;  C(0,2) = 0.0;
    C(1,0) = -s;  C(1,1) =  c;  C(1,2) = 0.0;
    C(2,0) = 0.0; C(2,1) = 0.0; C(2,2) = 1.0;
    return C;
  }
  
  Eigen::Matrix3d CfromRPH(double x, double y, double z)
  {     
    Eigen::Matrix3d C;
    double cx = cos(x);
    double sx = sin(x);
    double cy = cos(y);
    double sy = sin(y);
    double cz = cos(z);
    double sz = sin(z);
    
    C(0,0) =  cz*cy;  C(0,1) = sz*cx + cz*sy*sx;  C(0,2) = sz*sx - cz*sy*cx;
    C(1,0) = -sz*cy;  C(1,1) = cz*cx - sz*sy*sx;  C(1,2) = cz*sx + sz*sy*cx;
    C(2,0) =     sy;  C(2,1) =           -cy*sx;  C(2,2) =            cy*cx;

    return C;
  }
  Eigen::Matrix3d CfromRPH(const Eigen::Vector3d & x) 
  { 
    return CfromRPH(x[0], x[1], x[2]); 
  }

  Eigen::Vector3d CtoRPH(const Eigen::Matrix3d & C)
  {
    Eigen::Vector3d rph;
    rph[1] = asin(C(2,0));
    rph[2] = atan2(-C(1,0),C(0,0));
    rph[0] = atan2(-C(2,1),C(2,2));
    return rph;
  }  

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

  Eigen::Matrix3d AxisAngleToC(double a, double ax, double ay, double az)
  {
    ASRL_ASSERT_DBG(std::runtime_error, fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0) < 1e-4, "The axis is not a unit vector. ||a|| = " << (sqrt(ax*ax + ay*ay + az*az)));
		
		// If angle is very very small, just return Identity
    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();

    // Precalculate some values (testing showed its not worth precalculating ax*ax, ax*az, etc...)
    double sa = sin(a);
    double ca = cos(a);
    double const onemca = double(1) - ca;

    // Construct matrix
    Eigen::Matrix3d C;
    C(0,0) =  ca + onemca*ax*ax;     C(0,1) = onemca*ax*ay + sa*az;  C(0,2) = onemca*ax*az - sa*ay;
    C(1,0) =  onemca*ax*ay - sa*az;  C(1,1) = ca + onemca*ay*ay;     C(1,2) = onemca*ay*az + sa*ax;
    C(2,0) =  onemca*ax*az + sa*ay;  C(2,1) = onemca*ay*az - sa*ax;  C(2,2) = ca + onemca*az*az;
    return C;
  }
	
  Eigen::Matrix3d AxisAngleToC(double x, double y, double z) 
  {
    double a = sqrt(x*x + y*y + z*z); // Get angle
    if(a < 1e-12) return Eigen::Matrix3d::Identity(); // If angle is very very small, just return Identity   
    double d = 1.0/a; // Otherwise, get scaling
    return AxisAngleToC(a, x*d, y*d, z*d); // apply scaling and return C
  }
	
  Eigen::Matrix3d AxisAngleToC(const Eigen::Vector3d & aaxis) 
  { 
    return AxisAngleToC(aaxis[0], aaxis[1], aaxis[2]); 
  }  
	
	// Recover Axis-Angle from C
  Eigen::Vector3d CToAxisAngle(const Eigen::Matrix3d & C)
  {
    // Recover Angle
    double a = acos( (C(0,0) + C(1,1) + C(2,2) - double(1)) * double(0.5));
    
    // Check for nan
    if (boost::math::isnan(a)) 
    {
      std::cout << "[WARNING]: Improper rotation matrix: trace(C) > 3." << std::endl;
      return Eigen::Vector3d::Zero();
    }

    // If angle is very very small, just return zero.
    if(fabs(a) < 1e-10) 
      return Eigen::Vector3d::Zero();
		
		// Get axis directions
		Eigen::Vector3d axis;
    axis[0] = (C(2,1) - C(1,2));
    axis[1] = (C(0,2) - C(2,0));
    axis[2] = (C(1,0) - C(0,1));
    
    // Get normalization scaling
    double n2 = axis.norm();
    
    // If normalization is very very small, just return zero.
    if(fabs(n2) < 1e-10)
      return Eigen::Vector3d::Zero();
		
		// Properly scale vector back to axis-angle
    double scale = -a/n2;
    axis = scale * axis;
    return axis;
  }

  /// *************************************************************
  ///
  /// SO(3)
  ///
  /// *************************************************************

  /// Small angle approximation.
  Eigen::Matrix3d cross(double x, double y, double z)
  {
    Eigen::Matrix3d C;
    C(0,0) =  0.0;  C(0,1) =   -z;  C(0,2) =     y;
    C(1,0) =    z;  C(1,1) =  0.0;  C(1,2) =    -x;
    C(2,0) =   -y;  C(2,1) =    x;  C(2,2) =   0.0;
    return C;
  }
  Eigen::Matrix3d cross(const Eigen::Vector3d & aaxis)
  { 
    return cross(aaxis[0],aaxis[1],aaxis[2]); 
  }

  /// The SO(3) exponential mapping
  /// NumTerms 0: Use analytical solution
  /// numTerms > 0: Use approximation sum with numTerms terms
  /// e.g.: numTerms == 1:   eye(3) - cross(x)
  ///
  Eigen::Matrix3d ExpMap3(const Eigen::Vector3d & aaxis, unsigned int numTerms)
  {
    if (numTerms == 0)
    {
      return AxisAngleToC(aaxis);
    }
    else if (numTerms == 1)
    {
      return Eigen::Matrix3d::Identity() - cross(aaxis);
    }
    else if (numTerms == 2)
    {
      Eigen::Matrix3d x_small = cross(aaxis);
      return Eigen::Matrix3d::Identity() - x_small + 0.5*x_small*x_small;
    }
    else if (numTerms == 3)
    {
      Eigen::Matrix3d x_small = cross(aaxis);
      return Eigen::Matrix3d::Identity() - x_small + 0.5*x_small*x_small - 0.166666667*x_small*x_small*x_small;
    }
    else
    {
      std::cout << "[WARNING]: ExpMap3, numTerms > 3, reverting to more optimal analytical solution." << std::endl;
      return AxisAngleToC(aaxis);
      /*  ---------- Numerical Solution: Good for testing the analytical solution
      Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
    
      // Incremental variables
      bool sign = true; // positive
      double factorial = 1.0;
      Eigen::Matrix3d x_small = cross(aaxis); // small angle approximation
      Eigen::Matrix3d x_small_n = Eigen::Matrix3d::Identity(); // cross(angle_axis)^N - initialized for N = 0
    
      // Loop over sum up to the specified numTerms
      for (unsigned int n = 1; n <= numTerms; n++)
      {
        sign = !sign;
        factorial = factorial*n;
        x_small_n = x_small_n*x_small;
        if (sign) C += (1.0/factorial)*x_small_n;
        else      C -= (1.0/factorial)*x_small_n;
      }
      return C;
      --------------------------------------------------------------------------*/
    }
  }

  ///
  /// Axis Angle To "S" Matrix
  /// -- Relates the translations: r = S*rho
  ///
  /// axis_angle = angle*unitv = a*[ax ay az], where sqrt(ax*ax + ay*ay + az*az) == 1.0
  ///
  /// S = identity*sin(a)/a + (1 - sin(a)/a)*unitv*unitv^T - cross(unitv)*(1 - cos(a))/a
  ///   = [   saa + (1-saa)*ax*ax, (1-saa)*ax*ay + caa*az, (1-saa)*ax*az-caa*ay]
  ///     [(1-saa)*ax*ay - caa*az,    saa + (1-saa)*ay*ay, (1-saa)*ay*az+caa*ax]
  ///     [(1-saa)*ax*az + caa*ay, (1-saa)*ay*az - caa*ax,  saa + (1-saa)*az*az]
  ///
  /// Naive implementation:
  ///    Eigen::Vector3d unit(ax, ay, az);
  ///    return saa*Eigen::Matrix3d::Identity() + (double(1) - saa)*unit*unit.transpose() - caa*cross(ax, ay, az);
  ///
  Eigen::Matrix3d AxisAngleToS(double a, double ax, double ay, double az) 
  {
    ASRL_ASSERT_DBG(std::runtime_error,fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0) < 1e-4, "The axis is not a unit vector. ||a|| = " << (sqrt(ax*ax + ay*ay + az*az)));
	
    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();

    double saa = sin(a)/a;
    double onemsaa = double(1) - saa;
    double caa = (1-cos(a))/a;

    Eigen::Matrix3d S;
    S(0,0) =  saa + onemsaa*ax*ax;     S(0,1) = onemsaa*ax*ay + caa*az;  S(0,2) = onemsaa*ax*az - caa*ay;
    S(1,0) =  onemsaa*ax*ay - caa*az;  S(1,1) = saa + onemsaa*ay*ay;     S(1,2) = onemsaa*ay*az + caa*ax;
    S(2,0) =  onemsaa*ax*az + caa*ay;  S(2,1) = onemsaa*ay*az - caa*ax;  S(2,2) = saa + onemsaa*az*az;
    return S;
  }

  Eigen::Matrix3d AxisAngleToS(double x, double y, double z) 
  {
    double a = sqrt(x*x + y*y + z*z); // Get angle
    if(a < 1e-12) return Eigen::Matrix3d::Identity(); // If angle is very very small, just return Identity   
    double d = 1.0/a; // Otherwise, get scaling
    return AxisAngleToS(a, x*d, y*d, z*d); // apply scaling and return C
  }

  // "S" Matrix, relates the body centric translation (rho) to the transformation matrix translation (r)
  Eigen::Matrix3d AxisAngleToS(const Eigen::Vector3d & aaxis, unsigned int numTerms)
  {
    if (numTerms == 0)
    {
      return AxisAngleToS(aaxis[0], aaxis[1], aaxis[2]);
    }
    else if (numTerms == 1)
    {
      return Eigen::Matrix3d::Identity() - 0.5*cross(aaxis);
    }
    else if (numTerms == 2)
    {
      Eigen::Matrix3d x_small = cross(aaxis);
      return Eigen::Matrix3d::Identity() - 0.5*x_small + 0.166666667*x_small*x_small;
    }
    else if (numTerms == 3)
    {
      Eigen::Matrix3d x_small = cross(aaxis);
      return Eigen::Matrix3d::Identity() - 0.5*x_small + 0.166666667*x_small*x_small - 0.041666667*x_small*x_small*x_small;
    }
    else
    {
      std::cout << "[WARNING]: AxisAngleToS, numTerms > 3, reverting to more optimal analytical solution." << std::endl;
      return AxisAngleToS(aaxis[0], aaxis[1], aaxis[2]);
    }
  }

  void AxisAngleToCandS(const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Matrix3d * S, unsigned int numTerms)
  {
    (*S) = AxisAngleToS(aaxis, numTerms);
    (*C) = Eigen::Matrix3d::Identity() - cross(aaxis)*(*S);
  }

  ///
  /// Axis Angle To INVERSE "S" Matrix
  /// -- Relates the translations: r = S*rho
  ///
  /// axis_angle = angle*unitv = a*[ax ay az], where sqrt(ax*ax + ay*ay + az*az) == 1.0
  ///
  /// S = identity*(a/2)*cot(a/2) + (1 - (a/2)*cot(a/2))*unitv*unitv^T + cross(unitv)*a/2
  ///   = [   aca + (1-aca)*ax*ax, (1-aca)*ax*ay - a_2*az, (1-aca)*ax*az + a_2*ay]
  ///     [(1-aca)*ax*ay + a_2*az,    aca + (1-aca)*ay*ay, (1-aca)*ay*az - a_2*ax]
  ///     [(1-aca)*ax*az - a_2*ay, (1-aca)*ay*az + a_2*ax,    aca + (1-aca)*az*az]
  ///
  /// Naive implementation:
  ///    Eigen::Vector3d unit(ax, ay, az);
  ///    return aca*Eigen::Matrix3d::Identity() + (double(1) - aca)*unit*unit.transpose() + a_2*cross(ax, ay, az);
  ///
  Eigen::Matrix3d AxisAngleToSinv(double a, double ax, double ay, double az) 
  {
    ASRL_ASSERT_DBG(std::runtime_error,fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0) < 1e-4, "The axis is not a unit vector. ||a|| = " << (sqrt(ax*ax + ay*ay + az*az)));
	
    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();

    double a_2 = 0.5*a;
    double aca = a_2/tan(a_2);
    double onemaca = double(1) - aca;

    Eigen::Matrix3d S;
    S(0,0) =  aca + onemaca*ax*ax;     S(0,1) = onemaca*ax*ay - a_2*az;  S(0,2) = onemaca*ax*az + a_2*ay;
    S(1,0) =  onemaca*ax*ay + a_2*az;  S(1,1) = aca + onemaca*ay*ay;     S(1,2) = onemaca*ay*az - a_2*ax;
    S(2,0) =  onemaca*ax*az - a_2*ay;  S(2,1) = onemaca*ay*az + a_2*ax;  S(2,2) = aca + onemaca*az*az;
    return S;
  }
  Eigen::Matrix3d AxisAngleToSinv(double x, double y, double z) 
  {
    double a = sqrt(x*x + y*y + z*z); // Get angle
    if(a < 1e-12) return Eigen::Matrix3d::Identity(); // If angle is very very small, just return Identity   
    double d = 1.0/a; // Otherwise, get scaling
    return AxisAngleToSinv(a, x*d, y*d, z*d); // apply scaling and return C
  }
  Eigen::Matrix3d AxisAngleToSinv(const Eigen::Vector3d & aaxis) { return AxisAngleToSinv(aaxis[0], aaxis[1], aaxis[2]); }

  /// *************************************************************
  ///
  /// Utility Functions
  ///
  /// *************************************************************
  
  double angleMod(double radians) { return (double)(radians - (ASRL_2PI * rint(radians / ASRL_2PI))); }
  double deg2rad(double degrees)  { return (double)(degrees * ASRL_DEG2RAD); }
  double rad2deg(double radians)  { return (double)(radians * ASRL_RAD2DEG); }


  }} // asrl::math


