#include <asrl/math2/SpecialEuclidean3.hpp>

#include <asrl/assert_macros.hpp>

namespace asrl { 
namespace math2 {

  /// *************************************************************
  ///
  /// "Cross-Matrix" equivalents for the 4x4 and 6x6 matrice forms
  ///
  /// *************************************************************

  // "Box Plus"
  Eigen::Matrix4d cross4(const Eigen::Vector3d & translation, const Eigen::Vector3d & aaxis)
  {
    Eigen::Matrix4d mat;
    mat <<      0.0,  -aaxis[2],   aaxis[1],  -translation[0],
           aaxis[2],        0.0,  -aaxis[0],  -translation[1],
          -aaxis[1],   aaxis[0],        0.0,  -translation[2], 
  	            0.0,        0.0,        0.0,              0.0;
    return mat;
  }
  Eigen::Matrix4d cross4(const Eigen::Matrix<double,6,1> & se3vec) { return cross4(se3vec.head<3>(), se3vec.tail<3>()); }

  /// "Box Cross"
  Eigen::Matrix<double,6,6> cross6(const Eigen::Vector3d & trans, const Eigen::Vector3d & angle)
  {
    Eigen::Matrix<double,6,6> mat = Eigen::Matrix<double,6,6>::Zero();
    mat.topLeftCorner<3,3>() = mat.bottomRightCorner<3,3>() = asrl::math2::cross(angle);
    mat.topRightCorner<3,3>() = asrl::math2::cross(trans);
    return mat;
  }
  Eigen::Matrix<double,6,6> cross6(const Eigen::Matrix<double,6,1> & se3vec) { return cross6(se3vec.head<3>(), se3vec.tail<3>()); }
  
  /// *************************************************************
  ///
  /// Exponential Maps
  ///
  /// *************************************************************

  /// expMap4
  void ExpMap4(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Vector3d * r, unsigned int numTerms)
  {    
    if (numTerms == 0)
    {
      Eigen::Matrix3d S;
      asrl::math2::AxisAngleToCandS(aaxis, C, &S); // return C
      (*r) = S*trans; // return r
    }
    else if (numTerms == 1)
    {
      (*C) = Eigen::Matrix3d::Identity() - cross(aaxis);  // return
      (*r) = trans; // return
    }
    else if (numTerms == 2)
    {
      Eigen::Matrix4d x_small = cross4(trans, aaxis);
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity() - x_small + 0.5*x_small*x_small;
      (*C) = T.topLeftCorner<3,3>();  // return
      (*r) = T.topRightCorner<3,1>(); // return
    }
    else if (numTerms == 3)
    {
      Eigen::Matrix4d x_small = cross4(trans, aaxis);
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity() - x_small + 0.5*x_small*x_small - 0.166666667*x_small*x_small*x_small;
      (*C) = T.topLeftCorner<3,3>();  // return
      (*r) = T.topRightCorner<3,1>(); // return
    }
    else // numTerms => 4, it is more cost efficient to actually use the analytical solution (numTerms = 0)
    {
      std::cout << "[WARNING]: ExpMap4, numTerms > 3, reverting to more optimal analytical solution." << std::endl;
      ExpMap4(trans, aaxis, C, r, 0);
      
      /*  ---------- Numerical Solution: Good for testing the analytical solution
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); // numTerms 0

      // Incremental variables
      bool sign = true; // positive
      double factorial = 1.0;
      Eigen::Matrix4d x_small = cross4(trans, aaxis); // small angle approximation
      Eigen::Matrix4d x_small_n = Eigen::Matrix4d::Identity(); // cross4(angle_axis)^N - initialized for N = 0
      
      // Loop over sum up to the specified numTerms
      for (unsigned int n = 1; n <= numTerms; n++)
      {
        sign = !sign;
        factorial = factorial*n;
        x_small_n = x_small_n*x_small;
        if (sign) T += (1.0/factorial)*x_small_n;
        else      T -= (1.0/factorial)*x_small_n;
      }
      
      // Return
      (*C) = T.topLeftCorner<3,3>();
      (*r) = T.topRightCorner<3,1>();
      --------------------------------------------------------------------------*/
    }
  }
  void ExpMap4(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix3d * C, Eigen::Vector3d * r, unsigned int numTerms)
  { 
    ExpMap4(se3vec.head<3>(), se3vec.tail<3>(), C, r, numTerms); 
  }
  Eigen::Matrix4d ExpMap4(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms)
  {
    Eigen::Matrix3d C;
    Eigen::Vector3d r;
    ExpMap4(se3vec, &C, &r, numTerms);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>() = C;
    T.topRightCorner<3,1>() = r;
    return T;
  }
  
  void ExpMap4andS(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Vector3d * r, Eigen::Matrix3d * S, unsigned int numTerms)
  {    
    asrl::math2::AxisAngleToCandS(aaxis, C, S, numTerms); // return C and S(3x3)
    (*r) = (*S)*trans; // return r
  }
  
  void ExpMap4andS(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Vector3d * r, Eigen::Matrix<double,6,6> * S, unsigned int numTerms)
  {
    Eigen::Matrix3d S3;
    ExpMap4andS(trans, aaxis, C, r, &S3, numTerms);
    (*S) = Eigen::Matrix<double,6,6>::Zero();
    S->topLeftCorner<3,3>() = S->bottomRightCorner<3,3>() = S3;
    S->topRightCorner<3,3>() = se3vecToQ(trans, aaxis, numTerms);
  }
  
  void ExpMap4andS(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix3d * C, Eigen::Vector3d * r, Eigen::Matrix<double,6,6> * S, unsigned int numTerms)
  {
    ExpMap4andS(se3vec.head<3>(), se3vec.tail<3>(), C, r, S, numTerms); 
  }
  
  Eigen::Matrix<double,6,1> se3vecFromExpMap4(const Eigen::Matrix3d & C, const Eigen::Vector3d & r)
  {
    Eigen::Matrix<double,6,1> se3vec;
    se3vec.tail<3>() = asrl::math2::CToAxisAngle(C);
    se3vec.head<3>() = asrl::math2::AxisAngleToSinv(se3vec.tail<3>())*r;
    return se3vec;
  }
  
  /// expmap6
  Eigen::Matrix<double,6,6> ExpMap6(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, unsigned int numTerms)
  {    
    /// Instead of writing out the exponential map summation for the 6x6, it is
    /// much faster (although not exactly the same) to use the approximation of
    /// the ExpMap4 and then appropriately construct the 6x6
    Eigen::Matrix3d C;
    Eigen::Vector3d r;
    ExpMap4(trans, aaxis, &C, &r, numTerms);
    Eigen::Matrix<double,6,6> mat = Eigen::Matrix<double,6,6>::Zero();
    mat.topLeftCorner<3,3>() = mat.bottomRightCorner<3,3>() = C;
    mat.topRightCorner<3,3>() = -asrl::math2::cross(r)*C;
    return mat;
  }
  Eigen::Matrix<double,6,6> ExpMap6(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms)
  { 
    return ExpMap6(se3vec.head<3>(), se3vec.tail<3>(), numTerms); 
  }
  
  Eigen::Matrix<double,6,6> ExpMap6FromS(const Eigen::Matrix<double,6,1> & se3vec, const Eigen::Matrix<double,6,6> & S)
  {
    Eigen::Matrix<double,6,6> Tse = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix3d ac = asrl::math2::cross(se3vec.tail<3>());
    Tse.topLeftCorner<3,3>() = Tse.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() - ac*S.topLeftCorner<3,3>();
    Tse.topRightCorner<3,3>() = - ac*S.topRightCorner<3,3>() - asrl::math2::cross(se3vec.head<3>())*S.topLeftCorner<3,3>();
    return Tse;
  }
  
  /// *************************************************************
  ///
  /// Special SE(3)
  ///
  /// For the lack of better description, the current documentation
  /// refers to a special set of matrices, defined as S and Q.
  /// The S matrix in SO(3) relates translations "r" and "rho".
  /// The S6 matrix and Q matrix extend this to SE(3).
  ///
  /// *************************************************************
  
  Eigen::Matrix3d se3vecToQ(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, unsigned int numTerms)
  {
    double a = sqrt(aaxis[0]*aaxis[0] + aaxis[1]*aaxis[1] + aaxis[2]*aaxis[2]); // Get angle
    if (a < 1e-12)
      return Eigen::Matrix3d::Zero();
    
    if (numTerms == 0)
    {
      double one_a = 1.0/a;
      double one_a3 = one_a*one_a*one_a;
      double one_a4 = one_a3*one_a;
      double sa = sin(a);
      double ca = cos(a);    
      double b = one_a3*(a - sa);
      double c = one_a4*(1.0 - 0.5*a*a - ca);
      double d = -0.5*(c - 3.0*one_a4*one_a*(a - sa - 0.166666667*a*a*a));
      Eigen::Matrix3d tc = asrl::math2::cross(trans);
      Eigen::Matrix3d ac = asrl::math2::cross(aaxis);
      Eigen::Matrix3d B = ac*tc + tc*ac - ac*tc*ac;
      Eigen::Matrix3d C = ac*ac*tc + tc*ac*ac - 3.0*ac*tc*ac;
      Eigen::Matrix3d D = ac*tc*ac*ac + ac*ac*tc*ac;
      return -0.5*tc + b*B + c*C + d*D;
    }
    else if (numTerms == 1)
    {
      return -0.5*asrl::math2::cross(trans);
    }
    else if (numTerms == 2)
    {
      Eigen::Matrix3d tc = asrl::math2::cross(trans);
      Eigen::Matrix3d ac = asrl::math2::cross(aaxis);
      return -0.5*tc + 0.166666667*tc*ac + 0.166666667*ac*tc + 0.041666667*ac*tc*ac;
    }
    else
    {
      std::cout << "[WARNING]: se3vecToQ, numTerms > 2, reverting to more optimal analytical solution." << std::endl;
      return se3vecToQ(trans, aaxis, 0);
      
      /*  ---------- Numerical Solution: Good for testing the analytical solution
      Eigen::Matrix3d tc = asrl::math2::cross(trans);
      Eigen::Matrix3d ac = asrl::math2::cross(aaxis);
      Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
      
      double factorialN = 2.0;
      bool signN = false; // positive
      Eigen::Matrix3d left_ac = Eigen::Matrix3d::Identity();
      
      for (unsigned int n = 0; n <= numTerms - 1; n++)
      {
        if (n > 0) 
        { 
          factorialN = factorialN*(n+2); 
          signN = !signN;
          left_ac = left_ac*ac;
        }
        Eigen::Matrix3d right_ac = Eigen::Matrix3d::Identity();
        bool sign = signN;
        double factorial = factorialN;
        for (unsigned int m = 0; m <= numTerms - 1; m++)
        {
          if (m > 0) 
          { 
            factorial = factorial*(n+m+2); 
            sign = !sign;
            right_ac = right_ac*ac;
          }
          if (sign) { Q += (1.0/factorial)*left_ac*tc*right_ac; }
          else      { Q -= (1.0/factorial)*left_ac*tc*right_ac; }
        }
      }
      return Q;
      --------------------------------------------------------------------------*/
    }
  }
  
  Eigen::Matrix<double,6,6> se3vecToS(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, unsigned int numTerms)
  {
      Eigen::Matrix<double,6,6> S = Eigen::Matrix<double,6,6>::Zero();
      S.topLeftCorner<3,3>() = S.bottomRightCorner<3,3>() = asrl::math2::AxisAngleToS(aaxis, numTerms);
      S.topRightCorner<3,3>() = se3vecToQ(trans, aaxis, numTerms);
      return S;
  }
  Eigen::Matrix<double,6,6> se3vecToS(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms) { return se3vecToS(se3vec.head<3>(), se3vec.tail<3>(), numTerms); }
  
  
  Eigen::Matrix<double,6,6> se3vecToSinv(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis)
  {
      Eigen::Matrix3d Sinv3 = asrl::math2::AxisAngleToSinv(aaxis);
      Eigen::Matrix<double,6,6> Sinv = Eigen::Matrix<double,6,6>::Zero();
      Sinv.topLeftCorner<3,3>() = Sinv.bottomRightCorner<3,3>() = Sinv3;
      Sinv.topRightCorner<3,3>() = -Sinv3*se3vecToQ(trans, aaxis)*Sinv3;
      return Sinv;
  }
  Eigen::Matrix<double,6,6> se3vecToSinv(const Eigen::Matrix<double,6,1> & se3vec) { return se3vecToSinv(se3vec.head<3>(), se3vec.tail<3>()); }
  
  /// *************************************************************
  ///
  /// Homogeneous Point Operators
  ///
  /// *************************************************************
  
  // Box Minus
  Eigen::Matrix<double,4,6> pointTo4x6(double x, double y, double z, double s)
  { 
    Eigen::Matrix<double,4,6> mat;
    mat <<    s,     0.0,     0.0,     0.0,        -z,        y,
            0.0,       s,     0.0,       z,       0.0,       -x, 
            0.0,     0.0,       s,      -y,         x,      0.0,
            0.0,     0.0,     0.0,     0.0,       0.0,      0.0;
    return mat;
  }
  Eigen::Matrix<double,4,6> pointTo4x6(const Eigen::Vector3d & p, double s) { return pointTo4x6(p[0], p[1], p[2], s); }
  Eigen::Matrix<double,4,6> pointTo4x6(const Eigen::Vector4d & p) { return pointTo4x6(p[0], p[1], p[2], p[3]); }


  }} // namespace asrl::math
  
  
  
