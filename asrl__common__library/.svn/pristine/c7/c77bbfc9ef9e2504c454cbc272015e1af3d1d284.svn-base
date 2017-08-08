///
/// @file   SpecialEuclidean3.hpp
/// @author Sean Anderson <sean.anderson@mail.utoronto.ca>
/// @date   Friday, 24 May 2013
///
/// @brief  Utility functions for the Special Euclidean Group SE(3)
///         Should be agnostic of Transformation class. Only use Eigen.
///         Adapted and extended from the translations.cpp code

#ifndef ASRL_MATH2_SPECIALEUCLIDEAN3_HPP
#define ASRL_MATH2_SPECIALEUCLIDEAN3_HPP

#include <asrl/math2/SpecialOrthogonal3.hpp>
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
  /// "Cross-Matrix" equivalents for the 4x4 and 6x6 matrice forms
  ///
  /// *************************************************************

  /// 
  /// Cross Operator equivalent for the 4x4 matrix 
  /// -- "Box Plus", as of Monday, 03 June 2013
  /// 
  /// @param translation 3x1 translation vector
  /// @param translation 3x1 axis-angle vector
  ///
  /// @return 4x4 cross matrix
  ///
  Eigen::Matrix4d cross4(const Eigen::Vector3d & translation, const Eigen::Vector3d & aaxis);
  ///
  /// @param se3vec is the 6x1 vector = [translation' aaxis'] = [rho' phi'];
  ///
  Eigen::Matrix4d cross4(const Eigen::Matrix<double,6,1> & se3vec); 

  /// 
  /// Cross Operator equivalent for the 6x6 matrix 
  /// -- "Box Cross", as of Monday, 03 June 2013
  /// 
  /// @param translation 3x1 translation vector
  /// @param translation 3x1 axis-angle vector
  ///
  /// @return 6x6 cross matrix
  ///
  Eigen::Matrix<double,6,6> cross6(const Eigen::Vector3d & trans, const Eigen::Vector3d & angle);
  ///
  /// @param se3vec is the 6x1 vector = [translation' aaxis'] = [rho' phi'];
  ///
  Eigen::Matrix<double,6,6> cross6(const Eigen::Matrix<double,6,1> & se3vec);

  /// *************************************************************
  ///
  /// Exponential Maps for the:
  ///   -- 4x4 Homogeneous Transformation Matrix
  ///   -- 6x6 Algebraic Equivalent to the Homogeneous Transformation Matrix
  ///
  /// *************************************************************

  /// 
  /// Exponential Map for T_ba (4x4 homogeneous transformation matrix)
  /// 
  /// @param trans translation (3x1 vector)
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param *C return C_ba by reference
  /// @param *r return r_ab_inb by reference
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 4
  ///
  void ExpMap4(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Vector3d * r, unsigned int numTerms = 0);
  ///
  /// @param se3vec is the 6x1 vector = [translation' aaxis'] = [rho' phi'];
  ///
  void ExpMap4(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix3d * C, Eigen::Vector3d * r, unsigned int numTerms = 0);
  ///
  /// @return 4x4 homogeneous transformation matrix
  ///
  Eigen::Matrix4d ExpMap4(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms = 0);
  ///
  /// @param *S return S_ba by reference (both 3x3 and 6x6 available)
  ///
  void ExpMap4andS(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Vector3d * r, Eigen::Matrix3d * S, unsigned int numTerms = 0);
  void ExpMap4andS(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, Eigen::Matrix3d * C, Eigen::Vector3d * r, Eigen::Matrix<double,6,6> * S, unsigned int numTerms = 0);
  void ExpMap4andS(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix3d * C, Eigen::Vector3d * r, Eigen::Matrix<double,6,6> * S, unsigned int numTerms = 0);
  
  /// 
  /// "Inverse" of the Exponential Map (4x4 homogeneous transformation matrix)
  /// 
  /// @param C_ba (3x3 rotation matrix)
  /// @param r_ab_inb (3x1 translation vector)
  ///
  /// @return se3vec is the 6x1 vector = [translation' aaxis'] = [rho' phi'];
  ///
  Eigen::Matrix<double,6,1> se3vecFromExpMap4(const Eigen::Matrix3d & C, const Eigen::Vector3d & r);
  
  /// 
  /// Exponential Map for Tsix_ba (6x6 algebraic equivalent transformation matrix)
  /// 
  /// @param trans translation (3x1 vector)
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 4
  ///
  /// @return 6x6 transformation matrix equivalent
  ///
  Eigen::Matrix<double,6,6> ExpMap6(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, unsigned int numTerms = 0);
  ///
  /// @param se3vec is the 6x1 vector = [translation' aaxis'] = [rho' phi'];
  ///
  Eigen::Matrix<double,6,6> ExpMap6(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms = 0);
  ///
  /// @param S is the 6x6 S Matrix
  ///
  Eigen::Matrix<double,6,6> ExpMap6FromS(const Eigen::Matrix<double,6,1> & se3vec, const Eigen::Matrix<double,6,6> & S);
  
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
  
  /// 
  /// Q Matrix (3x3)
  /// -- is used in calculating the 6x6 S Matrix
  /// 
  /// @param trans translation (3x1 vector)
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 3
  ///
  /// @return 3x3 Q Matrix
  ///
  Eigen::Matrix3d se3vecToQ(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, unsigned int numTerms = 0);
  
  /// 
  /// Ssix_ba (6x6 form of the "S" Matrix)
  /// -- hint: For small angles/distances, it is much more efficient to use a numerical solution, numTerms = 2 usually does well.
  /// 
  /// @param trans translation (3x1 vector)
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  /// @param numTerms the number of terms to approximate with (0 implies analytical solution).
  ///        -- hint: analytical solution (numTerms = 0), is more efficient that numTerms => 3
  ///
  /// @return 6x6 S matrix
  ///
  Eigen::Matrix<double,6,6> se3vecToS(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis, unsigned int numTerms = 0);
  ///
  /// @param se3vec is the 6x1 vector = [translation' aaxis'] = [rho' phi'];
  ///
  Eigen::Matrix<double,6,6> se3vecToS(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms = 0);
  
  /// 
  /// Inverse Ssix_ba (6x6 form of the "S" Matrix)
  /// 
  /// @param trans translation (3x1 vector)
  /// @param aaxis angle-axis rotation (3x1 vector), where the mag. is the angle
  ///
  /// @return inverse of 6x6 S matrix
  ///
  /// todo: if se3vecToSinv is not fast enough, it is an option to add numTerms for Sinv(3) and Q
  Eigen::Matrix<double,6,6> se3vecToSinv(const Eigen::Vector3d & trans, const Eigen::Vector3d & aaxis);
  Eigen::Matrix<double,6,6> se3vecToSinv(const Eigen::Matrix<double,6,1> & se3vec);
  
  /// *************************************************************
  ///
  /// Homogeneous Point Operators
  ///
  /// *************************************************************
  
  ///
  /// Homogeneous Point To 4x6 Matrix Operator
  ///    cross4(w)*p == -pointTo4x6(p)*w
  ///
  /// As of Tuesday, 28 May 2013, this is referred to as "Box Minus"
  ///
  /// @param p homogeneous point
  ///
  /// @return 4x6 matrix
  ///
  Eigen::Matrix<double,4,6> pointTo4x6(double x, double y, double z, double s);
  Eigen::Matrix<double,4,6> pointTo4x6(const Eigen::Vector3d & p, double s = 1.0);
  Eigen::Matrix<double,4,6> pointTo4x6(const Eigen::Vector4d & p);
  

}} // end namespace asrl::math2


#endif
