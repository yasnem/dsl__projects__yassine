#include <asrl/math2/Transformation.hpp>
#include <asrl/math2/SpecialEuclidean3.hpp>
#include <boost/make_shared.hpp>

namespace asrl { 
namespace math2 {

  /// *************************************************************
  ///
  /// Constructors.
  ///
  /// *************************************************************

  Transformation::Transformation() :
    C_ba_(Eigen::Matrix3d::Identity()),
    r_ab_inb_(Eigen::Vector3d::Zero())
  {

  }


  Transformation::Transformation(const Transformation & T) :
    C_ba_(T.C_ba_),
    r_ab_inb_(T.r_ab_inb_)
  {

  }


  Transformation::Transformation(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ba_ina) :
    C_ba_(C_ba),
    r_ab_inb_(-C_ba*r_ba_ina)
  {
    
  }

  Transformation::Transformation(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ab_inb, bool stateofthisdoesnothing) :
    C_ba_(C_ba),
    r_ab_inb_(r_ab_inb)
  {
    
  }

  Transformation::Transformation(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms)
  {
    asrl::math2::ExpMap4(se3vec, &C_ba_, &r_ab_inb_, numTerms);
  }

  Transformation::Transformation(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix<double,6,6> * S, unsigned int numTerms)
  {
    asrl::math2::ExpMap4andS(se3vec, &C_ba_, &r_ab_inb_, S, numTerms);
  }

  Transformation & Transformation::operator=(Transformation T)
  {
    // Swap (this)'s parameters with the temporary object passed by value
    // The temporary object is then destroyed at end of scope
    std::swap( this->C_ba_, T.C_ba_ );
    std::swap( this->r_ab_inb_, T.r_ab_inb_ );
    return (*this);
  }

  /// *************************************************************
  ///
  /// "Get" Functions
  ///
  /// *************************************************************


  Eigen::Matrix4d Transformation::matrix() const
  {
      Eigen::Matrix4d T_ba = Eigen::Matrix4d::Identity();
      T_ba.topLeftCorner<3,3>() = C_ba_;
      T_ba.topRightCorner<3,1>() = r_ab_inb_;
      return T_ba;
  }


  const Eigen::Matrix3d & Transformation::C() const
  {
    return C_ba_;
  }

  Eigen::Vector3d Transformation::r_ba_ina() const
  {
    return -C_ba_.transpose()*r_ab_inb_;
  }

  const Eigen::Vector3d & Transformation::r_ab_inb() const
  {
    return r_ab_inb_;
  }
  
  Eigen::Matrix<double,6,1> Transformation::se3vec() const
  {
     return asrl::math2::se3vecFromExpMap4(C_ba_, r_ab_inb_);
  }

  /// *************************************************************
  ///
  /// Transform Operations
  ///
  /// *************************************************************


  Transformation Transformation::inverse() const
  {
    // Eigen's x.transpose() function only returns a pseudo object, 
    // this is faster than allocating a new Eigen::Matrix3d
    return Transformation(C_ba_.transpose(), -C_ba_.transpose()*r_ab_inb_, true);
  }

  Eigen::Matrix<double,6,6> Transformation::ExpMap6() const
  {
    Eigen::Matrix<double,6,6> Tsix_ba = Eigen::Matrix<double,6,6>::Zero();
    Tsix_ba.topLeftCorner<3,3>() = Tsix_ba.bottomRightCorner<3,3>() = C_ba_;
    Tsix_ba.topRightCorner<3,3>() = -asrl::math2::cross(r_ab_inb_) * C_ba_;
    return Tsix_ba;
  }

  Eigen::Matrix<double,6,6> Transformation::invExpMap6() const
  {
    Eigen::Matrix<double,6,6> Tsix_ab = Eigen::Matrix<double,6,6>::Zero();
    Tsix_ab.topLeftCorner<3,3>() = Tsix_ab.bottomRightCorner<3,3>() = C_ba_.transpose();
    Tsix_ab.topRightCorner<3,3>() = C_ba_.transpose()*asrl::math2::cross(r_ab_inb_);
    return Tsix_ab;
  }

  /// *************************************************************
  ///
  /// Operators
  ///
  /// *************************************************************
  
  
  Transformation & Transformation::operator*=(const Transformation & T_rhs) 
  {
    r_ab_inb_ = C_ba_*T_rhs.r_ab_inb_ + r_ab_inb_;
    C_ba_ = C_ba_*T_rhs.C_ba_;
    return *this;
  }


  const Transformation Transformation::operator*(const Transformation & T_rhs) const 
  {
    return Transformation(C_ba_*T_rhs.C_ba_, C_ba_*T_rhs.r_ab_inb_ + r_ab_inb_, true);
  }
  
  const Eigen::Vector4d Transformation::operator*(const Eigen::Vector4d & p_a) const
  {
    Eigen::Vector4d p_b;
    p_b.head<3>() = C_ba_*p_a.head<3>() + r_ab_inb_*p_a[3];
    p_b[3] = p_a[3];
    return p_b;
  }
  
  const Eigen::Vector3d Transformation::operator*(const Eigen::Vector3d & p_a) const // assumes homogeneous scale 1.0
  {
    return Eigen::Vector3d(C_ba_*p_a + r_ab_inb_);
  }
  
  /// *************************************************************
  ///
  /// Special Operators
  ///
  /// *************************************************************

  Transformation Transformation::invmult(const Transformation & T_rhs) const 
  {
    return Transformation(C_ba_.transpose()*T_rhs.C_ba_, C_ba_.transpose()*T_rhs.r_ab_inb_ - C_ba_.transpose()*r_ab_inb_, true);
  }

}} // namespace asrl::math2



/// *************************************************************
///
/// Extras
///
/// *************************************************************

std::ostream & operator<<(std::ostream & out, const asrl::math2::Transformation & T)
{
  out << std::endl << T.matrix() << std::endl;
  return out;
}









