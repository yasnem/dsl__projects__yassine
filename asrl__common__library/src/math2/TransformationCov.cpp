#include <asrl/math2/TransformationCov.hpp>
#include <asrl/math2/SpecialEuclidean3.hpp>
#include <boost/make_shared.hpp>

namespace asrl { 
namespace math2 {

  /// *************************************************************
  ///
  /// Constructors.
  ///
  /// *************************************************************

  TransformationCov::TransformationCov() :
    T_ba_( Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero() ),
    U_ba_( Eigen::Matrix<double,6,6>::Zero() )
  {

  }


  TransformationCov::TransformationCov(const TransformationCov & T) :
    T_ba_(T.T_ba_),
    U_ba_(T.U_ba_)
  {

  }
  
  TransformationCov::TransformationCov(const Transformation & T_ba, const Eigen::Matrix<double,6,6> & U_ba) :
    T_ba_(T_ba),
    U_ba_(U_ba)
  {
  
  }

  TransformationCov::TransformationCov(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ba_ina, const Eigen::Matrix<double,6,6> & U_ba) :
    T_ba_(C_ba, r_ba_ina),
    U_ba_(U_ba)
  {
  
  }

  TransformationCov::TransformationCov(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ab_inb, const Eigen::Matrix<double,6,6> & U_ba, bool stateofthisdoesnothing) :
    T_ba_(C_ba, r_ab_inb, stateofthisdoesnothing),
    U_ba_(U_ba)
  {
  
  }

  TransformationCov::TransformationCov(const Eigen::Matrix<double,6,1> & se3vec, const Eigen::Matrix<double,6,6> & U_ba, unsigned int numTerms) :
    T_ba_(se3vec, numTerms),
    U_ba_(U_ba)
  {

  }

  TransformationCov::TransformationCov(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix<double,6,6> * S, const Eigen::Matrix<double,6,6> & U_ba, unsigned int numTerms) :
    T_ba_(se3vec, S, numTerms),
    U_ba_(U_ba)
  {
    
  }

  TransformationCov & TransformationCov::operator=(TransformationCov T)
  {
    // Swap (this)'s parameters with the temporary object passed by value
    // The temporary object is then destroyed at end of scope
    std::swap( this->T_ba_, T.T_ba_ );
    std::swap( this->U_ba_, T.U_ba_ );
    return (*this);
  }

  /// *************************************************************
  ///
  /// "Get" Functions
  ///
  /// *************************************************************

  const Transformation & TransformationCov::T() const
  {
      return T_ba_;
  }
  
  const Eigen::Matrix3d & TransformationCov::C() const
  {
    return T_ba_.C();
  }

  Eigen::Vector3d TransformationCov::r_ba_ina() const
  {
    return T_ba_.r_ba_ina();
  }

  const Eigen::Vector3d & TransformationCov::r_ab_inb() const
  {
    return T_ba_.r_ab_inb();
  }
  
  const Eigen::Matrix<double,6,6> & TransformationCov::U() const
  {
    return U_ba_;
  }

  /// *************************************************************
  ///
  /// Transform Operations
  ///
  /// *************************************************************

  TransformationCov TransformationCov::inverse() const
  {
    Eigen::Matrix<double,6,6> Tsix_ab = this->invExpMap6();
    return TransformationCov(T_ba_.inverse(), Tsix_ab*U_ba_*Tsix_ab.transpose());
  }

  Eigen::Matrix<double,6,6> TransformationCov::ExpMap6() const
  {
    return T_ba_.ExpMap6();
  }

  Eigen::Matrix<double,6,6> TransformationCov::invExpMap6() const
  {
    return T_ba_.invExpMap6();
  }

  /// *************************************************************
  ///
  /// Operators
  ///
  /// *************************************************************
  
  
  TransformationCov & TransformationCov::operator*=(const TransformationCov & T_rhs) 
  {
    Eigen::Matrix<double,6,6> Tsix_ba = this->ExpMap6();
    U_ba_ += Tsix_ba*T_rhs.U_ba_*Tsix_ba.transpose();
    T_ba_*= T_rhs.T_ba_;
    return *this;
  }

  const TransformationCov TransformationCov::operator*(const TransformationCov & T_rhs) const 
  {
    Eigen::Matrix<double,6,6> Tsix_ba = this->ExpMap6();
    return TransformationCov(T_ba_*T_rhs.T_ba_, U_ba_ + Tsix_ba*T_rhs.U_ba_*Tsix_ba.transpose());
  }
  
  const Eigen::Vector4d TransformationCov::operator*(const Eigen::Vector4d & p_a) const
  {
    return T_ba_*p_a;
  }
  
  const Eigen::Vector3d TransformationCov::operator*(const Eigen::Vector3d & p_a) const // assumes homogeneous scale 1.0
  {
    return T_ba_*p_a;
  }
  
  /// *************************************************************
  ///
  /// Special Operators
  ///
  /// *************************************************************

  TransformationCov TransformationCov::invmult(const TransformationCov & T_rhs) const 
  {
    Eigen::Matrix<double,6,6> Tsix_ab = this->invExpMap6();
    return TransformationCov(T_ba_.invmult(T_rhs.T_ba_), Tsix_ab*U_ba_*Tsix_ab.transpose() + Tsix_ab*T_rhs.U_ba_*Tsix_ab.transpose());
  }

}} // namespace asrl::math2



/// *************************************************************
///
/// Extras
///
/// *************************************************************

std::ostream & operator<<(std::ostream & out, const asrl::math2::TransformationCov & T)
{
  out << std::endl << T.T() << std::endl;
  out << std::endl << T.U() << std::endl;
  return out;
}









