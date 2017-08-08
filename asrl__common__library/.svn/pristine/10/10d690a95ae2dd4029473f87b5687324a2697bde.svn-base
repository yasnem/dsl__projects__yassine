#include <asrl/math/Transformation.hpp>
#include <asrl/math/transformations.hpp>
#include <asrl/math/rotations.hpp>

namespace asrl { namespace math {


    Transformation::Transformation() :
      T_ab_(transformation_t::Identity()),
      U_ab_(covariance_t::Identity())
    {

    }


    Transformation::Transformation(transformation_t const & T_ab) :
      T_ab_(T_ab),
      U_ab_(covariance_t::Identity())
    {

    }


    Transformation::Transformation(transformation_t const & T_ab,
				   covariance_t const & U_ab) :
      T_ab_(T_ab),
      U_ab_(U_ab)
    {

    }


    Transformation::Transformation(Eigen::Matrix4d const & T_ab) :
      U_ab_(covariance_t::Identity())
    {
      // \todo validate that this is a valid transformation matrix
      T_ab_.matrix() = T_ab;
    }



    Transformation::Transformation(Eigen::Matrix4d const & T_ab,
				   covariance_t const & U_ab) :
      U_ab_(U_ab)
    {
      // \todo validate that this is a valid transformation matrix
      T_ab_.matrix() = T_ab;
    }


    Transformation::Transformation(Eigen::Matrix4d const & T_ab,
		     double sigma_ab)
      {
        T_ab_.matrix() = T_ab;
        U_ab_ = covariance_t::Identity() * sigma_ab * sigma_ab;
      }

    Transformation::Transformation(Eigen::Matrix3d const & C_ab, Eigen::Vector3d const & r_ab_inb) //Added by JDG
      : U_ab_(covariance_t::Identity())
    {
      //Makesure the matrix is Identity
      T_ab_ = transformation_t::Identity();
      //Then, put in the C matrix into the upper left 3x3
      T_ab_.matrix().topLeftCorner<3,3>() = C_ab;
      //Then the r component into the top right 3x1
      //r_ij_inj = -C_ji * r_ji_ini
      T_ab_.matrix().topRightCorner<3,1>() = -C_ab*r_ab_inb;
    }


    Transformation::transformation_t const & Transformation::T() const
    {
      return T_ab_;
    }


    Transformation::covariance_t const & Transformation::U() const
    {
      return U_ab_;
    }

    Transformation::transformation_t  & Transformation::T()
    {
      return T_ab_;
    }


    Transformation::covariance_t  & Transformation::U()
    {
      return U_ab_;
    }


    Eigen::Matrix<double,6,1>  Transformation::se3vector()
    {
      return asrl::math::fromTEuler(T_ab_.matrix());
    }

    Eigen::Matrix<double,6,1>  Transformation::se3vector() const
    {
      return asrl::math::fromTEuler(T_ab_.matrix());
    }


    Eigen::Matrix<double,6,1>  Transformation::xyzrpy()
    {
      //return value
      Eigen::Matrix<double, 6, 1> rval;

      //Get the xyz and place it in the had
      rval.head<3>() = this->r();
      //Get the rpy and place it in the tail
      rval.tail<3>() = this->rpy();

      //Return
      return rval;
    }

    Eigen::Matrix<double,6,1>  Transformation::xyzrpy() const
    {
      //return value
      Eigen::Matrix<double, 6, 1> rval;

      //Get the xyz and place it in the had
      rval.head<3>() = this->r();
      //Get the rpy and place it in the tail
      rval.tail<3>() = this->rpy();

      //Return
      return rval;
    }


    Eigen::Matrix3d Transformation::C() const
    {
      return T_ab_.matrix().topLeftCorner<3,3>();
    }

    Eigen::Matrix3d Transformation::C()
    {
      return T_ab_.matrix().topLeftCorner<3,3>();
    }


    Eigen::Vector3d Transformation::rpy() const
    {
      return C2rph(this->C());
    }

    Eigen::Vector3d Transformation::rpy()
    {
      return C2rph(this->C());
    }



    Eigen::Vector3d Transformation::r() const
    {
      return -C().transpose()*T_ab_.matrix().topRightCorner<3,1>();
    }

    Eigen::Vector3d Transformation::r()
    {
      return -C().transpose()*T_ab_.matrix().topRightCorner<3,1>();
    }


    Eigen::Vector2d Transformation::xy() const
    {
      return r().head<2>();
    }

    Eigen::Vector2d Transformation::xy()
    {
      return r().head<2>();
    }










    Transformation Transformation::inverse() const
    {
      // Invert the transformation.
      transformation_t T_ba = T_ab_.inverse();

      // Invert the uncertainty.
      covariance_t T_ba_boxtimes = boxTimes(T_ba.matrix());
      covariance_t U_ba = T_ba_boxtimes * U_ab_ * T_ba_boxtimes.transpose();

      return Transformation(T_ba,U_ba);
    }

    Transformation Transformation::composeWith( const Transformation & T_bc ) const
    {
      transformation_t T_ac = T_ab_ * T_bc.T();
      covariance_t T_ab_boxtimes = boxTimes(T_ab_.matrix());

      covariance_t U_ac = U_ab_ + T_ab_boxtimes * T_bc.U() * T_ab_boxtimes.transpose();

      return Transformation(T_ac, U_ac);
    }

    Transformation Transformation::inverseComposeWithLeft( const Transformation & T_bc ) const
    {
      // Aliases for readability.
      const transformation_t & T_ac = T_ab_;
      const covariance_t & U_ac = U_ab_;

      // Within this function, this object is {T_ac, U_ac} and we want to undo the composition
      // to recover {T_ab, U_ab}.

      transformation_t T_ab = T_ac * T_bc.T().inverse();
      covariance_t T_ab_boxtimes = boxTimes(T_ab.matrix());

      covariance_t U_ab = U_ac - T_ab_boxtimes * T_bc.U() * T_ab_boxtimes.transpose();

      return Transformation(T_ab, U_ab);
    }

    Transformation Transformation::inverseComposeWithRight( const Transformation & T_ac ) const
    {

      //transformation_t T_ac = T_ab_ * T_bc.T();
      //covariance_t T_ab_boxtimes = boxTimes(T_ab_.matrix());
      //covariance_t U_ac = U_ab_ + T_ab_boxtimes * T_bc.U() * T_ab_boxtimes.transpose();

      transformation_t T_ba = T_ab_.inverse();
      transformation_t T_bc =  T_ba * T_ac.T();
      covariance_t T_ba_boxtimes = boxTimes(T_ba.matrix());

      covariance_t U_bc =   T_ba_boxtimes * (T_ac.U() -  U_ab_) * T_ba_boxtimes.transpose() ;

      return Transformation(T_bc, U_bc);
    }




    void Transformation::checkTransformationIsValid( void ) const
    {
      ASRL_ASSERT_EQ(std::runtime_error, T_ab_.matrix().block(0,3,1,1), T_ab_.matrix().block(0,3,1,1),
		     "Transformation not valid, x!=x, is x-translation NAN?");
      ASRL_ASSERT_EQ(std::runtime_error, T_ab_.matrix().block(1,3,1,1), T_ab_.matrix().block(1,3,1,1),
		     "Transformation not valid, y!=y, is y-translation NAN?");
      ASRL_ASSERT_EQ(std::runtime_error, T_ab_.matrix().block(2,3,1,1), T_ab_.matrix().block(2,3,1,1),
		     "Transformation not valid, z!=z, is z-translation NAN?");
      ASRL_ASSERT_EQ(std::runtime_error, U_ab_.matrix().block(0,3,1,1), U_ab_.matrix().block(0,3,1,1),
		     "Transformation not valid, covz!=covz, is covz NAN?");
      ASRL_ASSERT_EQ(std::runtime_error, U_ab_.matrix().block(1,3,1,1), U_ab_.matrix().block(1,3,1,1),
		     "Transformation not valid, covy!=covy, is covy NAN?");
      ASRL_ASSERT_EQ(std::runtime_error, U_ab_.matrix().block(2,3,1,1), U_ab_.matrix().block(2,3,1,1),
		     "Transformation not valid, covz!=covz, is covz NAN?");
      return;
    }


  }} // namespace asrl::math

std::ostream & operator<<(std::ostream & out, const asrl::math::Transformation & T)
{
  out << std::endl << T.T().matrix() << std::endl << T.U() << std::endl;
  return out;
}

asrl::math::Transformation operator*(const asrl::math::Transformation &T1, const asrl::math::Transformation &T2)
{
  return T1.composeWith(T2);
}
