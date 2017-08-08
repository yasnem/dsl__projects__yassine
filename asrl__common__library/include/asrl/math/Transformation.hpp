#ifndef ASRL_MATH_TRANSFORMATION_HPP
#define ASRL_MATH_TRANSFORMATION_HPP

#include <Eigen/Geometry>
#include <asrl/eigen/EigenSerialization.hpp>

namespace asrl { namespace math {

    ///
    /// @class Transformation
    /// @brief a class that represents an uncertain transformation.
    /// @todo describe how these transformations work
    /// @todo overload operators so we can propagate uncertainty
    /// @todo overload operators so we can transform points.
    /// @todo convenience operators that provide translation and rotation components.
    ///
    class Transformation
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW


      ///
      /// A 3D Affine transformation (rigid-body translation and rotation).
      ///
      typedef Eigen::Isometry3d transformation_t;

      ///
      /// A \f$6 \times 6\f$ matrix that stores the uncertainty of an
      /// affine transformation matrix.
      ///
      typedef Eigen::Matrix<double, 6, 6> covariance_t;


      ///
      /// Default constructor. The transformation and uncertainty will
      /// both be set to identity.
      ///
      Transformation();

      ///
      /// Constructor. The transformation will be set to T_ab, and the
      /// uncertainty will be set to zero.
      ///
      /// @param T_ab the initializing transformation.
      ///
      Transformation(transformation_t const & T_ab);

      ///
      /// Constructor. The transformation will be set to T_ab, and the
      /// uncertainty will be set to zero.
      ///
      /// @param T_ab the initializing transformation.
      ///
      Transformation(Eigen::Matrix4d const & T_ab);

      ///
      /// Constructor. The transformation will be set to T_ab, and the
      /// uncertainty will be set to U_ab.
      /// Constructor.
      ///
      /// @param T_ab the initializing transformation
      /// @param U_ab the initializing uncertainty.
      ///
      Transformation(transformation_t const & T_ab,
		     covariance_t const & U_ab);


      ///
      /// Constructor. The transformation will be set to T_ab, and the
      /// uncertainty will be set to U_ab.
      /// Constructor.
      ///
      /// @param T_ab the initializing transformation
      /// @param U_ab the initializing uncertainty.
      ///
      Transformation(Eigen::Matrix4d const & T_ab,
		     covariance_t const & U_ab);


      ///
      /// Constructor. The transformation will be set to T_ab, and the
      /// uncertainty will be set to U_ab.
      /// Constructor.
      ///
      /// @param T_ab the initializing transformation
      /// @param U_ab the standard deviation for a diagonal uncertainty.
      ///
      Transformation(Eigen::Matrix4d const & T_ab,
		     double sigma_ab);


      ///
      /// Constructor. The transformation will be set to T_ab = [C_ab, -C_ab*r_ab_inb; 0 0 0 1], and the
      /// uncertainty will be set to zero.
      ///
      /// @param C_ab the rotation from b to a.
      /// @param r_ab_inb the translation from b to a, expressed in b.
      ///
      Transformation(Eigen::Matrix3d const & C_ab, Eigen::Vector3d const & r_ab_inb); //Added by JDG


      ///
      ///
      /// @return The underlying transformation
      ///
      transformation_t const & T() const;

      ///
      ///
      /// @return The underlying transformation
      ///
      transformation_t & T();

      ///
      ///
      /// @return The underlying covariance associated with the transformation.
      ///
      covariance_t const & U() const;

      ///
      ///
      /// @return The underlying covariance associated with the transformation.
      ///
      covariance_t & U();


      //********************** START HOOKS ADDED BY JON **********************//

      ///
      ///
      /// @return The underlying transformation as a 6x1 vector
      ///
      Eigen::Matrix<double,6,1> se3vector() const; //Be careful, this gives you back T_ab(1:3,4), i.e., the xyz points of a in frame b as per asrl::math::fromTEuler() in transformations.hpp;  -- JDG

      ///
      ///
      /// @return The underlying transformation as a 6x1 vector
      ///
      Eigen::Matrix<double,6,1> se3vector(); //Be careful, this gives you back T_ab(1:3,4), i.e., the xyz points of a in frame b as per asrl::math::fromTEuler() in transformations.hpp;  -- JDG

      ///
      ///
      /// @return The xyz and rpy components. Calls r() and rpy() and concatenates.
      ///
      Eigen::Matrix<double,6,1> xyzrpy() const;  //Added by JDG

      ///
      ///
      /// @return The xyz and rpy components. Calls r() and rpy() and concatenates.
      ///
      Eigen::Matrix<double,6,1> xyzrpy();  //Added by JDG

      ///
      ///
      /// @return The rotation matrix vector to a from b expressed in frame b (i.e., the rotation component C_ab of T_ab)
      ///
      Eigen::Matrix3d C() const;  //Added by JDG

      ///
      ///
      /// @return The rotation matrix vector to a from b expressed in frame b (i.e., the rotation component C_ab of T_ab)
      ///
      Eigen::Matrix3d C();  //Added by JDG

      ///
      ///
      /// @return The rpy rotation angles as per asrl::math::C2rph() in rotations.hpp
      ///
      Eigen::Vector3d rpy() const;  //Added by JDG

      ///
      ///
      /// @return The rpy rotation angles as per asrl::math::C2rph() in rotations.hpp
      ///
      Eigen::Vector3d rpy();  //Added by JDG


      ///
      ///
      /// @return The vector to a from b expressed in frame b (i.e., the translation component of T_ab expressed in b)
      ///
      Eigen::Vector3d r() const;  //Added by JDG

      ///
      ///
      /// @return The vector to a from b expressed in frame b (i.e., the translation component of T_ab expressed in b)
      ///
      Eigen::Vector3d r();  //Added by JDG

      ///
      ///
      /// @return The 2D component of the vector to a from b expressed in frame b (i.e., the 2D translation component of T_ab expressed in b)
      ///
      Eigen::Vector2d xy() const;  //Added by JDG

      ///
      ///
      /// @return The 2D component of the vector to a from b expressed in frame b (i.e., the 2D translation component of T_ab expressed in b)
      ///
      Eigen::Vector2d xy();  //Added by JDG


      //********************** END HOOKS ADDED BY JON **********************//


      /**
       * Invert the transformaiton and the uncertainty of the inverse
       *
       * @return The inverted transformation
       */
      Transformation inverse() const;

      /**
       * Compose this transformation matrix with another and return the
       * composition of the two. If this Transform is \f$\mathbf T_{01} \f$
       * and the other is \f$ \mathbf T_{12}$, this function returns
       * \f$ \mathbf T_{02} = \mathbf T_{01} \mathbf T_{12} \f$ along with
       * the corresponding uncertainty. See the documentation for more details.
       *
       * @param T The input transformatin matrix
       *
       * @return The composed transformation matrix.
       */
      Transformation composeWith( const Transformation & T ) const;


      /**
       *
       * This function implements the inverse of the composition operation.
       *
       * If this transformation is \f$\mathbf T_{02}\f$ produced through the
       * composeWith() function, \f$ \mathbf T_{02} = \mathbf T_{01} \mathbf T_{12} \f$,
       * this function will undo that operation on the transformation and the uncertainty
       * such that
       *
       * \f$ \mathbf T_{01} = \mathbf T_{02}.\text{inverseComposeWithLeft}( \mathbf T_{12} ) \f$,
       *
       * @param T The input transformatin matrix
       *
       * @return The inverse composed transformation matrix.
       */
      Transformation inverseComposeWithLeft( const Transformation & T ) const;

      /**
       *
       * This function implements the inverse of the composition operation.
       *
       * If this transformation is \f$\mathbf T_{01}\f$, formerly used to produce the
       * \f$\mathbf T_{02}\f$ using composeWith() function, \f$ \mathbf T_{02} = \mathbf T_{01} \mathbf T_{12} \f$,
       * this function will undo that operation on the transformation and the uncertainty
       * such that
       *
       * \f$ \mathbf T_{12} = \mathbf T_{01}.\text{inverseComposeWithRight}( \mathbf T_{02} ) \f$,
       *
       * @param T The input transformatin matrix
       *
       * @return The inverse composed transformation matrix.
       */
      Transformation inverseComposeWithRight( const Transformation & T ) const;



      void checkTransformationIsValid( void ) const;


      ///
      /// Serialize the Transformation to a boost::serialization archive.
      ///
      /// @param ar The archive
      /// @param version The archive file version number.
      ///
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version);

    private:

      /// The underlying transformation
      transformation_t T_ab_;

      /// The uncertainty of the transformation
      /// @todo Describe exactly what this matrix means
      /// @todo Show exactly how this relates to the ROS definition of uncertainty
      covariance_t U_ab_;
    };


    template<class Archive>
    void Transformation::serialize(Archive & ar, const unsigned int version)
    {
      ar & T_ab_;
      ar & U_ab_;
    }

  }} // namespace asrl::math

std::ostream & operator<<(std::ostream & out, const asrl::math::Transformation & T);
asrl::math::Transformation operator*(const asrl::math::Transformation &T1, const asrl::math::Transformation &T2);

#endif // ASRL_MATH_TRANSFORMATION_HPP
