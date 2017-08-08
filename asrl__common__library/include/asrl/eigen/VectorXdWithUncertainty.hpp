#ifndef ASRL_VECTORXD_WITH_UNCERTAINTY_HPP
#define ASRL_VECTORXD_WITH_UNCERTAINTY_HPP

#include<Eigen/Core>

namespace asrl { namespace eigen {

    ///
    /// \class VectorWithUncertainty
    /// \brief a X-Dimensional vector with uncertainty.
    /// \tparam the dimension of the vector
    ///
    template<int DIMENSION>
    struct VectorXdWithUncertainty {
      
      /// Default constructor
      VectorXdWithUncertainty(){}

      /// 
      /// Templated constructor that is able to deal with Eigen3 temporary types.
      /// For more details, see http://eigen.tuxfamily.org/dox-devel/TopicFunctionTakingEigenTypes.html
      ///
      /// @tparam DerivedV The EigenBase template parameter associated with the vector in
      /// @tparam DerivedU The EigenBase template parameter associated with the uncertainty in
      ///
      /// @param vector_in The initializing vector type
      /// @param uncertainty_in The initializing uncertainty type
      ///
      template <typename DERIVEDV, typename DERIVEDU>
      VectorXdWithUncertainty(const Eigen::EigenBase<DERIVEDV>& vector_in, const Eigen::EigenBase<DERIVEDU> & uncertainty_in)
      {
	vector = vector_in;
	uncertainty = uncertainty_in;
      }

      /// Specialize the class so that Eigen doesn't have problems with
      /// SSE instruction sets.
      /// http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      ///
      /// \var typedef Eigen::Matrix<double,DIMENSION,1> vector_t;
      ///
      /// A convenience typedef of the underlying vector type
      ///
      typedef Eigen::Matrix<double,DIMENSION,1> vector_t;

      ///
      /// \var typedef Eigen::Matrix<double,DIMENSION,DIMENSION> uncertainty_t;
      ///
      /// A convenience typedef of the underlying uncertainty type
      ///
      typedef Eigen::Matrix<double,DIMENSION,DIMENSION> uncertainty_t;
      enum {
	Dimension = DIMENSION /*!< The dimension of the underlying vector */
      };

      vector_t vector;
      uncertainty_t uncertainty;
    };


    ///
    /// \var typedef VectorXdWithUncertainty<3> Vector3dWithUncertainty
    ///
    /// A convenience typeder for 3d vector types
    ///
    typedef VectorXdWithUncertainty<3> Vector3dWithUncertainty;
    
    ///
    /// \var typedef VectorXdWithUncertainty<3> Vector3dWithUncertainty
    ///
    /// A convenience typeder for 6d vector types
    ///
    typedef VectorXdWithUncertainty<4> Vector4dWithUncertainty;
    
    ///
    /// \var typedef VectorXdWithUncertainty<3> Vector3dWithUncertainty
    ///
    /// A convenience typeder for 6d vector types
    ///
    typedef VectorXdWithUncertainty<6> Vector6dWithUncertainty;

  }}


#endif /* ASRL_VECTORXD_WITH_UNCERTAINTY_HPP */
