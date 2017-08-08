///
/// @file   TransformationCov.hpp
/// @author Sean Anderson <sean.anderson@mail.utoronto.ca>
/// @date   Monday, 20 May 2013
///
/// @brief  A more "full" transformation class, includes covariance
///         and adds additional functionalities.
///

#ifndef ASRL_MATH2_TRANSFORMATIONCOV_HPP
#define ASRL_MATH2_TRANSFORMATIONCOV_HPP

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

#include <asrl/math2/Transformation.hpp>

namespace asrl { 
namespace math2 {

  class TransformationCov
  {
    public:
      ///
      /// A note on EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// Sean Anderson, as of May 23, 2013
      ///
      /// Classes that include *fixed-size vectorizable Eigen types*, see
      /// http://eigen.tuxfamily.org/dox-devel/group__TopicFixedSizeVectorizable.html,
      /// must include this macro! Furthermore, special considerations must be taken if 
      /// you want to use them in STL containers, such as std::vector or std::map.
      ///
      /// The macro overload the dynamic "new" operator so that it generates 
      /// 16-byte-aligned pointers, this MUST be in the public section of the header!
      /// 
      /// Eigen::Matrix3d and Eigen::Vector3d are NOT 16-byte vectorizable,
      /// However, Eigen::Matrix<double,6,6> IS and therefore special consideration must
      /// be taken.
      ///
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      /// An STL TransformationCov vector must be allocated using an aligned allocator as described in the Eigen documentation:
      /// http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html
      typedef std::vector<TransformationCov, Eigen::aligned_allocator< TransformationCov > > vector_t;
      
      /// *************************************************************
      ///
      /// Constructors.
      ///
      /// *************************************************************
      
      ///
      /// Default constructor. The transformation will be set to identity.
      ///
      TransformationCov();
      
      ///
      /// Copy constructor.
      ///
      TransformationCov(const TransformationCov & T);

      /// 
      /// Constructor.
      ///
      /// @param T_ba the transformation from a to b.
      /// @param U_ba the uncertainty/covariance matrix over the pose change from a to b.
      ///      
      TransformationCov(const Transformation & T_ba, const Eigen::Matrix<double,6,6> & U_ba);
      
      /// 
      /// Constructor. The transformation will be set to T_ba = [C_ba, -C_ba*r_ba_ina; 0 0 0 1], with the uncertainty U_ba
      ///
      /// @param C_ba the rotation from a to b.
      /// @param r_ba_ina the translation from a to b, expressed in a.
      /// @param U_ba the uncertainty/covariance matrix over the pose from a to b.
      ///
      TransformationCov(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ba_ina, const Eigen::Matrix<double,6,6> & U_ba);
      
      /// 
      /// Constructor. The transformation will be set to T_ba = [C_ba, r_ab_inb; 0 0 0 1], with the uncertainty U_ba
      ///
      /// @param C_ba the rotation from a to b.
      /// @param r_ab_inb the translation from b to a, expressed in b.
      /// @param U_ba the uncertainty/covariance matrix over the pose from a to b.
      /// @param stateofthisdoesnothing this variable is only used to disambiguate the constructors for r_ba_ina and r_ab_inb, its actual value does nothing
      ///        -- It is unlikely this constructor will see use outside of the class. Therefore the more "default" C/r constructor is left for r_ab_inb.
      ///           Furthermore, it is intentional that we do not simply use the boolean with a default to determine the 'r' type. The "if" statement causes overhead.
      ///
      TransformationCov(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ab_inb, const Eigen::Matrix<double,6,6> & U_ba, bool stateofthisdoesnothing);
      
      /// Constructor. The transformation will be set to T_ba = expMap4(se3vec)
      ///
      /// @param se3vec the 6x1 vector, often referred to as pi in the literature. se3vec = [trans' phi']';
      /// @param U_ba the uncertainty/covariance matrix over the pose from a to b.
      /// @param numTerms the number of terms to approximate T_ba with (0 implies analytical solution).
      ///
      TransformationCov(const Eigen::Matrix<double,6,1> & se3vec, const Eigen::Matrix<double,6,6> & U_ba, unsigned int numTerms = 0);
      
      /// 
      /// Constructor. The transformation will be set to T_ba = expMap4(se3vec)
      ///
      /// @param se3vec the 6x1 vector, often referred to as pi in the literature. se3vec = [trans' phi']';
      /// @param return the 6x6 S matrix associated with se3vec by reference.
      /// @param U_ba the uncertainty/covariance matrix over the pose from a to b.
      /// @param numTerms the number of terms to approximate T_ba with (0 implies analytical solution).
      ///
      TransformationCov(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix<double,6,6> * S, const Eigen::Matrix<double,6,6> & U_ba, unsigned int numTerms = 0);
      
      ///
      /// Default destructor.
      ///
      ~TransformationCov(){};
      
      ///
      /// Assignment operator. Pass by value to use copy constructor, 
      /// and then swap our variables with the temporary object. 
      /// This is a new-ish standard approach to assignment.
      ///
      TransformationCov & operator=(TransformationCov T);
      
      /// *************************************************************
      ///
      /// "Get" Functions
      ///
      /// *************************************************************
      
      /// 
      /// Gets the Eigen representation of the matrix
      /// 
      /// @return Eigen::Matrix4d of the matrix
      /// 
      const Transformation & T() const;

      /// 
      /// Gets the underlying C_ba matrice.
      /// 
      /// @return Eigen::Matrix3d C_ba
      ///
      const Eigen::Matrix3d & C() const;
      
      /// 
      /// Gets the "forward" translation r_ba_ina = -C_ba.transpose()*r_ab_inb
      /// 
      /// @return Eigen::Vector3d r_ba_ina
      ///
      Eigen::Vector3d r_ba_ina() const;
      
      /// 
      /// Gets the underlying r_ab_inb vector.
      /// 
      /// @return Eigen::Vector3d r_ab_inb
      ///
      const Eigen::Vector3d & r_ab_inb() const;

      /// 
      /// Returns the 6x6 uncertainty matrix
      /// 
      /// @return The Eigen::Matrix<double,6,6> uncertainty matrix, if none was allocated, passes a Zero() matrix.
      ///
      const Eigen::Matrix<double,6,6> & U() const;
      

      /// *************************************************************
      ///
      /// Transform Operations
      ///
      /// *************************************************************

      /// 
      /// Creates the inverse matrix. 
      /// 
      /// @return TransformationCov inverse matrix
      /// 
      TransformationCov inverse() const;
  
      /// 
      /// Creates the 6x6 algebraic equivalent transformation
      /// -- a.k.a. BoxDot() as of Tuesday, 28 May 2013
      /// 
      /// @return 6x6 algebraic equivalent transformation
      /// 
      Eigen::Matrix<double,6,6> ExpMap6() const;
      Eigen::Matrix<double,6,6> invExpMap6() const;
      
      /// *************************************************************
      ///
      /// Operators
      ///
      /// *************************************************************
      
      TransformationCov & operator*=(const TransformationCov & T_rhs);
      const TransformationCov operator*(const TransformationCov & T_rhs) const;
      
      const Eigen::Vector4d operator*(const Eigen::Vector4d & p_a) const;
      const Eigen::Vector3d operator*(const Eigen::Vector3d & p_a) const; // assumes homogeneous scale 1.0
      
      /// *************************************************************
      ///
      /// Special Operators
      ///
      /// *************************************************************
  
      ///
      /// Multiply the inverse of this matrix with the input matrix
      /// More efficient than writing T_out = T_this.inverse()*T_in
      /// 
      /// @param Right hand side transformation matrix
      /// 
      /// @return The composed transformation matrix
      /// 
      TransformationCov invmult(const TransformationCov & T_rhs) const;
      
      /// ***************************************************************************
      /// "Private" variables. The underlying representation is kept public
      /// for speed advantages in multiplication of transforms.
      /// ***************************************************************************
    
    private:
    
      /// The underlying transformation
      Transformation T_ba_;
      
      /// The underlying 6x6 uncertainty matrix.
      Eigen::Matrix<double,6,6> U_ba_;
      
  };
      
}} // namespace asrl::math2
      
/// *************************************************************
///
/// Extras
///
/// *************************************************************
      
std::ostream & operator<<(std::ostream & out, const asrl::math2::TransformationCov & T);



#endif

