///
/// @file   Transformation.hpp
/// @author Sean Anderson <sean.anderson@mail.utoronto.ca>
/// @date   Monday, 20 May 2013
///
/// @brief  Lightweight transformation class, intended to be fast, and 
///         not to provide unnecessary functionality. Don't slow this down!
///

#ifndef ASRL_MATH2_TRANSFORMATION_HPP
#define ASRL_MATH2_TRANSFORMATION_HPP

#include <Eigen/Core>

namespace asrl { 
namespace math2 {

  class Transformation
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
      /// Fortunately, Eigen::Matrix3d and Eigen::Vector3d are NOT 16-byte vectorizable,
      /// therefore this class should not require alignment, and can be used normally in STL.
      ///
      
      /// *************************************************************
      ///
      /// Constructors.
      ///
      /// *************************************************************
      
      ///
      /// Default constructor. The transformation will be set to identity.
      ///
      Transformation();
      
      ///
      /// Copy constructor.
      ///
      Transformation(const Transformation & T);

      /// 
      /// Constructor. The transformation will be set to T_ba = [C_ba, -C_ba*r_ba_ina; 0 0 0 1]
      ///
      /// @param C_ba the rotation from a to b.
      /// @param r_ba_ina the translation from a to b, expressed in a.
      ///
      Transformation(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ba_ina);
      
      /// 
      /// Constructor. The transformation will be set to T_ba = [C_ba, r_ab_inb; 0 0 0 1]
      ///
      /// @param C_ba the rotation from a to b.
      /// @param r_ab_inb the translation from b to a, expressed in b. (the backward translation)
      /// @param stateofthisdoesnothing this variable is only used to disambiguate the constructors for r_ba_ina and r_ab_inb, its actual value does nothing
      ///        -- It is unlikely this constructor will see use outside of the class. Therefore the more "default" C/r constructor is left for r_ab_inb.
      ///           Furthermore, it is intentional that we do not simply use the boolean with a default to determine the 'r' type. The "if" statement causes overhead.
      ///
      Transformation(const Eigen::Matrix3d & C_ba, const Eigen::Vector3d & r_ab_inb, bool stateofthisdoesnothing);
      
      /// Constructor. The transformation will be set to T_ba = expMap4(se3vec)
      ///
      /// @param se3vec the 6x1 vector, often referred to as pi in the literature. se3vec = [trans' phi']';
      /// @param numTerms the number of terms to approximate T_ba with (0 implies analytical solution).
      ///
      Transformation(const Eigen::Matrix<double,6,1> & se3vec, unsigned int numTerms = 0);
      
      /// 
      /// Constructor. The transformation will be set to T_ba = expMap4(se3vec)
      ///
      /// @param se3vec the 6x1 vector, often referred to as pi in the literature. se3vec = [trans' phi']';
      /// @param fills in the 6x6 S matrix associated with se3vec.
      /// @param numTerms the number of terms to approximate T_ba with (0 implies analytical solution).
      ///
      Transformation(const Eigen::Matrix<double,6,1> & se3vec, Eigen::Matrix<double,6,6> * S, unsigned int numTerms = 0);
      
      ///
      /// Default destructor.
      ///
      ~Transformation(){};
      
      ///
      /// Assignment operator. Pass by value to use copy constructor, 
      /// and then swap our variables with the temporary object. 
      /// This is a new-ish standard approach to assignment.
      ///
      Transformation & operator=(Transformation T);
      
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
      Eigen::Matrix4d matrix() const;
      
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
      /// inversion of the exponential map
      /// 
      /// @return Eigen::Matrix<double,6,1> the 6x1 se3 vector, input to the exponential map
      ///
      Eigen::Matrix<double,6,1> se3vec() const;


      /// *************************************************************
      ///
      /// Transform Operations
      ///
      /// ************************************************************* 

      /// 
      /// Creates the inverse matrix. 
      /// 
      /// @return Transformation inverse matrix
      /// 
      Transformation inverse() const;
  
      
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
      
      Transformation & operator*=(const Transformation & T_rhs);
      const Transformation operator*(const Transformation & T_rhs) const;
      
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
      Transformation invmult(const Transformation & T_rhs) const;
      
      /// ***************************************************************************
      /// "Private" variables. The underlying representation is kept public
      /// for speed advantages in multiplication of transforms.
      /// ***************************************************************************
    
    private:
      
      /// The underlying rotation
      Eigen::Matrix3d C_ba_;
      
      /// The underlying translation vector
      Eigen::Vector3d r_ab_inb_;
      
  };
      
}} // namespace asrl::math2
      
/// *************************************************************
///
/// Extras
///
/// *************************************************************
      
std::ostream & operator<<(std::ostream & out, const asrl::math2::Transformation & T);



#endif

