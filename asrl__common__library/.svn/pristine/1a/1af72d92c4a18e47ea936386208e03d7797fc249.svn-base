#ifndef ASRL_MATH_HOMOGENEOUS_COORDINATES_HPP
#define ASRL_MATH_HOMOGENEOUS_COORDINATES_HPP

#include <Eigen/Core>

namespace asrl { namespace math {

    Eigen::Matrix<double,4,3> toHomogeneousJacobian(const Eigen::Vector3d & v);
    Eigen::Vector4d toHomogeneous(const Eigen::Vector3d & v, Eigen::Matrix<double,4,3> * jacobian = NULL);


    Eigen::Matrix<double,3,4> fromHomogeneousJacobian(const Eigen::Vector4d & v);
    Eigen::Vector3d fromHomogeneous(const Eigen::Vector4d & v, Eigen::Matrix<double,3,4> * jacobian = NULL);


    Eigen::MatrixXd toHomogeneousColumns(const Eigen::MatrixXd & M);
    Eigen::MatrixXd fromHomogeneousColumns(const Eigen::MatrixXd & M);

  }} // namespace asrl::math

#endif /* ASRL_MATH_HOMOGENEOUS_COORDINATES_HPP */
