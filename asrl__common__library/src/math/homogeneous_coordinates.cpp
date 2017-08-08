#include <asrl/math/homogeneous_coordinates.hpp>
#include <asrl/assert_macros.hpp>

namespace asrl { namespace math {

    Eigen::Matrix<double,4,3> toHomogeneousJacobian(const Eigen::Vector3d &)
    {
      return Eigen::Matrix<double,4,3>::Identity();
    }

    Eigen::Vector4d toHomogeneous(const Eigen::Vector3d & v, Eigen::Matrix<double,4,3> * jacobian)
    {
      if(jacobian)
	{
	  *jacobian = toHomogeneousJacobian(v);
	}
      return Eigen::Vector4d(v[0],v[1],v[2],1.0);
    }

    Eigen::Matrix<double,3,4> fromHomogeneousJacobian(const Eigen::Vector4d & v)
    {
      Eigen::Matrix<double,3,4> J;
      
      fromHomogeneous(v,&J);
      return J;
      
    }

    Eigen::Vector3d fromHomogeneous(const Eigen::Vector4d & v, Eigen::Matrix<double,3,4> * jacobian)
    {
      //ASRL_ASSERT_GT_DBG(std::runtime_error,fabs(v[3]),0.0,"This point is too distant to be represented in Euclidean space. p: " << v.transpose());

      //      ASRL_ASSERT_GT_DBG(std::runtime_error,fabs(v[3]),0.0000001,"This point is too distant to be represented in Euclidean space. p: " << v);
      //ASRL_ASSERT_GT_DBG(std::runtime_error,v[3],0.0,"This point is too distant to be represented in Euclidean space. p: " << v);

      double rv3 = 1.0/v[3];
      
      Eigen::Vector3d v3;
      v3 = v.head<3>() * rv3;

      if(jacobian)
	{
	  jacobian->topRightCorner<3,1>() = -v3 * rv3;
	  jacobian->topLeftCorner<3,3>() = Eigen::Matrix3d::Identity() * rv3;
	}
    
      return v3;
    }


    Eigen::MatrixXd toHomogeneousColumns(const Eigen::MatrixXd & M)
    {
      Eigen::MatrixXd hM(Eigen::MatrixXd::Ones(M.rows() + 1, M.cols()));
      hM.topRows(M.rows()) = M;
      return hM;
    }
    
    Eigen::MatrixXd fromHomogeneousColumns(const Eigen::MatrixXd & hM)
    {
      ASRL_ASSERT_GE_DBG(std::runtime_error,hM.rows(),2,"A homogeneous matrix must have at least 2 rows");
      
      Eigen::MatrixXd M(hM.topRows(hM.rows() - 1));
      Eigen::Matrix<double,1,Eigen::Dynamic> denom(Eigen::Matrix<double,1,Eigen::Dynamic>::Ones(hM.cols()).array()/hM.row(hM.rows()-1).array());

    
      for(int i = 0; i < M.rows(); i++)
	{
	  M.row(i).array() *= denom.array();
	}
	
      return M;
    }


    
  }} // namespace asrl::math
