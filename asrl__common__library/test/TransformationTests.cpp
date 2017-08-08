// Bring in my package's API, which is what I'm testing
#include <asrl/math/Transformation.hpp>
#include <asrl/math/transformations.hpp>
#include <Eigen/Core>
// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Serialization
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace asrl::math;

TEST(TransformationTestSuite, testSerialization)
{

  Transformation T;

  T.T() = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0.1,0.2,0.3);

  {
    std::ofstream ofs("test.ba", std::ios::binary);
    boost::archive::binary_oarchive oa(ofs);
    oa << T;
  }

  Transformation T2;

  {
    std::ifstream ifs("test.ba", std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    ia >> T2;
  }  


  Transformation::transformation_t::MatrixType & M1 = T.T().matrix();
  Transformation::transformation_t::MatrixType & M2 = T2.T().matrix();

  ASSERT_TRUE(M1 == M2) << "M1:\n" << M1 << "\nnot equal to M2\n" << M2 << std::endl;

  Transformation::covariance_t & C1 = T.U();
  Transformation::covariance_t & C2 = T2.U();

  ASSERT_TRUE(C1 == C2) << "C1:\n" << C1 << "\nnot equal to C2\n" << C2 << std::endl;

}

inline Eigen::Matrix4d randomTransformation()
{
  Eigen::Matrix<double,6,1> t = Eigen::Matrix<double,6,1>::Random();
  t.head<3>() *= 100.0;

  return toTEuler(t);
}

TEST(TransformationTestSuite, testComposition)
{
  Eigen::Matrix4d T_01 = randomTransformation();
  Eigen::Matrix4d T_12 = randomTransformation();
  Eigen::Matrix4d T_02 = T_01 * T_12;
  
  Transformation TT_01(T_01);
  Transformation TT_12(T_12);

  Eigen::Vector4d v_2 = Eigen::Vector4d::Random();
  Eigen::Vector4d v_0 = T_01 * T_12 * v_2;
  
  Transformation TT_02 = TT_01.composeWith(TT_12);
  

  Eigen::Vector4d vv_0 = TT_02.T().matrix() * v_2;

  for(int i = 0; i < v_0.size(); i++)
    {
      ASSERT_NEAR(vv_0(i),v_0(i),1e-10) << "transformed vector\n" << v_0 << "\nand Transformation transformed vector\n" << vv_0 << "\nwere not as close as expected\n";
    }

}


TEST(TransformationTestSuite, testInverse)
{
  Eigen::Matrix4d T_01 = randomTransformation();
  
  typedef Eigen::Matrix<double,6,6> cov_t;

  // Make a nice posdef matrix.
  cov_t U_01 = cov_t::Random();
  U_01 = U_01 * U_01.transpose() + cov_t::Identity();
  
  //  std::cout << "U_01:\n" << U_01 << std::endl;

  Transformation TT_01(T_01, U_01);
  Transformation TT_10 = TT_01.inverse();
  Transformation TTT_01 = TT_10.inverse();


  Eigen::Matrix4d r1_T_01 = TT_01.T().matrix();
  Eigen::Matrix4d r2_T_01 = TTT_01.T().matrix();

  for(int r = 0; r < r1_T_01.rows(); r++)
    {
      for(int c = 0; c < r1_T_01.cols(); c++)
	{
	  ASSERT_NEAR(r1_T_01(r,c),r2_T_01(r,c),1e-10) << "transformed matrix\n" << r1_T_01 << "\nand alternate path transformed matrix\n" << r2_T_01 << "\nwere not as close as expected\n";
	}
    }

  cov_t r1_U_01 = TT_01.U();
  cov_t r2_U_01 = TTT_01.U();

  // std::cout << "alt U_01\n" << r2_U_01 << std::endl;
  for(int r = 0; r < r1_U_01.rows(); r++)
    {
      for(int c = 0; c < r1_U_01.cols(); c++)
	{
	  ASSERT_NEAR(r1_U_01(r,c),r2_U_01(r,c),1e-10) << "transformed covariance matrix\n" << r1_U_01 << "\nand alternate path transformed covariance matrix\n" << r2_U_01 << "\nwere not as close as expected\n";
	}
    }
  
}



TEST(TransformationTestSuite, testComposition2)
{
  Eigen::Matrix4d T_01 = randomTransformation();
  Eigen::Matrix4d T_12 = randomTransformation();
  Eigen::Matrix4d T_02 = T_01 * T_12;
  
  typedef Eigen::Matrix<double,6,6> cov_t;

  // Make a nice posdef matrix.
  cov_t U_01 = cov_t::Random();
  U_01 = U_01 * U_01.transpose() + cov_t::Identity();
  
  cov_t U_12 = cov_t::Random();
  U_12 = U_12 * U_12.transpose() + cov_t::Identity();

  //  std::cout << "U_01:\n" << U_01 << std::endl;
  //  std::cout << "U_12:\n" << U_12 << std::endl;

  Transformation TT_01(T_01, U_01);
  Transformation TT_12(T_12, U_12);
  Transformation TT_02 = TT_01.composeWith(TT_12);


  Transformation TT_10 = TT_01.inverse();
  Transformation TT_21 = TT_12.inverse();
  Transformation TT_20 = TT_21.composeWith(TT_10);

  Transformation TT_20direct = TT_02.inverse();

  Eigen::Matrix4d r1_T_20 = TT_20.T().matrix();
  Eigen::Matrix4d r2_T_20 = TT_20direct.T().matrix();

  for(int r = 0; r < r1_T_20.rows(); r++)
    {
      for(int c = 0; c < r1_T_20.cols(); c++)
	{
	  ASSERT_NEAR(r1_T_20(r,c),r2_T_20(r,c),1e-10) << "transformed matrix\n" << r1_T_20 << "\nand alternate path transformed matrix\n" << r2_T_20 << "\nwere not as close as expected\n";
	}
    }

  cov_t r1_U_20 = TT_20.U();
  cov_t r2_U_20 = TT_20direct.U();
  for(int r = 0; r < r1_U_20.rows(); r++)
    {
      for(int c = 0; c < r1_U_20.cols(); c++)
	{
	  ASSERT_NEAR(r1_U_20(r,c),r2_U_20(r,c),1e-10) << "transformed covariance matrix\n" << r1_U_20 << "\nand alternate path transformed covariance matrix\n" << r2_U_20 << "\nwere not as close as expected\n";
	}
    }
  
}


TEST(TransformationTestSuite, testComposition3)
{
  Eigen::Matrix4d T_01 = randomTransformation();
  Eigen::Matrix4d T_12 = randomTransformation();
  Eigen::Matrix4d T_02 = T_01 * T_12;
  
  typedef Eigen::Matrix<double,6,6> cov_t;

  // Make a nice posdef matrix.
  cov_t U_01 = cov_t::Random();
  U_01 = U_01 * U_01.transpose() + cov_t::Identity();
  
  cov_t U_12 = cov_t::Random();
  U_12 = U_12 * U_12.transpose() + cov_t::Identity();

  //  std::cout << "U_01:\n" << U_01 << std::endl;
  //  std::cout << "U_12:\n" << U_12 << std::endl;

  Transformation TT_01(T_01, U_01);
  Transformation TT_12(T_12, U_12);
  Transformation TT_02 = TT_01.composeWith(TT_12);
  Transformation TTT_01 = TT_02.inverseComposeWithLeft(TT_12);


  // Test inverse composition left
  Eigen::Matrix4d r1_T_01 =  TT_01.T().matrix();
  Eigen::Matrix4d r2_T_01 = TTT_01.T().matrix();

  for(int r = 0; r < r1_T_01.rows(); r++)
    {
      for(int c = 0; c < r1_T_01.cols(); c++)
	{
	  ASSERT_NEAR(r1_T_01(r,c),r2_T_01(r,c),1e-10) << "transformed matrix\n" << r1_T_01 << "\nand alternate path transformed matrix\n" << r2_T_01 << "\nwere not as close as expected\n";
	}
    }

  cov_t r1_U_01 = TT_01.U();
  cov_t r2_U_01 = TTT_01.U();
  for(int r = 0; r < r1_U_01.rows(); r++)
    {
      for(int c = 0; c < r1_U_01.cols(); c++)
	{
	  ASSERT_NEAR(r1_U_01(r,c),r2_U_01(r,c),1e-10) << "transformed covariance matrix\n" << r1_U_01 << "\nand alternate path transformed covariance matrix\n" << r2_U_01 << "\nwere not as close as expected\n";
	}
    }


  // Test inverse composition right
  Transformation TTT_12 = TT_01.inverseComposeWithRight(TT_02);
  Eigen::Matrix4d r1_T_12 =  TT_12.T().matrix();
  Eigen::Matrix4d r2_T_12 = TTT_12.T().matrix();

  for(int r = 0; r < r1_T_12.rows(); r++)
    {
      for(int c = 0; c < r1_T_12.cols(); c++)
	{
	  ASSERT_NEAR(r1_T_12(r,c),r2_T_12(r,c),1e-10) << "transformed matrix\n" << r1_T_12 << "\nand alternate path transformed matrix\n" << r2_T_12 << "\nwere not as close as expected\n";
	}
    }

  cov_t r1_U_12 = TT_12.U();
  cov_t r2_U_12 = TTT_12.U();
  for(int r = 0; r < r1_U_12.rows(); r++)
    {
      for(int c = 0; c < r1_U_12.cols(); c++)
	{
	  ASSERT_NEAR(r1_U_12(r,c),r2_U_12(r,c),1e-10) << "transformed covariance matrix\n" << r1_U_12 << "\nand alternate path transformed covariance matrix\n" << r2_U_12 << "\nwere not as close as expected\n";
	}
    }


  
}
