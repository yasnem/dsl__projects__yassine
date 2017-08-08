#include <numpy_eigen/boost_python_headers.hpp>
#include <asrl/math/homogeneous_coordinates.hpp>


using namespace boost::python;
using namespace asrl::math;


tuple toHomogeneousAndJacobian(Eigen::Vector3d const & v)
{
  Eigen::Vector4d out_vh;
  Eigen::Matrix<double,4,3> out_H;
  out_vh = toHomogeneous(v,&out_H);
  return make_tuple(out_vh, out_H);
}

tuple fromHomogeneousAndJacobian(Eigen::Vector4d const & v)
{
  Eigen::Vector3d out_vh;
  Eigen::Matrix<double,3,4> out_H;
  out_vh = fromHomogeneous(v,&out_H);
  return make_tuple(out_vh, out_H);
}




  void export_homogeneous_coordinates()
  {
    using namespace boost::python;
    using namespace asrl;


    def("toHomogeneous", toHomogeneous);
    def("toHomogeneousAndJacobian", toHomogeneousAndJacobian);
    def("fromHomogeneous", fromHomogeneous);
    def("fromHomogeneousAndJacobian", fromHomogeneousAndJacobian);
    def("toHomogeneousColumns", toHomogeneousColumns);
    def("fromHomogeneousColumns", fromHomogeneousColumns);
    
  }
