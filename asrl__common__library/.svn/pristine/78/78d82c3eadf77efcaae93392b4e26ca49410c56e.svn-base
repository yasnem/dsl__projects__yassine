#include <numpy_eigen/boost_python_headers.hpp>
#include <asrl/math/Transformation.hpp>


using namespace boost::python;
using namespace asrl::math;

const Eigen::Matrix4d & T(const Transformation * Tp)
{
  return Tp->T().matrix();
}

const Eigen::Matrix<double,6,6> & U(const Transformation * Tp)
{
  return Tp->U();
}


void setT(Transformation * Tp, const Eigen::Matrix4d & Tmx)
{
  Tp->T() = Tmx;
}

void setU(Transformation * Tp, const Eigen::Matrix<double,6,6> & Umx)
{
  Tp->U() = Umx;
}

void export_Transformation()
{
    using namespace boost::python;
    using namespace asrl;
    using namespace asrl::math;


    class_<Transformation>("Transformation",init<>())
      .def(init<Eigen::Matrix4d>())
      .def(init<Eigen::Matrix4d,Eigen::Matrix<double,6,6> >())
      .def("T",T,return_value_policy<copy_const_reference>())
      .def("setT",setT)
      .def("U",U,return_value_policy<copy_const_reference>())
      .def("setU",setU)
      .def("inverse",&Transformation::inverse)
      .def("composeWith",&Transformation::composeWith)
      .def("inverseComposeWithLeft",&Transformation::inverseComposeWithLeft)
      .def("inverseComposeWithRight",&Transformation::inverseComposeWithRight);

}
