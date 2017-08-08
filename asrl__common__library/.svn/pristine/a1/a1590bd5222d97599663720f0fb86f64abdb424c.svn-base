#include <asrl/math/rotations.hpp>
#include <asrl/assert_macros.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

//Eigenvalue decomposition
#include <Eigen/Eigenvalues>

namespace asrl { namespace math {

  // Euler angle rotations.
  Eigen::Matrix3d Rx(double radians){
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) = 1;   C(0,1) = 0.0; C(0,2) =  0;
    C(1,0) = 0.0; C(1,1) = c;   C(1,2) = -s;
    C(2,0) = 0.0; C(2,1) = s;   C(2,2) =  c;

    return C;
  }

  Eigen::Matrix3d Ry(double radians){
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) =  c;   C(0,1) = 0.0; C(0,2) =  s;
    C(1,0) =  0.0; C(1,1) = 1;   C(1,2) = 0.0;
    C(2,0) = -s;   C(2,1) = 0.0; C(2,2) =  c;

    return C;
  }
  Eigen::Matrix3d Rz(double radians){
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) = c;   C(0,1) = -s;  C(0,2) = 0.0;
    C(1,0) = s;   C(1,1) =  c;  C(1,2) = 0.0;
    C(2,0) = 0.0; C(2,1) = 0.0; C(2,2) =  1;

    return C;
  }

  Eigen::Matrix3d rph2r(double x, double y, double z){
    Eigen::Matrix3d C;
    double cx = cos(x);
    double sx = sin(x);
    double cy = cos(y);
    double sy = sin(y);
    double cz = cos(z);
    double sz = sin(z);
    //[cos(z)*cos(y), -sin(z)*cos(x)+cos(z)*sin(y)*sin(x),  sin(z)*sin(x)+cos(z)*sin(y)*cos(x)]
    //[sin(z)*cos(y),  cos(z)*cos(x)+sin(z)*sin(y)*sin(x), -cos(z)*sin(x)+sin(z)*sin(y)*cos(x)]
    //[      -sin(y),                       cos(y)*sin(x),                       cos(y)*cos(x)]
    C(0,0) = cz*cy; C(0,1) = -sz*cx+cz*sy*sx; C(0,2) =   sz*sx+cz*sy*cx;
    C(1,0) = sz*cy; C(1,1) =  cz*cx+sz*sy*sx; C(1,2) =  -cz*sx+sz*sy*cx;
    C(2,0) = -sy;   C(2,1) =           cy*sx; C(2,2) =            cy*cx;

    return C;
  }
  Eigen::Matrix3d rph2r(Eigen::Vector3d const & x){
    return rph2r(x[0],x[1],x[2]);
  }

  Eigen::Vector3d r2rph(Eigen::Matrix3d const & C){
    double phi = asin(C(2,0));
    double theta = atan2(C(2,1),C(2,2));
    double psi = atan2(C(1,0), C(0,0));

    Eigen::Vector3d ret;
    ret[0] = theta;
    ret[1] = -phi;
    ret[2] = psi;

    return ret;
  }


  Eigen::Matrix3d rph_S(Eigen::Vector3d const & rph) {
    double s2 = sin(rph(1));
    double c2 = cos(rph(1));
    double s3 = sin(rph(2));
    double c3 = cos(rph(2));
    // This is in a negative sense to Tim's notes.
    Eigen::Matrix3d S;
    S(0,0) =  c2*c3;   S(0,1) = -s3;   S(0,2) = 0.0;
    S(1,0) =  c2*s3;   S(1,1) =  c3;   S(1,2) = 0.0;
    S(2,0) =  -s2  ;   S(2,1) = 0.0;   S(2,2) = 1.0;

    return S;
  }



  //// Small angle approximation.
  Eigen::Matrix3d crossMx(double x, double y, double z){
    Eigen::Matrix3d C;
    C(0,0) =  0.0; C(0,1) = -z;   C(0,2) =   y;
    C(1,0) =  z;   C(1,1) =  0.0; C(1,2) =  -x;
    C(2,0) = -y;   C(2,1) =  x;   C(2,2) =   0.0;

    return C;
  }

  Eigen::Matrix3d crossMx(Eigen::Vector3d const & x){
    return crossMx(x[0],x[1],x[2]);
  }

  Eigen::Matrix3d crossMx(Eigen::VectorXd const & x){
    return crossMx(x[0],x[1],x[2]);
  }

  // Axis Angle rotation.
  Eigen::Matrix3d axisAngle2r(double a, double ax, double ay, double az){

    ASRL_ASSERT_DBG(std::runtime_error,fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0) < 1e-4, "The axis is not a unit vector. ||a|| = " << (sqrt(ax*ax + ay*ay + az*az)));

    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();
    // e = [ax ay az]
    // e*(e') + (eye(3) - e*(e'))*cos(a) - crossMx(e) * sin(a) =
    //[         ax^2+ca*(1-ax^2), ax*ay-ca*ax*ay+sa*az, ax*az-ca*ax*az-sa*ay]
    //[ ax*ay-ca*ax*ay-sa*az,         ay^2+ca*(1-ay^2), ay*az-ca*ay*az+sa*ax]
    //[ ax*az-ca*ax*az+sa*ay, ay*az-ca*ay*az-sa*ax,         az^2+ca*(1-az^2)]
    double sa = sin(a);
    double ca = cos(a);
    double ax2 = ax*ax;
    double ay2 = ay*ay;
    double az2 = az*az;
    double const one = double(1);

    Eigen::Matrix3d C;
    C(0,0) =  ax2+ca*(one-ax2);       C(0,1) = ax*ay-ca*ax*ay+sa*az; C(0,2) = ax*az-ca*ax*az-sa*ay;
    C(1,0) =  ax*ay-ca*ax*ay-sa*az; C(1,1) = ay2+ca*(one-ay2);       C(1,2) = ay*az-ca*ay*az+sa*ax;
    C(2,0) =  ax*az-ca*ax*az+sa*ay; C(2,1) = ay*az-ca*ay*az-sa*ax; C(2,2) = az2+ca*(one-az2);

    return C;
  }
  Eigen::Matrix3d axisAngle2r(double x, double y, double z) {
    double a = sqrt(x*x + y*y + z*z);
    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();

    double d = 1/a;
    return axisAngle2r(a,x*d, y*d, z*d);
  }

  Eigen::Matrix3d axisAngle2r(Eigen::Vector3d const & x){
    return axisAngle2r(x[0], x[1], x[2]);
  }

  Eigen::Vector3d r2AxisAngle(Eigen::Matrix3d const & C){
    double a = acos( (C(0,0) + C(1,1) + C(2,2) - double(1)) * double(0.5));
    Eigen::Vector3d axis;

    if (boost::math::isnan(a)) {
      std::cout << "[WARNING]: Improper rotation matrix: trace(C) > 3." << std::endl;
      return Eigen::Vector3d::Zero();
    }

    if(fabs(a) < 1e-10) {
      return Eigen::Vector3d::Zero();
    }

    axis[0] = (C(2,1) - C(1,2));
    axis[1] = (C(0,2) - C(2,0));
    axis[2] = (C(1,0) - C(0,1));
    double n2 = axis.norm();
    if(fabs(n2) < 1e-10)
      return Eigen::Vector3d::Zero();

    double scale = -a/n2;
    axis = scale * axis;

    return axis;

  }


  Eigen::Vector3d r2AxisAngleRobust(Eigen::Matrix3d const & C){  //It kills me to write this in Paul's style. -- JDG
    double ph; //The scale of the rotation
    Eigen::Vector3d axisAngle; //The return axisAngle

    ph = std::acos( 0.5*(C.trace() - 1) );

    if ( std::abs(std::sin(ph)) > 1e-9 ) {//General case: ph is NOT near [0, pi, 2*pi]
      axisAngle = r2AxisAngle(C);
    }
    else if (abs(ph) > 1e-9) { //ph is near pi or 2*pi
      bool isEvFound = false; //whether we've found an eigen value that has a abs() of 1


      //Solve for the eigen values
      Eigen::EigenSolver<Eigen::Matrix3d> es(C);

      //Iterate over the eigen values and take the first one that is == 1.0
      for (unsigned int i = 0u; i < C.rows() && !isEvFound; ++i) {
        if (std::abs(es.eigenvalues()(i).real() - 1.0) < 1E-12 && es.eigenvalues()(i).imag() == 0.0) {
          isEvFound = true;
          axisAngle = ph * es.eigenvectors().col(i).real();

          if ( (axisAngle2r(axisAngle).transpose() * C).trace() - 3.0 > 1e-14) {
            axisAngle = -axisAngle;
          }
        }
      }
    }
    else { //ph is 0, therefore rotation is just identity
      axisAngle.setZero();
    }

     return axisAngle;
  }

  // Utility functions
  double angleMod(double radians){
    return (double)(radians - (ASRL_2PI * rint(radians / ASRL_2PI)));
  }
  double deg2rad(double degrees){
    return (double)(degrees * ASRL_DEG2RAD);
  }
  double rad2deg(double radians){
    return (double)(radians * ASRL_RAD2DEG);
  }


  Eigen::Matrix3d Cx(double radians)
  {
    return Rx(-radians);
  }
  Eigen::Matrix3d Cy(double radians)
  {
    return Ry(-radians);
  }
  Eigen::Matrix3d Cz(double radians)
  {
    return Rz(-radians);
  }

   Eigen::Matrix3d rph2C(double x, double y, double z)
   {
     return rph2r(-x,-y,-z);
   }

   Eigen::Matrix3d rph2C(Eigen::Vector3d const & x)
   {
     return rph2C(x[0],x[1],x[2]);
   }
   Eigen::Matrix3d rph2C(Eigen::VectorXd const & x)
   {
     ASRL_ASSERT_EQ_DBG(std::runtime_error,x.size(),3,"The input vector must have 3 components");
     return rph2C(x[0],x[1],x[2]);
   }

   Eigen::Matrix3d rph2C(Eigen::MatrixXd const & A, unsigned column)
   {
     ASRL_ASSERT_EQ_DBG(std::runtime_error,A.rows(),3,"The input matrix must have 3 rows");
     ASRL_ASSERT_LT_DBG(std::runtime_error,column,A.cols(),"The requested column is out of bounds");
     return rph2C(A(0,column),A(1,column),A(2,column));
   }

   Eigen::Vector3d C2rph(Eigen::MatrixXd const & C)
   {
     ASRL_ASSERT_EQ_DBG(std::runtime_error,C.rows(),3,"The input matrix must be 3x3");
     ASRL_ASSERT_EQ_DBG(std::runtime_error,C.cols(),3,"The input matrix must be 3x3");

     Eigen::Vector3d rph;

     rph[1] = asin(C(2,0));
     rph[2] = atan2(-C(1,0),C(0,0));
     rph[0] = atan2(-C(2,1),C(2,2));

     return rph;
   }

   Eigen::Vector3d C2rph(Eigen::Matrix3d const & C)
   {
     Eigen::Vector3d rph;

     rph[1] = asin(C(2,0));
     rph[2] = atan2(-C(1,0),C(0,0));
     rph[0] = atan2(-C(2,1),C(2,2));

     return rph;
   }



  }} // asrl::math


