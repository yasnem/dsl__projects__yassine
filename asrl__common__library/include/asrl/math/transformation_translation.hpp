#ifndef ASRL_MATH_TRANSFORMATION_TRANSLATION_HPP
#define ASRL_MATH_TRANSFORMATION_TRANSLATION_HPP

#include <asrl/math/Transformation.hpp>
#include <geometry_msgs/Transform.h>

#include <tf/tf.h>

namespace asrl {
  namespace math {

    /// \todo: I don't know if this is correct. Totally untested.
    asrl::math::Transformation transformationRosToAsrl(geometry_msgs::Transform& transform, boost::array<double,36>& covariance)
    {
      btTransform trans(btQuaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                        btVector3(transform.translation.x, transform.translation.y, transform.translation.z));

      asrl::math::Transformation::transformation_t t;
      t.linear() << trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z();
      t.matrix() << trans.getBasis().getRow(0).getX(), trans.getBasis().getRow(0).getY(), trans.getBasis().getRow(0).getZ(),
                    trans.getBasis().getRow(1).getX(), trans.getBasis().getRow(1).getY(), trans.getBasis().getRow(1).getZ(),
                    trans.getBasis().getRow(2).getX(), trans.getBasis().getRow(2).getY(), trans.getBasis().getRow(2).getZ();

      asrl::math::Transformation::covariance_t u;
      u <<  covariance[0], covariance[1], covariance[2], covariance[3], covariance[4], covariance[5],
            covariance[6], covariance[7], covariance[8], covariance[9],covariance[10],covariance[11],
           covariance[12],covariance[13],covariance[14],covariance[15],covariance[16],covariance[17],
           covariance[18],covariance[19],covariance[20],covariance[21],covariance[22],covariance[23],
           covariance[24],covariance[25],covariance[26],covariance[27],covariance[28],covariance[29],
           covariance[30],covariance[31],covariance[32],covariance[33],covariance[34],covariance[35] ;

      return asrl::math::Transformation(t,u);
    }

    /// \todo: I don't know if this is correct. Totally untested.
    asrl::math::Transformation transformationRosToAsrl(geometry_msgs::Transform& transform)
    {
      btTransform trans(btQuaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                        btVector3(transform.translation.x, transform.translation.y, transform.translation.z));

      asrl::math::Transformation::transformation_t t;
      t.linear() << trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z();
      t.matrix() << trans.getBasis().getRow(0).getX(), trans.getBasis().getRow(0).getY(), trans.getBasis().getRow(0).getZ(),
        trans.getBasis().getRow(1).getX(), trans.getBasis().getRow(1).getY(), trans.getBasis().getRow(1).getZ(),
        trans.getBasis().getRow(2).getX(), trans.getBasis().getRow(2).getY(), trans.getBasis().getRow(2).getZ();

      return asrl::math::Transformation(t);
    }

  } // namespace math
} // namespace asrl

#endif // ASRL_MATH_TRANSFORMATION_TRANSLATION_HPP
