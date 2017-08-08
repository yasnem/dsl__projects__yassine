#include <gtest/gtest.h>
#include <boost/cstdint.hpp>


#include <asrl/rosutil/time_utilities.hpp>

TEST(RosutilTestSuite,timeTest)
{
  ros::Time::init();
  ros::Time t(1304809309,591458797);
  
  boost::posix_time::ptime pt = asrl::rosutil::rosTimeToPTime(t);
  ros::Time rt = asrl::rosutil::ptimeTimeToRosTime(pt);

  ASSERT_NEAR(t.toSec(),rt.toSec(),1e-6);

}
