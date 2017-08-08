#ifndef ASRL_ROSUTIL_TIME_UTILITIES_HPP
#define ASRL_ROSUTIL_TIME_UTILITIES_HPP

#include <ros/time.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace asrl { namespace rosutil {

    std::string rosTimeToIsoString(const ros::Time & time);
    std::string rosTimeToIsoFilename(const ros::Time & time);
    boost::posix_time::ptime rosTimeToPTime(const ros::Time & time);
    ros::Time ptimeTimeToRosTime(const boost::posix_time::ptime & time);

  }} // namespace asrl::rosutil

namespace boost { namespace serialization {
    template<class Archive>
    void serialize(Archive & ar, ros::Time & t, const unsigned int version)
    {
      ar & t.sec;
      ar & t.nsec;
    }
    
    
  }} // boost::serialization


#endif /* ASRL_ROSUTIL_TIME_UTILITIES_HPP */
