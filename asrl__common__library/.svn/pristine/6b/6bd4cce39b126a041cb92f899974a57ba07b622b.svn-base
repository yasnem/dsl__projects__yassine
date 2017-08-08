#include <asrl/rosutil/time_utilities.hpp>
#include <asrl/timing/time_utils.hpp>
#include <asrl/string_routines.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

namespace asrl { namespace rosutil {

    std::string rosTimeToIsoString(const ros::Time & time)
    {
      return asrl::timing::to_iso_string(rosTimeToPTime(time));
    }

    std::string rosTimeToIsoFilename(const ros::Time & time)
    {
      std::string ts = rosTimeToIsoString(time);
      boost::replace_all(ts,":","_");
      boost::replace_all(ts,".","_");
      boost::replace_all(ts,"-","_");
      return ts;
    }

    boost::posix_time::ptime rosTimeToPTime(const ros::Time & time)
    {
      typedef boost::date_time::subsecond_duration<boost::posix_time::time_duration,1000000000> nanosec;
      boost::posix_time::ptime t = boost::posix_time::from_time_t(time.sec) + nanosec(time.nsec);

      return t;
    }
    

    ros::Time ptimeTimeToRosTime(const boost::posix_time::ptime & time)
    {
      ASRL_ASSERT_NE(std::runtime_error, time, boost::date_time::neg_infin, "The ptime does not hold a valid time"); 
      ASRL_ASSERT_NE(std::runtime_error, time, boost::date_time::pos_infin, "The ptime does not hold a valid time"); 
      
      static const boost::posix_time::ptime start(boost::gregorian::date(1970,1,1)); 
      boost::posix_time::time_duration d = (time-start);
      boost::int64_t sec = d.total_seconds();
      boost::int64_t nsec = d.total_nanoseconds() - sec * 1000000000;
      return ros::Time(sec,nsec); 

    }

  }} // namespace asrl::rosutil
