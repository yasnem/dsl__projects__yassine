
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/local_time_adjustor.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <asrl/assert_macros.hpp>

#include <iostream> 

namespace asrl { namespace timing {

	inline std::time_t to_time_t(boost::posix_time::ptime const & t) 
	{ 
		if( t == boost::date_time::neg_infin ) 
			return 0; 
		else if( t == boost::date_time::pos_infin ) 
			return LONG_MAX; 

		boost::posix_time::ptime start(boost::gregorian::date(1970,1,1)); 
		return (t-start).total_seconds(); 
	} 

	inline std::string to_iso_string(boost::posix_time::ptime const & utc_date_time)
	{
		// Adapted from:
		// boost::posix_time::to_iso_extended_string
		std::stringstream ss;
		
		std::string ts = boost::gregorian::to_iso_extended_string(utc_date_time.date());
		ss << ts;

		boost::posix_time::time_duration utc_time = utc_date_time.time_of_day();

		ASRL_ASSERT(std::runtime_error,!utc_time.is_special(),"Function not valid for special times");

		// Signify the end of the date and the beginning of the time.
		ss << 'T';
		char fill_char = '0';
		if(utc_time.is_negative()) {
			ss << '-';
		}
		ss  << std::setw(2) << std::setfill(fill_char) 
			<< boost::date_time::absolute_value(utc_time.hours());
		ss  << ':';
		ss  << std::setw(2) << std::setfill(fill_char) 
			<< boost::date_time::absolute_value(utc_time.minutes());
		ss  << ':';
		ss  << std::setw(2) << std::setfill(fill_char) 
			<< boost::date_time::absolute_value(utc_time.seconds());

		boost::posix_time::time_duration::fractional_seconds_type frac_sec = 
			boost::date_time::absolute_value(utc_time.fractional_seconds());

		ss  << "." << std::setw(boost::posix_time::time_duration::num_fractional_digits())
			<< std::setfill(fill_char) << frac_sec;

		// Signify that this is UTC time.
		ss << "Z";

		return ss.str();
	}

	// Parses an iso time string of the format 
	// 2009-07-23T10:19:22.858750-05:00
	// or
	// 2009-07-23T10:19:22.858750Z
	// (for UTC)
	// and returns the UTC time as a posix time object.
	inline boost::posix_time::ptime from_iso_string(std::string const & s)
	{
		boost::posix_time::ptime ret_time;
		// This function will be simple but rigid.
		// The model of an iso time string to be used is:
		// 2009-07-23T10:19:22.858750-05:00
		// so.
		ASRL_ASSERT_GE(std::runtime_error,s.size(),27,"The time string \"" << s << "\" is too short.");
		ASRL_ASSERT_EQ(std::runtime_error,s[ 4],'-',"The time string \"" << s << "\" does not conform to the expected format.");
		ASRL_ASSERT_EQ(std::runtime_error,s[ 7],'-',"The time string \"" << s << "\" does not conform to the expected format.");
		ASRL_ASSERT_EQ(std::runtime_error,s[10],'T',"The time string \"" << s << "\" does not conform to the expected format.");
		ASRL_ASSERT_EQ(std::runtime_error,s[13],':',"The time string \"" << s << "\" does not conform to the expected format.");
		ASRL_ASSERT_EQ(std::runtime_error,s[16],':',"The time string \"" << s << "\" does not conform to the expected format.");
		ASRL_ASSERT_EQ(std::runtime_error,s[19],'.',"The time string \"" << s << "\" does not conform to the expected format.");

		try {
			// parse the year.
			int year = boost::lexical_cast<int>(s.substr(0,4));

			// parse the month
			int month = boost::lexical_cast<int>(s.substr(5,2));

			// parse the day
			int day = boost::lexical_cast<int>(s.substr(8,2));

			// parse the hour
			int hour = boost::lexical_cast<int>(s.substr(11,2));

			// parse the minute
			int minute = boost::lexical_cast<int>(s.substr(14,2));

			// parse the whole seconds
			int second = boost::lexical_cast<int>(s.substr(17,2));

			// parse the fractional seconds
			int fractional_second = boost::lexical_cast<int>(s.substr(20,6));

			// build the nominal time.
			ret_time = boost::posix_time::ptime(
					boost::gregorian::date(year,month,day),
					boost::posix_time::time_duration(hour,minute,second,fractional_second)
				);

			// parse the time zone.
			// This is the tricky part.
			char tz = s[26];
			if(tz == 'Z') 
			{
				// The time zone is utc
				ASRL_ASSERT_EQ(std::runtime_error,s.size(),27,"The time string \"" << s << "\" time zone field does not conform to the expected format.");
				// There is nothing more to be done with the time zone offset.
			}
			else if(tz == '+' || tz == '-')
			{
				ASRL_ASSERT_EQ(std::runtime_error,s.size(),32,"The time string \"" << s << "\" time zone field does not conform to the expected format.");
				ASRL_ASSERT_EQ(std::runtime_error,s[29],':',"The time string \"" << s << "\" does not conform to the expected format.");
				
				// parse the time zone offset hours
				int tz_hours = boost::lexical_cast<int>(s.substr(27,2));
				 
				// parse the time zone offset minutes
				int tz_minutes = boost::lexical_cast<int>(s.substr(30,2));

				boost::posix_time::time_duration tz_offset = boost::posix_time::time_duration(tz_hours,tz_minutes,0,0);
				// Build the time duration time zone offset
				if(tz == '-')
				{
					ret_time += tz_offset;
				}
				else
				{
					ret_time -= tz_offset;
				}
			}
			else
			{
			  ASRL_THROW(std::runtime_error,"The time string \"" << s << "\" time zone field does not conform to the expected format.");
			}
		} 
		catch (boost::bad_lexical_cast const & e) 
		{
			ASRL_THROW(std::runtime_error,"Exception parsing time string: " << e.what());
		}
		return ret_time;
	}
  }} // namespace asrl::timing
