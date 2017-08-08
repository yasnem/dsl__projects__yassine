#ifndef ASRL_ROSPARAM_HPP
#define ASRL_ROSPARAM_HPP

#include <ros/ros.h>
#include <string>
#include <boost/lexical_cast.hpp>
#include <asrl/assert_macros.hpp>

namespace asrl { namespace rosutil {

    /** 
     * \brief gets a parameter value, setting the value on the parameter server if none exists already
     *
     * A simple wrapper function that makes sure that ros parameters with
     * default values end up on the parameter server (if no value exists yet).
     * This can help deal with the complexity of having many, many configuration
     * parameters and stop people from having to open up source files to determine
     * exactly what parameters a node has.
     *
     * After this funciton is called, the parameter server will have a value for this parameter.
     * 
     * @param nh  The node handle to get the parameter from
     * @param key Then name of the parameter
     * @param param OUTPUT: A reference to the value to fill in (either with a value from the parameter server, or with the default value)
     * @param defaultValue The default value to use if no value is available on the parameter server
     */
    template<typename T>
    inline void param(const ros::NodeHandle & nh, 
		      const std::string & key, 
		      T & param, 
		      const T & defaultValue)
    {
      // Try to get the value from the parameter server
      if(!nh.getParam(key,param))
	{
	  // If that wasn't successful, push the default value to the parameter server.
	  nh.setParam(key,defaultValue);
	  param = defaultValue;
	}      
    }

    ASRL_DEFINE_EXCEPTION(KeyNotFoundException,std::runtime_error);
    ASRL_DEFINE_EXCEPTION(BadLexicalCast,std::runtime_error);

    /**
     * @class ParameterContainer
     *
     * An abstraction used to wrap ROS or pure C++ parameters for
     * algorithms that may be pure C++ and don't need ROS. The
     * RosParameterContainer wraps a ROS node handle and the
     * MapParameterContainer wraps a std::map<string,string>
     * and has a very simple parser implemented.
     * 
     * Next steps involve separating these things into
     * headers that avoid linking with ROS...however, that is
     * beyond the current scope.
     */
    class ParameterContainer
    {
    public:
      ParameterContainer();
      virtual ~ParameterContainer();
      
      virtual int getInt(const std::string & key, int defaultValue) = 0; 
      virtual int getInt(const std::string & key) = 0; 

      virtual double getDouble(const std::string & key, double defaultValue) = 0; 
      virtual double getDouble(const std::string & key) = 0; 

      virtual std::string getString(const std::string & key, const std::string & defaultValue) = 0; 
      virtual std::string getString(const std::string & key) = 0; 

      virtual bool getBool(const std::string & key, bool defaultValue) = 0; 
      virtual bool getBool(const std::string & key) = 0; 
    };

    class RosParameterContainer : public ParameterContainer
    {
    public:
      RosParameterContainer(const ros::NodeHandle & nh);
      virtual ~RosParameterContainer();
      
      

      virtual int getInt(const std::string & key, int defaultValue); 
      virtual int getInt(const std::string & key); 

      virtual double getDouble(const std::string & key, double defaultValue); 
      virtual double getDouble(const std::string & key); 

      virtual std::string getString(const std::string & key, const std::string & defaultValue); 
      virtual std::string getString(const std::string & key); 

      virtual bool getBool(const std::string & key, bool defaultValue); 
      virtual bool getBool(const std::string & key); 


      template<typename T>
      T get(const std::string & key)
      {
        T rval;
        ASRL_ASSERT(KeyNotFoundException,nh_.getParam(key,rval),"Unable to find key \"" << key << "\" in configuration");
        return rval;
        
      }

      template<typename T>
      T get(const std::string & key, const T & defaultValue)
      {
        T rval;
        asrl::rosutil::param<T>(nh_, 
                key, 
                rval,
                defaultValue);
        return rval;
      }

    private:
      ros::NodeHandle nh_;
    };


    class MapParameterContainer : public ParameterContainer
    {
    public:
      MapParameterContainer();
      virtual ~MapParameterContainer();
      
      virtual int getInt(const std::string & key, int defaultValue); 
      virtual int getInt(const std::string & key); 

      virtual double getDouble(const std::string & key, double defaultValue); 
      virtual double getDouble(const std::string & key); 

      virtual std::string getString(const std::string & key, const std::string & defaultValue); 
      virtual std::string getString(const std::string & key); 

      virtual bool getBool(const std::string & key, bool defaultValue); 
      virtual bool getBool(const std::string & key); 

      template<typename T>
      void set(const std::string & key, const T & value)
      {
        try{
          map_[addPrefix(key)] = boost::lexical_cast<std::string>(value);
        }
        catch(const boost::bad_lexical_cast & e)
          {
            ASRL_THROW(BadLexicalCast,"Bad lexical cast for value \"" 
                       << value << "\" associated with key \"" 
                       << addPrefix(key) << "\" going to type std::string");
          }
      }

      template<typename T>
      T get(const std::string & key)
      {
        map_t::iterator i = map_.find(addPrefix(key));
        ASRL_ASSERT(KeyNotFoundException,i != map_.end(),"Unable to find key \"" << addPrefix(key) << "\" in configuration");
        T rval = T();

        try {
          rval = boost::lexical_cast<T>(i->second);
        }
        catch(const boost::bad_lexical_cast & e)
          {
            ASRL_THROW(BadLexicalCast,"Bad lexical cast for value \"" << i->second << "\" associated with key \"" << addPrefix(key) << "\" going to type " << typeid(T).name());
          }

        return rval;
      }

      template<typename T>
      T get(const std::string & key, const T & defaultValue)
      {
        map_t::iterator i = map_.find(addPrefix(key));
        T rval = defaultValue;
        if(i != map_.end())
          {
            try {
              rval = boost::lexical_cast<T>(i->second);
            }
            catch(const boost::bad_lexical_cast & e)
              {
                ASRL_THROW(BadLexicalCast,"Bad lexical cast for value \"" << i->second << "\" associated with key \"" << addPrefix(key) << "\" going to type " << typeid(T).name());
              }
          }

        return rval;
      }

      void clear();
      void load(const std::string & configFile);
      void setPrefix(const std::string & prefix); 

    private:
      std::string addPrefix(const std::string & key);

      std::string prefix_;
      typedef std::map<std::string,std::string> map_t;
      map_t map_;
    };


  }} // namespace asrl::rosutil


#endif /* ASRL_ROSPARAM_HPP */
