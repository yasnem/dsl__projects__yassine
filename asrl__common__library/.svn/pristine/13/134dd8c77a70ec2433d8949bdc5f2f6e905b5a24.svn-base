#ifndef ASRL_UTILITIES_NODES
#define ASRL_UTILITIES_NODES

#include <ros/ros.h>
#include <ros/exception.h>
#include <string>

namespace asrl {
  namespace rosutil {

      /*! \fn int spinNodeCatchExceptionNoInit(const std::string& nodeName)
       *  \brief A template function that runs a ros node and catches exceptions without first calling ros::init
       *  \param nodeName The name of the node.
       *  \return An int, 0
       *
       *  This function is a templated helper function that runs a node
       *  wrapped with a try/catch. Unlike spinNodeCatchException, it requires you to manually
       *  run ros::init(). This allows you to use the asrl try/catch wrapping while
       *  also doing custom post-init configuration (i.e., dynamic reconfigure).
       *  Note: ros::init() + spinNodeCatchExceptionNoInit == spinNodeCatchException.
       *  The template type must satisfy two interface requirements.
       *  First, it must have a constructor that takes a nodehandle. Second,
       *  it must have a spin() method.
       *
       */
      template <class NODE_CLASS_T>
      int spinNodeCatchExceptionNoInit(const std::string & nodeName)
      {
        ros::NodeHandle nh("~");
        int exitCode = 0;
        try {
          NODE_CLASS_T n(nh);
          n.spin();
        } catch (const ros::Exception & e) {
          ROS_FATAL_STREAM( nodeName << " node caught a ros exception: " << e.what() );
          exitCode = 1;
        } catch (const std::exception & e) {
          ROS_FATAL_STREAM( nodeName << " node caught an exception: " << e.what() );
          exitCode = 1;
        } catch (...) {
          ROS_FATAL_STREAM( nodeName << " node caught an unknown exception" );
          exitCode = 1;
        }

        ROS_INFO_STREAM( nodeName << " exiting with code: " << exitCode );
        return exitCode;
      }

      /*! \fn int spinNodeCatchException(int argc, char** argv, const std::string& nodeName)
       *  \brief A template function that runs a ros node and catches exceptions
       *  \param argc The number of command line arguments
       *  \param argv The command line arguments
       *  \param nodeName The name of the node.
       *  \return An int, 0
       *
       *  This function is a templated helper function that creates a node
       *  wrapped with a try/catch. It will catch exceptions and report, if
       *  possible. The template type must satisfy two interface requirements.
       *  First, it must have a constructor that takes a nodehandle. Second,
       *  it must have a spin() method.
       *
       */
      template <class NODE_CLASS_T>
      int spinNodeCatchException(int argc, char** argv, const std::string & nodeName)
      {
        ros::init(argc, argv, nodeName);
        return spinNodeCatchExceptionNoInit<NODE_CLASS_T>(nodeName);
      }


  } // namespace rosutil
} // namespace asrl

#endif // ASRL_UTILITIES_NODES
