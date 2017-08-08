#ifndef ASRL_ROS_VISUALIZATION_UTILITIES_HPP
#define ASRL_ROS_VISUALIZATION_UTILITIES_HPP

#include <ros/ros.h>
#include <boost/function.hpp>

namespace asrl {
  namespace rosutil {

    /**
     * \brief
     *
     *
     *
     * @param
     * @param
     * @param
     * @param
     */
    template <class MessageT, class VisualizationT>
    class Visualization
    {
    public :

      //*
      /**
       *
       */
      Visualization(ros::NodeHandle nh, std::string visualizationName, const boost::function<VisualizationT(MessageT)>& createVisualizationMessageFunction) :
        nodeHandle_(nh, "visualization"),
        createVisualizationMessage_(createVisualizationMessageFunction)
        {
          visualizationPublisher_ = nodeHandle_.advertise<VisualizationT>(visualizationName, 10);
        }

      //*
      /**
       *
       */
      void update(MessageT update)
        {
          visualizationPublisher_.publish( createVisualizationMessage_(update) );
          return;
        }

    protected :
      ros::NodeHandle nodeHandle_;
      ros::Publisher visualizationPublisher_;
      boost::function<VisualizationT(MessageT)> createVisualizationMessage_;
    };

  } // rosutil
} // asrl


#endif // ASRL_ROS_VISUALIZATION_UTILITIES_HPP

