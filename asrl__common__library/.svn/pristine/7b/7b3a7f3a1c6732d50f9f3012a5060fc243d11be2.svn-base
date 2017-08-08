#ifndef ASRL_BUMBLEBEEXB3_UTILITY_HPP
#define ASRL_BUMBLEBEEXB3_UTILITY_HPP

#include <sensor_msgs/Image.h>
#include "clahe.h"
#include <opencv2/core/core.hpp>
#include <ros/console.h>

namespace asrl { namespace rosutil {

    void createImageView(cv::Mat & outMat, sensor_msgs::Image const & imageMessage);
    void initImageView(cv::Mat & outMat, sensor_msgs::Image & imageMessage, int width, int height, int cvtype);
    void initImage(sensor_msgs::Image & imageMessage, int width, int height, int cvtype);

    /*   pImage - Pointer to the input/output image  
     *   uiXRes - Image resolution in the X direction  
     *   uiYRes - Image resolution in the Y direction  
     *   Min - Minimum greyvalue of input image (also becomes minimum of output image)  
     *   Max - Maximum greyvalue of input image (also becomes maximum of output image)  
     *   uiNrX - Number of contextial regions in the X direction (min 2, max uiMAX_REG_X)  
     *   uiNrY - Number of contextial regions in the Y direction (min 2, max uiMAX_REG_Y)  
     *   uiNrBins - Number of greybins for histogram ("dynamic range")  
     *   float fCliplimit - Normalized cliplimit (higher values give more contrast)  
     * The number of "effective" greylevels in the output image is set by uiNrBins; selecting  
     * a small value (eg. 128) speeds up processing and still produce an output image of  
     * good quality. The output image will have the same minimum and maximum value as the input  
     * image. A clip limit smaller than 1 results in standard (non-contrast limited) AHE.  
     */   
    void adaptiveHistogramEqualization(cv::Mat& inputImage, unsigned int uiXres, unsigned int uiYres, unsigned int Min,
    		unsigned int Max, unsigned int uiNrX, unsigned int uiNrY, unsigned int uiNrBins, float fCliplimit);

  }} // namespace asrl::bumblebeexb3


#endif // ASRL_BUMBLEBEEXB3_UTILITY_HPP
