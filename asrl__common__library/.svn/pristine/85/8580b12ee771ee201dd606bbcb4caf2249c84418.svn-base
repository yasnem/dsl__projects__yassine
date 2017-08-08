#include <asrl/rosutil/image_utilities.hpp>
#include <sensor_msgs/image_encodings.h>
#include <asrl/assert_macros.hpp>

namespace asrl { namespace rosutil {

    void createImageView(cv::Mat & outMat, sensor_msgs::Image const & imageMessage)
    {
        int cvtype = CV_8UC1;
        if(imageMessage.encoding == sensor_msgs::image_encodings::MONO8 ||
           imageMessage.encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
           imageMessage.encoding == sensor_msgs::image_encodings::BAYER_RGGB8 ||
           imageMessage.encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||
           imageMessage.encoding == sensor_msgs::image_encodings::BAYER_GBRG8 ||
           imageMessage.encoding == sensor_msgs::image_encodings::BAYER_GRBG8
        )
        {
            outMat = cv::Mat(imageMessage.height, imageMessage.width, cvtype, 
                             const_cast<void *>(static_cast<const void *>(&imageMessage.data[0])), 
                             imageMessage.step);
        }
        else if(imageMessage.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            cvtype = CV_16UC1;
            outMat = cv::Mat(imageMessage.height, imageMessage.width, cvtype, 
                             const_cast<void *>(static_cast<const void *>(&imageMessage.data[0])), 
                             imageMessage.step);
        }
        else if(imageMessage.encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
                imageMessage.encoding == sensor_msgs::image_encodings::RGB8 ||
                imageMessage.encoding == sensor_msgs::image_encodings::BGR8 )
        {
            cvtype = CV_8UC3;
            outMat = cv::Mat(imageMessage.height, imageMessage.width, cvtype, 
                             const_cast<void *>(static_cast<const void *>(&imageMessage.data[0])), 
                             imageMessage.step);
        }
        else if(imageMessage.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        {
            cvtype = CV_32FC1;
            outMat = cv::Mat(imageMessage.height, imageMessage.width, cvtype, 
                             const_cast<void *>(static_cast<const void *>(&imageMessage.data[0])), 
                             imageMessage.step);
        }
        else if(imageMessage.encoding == sensor_msgs::image_encodings::TYPE_64FC1)
        {
            cvtype = CV_64FC1;
            outMat = cv::Mat(imageMessage.height, imageMessage.width, cvtype, 
                             const_cast<void *>(static_cast<const void *>(&imageMessage.data[0])), 
                             imageMessage.step);
        }
        else
        {
            ASRL_THROW(std::runtime_error, "Unsupported image encoding [" << imageMessage.encoding << "]. Supported encodings: {"
                                            << sensor_msgs::image_encodings::MONO8 << ", "
                                            << sensor_msgs::image_encodings::RGB8 << ", "
                                            << sensor_msgs::image_encodings::BGR8 << ", "
                                            << sensor_msgs::image_encodings::TYPE_8UC1 << ", "
                                            << sensor_msgs::image_encodings::TYPE_32FC1 << ", "
                                            << sensor_msgs::image_encodings::TYPE_8UC3 << "}");
        }
    }

    void initImageView(cv::Mat & outMat, sensor_msgs::Image & imageMessage, int width, int height, int cvtype)
    {
    	switch(cvtype)
    	{
    	case CV_8UC1:
    		// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width;
    		imageMessage.encoding = sensor_msgs::image_encodings::MONO8;
    		imageMessage.data.resize(height*width);

    		break;
    	case CV_8UC3:
    		// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*3;
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    		imageMessage.data.resize(height*width*3);

    		break;
    	case CV_16UC1:
    		// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*sizeof(CV_16UC1);
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
    	case CV_32FC1:
        	// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*sizeof(float);
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
        case CV_64FC1:
            // Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*sizeof(double);
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
    	default:
    		ASRL_THROW(std::runtime_error,"Unsupported CV type. Supported types are { CV_8UC1, CV_8UC3, CV_16UC1, CV_32FC1, CV_64FC1 }");
    	}

    	//initImage(imageMessage, width, height, cvtype);

        // Initialize the CV Mat.
        outMat = cv::Mat(height, width, cvtype, &imageMessage.data[0], imageMessage.step);
    }

    void initImage(sensor_msgs::Image & imageMessage, int width, int height, int cvtype)
    {
    	switch(cvtype)
    	{
    	case CV_8UC1:
    		// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width;
    		imageMessage.encoding = sensor_msgs::image_encodings::MONO8;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
    	case CV_8UC3:
    		// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*3;
    		imageMessage.encoding = sensor_msgs::image_encodings::RGB8;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
    	case CV_16UC1:
    		// Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*2;
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
    	case CV_32FC1:
    	    // Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*sizeof(float);
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
		case CV_64FC1:
		    // Initialize the image.
    		imageMessage.height   = height;
    		imageMessage.width    = width;
    		imageMessage.step     = width*sizeof(double);
    		imageMessage.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
    		imageMessage.data.resize(height*imageMessage.step);
    		break;
    	default:
    		ASRL_THROW(std::runtime_error,"Unsupported CV type. Supported types are { CV_8UC1, CV_8UC3, CV_16UC1, CV_32FC1, CV_64FC1 }");
    	}
    }

    void adaptiveHistogramEqualization(cv::Mat& inputImage, unsigned int uiXres, unsigned int uiYres, unsigned int Min,
    		                           unsigned int Max, unsigned int uiNrX, unsigned int uiNrY, unsigned int uiNrBins, float fCliplimit)
    {
    	int rtn = CLAHE(inputImage.ptr(), uiXres, uiYres, Min, Max, uiNrX, uiNrY, uiNrBins, fCliplimit);

    	// TODO: need to fill in all these codes
    	if (rtn != 0)
    	{
    		ROS_WARN("Adaptive histogram did not work. Code = %d",rtn);
    	}
	}

  }} // namespace asrl::bumblebeexb3
