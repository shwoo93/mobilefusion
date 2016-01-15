#ifndef _KINECT_FRAME_LISTENER_
#define _KINECT_FRAME_LISTENER_

#include <opencv2/opencv.hpp>

namespace mobilefusion{

	class KinectFrameListener {
	public:
		KinectFrameListener() {};
		virtual ~KinectFrameListener() {};
		virtual void OnFrame(cv::Mat &rgb, cv::Mat &depth) = 0;
	};
}



#endif