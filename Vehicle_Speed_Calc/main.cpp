#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

#include "videoprocessor.h"
#include "Detection.h"

int main()
{
	// Create video procesor instance
	VideoProcessor processor;

	// Create background/foreground segmentor 
	Detection detector;
	detector.setThreshold(25);

	// Open video file
	processor.setInput("video.avi");

	// set frame processor
	processor.setFrameProcessor(&detector);

	// Declare a window to display the video
	processor.displayOutput("Extracted Foreground");

	// Play the video at the original frame rate
	processor.setDelay(1000./processor.getFrameRate());

	// Start the process
	processor.run();

	cv::waitKey();
}