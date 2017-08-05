#if !defined DETECTION_H
#define DETECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "videoprocessor.h"
#include "vehicle.h"

class Detection : public FrameProcessor 
{
	cv::Mat gray;			// current gray-level image
	cv::Mat background;		// accumulated background
	cv::Mat backImage;		// background image
	cv::Mat foreground;		// foreground image
	double learningRate;    // learning rate in background accumulation
	int threshold;			// threshold for foreground extraction

	cv::Rect DetectArea;    //Rect of Detection vehicle
	cv::Rect TrackArea;		//Rect of Tracking  vehicle
	cv::Mat  DetectRegion;	//sub image for detection vehicle
	cv::Mat  TrackRegion;	//sub image for Tracking  vehicle

	std::vector<vehicle> vehicles;	//list of all available vehicle in frame
	bool write_flg;                 //checking for add vehicle or not
	int  frm_count;					//counting frame
	cv::Mat PerspectiveTransform;	//Perspective Transform

  public:
	
	//Constructor of Class for initalize data member
	//magic numbers are hard coded value for Calibrating video file
	Detection() : threshold(40), learningRate(0.01)
				, DetectArea(440,50,100,100)
				, TrackArea(440,50,180,330)
	{
		//Compute Perspective Transform of Video File
		cv::Point2f src_pnt[4] = {cv::Point(6,6),cv::Point(73,6),cv::Point(22,305),cv::Point(179,305)};
		cv::Point2f des_pnt[4] = {cv::Point(6,6),cv::Point(179,6),cv::Point(6,305),cv::Point(179,305)};
		PerspectiveTransform = cv::getPerspectiveTransform(src_pnt,des_pnt);
	}
	
	//Set Threshold for Create Binary Frame
	void setThreshold(int t) 
	{
		threshold = t;
	}

	//Set Learning Rate for Create Background Frame
	void setLearningRate(double r) 
	{
		learningRate= r;
	}

	//**************************************************************************************
	// processing method
	//**************************************************************************************
	
	//overrided Process Method for Detecting Vehicles
	void process(cv:: Mat &frame, cv:: Mat &output) 
	{
		//delay for read next vehicle
		frm_count++;
		if(frm_count == 20)
			write_flg = 1;

		//Grab sub image from Defined area
		DetectRegion = (frame(DetectArea));
		TrackRegion  = (frame(TrackArea));

		cv::cvtColor(DetectRegion, gray, CV_BGR2GRAY);

		// initialize background to 1st frame
		if (background.empty())
		{
			gray.convertTo(background, CV_32F);
			cv::blur(background,background,cv::Size(5,5));
			background.convertTo(backImage,CV_8U);
		}

		// compute difference between current image and background
		cv::absdiff(backImage,gray,foreground);

		// apply threshold to foreground image
		cv::threshold(foreground,output,threshold,255,cv::THRESH_BINARY);

		// update BackGround Frame
		cv::accumulateWeighted(gray, background, learningRate, output);
		cv::imshow("back",backImage);

		// Set Morphological Operation For Detect Vehicle
		cv::erode(output,output,cv::Mat(),cv::Point(-1,-1),3);
		cv::dilate(output,output,cv::Mat(),cv::Point(-1,-1),2);
		cv::morphologyEx(output,output,cv::MORPH_CLOSE,cv::Mat(),cv::Point(-1,-1),6);
		cv::morphologyEx(output,output,cv::MORPH_OPEN,cv::Mat(),cv::Point(-1,-1),2);

		//detect vehicle from white contours
		std::vector<std::vector<cv::Point>> contours;
	    cv::findContours(output,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		int cmin= 220;  // minimum contour length
		int cmax= 5000; // maximum contour length

		//Eliminate samll object from list
		std::vector<std::vector<cv::Point>>::const_iterator itc= contours.begin();
		while (itc!=contours.end()) 
		{
			if (itc->size() < cmin || itc->size() > cmax)
				itc= contours.erase(itc);
			else 
				++itc;
		}

		//Create Vehicle and Add it to list
		if(!contours.empty())
		{
			itc= contours.begin();
			while (itc!=contours.end()) 
			{
				cv::Rect r0 = cv::boundingRect(cv::Mat(*itc));
				if(write_flg)
				{
					vehicles.push_back(vehicle((DetectRegion), PerspectiveTransform));
					write_flg = 0;
					frm_count = 0;
				}
				++itc;
			}
		}

		//track and draw line of vehicle
		std::vector<vehicle>::iterator vit = vehicles.begin();
		while(vit!=vehicles.end())
		{
			//Track vehicle in WrapedPerspectived image of frame
			cv::Mat wp_img = /*wp*/((TrackRegion));
			//frame = wp_img;
			(*vit).Track(wp_img);
			(*vit).DrawLine(wp_img);
			cv::imshow("wp",wp_img);
			
			
			vit++;
		}

		//Calc wee speed of vehicle 
		vit = vehicles.begin();
		while(vit!=vehicles.end())
		{
			(*vit).CalcSpeed();
			vit++;
		}

		//Delete vehicle that out from screen
		vit = vehicles.begin();
		while(vit!=vehicles.end())
		{
			cv::Rect TrackBound = cv::Rect(0,0,TrackArea.width,TrackArea.height-30);
			for(int i = 0; i<vit->Position[0].size(); i++)
			{
				if(!(TrackBound.contains(vit->Position[vit->Position.size()-1][i])))
				{
					std::cout << (*vit).Position[i].size() << "\t" <<"\n";
					vit = vehicles.erase(vit);
					cv::waitKey();
					break;
				}
			}
			if(vehicles.empty())
				break;
			vit++;
		}
		output = frame;
	}

	/*cv::Mat wp(cv::Mat& img,int type=1)//,cv::Point2f in_ptn[4], cv::Point2f out_ptn[4])
	{
		//Eliminate Perspective of input image 
		//Clibrated only for defualt video file

		cv::Mat result;

		if(type == 1)
			cv::warpPerspective(img,result,mm,img.size());
		else
			cv::warpPerspective(img,result,mm2,img.size());
		return result;
	}*/
};

#endif

