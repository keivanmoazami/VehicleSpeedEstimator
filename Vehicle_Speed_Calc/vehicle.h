#ifndef VEHICLE_H
#define VEHICLE_H

#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video\video.hpp>

class vehicle
{
public:
	cv::Mat mImg;						//Current Frame
	cv::Mat mImg_prev;					//Previous Frame
	std::vector<std::vector<cv::Point2f>> Position; //List of all tracked point
	
	std::vector<float> mSpeed;			//List of all speed of vehicle
	float meter_per_pix;				//Scale of pixel per meter 
	cv::Mat Perspective;				//Perspective Transform of Video

	//std::vector<double> mTime;		//List of difference time between two last frame
	//double lastTime;					//Time of last computed frame

public:
	vehicle(cv::Mat& vehicle_img, cv::Mat& perspective)
	{
		Perspective = perspective;

		cv::cvtColor(vehicle_img, vehicle_img, cv::COLOR_BGR2GRAY);
		
		//Extract Good Feature For Tracking With Corner Harris
		std::vector<cv::Point2f> features;
		cv::goodFeaturesToTrack(vehicle_img, features, 50, 0.01f, 10);

		//Eliminate none Vehicle Feature Points
		cv::Rect feature_rect = cv::Rect(vehicle_img.cols/4, vehicle_img.rows/4, vehicle_img.cols/2, vehicle_img.rows/2);
		std::vector<cv::Point2f>::iterator fit = features.begin();
		while(fit != features.end())
		{
			if(!feature_rect.contains(*fit))
			{
				fit = features.erase(fit);
			}
			else
				++fit;
		}

		//insert First Points of Vehicle in List of Position
		Position.insert(Position.end(),features);

		meter_per_pix = 0.0f;
		//lastTime = 0.0;
	}

	//Update New Position of Track Points in last Frame
	void Track(cv::Mat& img)
	{
		if(!Position.empty())
		{
			std::vector<cv::Point2f> lastPos;
			std::vector<cv::Point2f> lastPosNew;
			std::vector<uchar> status;
			std::vector<float> err;

			//Copy first frame in perv_img and calc pixel per meter 
			img.copyTo(mImg);
			if(mImg_prev.empty())
			{
				int height = img.size().height;
				meter_per_pix = 23.0f/height;
				img.copyTo(mImg_prev);
			}

			//get last position of vehicle
			lastPos = Position[Position.size()-1];

			//Track and get new pos of point in current frame
			cv::calcOpticalFlowPyrLK(mImg_prev, mImg, lastPos, lastPosNew, status, err); 
			Position.insert(Position.end(),lastPosNew);
			
			std::swap(mImg_prev,mImg);

			/*double  duration = static_cast<double>(cv::getTickCount());
			if(!lastTime)
				lastTime = duration;
			mTime.push_back((duration - lastTime)/cv::getTickFrequency());
			lastTime = duration;*/
		}
	}

	//show Tracking outpot
	void DrawLine(cv::Mat& img)
	{
		int col = 255;
		int col2 = 255;
		for(int j = 0; j<Position[0].size(); j++)
		{
			for(int i = 0 ; i<Position.size()-1 ; i++)
			{
				col2 = col = abs(255 - col);
				cv::line(img,Position[i][j],Position[i+1][j],cv::Scalar(0, 0, col), 2);

				std::vector<cv::Point2f> src_distance;
				std::vector<cv::Point2f> dst_distance;
				src_distance.push_back(Position[i][j]);
				src_distance.push_back(Position[i+1][j]);
				dst_distance.resize(src_distance.size());
				cv::perspectiveTransform(src_distance,dst_distance,Perspective);
				cv::line(img,dst_distance[0],dst_distance[1],cv::Scalar(0, col2, 0), 2);
			}
		}
		
	}

	//calc Speed of Vehicle in each frame
	void CalcSpeed()
	{
		std::vector<float> distance;
		std::vector<float> multiple;
		int j = 1;
		if(Position.size() < 3)
			return;
		int counter = 0;
		float x1,y1,x2,y2;
		float constmul1,constmul2;

		distance.resize(Position[j].size());
		multiple.resize(Position[j].size());
		
		for(j ; j<Position.size()-1; j++)
		{
			
			for(int i = 0 ; i<Position[j].size(); i++)
			{
				// Compute distance of two point
				std::vector<cv::Point2f> src_distance;
				std::vector<cv::Point2f> dst_distance;
				src_distance.push_back(Position[j][i]);
				src_distance.push_back(Position[j+1][i]);
				dst_distance.resize(src_distance.size());

				// Compute Position of Track Points With out Perspective
				cv::perspectiveTransform(src_distance,dst_distance,Perspective);

				// Compute Distance of Point Between Two Frame
				distance[i] += sqrt(pow(dst_distance[1].x - dst_distance[0].x,2)+pow(dst_distance[1].y - dst_distance[0].y,2));
			}
		}

		//mean of distance 
		float totaldistance = 0;
		float size = Position.size() - 2;
		for(int i=0 ; i<distance.size();i++)
		{
			totaldistance += distance[i] / size;
			counter++;
		}
		if(totaldistance)
		{
			//convert pixel height to meter height
			totaldistance /= counter;
			//distance *= ((constmul1 + constmul2) / 2);
			totaldistance *= meter_per_pix;
		}

		//compute km/h speed of vehicle 
		mSpeed.push_back(totaldistance/0.04f*3.6f);

		//mSpeed.push_back(distance/mTime[i-1]*3.6f);
		//if(mSpeed.size() > 5)
		std::cout <<j<<"- " << totaldistance <<" :\t" << ((!mSpeed.empty()) ? (mSpeed[mSpeed.size()-1]) : 0) <<"\t KM/H \n";
	}
};
#endif
