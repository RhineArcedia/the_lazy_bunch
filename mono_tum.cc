/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <math.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	
#include<opencv2/core/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include "/usr/local/include/ctello.h"
#include "MapDrawer.h"

#include<System.h>
#include <Converter.h>	

using namespace std;
using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

#define MAP_FILE_NAME "map.csv" //Use a path or put the file at the dame directory as the program
#define NUMBER_OF_SLICES 100 //Determains the number of slices, *do not use a high number*
#define MIN_POINT_TO_CONSIDER_SLICE 20 //The minimum amount of points in a slice to consider it as viable

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
bool rotating = true;
bool mapInit = false;
bool finished = false;
//VideoCapture capture(0);
Tello tello{}; 
cv::Mat im;


//Written by Erel Afoota for locating a path exiting a room as a group project
//Group members: David Doner, Elad Hirshel, Erel Afoota

// baseX / baseY - x / y value of some point, originX / originY - x / y value for the origin of all slices
//Returns the index of desired slice
int read_record(double baseX, double baseY, double originX, double originY)
{
    //Slices arrays
    double sliceSumOfDistances[NUMBER_OF_SLICES]; //Holds the sum of associated points distances
    for(int i = 0; i < NUMBER_OF_SLICES; i++)
    {
    	sliceSumOfDistances[NUMBER_OF_SLICES] = 0.0;
    }
    int numberOfPoints[NUMBER_OF_SLICES]; //Holds the number of associated points
    for(int i = 0; i < NUMBER_OF_SLICES; i++)
    {    
        numberOfPoints[NUMBER_OF_SLICES] = 0;
    }
    
    // File pointer
    fstream fin;
  
    // Open map file (.csv)
    fin.open(MAP_FILE_NAME, ios::in);
    
    //Calculate normal for base line
    double baseNormal = sqrt(pow(baseX - originX, 2.0) + pow(baseY - originY, 2.0));
  
    //Some variables
    string line, word, temp;
    double x, y, angle;
    double vectorMultiplication = 0.0, normal = 0.0;
    double sliceSpacing = 360/NUMBER_OF_SLICES;
    
    //Calculate space between slices (note that slices intersect)
    
    
    //Runs for each line in the csv file (meaning each point)
    while (fin >> temp) {
  
        getline(fin, line);
  
        stringstream s(line);
        getline(s, word, ',');
        x = stod(word);
        getline(s, word, ',');
        getline(s, word, ',');
        y = stod(word);
        
        //Calculating the angle compered to choosen line
        vectorMultiplication = x * baseX + y * baseY;
        normal = sqrt(pow(x - originX, 2.0) + pow(y - originY, 2.0));
        if(normal * baseNormal != 0.0)
        {
			angle = acos(vectorMultiplication / (normal * baseNormal));
			
			//Determine the correct slices for the point
			int lowerSlice = (int)(angle / sliceSpacing);
			int higherSlice;
			if((lowerSlice * sliceSpacing) >= (sliceSpacing / 2.0))
			{
				higherSlice = lowerSlice + 1;
				if (higherSlice >= NUMBER_OF_SLICES)
				    higherSlice = 0;
			}
			else
			{
				higherSlice = lowerSlice - 1;
				if (higherSlice < 0)
				    higherSlice = NUMBER_OF_SLICES - 1;
			}
			
			//Update slice's arrays
			numberOfPoints[lowerSlice] += 1;
			numberOfPoints[higherSlice] += 1;
			sliceSumOfDistances[lowerSlice] += normal;
			sliceSumOfDistances[higherSlice] += normal;
		}
    }
    
    //Now we find our desired line
    int result = 0; //The index of the desired slice
    double resultAdjustedLen = 0.0, tempAdjustedLen = 0.0; //Average len of a slice
    for(int i = 0; i < NUMBER_OF_SLICES; i++)
    {
        tempAdjustedLen = (sliceSumOfDistances[i] / numberOfPoints[i]);
        if(tempAdjustedLen > resultAdjustedLen && numberOfPoints[i] >= MIN_POINT_TO_CONSIDER_SLICE)
        {
            result = i;
            resultAdjustedLen = tempAdjustedLen;
        }
    }
    return result;
}


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void saveMap(ORB_SLAM2::System& SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("~/ORB_SLAM2/Examples/Monocular/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
}

void takeImage() // constantly saves the image from the camera
{
    // turns on video stream and makes the drone fly
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse())) { sleep(0.5);}
    
    
    VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
    while(true)
    {
    	capture >> im;
    	/*if(im.empty())
    	{
    		cout << "image is empty" << endl;
    	} */
    	sleep(0.1);
    }
}

void runDrone() // constantly saves the image from the camera
{
    tello.SendCommand("takeoff");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    while(!mapInit)
    {
    	sleep(2);
    	tello.SendCommand("up 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.5);} 
    	sleep(2);
    	tello.SendCommand("down 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.5); } 
    }
    for(int i = 0; i < 16; i++)
    {
    	tello.SendCommand("cw 25");
    	while (!(tello.ReceiveResponse())) {sleep(0.5); }
    	sleep(2);
    	tello.SendCommand("up 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.5); }
    	sleep(1); 
    	tello.SendCommand("down 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.5); }
    	sleep(1); 
    	
    }
    tello.SendCommand("forward 20");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    sleep(2);
    rotating = false;
}

void goToDoor(int angle)
{
	if(angle < 180)
	{
	    tello.SendCommand("ccw 20"); // in case the angle is lower than 20
    	while (!(tello.ReceiveResponse())) { sleep(0.5); }
    	tello.SendCommand("cw " + (angle+20)); // adds 20 to match the earlier rotation
    	while (!(tello.ReceiveResponse())) { sleep(0.5); }
	}	
	else
	{
		tello.SendCommand("cw 20"); // in case the angle we need to rotate is lower than 20
    	while (!(tello.ReceiveResponse())) { sleep(0.5); }
    	tello.SendCommand("ccw " + (380-angle)); // 360-angle is the angle we need to rotate and add 20 to match the earlier rotation
    	while (!(tello.ReceiveResponse())) { sleep(0.5); }
	}
	tello.SendCommand("forward 100");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    tello.SendCommand("land");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    finished = true;
    tello.SendCommand("streamoff");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
}

void showCamera()
{
	while(!finished)
	{
		if(!im.empty())
		{
			imshow("finally this project is over", im);
		}
		sleep(0.5);
	}
}

int main(int argc, char **argv)
{
	double last_camera_x, last_camera_z;
	double extra_camera_x, extra_camera_z;
	double angle;
	int images = 0;
    cv::Mat Twc;
    
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    cout << endl << "main started" << endl;
    
    //connects to drone
    if (!tello.Bind()) 
    {
        return 0;
    }
    cout << endl <<  "after binding" << endl;
    
    thread t1(takeImage); // creates the thread to constantly save image from camera
    thread t2(runDrone); // to make drone spin

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, true);
    
    
    cout << endl << "after thread" << endl;
    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl; 
    
    // Main loop
    //for(int ni=0; ni<nImages; ni++)
    while(rotating)
    {
	
        // Pass the image to the SLAM system
        if(!im.empty())
        {
    		/*cout << "image not empty" << endl; */
            Twc = SLAM.TrackMonocular(im,images++);
            if(!mapInit && !Twc.empty()){ mapInit = true;}
        }
        sleep(0.5); 
    }
    
    // position after going forward to find direction vector
    extra_camera_x = Twc.at<double>(0,3);
    extra_camera_z = Twc.at<double>(2,3);
    
    // go back so that we have direction vector for current camera orientation
    tello.SendCommand("back 20");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    for(int i = 0; i < 100; i++)
    {
    	if(!im.empty())
        {
    		/*cout << "image not empty" << endl; */
            Twc = SLAM.TrackMonocular(im,images++);
        }
        sleep(0.5); 
    }
    //final position
    last_camera_x = Twc.at<double>(0,3);
    last_camera_z = Twc.at<double>(2,3);

    // Stop all threads
    SLAM.Shutdown();
    
    
    // Save camera trajectory and the map
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    saveMap(SLAM);
    
    thread t3(showCamera);
    
    // find the correct angle to the middle of the slice where the door is
    angle = read_record(extra_camera_x, extra_camera_z, last_camera_x, last_camera_z) * (360.0/NUMBER_OF_SLICES) + 180.0/NUMBER_OF_SLICES;
    goToDoor((int)(angle + 0.5));
    
    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
