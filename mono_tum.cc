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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	
#include<opencv2/core/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include "/usr/local/include/ctello.h"

#include<System.h>
#include <Converter.h>	

using namespace std;
using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;


const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
//VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
VideoCapture capture(0);
Tello tello{}; 
cv::Mat im;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void saveMap(ORB_SLAM2::System &SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
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
    while(true)
    {
    	capture >> im;
    	sleep(0.1);
    }
}

void runDrone() // constantly saves the image from the camera
{
    while(true)
    {
    	tello.SendCommand("cw 20");
    	while (!(tello.ReceiveResponse())); 
    	sleep(0.1);
    }
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    cout << endl << "main started" << endl;
    thread t1(takeImage); // creates the thread to constantly save image from camera
    //thread t2(runDrone);
    cout << endl << "after thread" << endl;
    
    // connects to drone
    /*if (!tello.Bind()) 
    {
        return 0;
    }*/
    cout << endl <<  "after binding" << endl;
    
    // turns on video stream and makes the drone fly
    /*tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()));
    tello.SendCommand("takeoff");
    while (!(tello.ReceiveResponse())); */

    // sets the length of the main loop, tframe is for time statistics
    int nImages = 10000;
    double tframe;
    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, true);
	
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl; 
    
    // Main loop
    for(int ni=0; ni<nImages; ni++)
    {
        tframe = ni; // time statistic
	
        // Pass the image to the SLAM system
        if(!im.empty())
        {
                SLAM.TrackMonocular(im,tframe);
        }
        sleep(0.5);
    }

    // Stop all threads
    SLAM.Shutdown();
    
    
    // Save camera trajectory and the map
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    saveMap(SLAM);
    
    // land and turns off stream from drone
    /*tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));
    tello.SendCommand("streamoff");
    while (!(tello.ReceiveResponse())); */
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
