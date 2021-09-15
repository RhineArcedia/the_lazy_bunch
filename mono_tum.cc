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

#define MAP_FILE_NAME "map.csv" //Use a path or put the file at the dame directory as the program
#define NUMBER_OF_SLICES 300 //Determains the number of slices, *do not use a high number*
#define MIN_POINT_TO_CONSIDER_SLICE 10 //The minimum amount of points in a slice to consider it as viable
#define SLICE_QUALITY_TEST 0.7 //Slices that are probably good
#define SLICES_LIMIT 7 //Limits the number of good slices

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
bool mapInit = false;
bool rotating = true;
bool updating = false;
bool finished = false;
bool moving = false;
//VideoCapture capture(0);
Tello tello{}; 
cv::Mat im;
cv::Mat Tcw;

//Written by Erel Afoota for locating a path exiting a room as a group project
//Group members: David Doner, Elad Hirshel, Erel Afoota

// baseX / baseY - x / y value of some point, originX / originY - x / y value for the origin of all slice 
//Returns the angle of median of desired slice and average distance of points in desired slice to origin and the xy coordinates of the median point in that slice
void find_route(double baseX, double baseY, double originX, double originY, double* pointer )
{
	pointer[0] = 0;
	pointer[1] = 0; // first is index of desired slice, second is average distance of points in desired slice to origin
    //Slices arrays
    double sliceSumOfDistances[NUMBER_OF_SLICES]; //Holds the sum of associated points distances
    for(int i = 0; i < NUMBER_OF_SLICES; i++)
    {
    	sliceSumOfDistances[i] = 0.0;
    }
    int numberOfPoints[NUMBER_OF_SLICES]; //Holds the number of associated points
    //cout << "Init..." << endl;
    for(int i = 0; i < NUMBER_OF_SLICES; i++)
    {    
        numberOfPoints[i] = 0;
        //cout << "Slice: " << i << " = " << numberOfPoints[i] << endl;
    }
    baseX = baseX - originX;
    baseY = baseY - originY;
    // File pointer
    fstream fin;
  
    // Open map file (.csv)
    fin.open(MAP_FILE_NAME, ios::in);
    
    //Calculate normal for base line
    double baseNorm = sqrt(pow(baseX, 2.0) + pow(baseY, 2.0));
    
    //Some variables
    string line, word, temp;
    double x, y, angle;
    double vectorMultiplication = 0.0, crossProduct = 0.0;
    double normal = 0.0;
    double sliceSpacing = 360.0/NUMBER_OF_SLICES; //Calculate space between slices (note that slices intersect)
    int higherSlice = 0, lowerSlice = 0;
    
    //Runs for each line in the csv file (meaning each point)
    getline(fin, line);
    while (!fin.eof()) 
    {
        stringstream s(line);
        getline(s, word, ',');
        if(word.empty()) break;
        //cout << " calculating point " << endl;
        x = stod(word);
        getline(s, word, ',');
        y = stod(word); 
        getline(s, word, ',');
        //cout << "x is " << x << " and y is " << y << endl;
        x = x - originX;
        y = y - originY;
        
        //Calculating the angle compered to the chosen line
        vectorMultiplication = x * baseX + y * baseY;
        crossProduct =  baseX * y - baseY * x;
        
        normal = sqrt(pow(x, 2.0) + pow(y, 2.0));
        if(normal != 0)
        {
			angle = acos(vectorMultiplication / (normal * baseNorm)) * 180/M_PI;
			if(crossProduct > 0) { angle = 360 - angle; }
			//cout << "angle is " << angle << endl;			
			//Determine the correct slices for the point
			lowerSlice = (int)(angle / sliceSpacing);
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
			numberOfPoints[lowerSlice] = numberOfPoints[lowerSlice] + 1;
			numberOfPoints[higherSlice] = numberOfPoints[higherSlice] + 1;
			sliceSumOfDistances[lowerSlice] = sliceSumOfDistances[lowerSlice] + normal;
			sliceSumOfDistances[higherSlice] = sliceSumOfDistances[higherSlice] + normal;
			//cout << "Slice are: " << lowerSlice << ", " << higherSlice << endl;
			//cout << "Num: " << numberOfPoints[lowerSlice] << ", " << numberOfPoints[higherSlice] << endl;
		}
		getline(fin, line);
    }
    
    //Now we find our desired line
    double tempAdjustedLen = 0.0; //Average len of a slice
    for(int i = 0; i < NUMBER_OF_SLICES; i++)
    {
    	if (numberOfPoints[i] > 0)
    	{
			tempAdjustedLen = (sliceSumOfDistances[i] / numberOfPoints[i]);
		    cout << "slice: " << i <<", avg len " << tempAdjustedLen << ", num of points " << numberOfPoints[i] << endl;
		    if(tempAdjustedLen > pointer[1] && numberOfPoints[i] >= MIN_POINT_TO_CONSIDER_SLICE)
		    {
		        pointer[0] = i;
		        pointer[1] = tempAdjustedLen;
		    }
        }
    }
    cout << "Best slice: " << pointer[0] << ", points: " << numberOfPoints[(int)pointer[0]] << ", total len: " << sliceSumOfDistances[(int)pointer[0]] << endl;
    
    //Find a prefered slice
    //First higher slices
    higherSlice = (int)pointer[0];
    lowerSlice = (int)pointer[0];
    while(numberOfPoints[higherSlice + 1] > 0 && (sliceSumOfDistances[higherSlice + 1] / numberOfPoints[higherSlice + 1]) >= SLICE_QUALITY_TEST * pointer[1])
    {
    	higherSlice++;
    	if (higherSlice >= NUMBER_OF_SLICES)
    		higherSlice = 0;
    }
    
    //Now lowest slice
    while(numberOfPoints[lowerSlice - 1] > 0 && (sliceSumOfDistances[lowerSlice - 1] / numberOfPoints[lowerSlice - 1]) >= SLICE_QUALITY_TEST * pointer[1])
    {
    	lowerSlice--;
    	if (lowerSlice < 0)
    		lowerSlice = NUMBER_OF_SLICES;
    }
    
    //Now update to desired slice
    pointer[0] = (int)((higherSlice - lowerSlice)/2) + lowerSlice;
    pointer[1] = (sliceSumOfDistances[(int)pointer[0]] / numberOfPoints[(int)pointer[0]]);
    
    cout << "Chosen slice: " << pointer[0] << ", points: " << numberOfPoints[(int)pointer[0]] << ", total len: " << sliceSumOfDistances[(int)pointer[0]] << endl;
    
    map<double, vector<double>> mapOfPoints;
    vector<double> v1;
    
    
    fin.clear();
    fin.seekg(0, ios::beg);
    //Runs for each line in the csv file (meaning each point)
    getline(fin, line); 
    while (!fin.eof()) 
    {
        stringstream s(line);
        getline(s, word, ',');
        if(word.empty()) break;
        x = stod(word);
        getline(s, word, ',');
        y = stod(word); 
        getline(s, word, ',');
        x = x - originX;
        y = y - originY;
        v1.push_back(x);
        v1.push_back(y);
        //Calculating the angle compered to the chosen line
        vectorMultiplication = x * baseX + y * baseY;
        crossProduct =  baseX * y - baseY * x;
        
        normal = sqrt(pow(x, 2.0) + pow(y, 2.0));
        if(normal != 0)
        {
			angle = acos(vectorMultiplication / (normal * baseNorm)) * 180/M_PI;
			if(crossProduct > 0) { angle = 360 - angle; }
			//Determine the correct slices for the point
			lowerSlice = (int)(angle / sliceSpacing);
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
			
			if(lowerSlice == pointer[0] || higherSlice == pointer[0])
			{
				mapOfPoints.insert(make_pair(normal, v1));
			}
		}
		v1.clear();
		getline(fin, line);
    }
    // finding the median
    double median_x, median_y;
    if(!mapOfPoints.empty())
    {
    	auto iter = mapOfPoints.begin();
    	advance(iter, (mapOfPoints.size() / 2) - 1); // to reach middle of list
    	median_x = iter->second.front();
    	median_y = iter->second.back();
    	if (mapOfPoints.size() % 2 == 0) // checking if number of elements is even
    	{
    		iter++;
    		median_x = (median_x + iter->second.front()) / 2;
    		median_y = (median_y + iter->second.back()) / 2;
    	}
    }
    cout << "The median x is " << median_x << "\nThe median y is " << median_y << endl;
    pointer[2] = median_x;
    pointer[3] = median_y;
    median_x = median_x - originX;
    median_y = median_y - originY;
    vectorMultiplication = median_x * baseX + median_y * baseY;
    crossProduct =  baseX * median_y - baseY * median_x;
    normal = sqrt(pow(median_x, 2.0) + pow(median_y, 2.0));
    if(normal == 0) { angle = 0; }
    else 
    {
    	angle = acos(vectorMultiplication / (normal * baseNorm)) * 180/M_PI;
    	if(crossProduct > 0) { angle = 360 - angle; }
    }
    pointer[0] = angle;
}

void saveMap(ORB_SLAM2::System& SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open(MAP_FILE_NAME);
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
    while (!(tello.ReceiveResponse())) { sleep(0.2);}
    
    VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
    while(!finished)
    {
    	capture >> im;
    	/*if(im.empty())
    	{
    		cout << "image is empty" << endl;
    	} */
    	sleep(0.05);
    }
}

void runDrone() // thread runing the drone
{
    tello.SendCommand("takeoff");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    while(!mapInit)
    //while(1)
    {
    	sleep(0.5);
    	tello.SendCommand("up 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.5);} 
    	sleep(0.5);
    	tello.SendCommand("down 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.5); } 
    }
    for(int i = 0; i < 12; i++)
    {
    	cout << "Rotating " << i << endl;
    	tello.SendCommand("cw 30");
    	while (!(tello.ReceiveResponse())) { sleep(0.1); }
    	sleep(0.5);
    	tello.SendCommand("up 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.1); }
    	sleep(0.5); 
    	tello.SendCommand("down 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.1); }
    	sleep(0.5); 
    }
    //sleep(150);
    rotating = false;
}

// Vars is the angle and target point
void goToDoor(double* Vars, double originX, double originY) // rotate to face door and go to it
{	
	Vars[2] = Vars[2] - originX;
	Vars[3] = Vars[3] - originY;
	double x = Tcw.at<double>(0,3) - originX, y = Tcw.at<double>(1,3) - originY;
	double medianNorm = sqrt(pow(Vars[2], 2) + pow(Vars[3], 2));
	double directionNorm = sqrt(pow(x, 2) + pow(y, 2));
	double vectorMultiplication = 0.0, crossProduct = 0.0;
	double angle;
	
	tello.SendCommand("ccw 20"); // in case the angle is smaller than 20 because the tello can rotate a minimum of 20
    while (!(tello.ReceiveResponse())) { sleep(0.2); }
    sleep(0.2);
    tello.SendCommand("cw " + to_string((int)(angle + 0.5)));
    while (!(tello.ReceiveResponse())) { sleep(0.2); }
    sleep(1);
    
    while(medianNorm > directionNorm)
    {
    	tello.SendCommand("forward 30");
    	while(!(tello.ReceiveResponse())) { sleep(0.2); }
    	sleep(0.2);
    	updating = true;
    	while(!updating) { sleep(0.01); }
    	
    	x = Tcw.at<double>(0,3) - originX;
    	y = Tcw.at<double>(1,3) - originY;
    	vectorMultiplication = x * Vars[2] + y * Vars[3];
        crossProduct =  x * Vars[2] - y * Vars[3];
        directionNorm = sqrt(pow(x, 2.0) + pow(y, 2.0));
        if(directionNorm == 0 || medianNorm == 0) { angle = 0; }
        else 
        {
        	angle = acos(vectorMultiplication / (medianNorm * directionNorm)) * 180/M_PI;
			if(crossProduct > 0) { angle = 360 - angle; }
		}
			
    	tello.SendCommand("ccw 20");
    	while(!(tello.ReceiveResponse())) { sleep(0.2); }
    	sleep(0.2);
    	tello.SendCommand("cw " + to_string((int)(angle + 0.5)));
    	while(!(tello.ReceiveResponse())) { sleep(0.2); }
    	sleep(0.2);
    }
    
    tello.SendCommand("land");
    while (!(tello.ReceiveResponse())) { sleep(0.2); }
    finished = true;
}

int main(int argc, char **argv)
{
	cv::Mat tmpTcw;
	double last_camera_x = 0.0, last_camera_y = 0.0, last_camera_z = 0.0;
	double direction_vector_x = 0.0, direction_vector_y = 0.0, direction_vector_z = 0.0;
	double yaw, pitch, roll;
	double results[4] = {0, 0, 0, 0}; 
	int images = 0;
	
	
    
    
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
    
    thread t1(runDrone); // to make drone spin
    thread t2(takeImage); // creates the thread to constantly save image from camera

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, true);
    
    
    cout << endl << "after thread" << endl;
    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl; 
    
    // Main loop
    //for(int ni=0; ni<2500; ni++)
    while(rotating)
    {
        // Pass the image to the SLAM system
        if(!im.empty())
        {
    		/*cout << "image not empty" << endl; */
            tmpTcw = SLAM.TrackMonocular(im,images++);
            if(!tmpTcw.empty()) 
            {
            	Tcw = tmpTcw;
    			//cout << "the x is " << Tcw.at<double>(0) << "\nthe y is " << Tcw.at<double>(1) << "\nthe z is " << Tcw.at<double>(2) << "\nthe normal is " << Tcw.at<double>(3) << endl;
        	}
            if(!mapInit && !Tcw.empty()){ mapInit = true;}
        }
        sleep(0.05); 
    }
    
    
    cout << "final position**********************************************************************" << endl;
    //final position
    last_camera_x = Tcw.at<double>(0);
    last_camera_y = Tcw.at<double>(1);
    last_camera_z = Tcw.at<double>(2);
    yaw = atan(Tcw.at<double>(1,0)/Tcw.at<double>(0,0));
    pitch = atan(-Tcw.at<double>(2,0)/(sqrt(pow(Tcw.at<double>(2,1), 2)+pow(Tcw.at<double>(2,2), 2))));
    roll = atan(Tcw.at<double>(2,1)/Tcw.at<double>(2,2));
    direction_vector_x = sin(yaw)*cos(pitch);
    direction_vector_y = cos(yaw)*cos(pitch);
    direction_vector_z = sin(pitch);
    
    // Save camera trajectory and the map
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    saveMap(SLAM); 
    cout << "map saved " << endl;
    cout << "direction_vector_x is " << direction_vector_x << endl;
    cout << "direction_vector_y is " << direction_vector_y << endl;
    cout << "direction_vector_z is " << direction_vector_z << endl;
    cout << "last_camera_x is " << last_camera_x << endl;
    cout << "last_camera_y is " << last_camera_y << endl;
    cout << "last_camera_z is " << last_camera_z << endl;
    
    // find the correct angle to the middle of the slice where the door is
    find_route(direction_vector_x, direction_vector_y, last_camera_x, last_camera_y, results);
    cout << "find_route worked " << endl;
    cout << "angle is " << results[0] << " and the average distance is " << results[1] << endl;
    thread t3(goToDoor, results, last_camera_x, last_camera_y); // thread for going to the door
    cout << "another thread " << endl;
    
	while(!finished)
	{
		if(!im.empty())
		{
			tmpTcw = SLAM.TrackMonocular(im, images++);
			if(!tmpTcw.empty()) 
			{
				Tcw = tmpTcw;
				updating = false;
    		}
		}
		sleep(0.1);	
	}    
	
	// stop streaming
    tello.SendCommand("streamoff");
    while (!(tello.ReceiveResponse())) { sleep(0.5); }
    
    // Stop all threads
    SLAM.Shutdown();
    return 0;
}
