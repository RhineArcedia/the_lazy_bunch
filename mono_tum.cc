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
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>	
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
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
#define SLICE_QUALITY_TEST 0.55 //Slices that are probably good
#define SLICES_LIMIT 20 //Limits the number of good slices
#define MAX_TIME 20 // max time to wait for location from orb slam

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
bool mapInit = false;
bool rotating = true;
bool updating = false;
bool finished = false;
int counting = 0;
//VideoCapture capture(0);
Tello tello{}; 
cv::Mat im;
cv::Mat Twc;
cv::Mat euler;


// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = (double)rotationMatrix.at<float>(0,0);
  double m02 = (double)rotationMatrix.at<float>(0,2);
  double m10 = (double)rotationMatrix.at<float>(1,0);
  double m11 = (double)rotationMatrix.at<float>(1,1);
  double m12 = (double)rotationMatrix.at<float>(1,2);
  double m20 = (double)rotationMatrix.at<float>(2,0);
  double m22 = (double)rotationMatrix.at<float>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = (double)(x * 180/M_PI);
  euler.at<double>(1) = (double)(y * 180/M_PI);
  euler.at<double>(2) = (double)(z * 180/M_PI);
  
  if(x < 0) { euler.at<double>(0) = euler.at<double>(0) + 360; }
  if( y < 0) { euler.at<double>(1) = euler.at<double>(1) + 360; }
  if(z < 0) { euler.at<double>(2) = euler.at<double>(2) + 360; }
  
  return euler;
}


//Written by Erel Afoota for locating a path exiting a room as a group project
//Group members: David Doner, Elad Hirshel, Erel Afoota

// baseX / baseZ - x / z value of some point, originX / originZ - x / z value for the origin of all slice 
//Returns the angle of median of desired slice and average distance of points in desired slice to origin and the xz coordinates of the median point in that slice
void find_route(double baseX, double baseZ, double originX, double originZ, double* pointer )
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
    // File pointer
    fstream fin;
  
    // Open map file (.csv)
    fin.open(MAP_FILE_NAME, ios::in);
    
    //Calculate normal for base line
    double baseNorm = sqrt(pow(baseX, 2.0) + pow(baseZ, 2.0));
    cout << "The baseNorm is " << baseNorm << endl;
    
    //Some variables
    string line, word, temp;
    double x, z, angle;
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
        getline(s, word, ',');
        z = stod(word);
        //cout << "x is " << x << " and y is " << y << endl;
        x = x - originX;
        z = z - originZ;
        
        //Calculating the angle compered to the chosen line
        vectorMultiplication = x * baseX + z * baseZ;
        crossProduct =  baseX * z - baseZ * x;
        
        normal = sqrt(pow(x, 2.0) + pow(z, 2.0));
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
    bool over = false, below = false;
    int counter = 0;
    while(counter < SLICES_LIMIT && numberOfPoints[higherSlice + 1] > MIN_POINT_TO_CONSIDER_SLICE && (sliceSumOfDistances[higherSlice + 1] / numberOfPoints[higherSlice + 1]) >= SLICE_QUALITY_TEST * pointer[1])
    {
    	higherSlice++;
    	counter++;
    	if (higherSlice >= NUMBER_OF_SLICES){
    		higherSlice = 0;
    		over = true;
    	}
    }
    
    //Now lowest slice
    counter = 0;
    while(counter < SLICES_LIMIT && numberOfPoints[lowerSlice - 1] > MIN_POINT_TO_CONSIDER_SLICE && (sliceSumOfDistances[lowerSlice - 1] / numberOfPoints[lowerSlice - 1]) >= SLICE_QUALITY_TEST * pointer[1])
    {
    	lowerSlice--;
    	counter++;
    	if (lowerSlice < 0){
    		lowerSlice = NUMBER_OF_SLICES - 1;
    		below = true;
    	}
    }
    
    cout << "higher slice is " << higherSlice << "\nlower slice is " << lowerSlice << endl;
    if(over) { higherSlice = higherSlice + NUMBER_OF_SLICES; }
    if(below) { lowerSlice = lowerSlice - NUMBER_OF_SLICES; }
    cout << "higher slice is " << higherSlice << "\nlower slice is " << lowerSlice << endl;
    
    //Now update to desired slice
    pointer[0] = (int)((higherSlice + lowerSlice)/2);
    if(pointer[0] < 0) { pointer[0] = NUMBER_OF_SLICES + pointer[0]; }
    if(pointer[0] >= NUMBER_OF_SLICES) { pointer[0] = pointer[0] - NUMBER_OF_SLICES; }
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
        getline(s, word, ',');
        z = stod(word); 
        v1.push_back(x);
        v1.push_back(z);
        x = x - originX;
        z = z - originZ;
        //Calculating the angle compered to the chosen line
        vectorMultiplication = x * baseX + z * baseZ;
        crossProduct =  baseX * z - baseZ * x;
        
        normal = sqrt(pow(x, 2.0) + pow(z, 2.0));
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
    double median_x = 0, median_z = 0;
    if(!mapOfPoints.empty())
    {
    	auto iter = mapOfPoints.begin();
    	advance(iter, (mapOfPoints.size() / 2) - 1); // to reach middle of list
    	median_x = iter->second.front();
    	median_z = iter->second.back();
    	if (mapOfPoints.size() % 2 == 0) // checking if number of elements is even
    	{
    		iter++;
    		median_x = (median_x + iter->second.front()) / 2;
    		median_z = (median_z + iter->second.back()) / 2;
    	}
    }
    cout << "The median x is " << median_x << "\nThe median z is " << median_z << endl;
    pointer[2] = median_x;
    pointer[3] = median_z;
    median_x = median_x - originX;
    median_z = median_z - originZ;
    vectorMultiplication = median_x * baseX + median_z * baseZ;
    crossProduct =  baseX * median_z - baseZ * median_x;
    normal = sqrt(pow(median_x, 2.0) + pow(median_z, 2.0));
    if(normal == 0) { angle = 0; }
    else 
    {
    	angle = acos(vectorMultiplication / (normal * baseNorm)) * 180/M_PI;
    	if(crossProduct > 0) { angle = 360 - angle; }
    }
    pointer[0] = angle;
    pointer[1] = sqrt(pow(median_x, 2) + pow(median_z, 2));
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
    while (!(tello.ReceiveResponse())) { sleep(0.1); }
    while(!mapInit)
    //while(1)
    {
    	sleep(0.5);
    	tello.SendCommand("up 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.1);} 
    	sleep(0.5);
    	tello.SendCommand("down 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.1); } 
    }
    for(int i = 0; i < 18; i++)
    {
    	tello.SendCommand("cw 20");
    	while (!(tello.ReceiveResponse())) {sleep(0.1); }
    	sleep(1);
    	tello.SendCommand("up 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.1); }
    	sleep(1); 
    	tello.SendCommand("down 20");
    	while (!(tello.ReceiveResponse())) { sleep(0.1); }
    	sleep(1); 
    	cout << "finished rotating" << endl;
    } 
	rotating = false;
}

// Vars is the angle and target point
void goToDoor(double* Vars, double originX, double originZ) // rotate to face door and go to it
{	
	double direction_x = (double)(sin(euler.at<double>(2) * M_PI/180)), direction_z = (double)(cos(euler.at<double>(2) * M_PI/180));
	double x = (double)Twc.at<float>(0, 3), z = (double)Twc.at<float>(2, 3);
	double medianNorm = sqrt(pow(Vars[2] - x, 2) + pow(Vars[3] - z, 2));
	double Norm = sqrt(pow(x - originX, 2) + pow(z - originZ, 2));
	double baseNorm = sqrt(pow(direction_x, 2) + pow(direction_z, 2));
	double vectorMultiplication = 0.0, crossProduct = 0.0;
	double angle = 0;
	
	tello.SendCommand("ccw 20"); // in case the angle is smaller than 20 because the tello can rotate a minimum of 20
    while (!(tello.ReceiveResponse())) { sleep(0.2); }
    sleep(0.5);
    tello.SendCommand("cw " + to_string((int)(Vars[0] + 20)));
    while (!(tello.ReceiveResponse())) { sleep(0.2); }
    sleep(2.5);
    
    while(Vars[1] > Norm && counting < MAX_TIME)
    {
    	tello.SendCommand("forward 50");
    	while(!(tello.ReceiveResponse())) { sleep(0.2); }
    	sleep(0.5);
    	updating = true;
    	while(updating && counting < MAX_TIME) { sleep(0.01); }
    	if(counting < MAX_TIME) 
    	{ 
			x = (double)Twc.at<float>(0,3);
			z = (double)Twc.at<float>(2,3);
			direction_x = (double)(sin(euler.at<double>(2) * M_PI/180));
			direction_z = (double)(cos(euler.at<double>(2) * M_PI/180));
			
			vectorMultiplication = direction_x * (Vars[2] - x) + direction_z * (Vars[3] - z);
			crossProduct =  direction_x * (Vars[3] - z) - direction_z * (Vars[2] - x);
			baseNorm = sqrt(pow(direction_x, 2.0) + pow(direction_z, 2.0));
			Norm = sqrt(pow(x - originX, 2) + pow(z - originZ, 2));
			medianNorm = sqrt(pow(Vars[2] - x, 2) + pow(Vars[3] - z, 2));
			
			if(baseNorm == 0 || medianNorm == 0) { angle = 0; }
			else 
			{
				angle = acos(vectorMultiplication / (medianNorm * baseNorm)) * 180/M_PI;
			}
			
			if(crossProduct < 0)
			{
				tello.SendCommand("ccw 20");
				while(!(tello.ReceiveResponse())) { sleep(0.2); }
				sleep(0.5);
				tello.SendCommand("cw " + to_string((int)(angle + 20.5)));
				while(!(tello.ReceiveResponse())) { sleep(0.2); }
				sleep(0.5);
			}
			else
			{
				tello.SendCommand("cw 20");
				while(!(tello.ReceiveResponse())) { sleep(0.2); }
				sleep(0.5);
				tello.SendCommand("ccw " + to_string((int)(angle + 20.5)));
				while(!(tello.ReceiveResponse())) { sleep(0.2); }
				sleep(0.5);
			}
		}
    } 
    
    tello.SendCommand("forward 400");
    while(!(tello.ReceiveResponse())) { sleep(0.2); }
    sleep(1);
    tello.SendCommand("land");
    while (!(tello.ReceiveResponse())) { sleep(0.2); }
    finished = true;
}

int main(int argc, char **argv)
{
	cv::Mat tmpTcw;
	double direction_camera_x = 0.0, direction_camera_y = 0.0, direction_camera_z = 0.0;
	double last_camera_x = 0.0, last_camera_y = 0.0, last_camera_z = 0.0;
	double results[4] = {0, 0, 0, 0}; 
	int images = 0;
	
	
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    cout << endl << "main started" << endl;
    
    //c onnects to drone
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
    //for(int ni=0; ni<2500; ni++)
    while(rotating)
    {
        // Pass the image to the SLAM system
        if(!im.empty())
        {
    		/*cout << "image not empty" << endl; */
            tmpTcw = SLAM.TrackMonocular(im,images++);
            if(!tmpTcw.empty()) {
            	Twc = tmpTcw.clone().inv();
    			//cout << "the other x is " << Twc.at<float>(0, 3) << "\nthe other y is " << Twc.at<float>(1, 3) << "\nthe other z is " << Twc.at<float>(2, 3) << "\nthe other normal is " << Twc.at<float>(3, 3) << endl;
    			euler = rot2euler(Twc);
    			//cout << "euler is: " << endl << euler << endl;
    			//cout << "normal is: " << endl << Tcw << endl;
    			//cout << "inverse is: " << endl << Twc << endl;
        	}
            if(!mapInit && !Twc.empty()){ mapInit = true;}
        }
        sleep(0.05); 
    } 
    
    SLAM.ActivateLocalizationMode();
    
    cout << "final position*********************************************************************8" << endl;
    //final position
    last_camera_x = (double)Twc.at<float>(0, 3);
    last_camera_y = (double)Twc.at<float>(1, 3);
    last_camera_z = (double)Twc.at<float>(2, 3);
    
    direction_camera_x = (double)(sin(euler.at<double>(2) * M_PI/180));
    direction_camera_y = (double)(sin(euler.at<double>(0) * M_PI/180));
    direction_camera_z = (double)(cos(euler.at<double>(2) * M_PI/180));
    
    
    // Save camera trajectory and the map
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    saveMap(SLAM); 
    cout << "map saved " << endl;
    cout << "direction_camera_x is " << direction_camera_x << endl;
    cout << "direction_camera_y is " << direction_camera_y << endl;
    cout << "direction_camera_z is " << direction_camera_z << endl;
    cout << "last_camera_x is " << last_camera_x << endl;
    cout << "last_camera_y is " << last_camera_y << endl;
    cout << "last_camera_z is " << last_camera_z << endl;
    
    // find the correct angle to the middle of the slice where the door is
    find_route(direction_camera_x, direction_camera_z, last_camera_x, last_camera_z, results);
    cout << "find_route worked " << endl;
    cout << "angle is " << results[0] << " and the average distance is " << results[1] << endl;
    thread t3(goToDoor, results, last_camera_x, last_camera_z); // thread for going to the door
    cout << "another thread " << endl;
	while(!finished)
	{
		if(!im.empty())
		{
			tmpTcw = SLAM.TrackMonocular(im, images++);
			counting++;
			if(!tmpTcw.empty()) {
			counting = 0;
             	Twc = tmpTcw.clone().inv();
    			//cout << "the other x is " << Twc.at<float>(0, 3) << "\nthe other y is " << Twc.at<float>(1, 3) << "\nthe other z is " << Twc.at<float>(2, 3) << "\nthe other normal is " << 	Twc.at<float>(3, 3) << endl;
    			euler = rot2euler(Twc);
    			//cout << "euler is: " << endl << euler << endl;
    			//cout << "normal is: " << endl << Tcw << endl;
    			//cout << "inverse is: " << endl << Twc << endl;
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
