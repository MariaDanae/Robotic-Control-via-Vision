#include <iostream>
#include<sstream>
#include<string>
#include<cmath>
#include<ctime> 

#include<opencv\cv.h>
#include<opencv\highgui.h>
#include "opencv2/opencv.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#define UNICODE 1 
#include <conio.h>
#include<stdlib.h>
#include<stdio.h>
#include <Windows.h> 
using namespace std;
using namespace cv;

//global variables

const bool debug = 0;
bool Manual_CTRL = false;

const int NMAX_x = 100;
const int NMAX_y = 2;

int initial_block_number;

//////////////////Class Definitions//////////////////////////////////

class bluetooth {

public:

	char outputchar[1];

	HANDLE hSerial;

	DWORD btsIO;

	bool connected;
	bool drop_fix;

	bluetooth(int n);  //constructor

	~bluetooth(); //destructor
};

class lego_collection{

public:

	Mat MAT_Original_image;
	Mat MAT_Gray;
	Mat MAT_threshold;
	Mat MAT_HSV;
	Mat MAT_HSV_THRESHOLD_RED;
	Mat MAT_HSV_THRESHOLD_BLUE;
	Mat MAT_HSV_THRESHOLD_GREEN;
	Mat MAT_HSV_THRESHOLD_ROBOT;			
	int x_y_cordinate_storage[NMAX_x][NMAX_y];	
	int number_of_blocks_color;			 
	int X_COORD;
	int X_COORD_CIRCLE_front;
	int X_COORD_CIRCLE_back;
	int Y_COORD;
	int	Y_COORD_CIRCLE_front;
	int	Y_COORD_CIRCLE_back;
	int min_X_COORD;
	int min_Y_COORD;
	int DISTANCE_TO_BLOCK;
	int block_found;
	bool Block_in_claw;
	Point Robot_Centroid;
	Point Closest_Block_PT;
	double angle_circle_front_to_block;
	double angle_circle_back_to_block;
	float Angle_DIFF;
	void Image_Filtering();
	vector<Point2f>  Block_locations(Mat MAT_threshold, char debug_name[]);		
	void Robot_location(Mat MAT_threshold, char debug_name[]);					void Closest_block_distance(vector<Point2f> Block_Centroid);	
	char Closest_block_angle();			
	char Move_robot(int block_location);
	void Find_closest_block(int block_color);
	void Update_block_distance(Point Block_Centroid);
	Mat line_image;
	void update_image();
	char Angle_to_DROPZONE(float xcord, float ycord);
	
	int angle_counter;
	bool Go_FWD;
	int flag;

	int block_number;
	int dropoff_array [27][3];
	int coord_array[27][2];

	bool backwards;
	bool home;

	lego_collection();

	~lego_collection();

};


//////////////////////////////////Constructors//////////////////////////////


bluetooth::bluetooth(int n){

	outputchar[0] = '0';

	connected = false;
	drop_fix = false;

	// Setup serial port connection and needed variables.
	hSerial = CreateFile(L"\\\\.\\COM14", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	while (!connected){
		if (hSerial != INVALID_HANDLE_VALUE)
		{
			printf("Port opened! \n");

			DCB dcbSerialParams;
			GetCommState(hSerial, &dcbSerialParams);

			dcbSerialParams.BaudRate = CBR_9600;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.Parity = NOPARITY;
			dcbSerialParams.StopBits = ONESTOPBIT;

			SetCommState(hSerial, &dcbSerialParams);
			connected = true;
		}
		else
		{
				if (GetLastError() == ERROR_FILE_NOT_FOUND)
				{
					printf("Serial port doesn't exist! \n");
				}

				printf("Error while setting up serial port! \n");

				//delete file
				DeleteFile(L"\\\\.\\COM14");

				ClearCommError(hSerial, &btsIO, NULL);

				//try to reconnect
				hSerial = CreateFile(L"\\\\.\\COM14", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
		
		}
	}

}

lego_collection::lego_collection(){

	block_found = 0;
	Block_in_claw = false;
	angle_counter = 0;
	Go_FWD = false;
	flag = 0;
	block_number = initial_block_number;

	backwards = false;
	home = false;

	//dropoff_array[block number][color]
	//dropoff_array[block number][size]
	//dropoff_array[block number][location]

	//color: 0=red, 1=blue, 2=green
	//size: 0=small, 1=large

	//Letter I with small blocks only, color sequence as per spec
	dropoff_array[0][0] = 1;
	dropoff_array[0][1] = 1;
	dropoff_array[0][2] = 0;

	dropoff_array[5][0] = 0;
	dropoff_array[5][1] = 1;
	dropoff_array[5][2] = 1;

	dropoff_array[10][0] = 2;
	dropoff_array[10][1] = 0;
	dropoff_array[10][2] = 2;

	dropoff_array[17][0] = 0;
	dropoff_array[17][1] = 1;
	dropoff_array[17][2] = 3;

	dropoff_array[24][0] = 1;
	dropoff_array[24][1] = 1;
	dropoff_array[24][2] = 4;

	//Letter H
	dropoff_array[1][0] = 1;
	dropoff_array[1][1] = 1;
	dropoff_array[1][2] = 5;

	dropoff_array[6][0] = 0;
	dropoff_array[6][1] = 1;
	dropoff_array[6][2] = 6;

	dropoff_array[11][0] = 2;
	dropoff_array[11][1] = 0;
	dropoff_array[11][2] = 7;


	dropoff_array[2][0] = 1;
	dropoff_array[2][1] = 1;
	dropoff_array[2][2] = 8;

	dropoff_array[7][0] = 0;
	dropoff_array[7][1] = 1;
	dropoff_array[7][2] = 9;

	dropoff_array[14][0] = 2;
	dropoff_array[14][1] = 0;
	dropoff_array[14][2] = 10;


	dropoff_array[13][0] = 0;
	dropoff_array[13][1] = 1;
	dropoff_array[13][2] = 11;

	dropoff_array[12][0] = 2;
	dropoff_array[12][1] = 0;
	dropoff_array[12][2] = 12;


	dropoff_array[18][0] = 0;
	dropoff_array[18][1] = 1;
	dropoff_array[18][2] = 13;

	dropoff_array[25][0] = 1;
	dropoff_array[25][1] = 1;
	dropoff_array[25][2] = 14;


	dropoff_array[19][0] = 0;
	dropoff_array[19][1] = 1;
	dropoff_array[19][2] = 15;

	dropoff_array[26][0] = 1;
	dropoff_array[26][1] = 1;
	dropoff_array[26][2] = 16;

	//Letter U
	dropoff_array[3][0] = 1;
	dropoff_array[3][1] = 1;
	dropoff_array[3][2] = 17;

	dropoff_array[8][0] = 0;
	dropoff_array[8][1] = 1;
	dropoff_array[8][2] = 18;

	dropoff_array[15][0] = 2;
	dropoff_array[15][1] = 0;
	dropoff_array[15][2] = 19;

	dropoff_array[20][0] = 1;
	dropoff_array[20][1] = 1;
	dropoff_array[20][2] = 20;


	dropoff_array[4][0] = 1;
	dropoff_array[4][1] = 1;
	dropoff_array[4][2] = 21;

	dropoff_array[9][0] = 0;
	dropoff_array[9][1] = 1;
	dropoff_array[9][2] = 22;

	dropoff_array[16][0] = 2;
	dropoff_array[16][1] = 0;
	dropoff_array[16][2] = 23;

	dropoff_array[23][0] = 1;
	dropoff_array[23][1] = 1;
	dropoff_array[23][2] = 24;


	dropoff_array[22][0] = 0;
	dropoff_array[22][1] = 1;
	dropoff_array[22][2] = 25;

	dropoff_array[21][0] = 2;
	dropoff_array[21][1] = 0;
	dropoff_array[21][2] = 26;

	//coordinates for dropoff
	//coord_array[location][x]
	//coord_array[location][y]

	//Letter I
	coord_array[0][0] = 580;
	coord_array[0][1] = 40;

	coord_array[1][0] = 580;
	coord_array[1][1] = 60;

	coord_array[2][0] = 580;
	coord_array[2][1] = 80;

	coord_array[3][0] = 580;
	coord_array[3][1] = 100;

	coord_array[4][0] = 580;
	coord_array[4][1] = 120;

	//Letter H
	coord_array[5][0] = 530;
	coord_array[5][1] = 40;

	coord_array[6][0] = 530;
	coord_array[6][1] = 60;

	coord_array[7][0] = 530;
	coord_array[7][1] = 80;

	coord_array[8][0] = 470;
	coord_array[8][1] = 40;

	coord_array[9][0] = 470;
	coord_array[9][1] = 60;

	coord_array[10][0] = 470;
	coord_array[10][1] = 80;

	coord_array[11][0] = 490;
	coord_array[11][1] = 80;

	coord_array[12][0] = 510;
	coord_array[12][1] = 80;

	coord_array[13][0] = 530;
	coord_array[13][1] = 100;

	coord_array[14][0] = 530;
	coord_array[14][1] = 120;

	coord_array[15][0] = 470;
	coord_array[15][1] = 100;

	coord_array[16][0] = 470;
	coord_array[16][1] = 120;

	//Letter U

	coord_array[17][0] = 420;
	coord_array[17][1] = 40;

	coord_array[18][0] = 420;
	coord_array[18][1] = 60;

	coord_array[19][0] = 420;
	coord_array[19][1] = 80;

	coord_array[20][0] = 420;
	coord_array[20][1] = 100;

	coord_array[21][0] = 360;
	coord_array[21][1] = 40;

	coord_array[22][0] = 360;
	coord_array[22][1] = 60;

	coord_array[23][0] = 360;
	coord_array[23][1] = 80;

	coord_array[24][0] = 360;
	coord_array[24][1] = 100;

	coord_array[25][0] = 380;
	coord_array[25][1] = 100;

	coord_array[26][0] = 400;
	coord_array[26][1] = 100;

}


//////////////////////////////////Destructors///////////////////////////////


bluetooth::~bluetooth(){

	CloseHandle(hSerial);

	//clear stuff for reconnect
	DeleteFile(L"\\\\.\\COM14");
	ClearCommError(hSerial, &btsIO, NULL);

	printf("Port closed! \n");

}

lego_collection::~lego_collection(){

	printf("lego destructed \n");

}


//////////////////////////////////Member Functions/////////////////////////////////////


void lego_collection::update_image(){

	MAT_Gray.create(MAT_Original_image.size(), MAT_Original_image.type());
	cvtColor(MAT_Original_image, MAT_Gray, CV_BGR2GRAY);
	Mat MAT_myblur;
	blur(MAT_Original_image, MAT_myblur, Size(10, 10), Point(-1, -1), BORDER_DEFAULT);


	cvtColor(MAT_myblur, MAT_HSV, COLOR_BGR2HSV);	

	inRange(MAT_HSV, Scalar(23, 114, 0), Scalar(47, 255, 255), MAT_HSV_THRESHOLD_ROBOT);	erode(MAT_HSV_THRESHOLD_ROBOT, MAT_HSV_THRESHOLD_ROBOT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(MAT_HSV_THRESHOLD_ROBOT, MAT_HSV_THRESHOLD_ROBOT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(MAT_HSV_THRESHOLD_ROBOT, MAT_HSV_THRESHOLD_ROBOT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	return;
}


void lego_collection::Image_Filtering()
{

	MAT_Gray.create(MAT_Original_image.size(), MAT_Original_image.type());
	cvtColor(MAT_Original_image, MAT_Gray, CV_BGR2GRAY);

	Mat MAT_myblur;		// blur image to take out noise
	blur(MAT_Original_image, MAT_myblur, Size(10, 10), Point(-1, -1), BORDER_DEFAULT);
	
	cvtColor(MAT_myblur, MAT_HSV, COLOR_BGR2HSV);	// change to HSV

	// red

	inRange(MAT_HSV, Scalar(0, 26, 0), Scalar(20, 255, 255), MAT_HSV_THRESHOLD_RED); 
	erode(MAT_HSV_THRESHOLD_RED, MAT_HSV_THRESHOLD_RED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
	dilate(MAT_HSV_THRESHOLD_RED, MAT_HSV_THRESHOLD_RED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	if (debug == 1) imshow("Red threshold", MAT_HSV_THRESHOLD_RED);


	// blue

	inRange(MAT_HSV, Scalar(103, 35, 0), Scalar(178, 204, 255), MAT_HSV_THRESHOLD_BLUE);	erode(MAT_HSV_THRESHOLD_BLUE, MAT_HSV_THRESHOLD_BLUE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(MAT_HSV_THRESHOLD_BLUE, MAT_HSV_THRESHOLD_BLUE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	if (debug == 1) imshow("Blue threshold", MAT_HSV_THRESHOLD_BLUE);


	// green
	
	inRange(MAT_HSV, Scalar(61, 23, 47), Scalar(82, 100, 206), MAT_HSV_THRESHOLD_GREEN);
	erode(MAT_HSV_THRESHOLD_GREEN, MAT_HSV_THRESHOLD_GREEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(MAT_HSV_THRESHOLD_GREEN, MAT_HSV_THRESHOLD_GREEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	if (debug == 1) imshow("Green threshold", MAT_HSV_THRESHOLD_GREEN);

	
	//yellow

	inRange(MAT_HSV, Scalar(23, 114, 0), Scalar(47, 255, 255), MAT_HSV_THRESHOLD_ROBOT); 
	erode(MAT_HSV_THRESHOLD_ROBOT, MAT_HSV_THRESHOLD_ROBOT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(MAT_HSV_THRESHOLD_ROBOT, MAT_HSV_THRESHOLD_ROBOT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(MAT_HSV_THRESHOLD_ROBOT, MAT_HSV_THRESHOLD_ROBOT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		
}

vector<Point2f>  lego_collection::Block_locations(Mat MAT_threshold, char debug_name[])	
{
	int	X_COORD = 0; // temporary coordinate
	int	Y_COORD = 0; // temporary coordinate

	//possibly more filtering
	Mat Overaly_gray = MAT_Gray.clone();
	vector<vector<Point> > contours;
	findContours(MAT_threshold.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	vector<Point> APPROX_POLYGON;

	number_of_blocks_color = 1;

	vector<Moments> Moment(contours.size());	// moment
	vector<Point2f> Block_Centroids_Container(contours.size());	// center of mass


	for (int i = 0; i < contours.size(); i++)
	{

		drawContours(Overaly_gray, contours, i, (0, 0, 255), 4, 8, vector<Vec4i>(), 0, Point());  
		
		approxPolyDP(Mat(contours[i]), APPROX_POLYGON, arcLength(Mat(contours[i]), true)*0.02, true);
		Moment[i] = moments(contours[i], false);
		if (Moment[i].m00 > 3)				// reduce noise
		{
					vector<Rect> boundRect(contours.size());
					vector<vector<Point> > contours_poly(contours.size());
					approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
					boundRect[i] = boundingRect(Mat(contours_poly[i]));
					Scalar color(0, 0, 255);
					drawContours(Overaly_gray, contours_poly, i, color, 4, 8, vector<Vec4i>(), 0, Point());
					// Find center point of rectangles
					X_COORD = Moment[i].m10 / Moment[i].m00;
					Y_COORD = Moment[i].m01 / Moment[i].m00;

					// delete objects in work zone
					if (X_COORD  > 320 && Y_COORD < 240){

						break;
					}
					Block_Centroids_Container[i] = Point2f(X_COORD, Y_COORD);	
					circle(Overaly_gray, Block_Centroids_Container[i], 4, Scalar(0, 255, 0), -1, 8, 0);					// draws center point 							
					std::string my_rectangle;
					std::stringstream my_number_of_blocks_color;
					my_number_of_blocks_color << number_of_blocks_color;
					number_of_blocks_color++;
					my_rectangle = "RECT " + my_number_of_blocks_color.str();
					putText(Overaly_gray, my_rectangle, Block_Centroids_Container[i], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

					// stores coordinates of blocks
					x_y_cordinate_storage[number_of_blocks_color][0] = X_COORD;
					x_y_cordinate_storage[number_of_blocks_color][1] = Y_COORD;
		}
	}
	if (debug == 1) imshow(debug_name, Overaly_gray);
	return Block_Centroids_Container;

}



void lego_collection::Robot_location(Mat MAT_threshold, char debug_name[])
{
	Mat Overaly_gray = MAT_Gray.clone();
	vector<vector<Point> > contours;
	findContours(MAT_threshold.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	vector<Point> APPROX_POLYGON;

	vector<Moments> robot_Moment(contours.size());	// moment
	vector<Point2f> robot_Centroid_temp(contours.size());	// center of mass


	for (int i = 0; i < contours.size(); i++)
	{

		drawContours(Overaly_gray, contours, i, (0, 0, 255), 4, 8, vector<Vec4i>(), 0, Point());		
		robot_Moment[i] = moments(contours[i], false);
		if (robot_Moment[i].m00 > 20)			
		{
			double area = cv::contourArea(contours[i]);
			cv::Rect r = cv::boundingRect(contours[i]);
			int radius = r.width / 2;

			if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
			{
					//// Draw circle
					vector<Rect> boundRect(contours.size());
					vector<vector<Point> > contours_poly(contours.size());
					approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
					boundRect[i] = boundingRect(Mat(contours_poly[i]));
					Scalar color(0, 0, 255);															
					drawContours(Overaly_gray, contours_poly, i, color, 2, 8, vector<Vec4i>(), 0, Point());			

					// Find center point of circle
					robot_Moment[i] = moments(contours[i], false);

					if (robot_Moment[i].m00 > 20 && robot_Moment[i].m00 <1000)								// for small circle
					{
					X_COORD_CIRCLE_front = robot_Moment[i].m10 / robot_Moment[i].m00;
					Y_COORD_CIRCLE_front = robot_Moment[i].m01 / robot_Moment[i].m00;
					if (X_COORD_CIRCLE_front == 0 || Y_COORD_CIRCLE_front == 0) break;
					robot_Centroid_temp[i] = Point2f(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front);						// coordinates
					circle(Overaly_gray, robot_Centroid_temp[i], 4, Scalar(0, 255, 0), -1, 8, 0);				
					std::string my_rbg;
					std::stringstream my_array_rbg;
					my_array_rbg << "Robot front";		
					my_rbg = my_array_rbg.str();
					putText(Overaly_gray, my_rbg, robot_Centroid_temp[i], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
					}

					if (robot_Moment[i].m00 > 1000)	// for large circle
					{
					X_COORD_CIRCLE_back = robot_Moment[i].m10 / robot_Moment[i].m00;
					Y_COORD_CIRCLE_back = robot_Moment[i].m01 / robot_Moment[i].m00;
					robot_Centroid_temp[i] = Point2f(X_COORD_CIRCLE_back, Y_COORD_CIRCLE_back);						// coordinates
					circle(Overaly_gray, robot_Centroid_temp[i], 4, Scalar(0, 255, 0), -1, 8, 0);						// draws center point
						
					}
			}
		}
			
	}
	if (debug == 1) imshow(debug_name, Overaly_gray); 
}

void lego_collection::Closest_block_distance(vector<Point2f> Block_Centroid)
{
	int tempDISTANCE_TO_BLOCK;

	for (int n = 0; n < Block_Centroid.size(); n++){

		if (Block_Centroid[n].x != 0 || Block_Centroid[n].y != 0){
			tempDISTANCE_TO_BLOCK = sqrt((pow((Block_Centroid[n].x - X_COORD_CIRCLE_front), 2)) + (pow((Block_Centroid[n].y - Y_COORD_CIRCLE_front), 2)));	

			if (DISTANCE_TO_BLOCK > tempDISTANCE_TO_BLOCK)
			{
				DISTANCE_TO_BLOCK = tempDISTANCE_TO_BLOCK;
				Closest_Block_PT = Block_Centroid[n];
			}
		}
	}
	
}

void lego_collection::Update_block_distance(Point Block_Centroid)
{
	
	DISTANCE_TO_BLOCK = sqrt((pow((Block_Centroid.x - X_COORD_CIRCLE_front), 2)) + (pow((Block_Centroid.y - Y_COORD_CIRCLE_front), 2)));		
	
}

void lego_collection::Find_closest_block(int block_color){

	// IGNORE DROP OFF ZONE LOCATIONS
	Robot_location(MAT_HSV_THRESHOLD_ROBOT, "robot");

	// reset block distance
	DISTANCE_TO_BLOCK = 100000;

	if (block_color == 0)
	{
		Closest_block_distance(Block_locations(MAT_HSV_THRESHOLD_RED, "red"));
		// check distance with temp var
		cout << "locating closest red block" << endl;
	}
	else if (block_color == 2)
	{
		Closest_block_distance(Block_locations(MAT_HSV_THRESHOLD_GREEN, "Green"));
		// check distance with temp var
		cout << "locating closest green block" << endl;
	}
	else if (block_color == 1)
	{
		Closest_block_distance(Block_locations(MAT_HSV_THRESHOLD_BLUE, "Blue"));
		// check distance with temp var
		cout << "locating closest blue block" << endl;
	}
	else {
		cout << "invalid color choice" << endl;
	}

	line_image = MAT_Gray.clone();

	//draw line to closest block

	cout << "closest block at:\t" << Closest_Block_PT << endl;
	line(line_image, Closest_Block_PT, Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), Scalar(0, 255, 0), 4, 8, 0);	// closest block to robot

	imshow("Line image", line_image);
}

char lego_collection::Closest_block_angle()
{

	Point v1 = Closest_Block_PT;
	Point v2 = Closest_Block_PT;

	v1.x = v1.x - X_COORD_CIRCLE_front;
	v1.y = v1.y - Y_COORD_CIRCLE_front;
	v2.x = v2.x - X_COORD_CIRCLE_back;
	v2.y = v2.y - Y_COORD_CIRCLE_back;

float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    	float dot = v1.x * v2.x + v1.y * v2.y;

     	Angle_DIFF = dot / (len1 * len2);	

	Angle_DIFF = atan2((float)v1.y, (float)v1.x) - atan2((float)v2.y, (float)v2.x);
	
	if (Angle_DIFF < 0) Angle_DIFF += 2 * CV_PI;

	Angle_DIFF = Angle_DIFF * (180.0 / CV_PI);

	if (len2 < len1){
		 backwards = true;
	 }
	
	return '0';
}

char lego_collection::Angle_to_DROPZONE(float xcord, float ycord)
{

	Point v1;
	Point v2;

	v1.x = xcord - X_COORD_CIRCLE_front;
	v1.y = ycord - Y_COORD_CIRCLE_front;
	v2.x = xcord - X_COORD_CIRCLE_back;
	v2.y = ycord - Y_COORD_CIRCLE_back;

	float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

	float dot = v1.x * v2.x + v1.y * v2.y;

	Angle_DIFF = dot / (len1 * len2);

	Angle_DIFF = atan2((float)v1.y, (float)v1.x) - atan2((float)v2.y, (float)v2.x);

	if (Angle_DIFF < 0) Angle_DIFF += 2 * CV_PI;

	Angle_DIFF = Angle_DIFF * (180.0 / CV_PI);

	if (len2 < len1){
		backwards = true;
	}

	return '0';
}


char lego_collection::Move_robot(int block_location){

	Robot_location(MAT_HSV_THRESHOLD_ROBOT, "robot");

	Update_block_distance(Closest_Block_PT);

	line_image = MAT_Gray.clone();

	backwards = false; //reset this every loop

	//draw line to closest block	
	if (Block_in_claw == false){

		if (home == true){
			Update_block_distance(Point(500, 350));
			Angle_to_DROPZONE(500, 350);
			line(line_image, Point(500, 350), Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), Scalar(0, 255, 0), 4, 8, 0);
			cout << "going to home location" << endl;
		}
		else{
			Closest_block_angle();
			line(line_image, Closest_Block_PT, Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), Scalar(0, 255, 0), 4, 8, 0);	// closest block to robot
		}

		// grid lines
		line(line_image, Point(320, 240), Point(640, 240), Scalar(0, 255, 0), 4, 8, 0);
		line(line_image, Point(320, 0), Point(320, 480), Scalar(0, 255, 0), 4, 8, 0);

		// text on robot (distance and angle)
		std::string my_rbg;
		std::stringstream my_array_rbg;
		my_array_rbg << DISTANCE_TO_BLOCK;									my_rbg = my_array_rbg.str();

		std::string my_rbg2;
		std::stringstream my_array_rbg2;
		my_array_rbg2 << Angle_DIFF;
		my_rbg2 = my_array_rbg2.str();

		putText(line_image, my_rbg, Point(X_COORD_CIRCLE_back, Y_COORD_CIRCLE_back), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
		putText(line_image, my_rbg2, Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

		imshow("Line image", line_image);

		if (DISTANCE_TO_BLOCK >= 120){

			block_found = 1;

			//angle robot to closest block
			if ((Angle_DIFF > 5) && (Angle_DIFF < 50) && Go_FWD == false)
			{
				cout << "positioning - right" << endl;
				return 'j'; //fast turn
			}
			else if ((Angle_DIFF > 310) && (Angle_DIFF < 355) && Go_FWD == false)
			{
				cout << "positioning - left" << endl;
				return 'l'; //fast turn
			}

			if ((Angle_DIFF > 0.5) && (Angle_DIFF < 5) && Go_FWD == false)
				{
					cout << "positioning - right" << endl;
					return 'd'; //slow turn
				}
			else if ((Angle_DIFF > 355) && (Angle_DIFF < 359.5) && Go_FWD == false)
				{
					cout << "positioning - left" << endl;
					return 'a'; //slow turn
				}

			if (((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && backwards == true)
			{
				return 'd'; //spin robot if directly backwards
			}

			if (((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && backwards==false)
			{
				Go_FWD = true;
			}

			//move robot forward
			if (Go_FWD == true)
			{
				cout << "booking it!" << endl;
				return 'i';
			}
			
		}

		
		if (DISTANCE_TO_BLOCK >= 80 && DISTANCE_TO_BLOCK < 120){

			block_found = 1;

			if (!flag){
				Go_FWD = false; //reset Go_FWD to allow re-positioning
				flag = 1;
			}

			if (home == true){
				Block_in_claw = false;
				home = false;
				flag = 0;
				Go_FWD = false;
				return '0'; //dont try to pickup when going home
			}

			//angle robot to closest block
			if ((Angle_DIFF > 0.5) && (Angle_DIFF < 50) && Go_FWD == false)
			{
				//cout << "positioning - right" << endl;
				return 'd';
			}
			else if ((Angle_DIFF > 310) && (Angle_DIFF < 359.5) && Go_FWD == false)
			{
				//cout << "positioning - left" << endl;
				return 'a';
			}

			if (((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && backwards==false)
			{
				Go_FWD = true;
			}

			//move robot forward
			if (Go_FWD == true)
			{
				//cout << "on the move!" << endl;
				return 'w';
			}

		}
		

		if (DISTANCE_TO_BLOCK > 70 && DISTANCE_TO_BLOCK < 80)  {

			block_found = 1;

			// FINE TUNE ANGLE
			if ((Angle_DIFF > 0.5) && (Angle_DIFF < 50) && angle_counter<5 && home==false)
			{
				return 'h';
			}
			else if ((Angle_DIFF > 310) && (Angle_DIFF < 359.5) && angle_counter<5 && home == false)
			{
				return 'f';
			}
			
			if (angle_counter<5 && ((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && home == false)
			{
				cout << "locked on! #" << angle_counter + 1 << endl;
				angle_counter++;
				return '0'; //send a void command to arduino
			}

			if (angle_counter >= 5 && home == false){ //ensures angle is small
				cout << "final lock!" << endl;
				cout << "Picking up block!" << endl;
				angle_counter = 0;
				flag = 0;
				Go_FWD = false;
				Block_in_claw = true;
				home = true;
				return 'g'; //pick up block
				
			}

		}

		if (DISTANCE_TO_BLOCK <= 70)  {
			return 'y'; //fine backup
		}

	}

	if (Block_in_claw == true){

		cout << "i have a block" << endl;

		if (home == true){
			Update_block_distance(Point(coord_array[block_location][0]+50, 350));
			Angle_to_DROPZONE(coord_array[block_location][0]+50, 350);
			line(line_image, Point(coord_array[block_location][0]+50, 350), Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), Scalar(0, 255, 0), 4, 8, 0);
			cout << "going to home location" << endl;
		}
		else {
			Update_block_distance(Point(coord_array[block_location][0], coord_array[block_location][1]));
			Angle_to_DROPZONE(coord_array[block_location][0], coord_array[block_location][1]);
			line(line_image, Point(coord_array[block_location][0], coord_array[block_location][1]), Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), Scalar(0, 255, 0), 4, 8, 0);
			cout << "going to dropoff location #" << block_location << endl;
		}
		

		//draw lines
		line(line_image, Point(320, 240), Point(640, 240), Scalar(0, 255, 0), 4, 8, 0);
		line(line_image, Point(320, 0), Point(320, 480), Scalar(0, 255, 0), 4, 8, 0);

		//text on robot (distance and angle)
		std::string my_rbg;
		std::stringstream my_array_rbg;
		my_array_rbg << DISTANCE_TO_BLOCK;						
		my_rbg = my_array_rbg.str();

		std::string my_rbg2;
		std::stringstream my_array_rbg2;
		my_array_rbg2 << Angle_DIFF;
		my_rbg2 = my_array_rbg2.str();

		putText(line_image, my_rbg, Point(X_COORD_CIRCLE_back, Y_COORD_CIRCLE_back), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
		putText(line_image, my_rbg2, Point(X_COORD_CIRCLE_front, Y_COORD_CIRCLE_front), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

		imshow("Line image", line_image);

		if (DISTANCE_TO_BLOCK >= 100){

			block_found = 1;

			//angle robot to dropzone
			if (home == true){
				if ((Angle_DIFF > 5) && (Angle_DIFF < 50) && Go_FWD == false)
				{
					cout << "positioning - right" << endl;
					return 'j'; //fast turn
				}
				else if ((Angle_DIFF > 310) && (Angle_DIFF < 355) && Go_FWD == false)
				{
					cout << "positioning - left" << endl;
					return 'l'; //fast turn
				}

				if ((Angle_DIFF > 0.5) && (Angle_DIFF < 5) && Go_FWD == false)
				{
					cout << "positioning - right" << endl;
					return 'd'; //slow turn
				}
				else if ((Angle_DIFF > 355) && (Angle_DIFF < 359.5) && Go_FWD == false)
				{
					cout << "positioning - left" << endl;
					return 'a'; //slow turn
				}
			}
			else{

				if ((Angle_DIFF > 0.5) && (Angle_DIFF < 50) && Go_FWD == false)
				{
					cout << "positioning - right" << endl;
					return 'd'; //slow turn
				}
				else if ((Angle_DIFF > 310) && (Angle_DIFF < 359.5) && Go_FWD == false)
				{
					cout << "positioning - left" << endl;
					return 'a'; //slow turn
				}

			}

			if (((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && backwards == true)
			{
				return 'd'; //spin robot if directly backwards
			}

			if (((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && backwards==false)
			{
				Go_FWD = true;
			}

			//move robot forward
			if (Go_FWD == true)
			{
				cout << "booking it!" << endl;
				return 'i';
			}

		}

		if (DISTANCE_TO_BLOCK > 42 && DISTANCE_TO_BLOCK < 100){

			block_found = 1;

			if (!flag){
				Go_FWD = false; //reset Go_FWD to allow re-positioning
				flag = 1;
			}

			if (home == true){
				Block_in_claw = true;
				home = false;
				Go_FWD = false;
				flag = 0;
				return '0'; //send a void command to arduino
			}

			//angle robot to dropzone
			if ((Angle_DIFF > 0.5) && (Angle_DIFF < 50) && Go_FWD == false)
			{
				return 'd';
			}
			else if ((Angle_DIFF > 310) && (Angle_DIFF < 359.5) && Go_FWD == false)
			{
				return 'a';
			}

			if (((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && backwards==false)
			{
				Go_FWD = true;
			}

			//move robot forward
			if (Go_FWD == true)
			{
				cout << "on the move!" << endl;
				return 'w';
			}

		}


		if (DISTANCE_TO_BLOCK <= 42 && DISTANCE_TO_BLOCK >= 38) 
		{

			// FINE TUNE ANGLE
			if ((Angle_DIFF > 0.5) && (Angle_DIFF < 50) && angle_counter<5 && home==false)
			{
				return 'h';
			}
			else if ((Angle_DIFF > 310) && (Angle_DIFF < 359.5) && angle_counter<5 && home == false)
			{
				return 'f';
			}

			if (angle_counter<5 && ((Angle_DIFF <= 0.5) || (Angle_DIFF >= 359.5)) && home == false)
			{
				cout << "locked on! #" << angle_counter + 1 << endl;
				angle_counter++;
				return '0'; //send a void command to arduino
			}

			if (angle_counter >= 5 && home == false){ //ensures angle is small
				cout << "dropping off lego, great success!" << endl;
				angle_counter = 0;
				flag = 0;
				block_found = 0;
				Go_FWD = false;
				Block_in_claw = false;
				home = true;
				block_number++; //increment block number 
				return 'r'; //drop off block
			}
		}

		if (DISTANCE_TO_BLOCK < 38){
			return 'y'; //fine backup
		}

	}

	cout << "NO COMMAND, IM LOST!!!" << endl;

	return 'q';
	
}

////////////////////		MAIN LOOP		//////////////////////////////////////

int edgeThresh = 1;
int lowThreshold = 10;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";

int main()
{
	//time variables
	double t_clock;
	double t0;
	static int init = 0;

	//set the block number
	cout << "Which block would you like to start with ? ";
	cin >> initial_block_number;
	cout << "Starting at block " << initial_block_number << "\n";

	//array starts at 0 so correct human input.
	initial_block_number = initial_block_number - 1;

	lego_collection Blocks;	//Object initialization

	bluetooth *panther; //pointer for dynamic object

	panther = new bluetooth(8); //dynamic object

	if (panther == NULL) {
		printf("\nerror creating dynamic panther");
		return 0;
	}

	//debug variables
	bool debugMode = false;
	bool trackingEnabled = false;
	bool pause = false; //pause and resume code
	int counter = 0;

	// open webcam
	VideoCapture capWebcam(0);		
	cout << "starting code  " << endl;
	if (!capWebcam.isOpened())  { cout << "Cannot open webcam/n"; return -1; }


	/////////////////////////Continuous Loop/////////////////
	while (1)
	{

		if (!init){
			t0 = clock();
			init = 1;
		}

		t_clock = clock() - t0; // time since start of program

		bool bSuccess = capWebcam.read(Blocks.MAT_Original_image);	
		if (!bSuccess) { cout << "Cannot read a frame from video/n"; break; }
	
		Mat dst = Blocks.MAT_Gray.clone();
		Mat test1 = Blocks.MAT_Gray.clone();

		if (!Blocks.block_found || debug){
			Blocks.Image_Filtering();
						Blocks.Find_closest_block(Blocks.dropoff_array[Blocks.block_number][0]);
			Blocks.block_found = 1;
		}
		else{
			Blocks.update_image();

		}

		
		
		if (Blocks.block_number == 5 || Blocks.block_number == 10 || Blocks.block_number==15 || Blocks.block_number==20){

			if (panther->drop_fix==false){
				cout << "Reconnecting the robot for safety!" << endl;
				if (panther != NULL) {
					delete panther;
					panther = NULL;
				}
				else {
				cout << "<<<error deleting dynamic panther >>>" << endl;
				}

				panther = new bluetooth(14); //dynamic object

				if (panther == NULL) {
				cout << "<<<error creating dynamic panther >>>" << endl;
					return 0;
				}
			}
			panther->drop_fix = true;
		}
		
		if (Blocks.block_number == 6 || Blocks.block_number == 11 || Blocks.block_number==16 || Blocks.block_number==21){
			panther->drop_fix = false; // reset this so it will reconnect again 
		}

	////////	/////////Char sent to Robot/////////////////////////////////////
		if (t_clock > 80) { //time delay line 1

			panther->outputchar[0] = Blocks.Move_robot(Blocks.dropoff_array[Blocks.block_number][2]);

			if (Manual_CTRL == false){

				if (panther != NULL){ //only try to write when panther exists
					WriteFile(panther->hSerial, panther->outputchar, strlen(panther->outputchar), &panther->btsIO, NULL);
				}
			}

			t0 = clock(); //time delay line 2
		} 


	/////////////////////Switch for exiting program////////////////////////////
		int keypress;
		keypress = waitKey(10);

		switch (keypress){

		case 27: //'esc' key has been pressed, exit program.

			if (panther != NULL) {
				delete panther;
				return 0;
			}
			else {
				printf("\nerror deleteing dynamic panther");
				return 0;
			}
			
		case 'q': //'esc' key has been pressed, exit program.

			if (panther != NULL) {
				delete panther;
				return 0;
			}
			else {
				printf("\nerror deleteing dynamic panther");
				return 0;
			}
		case 'z': //'esc' key has been pressed, exit program.

			cout << "Reconnecting the robot for safety!" << endl;
			if (panther != NULL) {
				delete panther;
				panther = NULL;
			}
			else {
				cout << "<<<error deleting dynamic panther >>>" << endl;
			}

			panther = new bluetooth(14); //dynamic object

			if (panther == NULL) {
				cout << "<<<error creating dynamic panther >>>" << endl;
				return 0;
			}

		case '1': // toggle Manual mode
				Manual_CTRL = ! Manual_CTRL; 
				cout << "Manual_CTRL =  " << Manual_CTRL << endl;
				
				break;


		case '2':
				cout << "Block_in_claw =  " << Blocks.Block_in_claw << endl;
				Blocks.Block_in_claw = !Blocks.Block_in_claw;

				break;
		
			default:
				if (keypress > 10){
					panther->outputchar[0] = keypress;
					keypress = 0;

					if (panther != NULL){											WriteFile(panther->hSerial, panther->outputchar, strlen(panther->outputchar), &panther->btsIO, NULL);
					}
				}
				break;


		}
	

	} ///////end of while loop

	if (panther != NULL) {
		delete panther;
		return 0;
	}
	else {
		printf("\nerror deleting dynamic panther");
		return 0;
	}

}
