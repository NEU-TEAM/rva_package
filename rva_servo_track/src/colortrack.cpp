#include "colortrack.h"

using namespace cv;

ColorTrack::ColorTrack()
{
}

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 400;
const int MAX_OBJECT_AREA =  204800;
//names that will appear at the top of each window
const string windowName = "Thresholded Image";
const string trackbarWindowName = "Trackbars";

void on_trackbar( int, void* )
{//This function gets called whenever a
		// trackbar position is changed
}

void createTrackbars(){
		//create window for trackbars

		namedWindow(trackbarWindowName,0);
		//create memory to store trackbar name on window
		char TrackbarName[50];
		sprintf( TrackbarName, "H_MIN", H_MIN);
		sprintf( TrackbarName, "H_MAX", H_MAX);
		sprintf( TrackbarName, "S_MIN", S_MIN);
		sprintf( TrackbarName, "S_MAX", S_MAX);
		sprintf( TrackbarName, "V_MIN", V_MIN);
		sprintf( TrackbarName, "V_MAX", V_MAX);
		//create trackbars and insert them into window
		//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
		//the max value the trackbar can move (eg. H_HIGH),
		//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
		//                                  ---->    ---->     ---->
		createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
		createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
		createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
		createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
		createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
		createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}


void morphOps(Mat &thresh)
{
		//create structuring element that will be used to "dilate" and "erode" image.
		//the element chosen here is a 3px by 3px rectangle

		Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
		//dilate with larger element so make sure object is nicely visible
		Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

		erode(thresh,thresh,erodeElement);
		erode(thresh,thresh,erodeElement);

		dilate(thresh,thresh,dilateElement);
		dilate(thresh,thresh,dilateElement);
}

bool trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

		Mat temp;
		threshold.copyTo(temp);
		//these two vectors needed for output of findContours
		vector< vector<Point> > contours;
		vector<Vec4i> hierarchy;
		//find contours of filtered image using openCV findContours function
		findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
		//use moments method to find our filtered object
		double refArea = 0;
		bool objectFound = false;
		if (hierarchy.size() > 0)
				{
						int numObjects = hierarchy.size();
						//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter

						for (int index = 0; index >= 0; index = hierarchy[index][0])
								{
										Moments moment = moments((cv::Mat)contours[index]);
										double area = moment.m00;

										//if the area is less than 20 px by 20px then it is probably just noise
										//if the area is the same as the 3/2 of the image size, probably just a bad filter
										//we only want the object with the largest area so we safe a reference area each
										//iteration and compare it to the area in the next iteration.
										if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea)
												{
														x = moment.m10/area;
														y = moment.m01/area;
														objectFound = true;
														refArea = area;
												}
								}

						if(objectFound == true)
								{
										putText(cameraFeed, "Tracking Object", Point(0,50),2,1,Scalar(0,255,0),2);
										//draw object location on screen
										Utilities::drawObject(x, y, cameraFeed);
								}
						else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
				}
		return objectFound;
}


bool ColorTrack::process(cv::Mat img_in, cv::Mat &img_out, int &x_out, int &y_out)
{
		//matrix storage for HSV image
		Mat HSV;
		//matrix storage for binary threshold image
		Mat threshold;

		//convert frame from BGR to HSV colorspace
		cvtColor(img_in,HSV, COLOR_BGR2HSV);

		inRange(HSV, Scalar(33, 55, 0), Scalar(78, 256, 256), threshold);

		morphOps(threshold);

		bool found = trackFilteredObject(x_out, y_out, threshold, img_out);

		//show frames
		imshow(windowName,threshold);

		waitKey(10);
		return found;
}
