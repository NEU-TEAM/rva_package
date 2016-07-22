#include "utilities.h"

using namespace cv;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

Utilities::Utilities()
{
}

std::string Utilities::intToString(int number)
{
		std::stringstream ss;
		ss << number;
		return ss.str();
}

void Utilities::drawObject(int x, int y, Mat &frame)
{
		//use some of the openCV drawing functions to draw crosshairs
		//on your tracked image!

		circle(frame,Point(x,y),12,Scalar(0,255,0),2);
		if(y-25>0)
				line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
		else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
		if(y+25<FRAME_HEIGHT)
				line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
		else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
		if(x-25>0)
				line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
		else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
		if(x+25<FRAME_WIDTH)
				line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
		else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

		putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
}
