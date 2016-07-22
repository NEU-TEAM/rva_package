#include "camshift.h"

using namespace cv;
using namespace std;

//minimum and maximum object area
const int MIN_OBJECT_AREA = 400;
const int MAX_OBJECT_AREA =  204800;

CamShift::CamShift()
{
		hue_size_ = 30;
		sat_size_ = 32;

		sat_min_ = 90;

		val_min_ = 50;
		val_max_ = 244;

		hue_range_[0] = 0;
		hue_range_[1] = 180;

		sat_range_[0] = 0;
		sat_range_[1] = 256;

		pranges_[0] = hue_range_;
		pranges_[1] = sat_range_;

		tracker_initialized = false;
}

RotatedRect CamShift::camshift_init (Mat img, Rect detection)
{
		Mat hsv, backproj, mask;
		const int hist_size[] = {hue_size_, sat_size_};
		int ch[2] = {0, 1};

		Mat img_b;
		blur(img, img_b, cv::Size(5, 5));
		cvtColor(img_b, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(0, sat_min_, val_min_), Scalar(256, 256, val_max_), mask);// black pixels are masked

		Mat roi(hsv, detection);
		Mat maskroi(mask, detection);

		calcHist(&roi, 1, ch, maskroi, hist_, 1, hist_size, pranges_);
		normalize(hist_, hist_, 0, 255, CV_MINMAX);

		calcBackProject(&hsv, 1, ch, hist_, backproj, pranges_);
		backproj &= mask;
//		imshow("sls",backproj);
//		waitKey(10);

		last_trackbox_ = cv::CamShift(backproj, detection,TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

		last_detection_ = last_trackbox_.boundingRect ();

		tracker_initialized = true;
		return last_trackbox_;
}

RotatedRect CamShift::camshift_track (Mat img)
{
		Mat hsv, backproj, mask;
		int ch[2] = {0, 1};

		cvtColor(img, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(0, sat_min_, val_min_), Scalar(256, 256, val_max_), mask);

		calcBackProject(&hsv, 1, ch, hist_, backproj, pranges_);
		backproj &= mask;

		Rect detection = last_trackbox_.boundingRect ();
		last_trackbox_ = cv::CamShift(backproj, detection, TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
		return last_trackbox_;
}

bool CamShift::initialized ()
{
		return tracker_initialized;
}

bool CamShift::process(Mat img_in, Rect detection, Mat &img_out, int &center_x, int &center_y, RotatedRect &roi)
{
		if (tracker_initialized)
				{
						roi = camshift_track (img_in);
				}
		else
				{
						roi = camshift_init(img_in, detection);
				}

		// draw rotate rect on image
		Point2f vertices[4];
		roi.points(vertices);

		center_x = roi.center.x;
		center_y = roi.center.y;

		if (roi.boundingRect().area() < MIN_OBJECT_AREA || roi.boundingRect().area() > MAX_OBJECT_AREA)
				{
						return false;
				}

		Utilities::drawObject(center_x, center_y, img_out);

		for (int i = 0; i < 4; i++)
				{
						line(img_out, vertices[i], vertices[(i+1)%4], Scalar(232,228,53), 2);
				}

		waitKey(20);
		return true;
}
