#ifndef __UTIL_H_INCLUDED__
#define __UTIL_H_INCLUDED__   

#include <opencv2/opencv.hpp>
#include <time.h>
#include <sys/time.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;

Point2f intersect(Point2f a, Point2f b, Point2f c, Point2f d);
Mat imscale(int width, Mat img);
UMat imscale(int width, UMat img);
void getCameraProfile(int W, Mat& cameraMatrix, Mat& distCoeffs);
double getTime();
void drawText(UMat img, string text, int bottom=0);
void drawGridText(UMat img, Rect r, string text, int bottom=0);
void drawGridTextSmall(UMat img, Rect r, string text, int bottom=0, int mode=0);
void saveOffsets(float gx, float gy);
void getOffsets(float &gx, float &gy);
float getAngle(Matx33f H);
float getScale(Matx33f H);

#endif
