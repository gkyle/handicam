#include <sys/ioctl.h>
#include "util.hpp"

// Lines AB, CD
Point2f intersect(Point2f a, Point2f b, Point2f c, Point2f d) {
  float m1 = (b.y-a.y) / (b.x-a.x);
  float c1 = a.y-m1*a.x;

  float m2 = (d.y-c.y) / (d.x-c.x);
  float c2 = c.y-m2*c.x;

  float x = (c2-c1)/(m1-m2);
  float y = m1 * x + c1;

  return Point2f(x, y);
}

Mat imscale(int width, Mat img) {
  Mat imgScale;
  float scale = (float)width / (float)img.cols;
  resize(img, imgScale, Size(), scale, scale, INTER_LINEAR);
  return imgScale;
}

UMat imscale(int width, UMat uimg) {
  return imscale(width, uimg.getMat(ACCESS_READ)).getUMat(ACCESS_READ);
}

double getTime(){
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    //  Handle error
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void getCameraProfile(int W, Mat& cameraMatrix, Mat& distCoeffs) {
  std::string filename = "./calib/training-" + std::to_string(W) + "/out_webcam_camera_data.xml";
  FileStorage fs(filename, FileStorage::READ);

  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  fs.release();
}

float getAngle(Matx33f H) {
  double t = atan(H(1,0) / H(0,0));
  double deg = t * (180/3.1415926535897);
  return deg * -1;
}

float getScale(Matx33f H) {
  float a = H(0,0);
  float b = H(0,1);
  float s = sqrt(pow(a, 2) + pow(b, 2));
  return s;
}

void drawText(UMat img, string text, int bottom) {
  int fontFace = FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.5 * img.cols / 600; // compensate for rescaling for UI.
  int thickness = 2.0 * img.cols/800;

  int baseline = 0;
  Size textSize = getTextSize(text, fontFace,
			      fontScale, thickness, &baseline);

  // Center the text at bottom with vertical offset.
  Point textOrg((img.cols - textSize.width)/2,
		(img.rows - (textSize.height*bottom*2)-20));
  putText(img, text, textOrg, fontFace, fontScale,
	  Scalar(255, 0, 0), thickness, 8);
}

void drawGridText(UMat img, Rect r, string text, int bottom) {
  int fontFace = FONT_HERSHEY_SIMPLEX;
  double fontScale = 10.0; // compensate for rescaling for UI.
  int thickness = 20.0;

  int baseline = 0;
  Size textSize = getTextSize(text, fontFace,
			      fontScale, thickness, &baseline);

  // Center the text at bottom with vertical offset.
  Point textOrg(r.x + (r.width - textSize.width)/2,
		r.y + (r.height - (textSize.height*3)-20));
  putText(img, text, textOrg, fontFace, fontScale,
	  Scalar(255, 0, 0), thickness, 8);
}

void drawGridTextSmall(UMat img, Rect r, string text, int bottom, int mode) {
  int fontFace = FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.0 * (r.width/(double)600); // compensate for rescaling for UI.
  int thickness = 3.0 * (r.width/(double)600);

  int baseline = 0;
  Size textSize = getTextSize(text, fontFace,
			      fontScale, thickness, &baseline);

  Scalar color = mode == 0 ? Scalar(0, 255, 0) : Scalar(255, 255, 0);
  
  // Center the text at bottom with vertical offset.
  Point textOrg(r.x + (r.width - textSize.width)/2,
		r.y + (r.height - (textSize.height*bottom)));
  putText(img, text, textOrg, fontFace, fontScale,
	  color, thickness, 8);
}

void saveOffsets(float gx, float gy) {
  FileStorage fs("save.xml", FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "gx" << gx;
    fs << "gy" << gy;
    fs.release();
  } else {
    LOG(ERROR) << "Failed to load save file...." << endl;
  }
}

void getOffsets(float &gx, float &gy) {
  FileStorage fs("save.xml", FileStorage::READ);
  if (fs.isOpened()) {
    fs["gx"] >> gx;
    fs["gy"] >> gy;
    fs.release();
  } else {
    LOG(ERROR) << "Failed to load config file...." << endl;
  }
}

double angle(Mat R) {
  double t = atan(R.at<float>(1,0) / R.at<float>(0,0));
  double deg = t * (180/3.1415926535897);
  return deg * -1;
}
