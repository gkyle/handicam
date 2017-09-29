#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <sys/ioctl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>

#ifndef CONFIG
#define CONFIG

using namespace cv;
using namespace std;

class Config {
  public:
    Config(const string filename="config.xml");
  
    void setv4l();

    void savev4l();

    string config_file;
  
    int video_source = 0;

    int image_width = 1280;

    int image_height = 1024;

    string calibration_file = "";
    
    float markerboard_width = 6.0;

    float markerboard_height = 8.0;

    float markerboard_offset = 0.0;

    float markerboard_project_width = 6.0;

    float markerboard_project_height = 8.0;

    int focus_auto = 0;

    int focus_absolute = 40;

    int exposure_auto = 1;

    int exposure_absolute = 700;

    int zoom_absolute = 100;

    Mat cameraMatrix;
  
    Mat distCoeffs;
  
 private:
    void getCameraProfile(string filename);
  
    float getFloat(string name, FileStorage &fs);

    int getInt(string name, FileStorage &fs);

    string getString(string name, FileStorage &fs);
};

#endif
