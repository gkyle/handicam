#include "config.hpp"

using namespace std;
using namespace cv;

Config::Config(string filename) {
  google::InitGoogleLogging("");

  FileStorage fs(filename, FileStorage::READ);
  config_file = filename;
  
  if (fs.isOpened()) {
    video_source = getInt("video_source", fs);
    calibration_file = getString("calibration_file", fs);
    image_width = getInt("image_width", fs);
    image_height = getInt("image_height", fs);

    markerboard_width = getFloat("markerboard_width", fs);
    markerboard_height = getFloat("markerboard_height", fs);
    markerboard_offset = getFloat("markerboard_offset", fs);
    markerboard_project_width = getFloat("markerboard_project_width", fs);
    markerboard_project_height = getFloat("markerboard_project_height", fs);

    focus_auto = getInt("focus_auto", fs);
    focus_absolute = getInt("focus_absolute", fs);
    exposure_auto = getInt("exposure_auto", fs);
    exposure_absolute = getInt("exposure_absolute", fs);
    zoom_absolute = getInt("zoom_absolute", fs);
    
    getCameraProfile(calibration_file);

    fs.release();
  } else {
    cerr << "Failed to load config file...." << endl;
  }

  setv4l();
}

void Config::getCameraProfile(string filename) {
  FileStorage fs(filename, FileStorage::READ);
  if (fs.isOpened()) {
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    fs.release();
  } else {
    LOG(ERROR) << "Failed to load camera profile..." << endl;
  }
}

float Config::getFloat(string name, FileStorage &fs) {
  float value;
  if (fs[name].isNone() || fs[name].empty()) {
    LOG(ERROR) << "node \"" << name << "\" does not exist." << endl;
  } else if (fs[name].type() != FileNode::FLOAT) {
    LOG(ERROR) << "node \"" << name << "\" type is not FLOAT." << endl;
  } else {
    fs[name] >> value;
    LOG(INFO) << name << ": " << value << endl;
  }
  return value;
}

int Config::getInt(string name, FileStorage &fs) {
  int value;
  if (fs[name].isNone() || fs[name].empty()) {
    LOG(ERROR) << "node \"" << name << "\" does not exist." << endl;
  } else if (fs[name].type() != FileNode::INT) {
    LOG(ERROR) << "node \"" << name << "\" type is not INT." << endl;
  } else {
    fs[name] >> value;
    LOG(INFO) << name << ": " << value << endl;
  }
  return value;
}

string Config::getString(string name, FileStorage &fs) {
  string value;
  if (fs[name].isNone() || fs[name].empty()) {
    LOG(ERROR) << "node \"" << name << "\" does not exist." << endl;
  } else if (fs[name].type() != FileNode::STRING) {
    LOG(ERROR) << "node \"" << name << "\" type is not STRING." << endl;
  } else {
    fs[name] >> value;
    LOG(INFO) << name << ": " << value << endl;
  }
  return value;
}

void setParam(int descriptor, int id, int val) {
  v4l2_control c;
  c.id = id;
  c.value = val;
  if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
    LOG(INFO) << "Set v4l2: " << id << " to " << val << endl;
  } else {
    LOG(ERROR) << "Failed to set v4l2: " << id << " to " << val << endl;
  }    
}

void Config::setv4l() {
  int descriptor = v4l2_open("/dev/video0", O_RDWR);

  setParam(descriptor, V4L2_CID_FOCUS_AUTO, focus_auto);
  setParam(descriptor, V4L2_CID_FOCUS_ABSOLUTE, focus_absolute);
  setParam(descriptor, V4L2_CID_EXPOSURE_AUTO, exposure_auto);
  setParam(descriptor, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute);
  setParam(descriptor, V4L2_CID_ZOOM_ABSOLUTE, zoom_absolute);

  //setParam(descriptor, V4L2_CID_AUTO_WHITE_BALANCE, 0);
  //setParam(descriptor, V4L2_CID_WHITE_BALANCE_TEMPERATURE, 2000);

  v4l2_close(descriptor);
}

void Config::savev4l() {
  FileStorage fs(config_file, FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "video_source" << video_source;
    fs << "calibration_file" << calibration_file;
    fs << "image_width" << image_width;
    fs << "image_height" << image_height;

    fs << "markerboard_width" << markerboard_width;
    fs << "markerboard_height" << markerboard_height;
    fs << "markerboard_offset" << markerboard_offset;
    fs << "markerboard_project_width" << markerboard_project_width;
    fs << "markerboard_project_height" << markerboard_project_height;

    fs << "focus_auto" << focus_auto;
    fs << "focus_absolute" << focus_absolute;
    fs << "exposure_auto" << exposure_auto;
    fs << "exposure_absolute" << exposure_absolute;
    fs << "zoom_absolute" << zoom_absolute;
    fs.release();
  } else {
    LOG(ERROR) << "Failed to load config file...." << endl;
  }
  setv4l();
}
