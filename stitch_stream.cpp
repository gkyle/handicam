#include <math.h>

#include "util.hpp"
#include "markers.hpp"
#include "config.hpp"
#include "stitcher.hpp"
#include "source.hpp"

using namespace std;
using namespace cv;

vector<double> dmtime;

void showError(string error, UMat img) {
  LOG(ERROR) << error << endl;
  drawText(img, error, 2);
}

void stats() {
  int count = dmtime.size();
  double max=-1;
  double min=100000;
  double sum=0;
  double mean=0;
  for (int i=0; i<count; i++) {
    sum += dmtime[i];
    if (dmtime[i] > max) max = dmtime[i];
    if (dmtime[i] < min) min = dmtime[i];
  }
  mean = sum / (double)count;
  LOG(INFO) << "MAX: " << max << endl;
  LOG(INFO) << "MIN: " << min << endl;
  LOG(INFO) << "MEAN: " << mean << endl;
}

IncrementalStitcher::Status checkTransform(Mat R, IncrementalStitcher& stitcher, Config& config) {
  IncrementalStitcher::Status status = IncrementalStitcher::Status::OK;
  if (abs(R.at<float>(0,2)) > config.image_width/3) {
    status = IncrementalStitcher::Status::EXCEEDS_X_THRESHOLD_ERR;
  } else if (abs(R.at<float>(1,2)) > config.image_height/3) {
    status = IncrementalStitcher::Status::EXCEEDS_X_THRESHOLD_ERR;
  } else if (abs(angle(R)) > 15.0) {
    status = IncrementalStitcher::Status::EXCEEDS_X_THRESHOLD_ERR;
  }

  return status;
}

int main( int argc, char** argv ) {
  Config config;
  Markers markers(config, false);
  char key;
  namedWindow("Stitched Image");
  moveWindow("Stitched Image", 20,20);

  Source *source;
  if (argc==1) {
    VideoCapture cap;
    if(!cap.open(config.video_source)) {
      LOG(ERROR) << "Could not open camera." << endl;
      return 0;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, config.image_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, config.image_height);
    // TODO: Make this a command line option. Useful when capturing video for replay, but
    // otherwise unecessarily stalls startup.
    Mat junk;
    for (int i=0; i<60; i++) cap >> junk;
    config.setv4l(); // Exposure settings don't take unless we read some frames first.
    source = new VideoSource(cap);
  } else {
    vector<UMat> imgs;
    for(int i=1; i<argc; i++) {
      UMat img = imread( argv[i]).getUMat(ACCESS_READ);
      if (!img.cols) {
	LOG(ERROR) << "Bad file: " << argv[i] << endl;
	return -1;
      }
      LOG(INFO) << argv[i] << endl;
      imgs.push_back(img);
    }
    source = new ImageSource(imgs);
  }
  
  IncrementalStitcher stitcher(1.0,
			       IncrementalStitcher::MatchMode::PAIRWISE,
			       IncrementalStitcher::DetectMethod::DETECT_SURF,
			       IncrementalStitcher::ExtractMethod::EXTRACT_FREAK);
  UMat img1;
  Markers::Status status = Markers::Status::ERR;
  while (status != Markers::Status::OK) {
    status = source->nextImage(markers, img1);
    if (status != Markers::Status::OK) {
      UMat dummy = UMat::zeros(600, 600, CV_8UC3);
      showError(markers.getError(status), dummy);
      imshow("Stitched Image", dummy);
      key = waitKey(100);
    }
  }
  
  double t1, dt;
  int seq = 0;
  while (!source->done()) {
    UMat img2;
    t1 = getTime();
    status = source->nextImage(markers, img2);
    dt = (getTime() - t1);
    LOG(INFO) << "AM Time: " << dt << endl;

    UMat stitchedImg;
    if (status == 0) {
      Mat R;
      t1 = getTime();
      IncrementalStitcher::Status status = stitcher.detectAndMatch(img1, img2, R);
      dt = (getTime() - t1);
      dmtime.push_back(dt);
      LOG(INFO) << "DM Time: " << dt << endl;
      if (status == IncrementalStitcher::Status::OK &&
	  checkTransform(R, stitcher, config) == IncrementalStitcher::Status::OK) {
	stitcher.composeImages(img1, img2, R);
	stitcher.getNextBaseImage().copyTo(img1);
	stitchedImg = stitcher.getStitchedImage();
      } else {
	stitcher.getStitchedImage().copyTo(stitchedImg);
	showError(stitcher.getError(status), stitchedImg);
      }
    } else {
      stitcher.getStitchedImage().copyTo(stitchedImg);
      showError(markers.getError(status), stitchedImg);
    }

    if (stitchedImg.cols > 0) {
      imshow("Stitched Image", imscale(600, stitchedImg));
    }
    
    // Pause for any drawing to catch up.
    key = waitKey(100);
    if (key == 27) {
      break;
    }
  }
  LOG(INFO) << "done" << endl;
  stats();
  imwrite("stitched.jpeg", stitcher.getStitchedImage());
  LOG(INFO) << "Wrote file." << endl;
  key = (char) waitKey(0);
  
  return 0;
}

