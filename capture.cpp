#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include "util.hpp"
#include "config.hpp"
#include "markers.hpp"
#include "stitcher.hpp"

#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>

using namespace std;
using namespace cv;

int MAX = 10;
vector<float> ax, ay, ar;
void logStats(Matx33f aH) {
  ax.insert(ax.begin(), aH(0,2));
  ay.insert(ay.begin(), aH(1,2));
  ar.insert(ar.begin(), getAngle(aH));

  while (ax.size() > MAX) ax.pop_back();
  while (ay.size() > MAX) ay.pop_back();
  while (ar.size() > MAX) ar.pop_back();
}

void avg(vector<float> list, float& mean, float& range) {
  int count = list.size();
  float sum=0;
  mean=0;
  float min = 1000000;
  float max = -1000000;
  for (int i=0; i<count; i++) {
    sum += list[i];
    if (list[i] < min) min = list[i];
    if (list[i] > max) max = list[i];
  }
  mean = sum / (float)count;
  range = max - min;
}

int main(int argc, char** argv) {
  Config config;
  Markers markers(config);

  namedWindow("Capture");
  moveWindow("Capture", 20,20);

  VideoCapture cap;
  VideoWriter vw("capture_out.avi",  VideoWriter::fourcc('M','J','P','G'), 20.0,
		 Size(config.image_width, config.image_height), true);

  if(!cap.open(config.video_source)) {
    cerr << "Could not open camera." << endl;
    return 0;
  }
  cap.set(CV_CAP_PROP_FRAME_WIDTH, config.image_width);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, config.image_height);
    
  // TODO: Make this a command line option. Useful when capturing video for replay.
  Mat junk;
  for (int i=0; i<30; i++) cap >> junk;
  config.setv4l(); // Exposure settings don't take unless we read some frames first.

  UMat img, imgCopy, imgProj, lastProj;
  int sequence = 0;
  bool doProjection = false;
  bool doStability = false;
  bool drawMarkers = false;

  IncrementalStitcher stitcher(1.0,
			       IncrementalStitcher::MatchMode::AGGREGATE,
			       IncrementalStitcher::DetectMethod::DETECT_SURF,
			       IncrementalStitcher::ExtractMethod::EXTRACT_FREAK);
  while (true) {
    const int SKIP_FRAMES = 5; // Need to blow away buffered frames.
    for (int i=0; i<SKIP_FRAMES; i++) {
      //cap >> img;
      cap.grab();
      cap.retrieve(img);
      LOG(INFO) << img.cols << " x " << img.rows << endl;
      vw.write(img.getMat(ACCESS_READ));
    }
    img.copyTo(imgCopy);

    UMat imgProj;
    int status = markers.getArucoOrientedImage(imgCopy, imgProj, drawMarkers,
					       doProjection || doStability);
    if (doProjection && imgProj.cols > 0) {
      imshow("Projection", imscale(600, imgProj));
    }
    UMat scaleCopy = imscale(800, imgCopy);
    if (status==Markers::Status::OK &&  doStability && lastProj.cols > 0) {
      Mat R;
      IncrementalStitcher::Status status = stitcher.detectAndMatch(lastProj, imgProj, R);
      Matx33f H = R;
      logStats(H);
      float mx, my, mr;
      float rx, ry, rr;
      avg(ax, mx, rx);
      avg(ay, my, ry);
      avg(ar, mr, rr);
      //float dx = H(0,2);
      //float dy = H(1,2);
      //float dr = getAngle(H);
      char dbuf [7];
      sprintf(dbuf, "% 4.3f", rr);
      char xbuf [7];
      sprintf(xbuf, "% 4.3f", rx);
      char ybuf [7];
      sprintf(ybuf, "% 4.3f", ry);

      drawGridTextSmall(scaleCopy, Rect(scaleCopy.cols/4, scaleCopy.rows/4,
					scaleCopy.cols/2, scaleCopy.rows/2),
			"Rotation: " + string(dbuf) + "d", 0.0);
      drawGridTextSmall(scaleCopy, Rect(scaleCopy.cols/4, scaleCopy.rows/4,
					scaleCopy.cols/2, scaleCopy.rows/2),
			"X: " + string(xbuf) + "px", 4.0);
      drawGridTextSmall(scaleCopy, Rect(scaleCopy.cols/4, scaleCopy.rows/4,
					scaleCopy.cols/2, scaleCopy.rows/2),
			"Y: " + string(ybuf) + "px", 2.0);
    }
    drawText(scaleCopy, "+/- (F/f)ocus  (E/e)xposure  (Z/z)oom", 1.5);
    drawText(scaleCopy, "(C)apture  (P)rojection  (M)arkers  (S)tability");
    imshow("Capture", scaleCopy);
    lastProj = imgProj;
    
    char key = (char) cv::waitKey(10);
    if (key == 27) {
      break;
    } else if (key == 'p') {
      doProjection = !doProjection;
      if (doProjection) {
	namedWindow("Projection");
	moveWindow("Projection", 400,20);
      } else {
	cvDestroyWindow("Projection");
      }
    } else if (key == 's') {
      doStability = !doStability;
    } else if (key == 'm') {
      drawMarkers = !drawMarkers;
    } else if (key == 'c' || key == '\n') {
      char buf [3];
      sprintf(buf, "%03d", ++sequence);
      string filename = "calib-" + to_string(config.image_width) + "-" + buf + ".jpeg";
      LOG(INFO) << filename << endl;
      imwrite(filename, img);
    } else if (key == 'f') {
      config.focus_absolute -= 5;
      config.savev4l();
    } else if (key == 'F') {
      config.focus_absolute += 5;
      config.savev4l();
    } else if (key == 'e') {
      config.exposure_absolute -= 100;
      config.savev4l();
    } else if (key == 'E') {
      config.exposure_absolute += 100;
      config.savev4l();
    } else if (key == 'z') {
      config.zoom_absolute -= 10;
      config.savev4l();
    } else if (key == 'Z') {
      config.zoom_absolute += 10;
      config.savev4l();
    }
  }
}
