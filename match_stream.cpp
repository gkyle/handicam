#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <math.h>

#include "util.hpp"
#include "markers.hpp"
#include "config.hpp"
#include "stitcher.hpp"
#include "source.hpp"
#include "grid.hpp"

using namespace std;
using namespace cv;

void showError(string error, UMat img) {
  LOG(ERROR) << error << endl;
  drawText(img, error, 2);
}

class Projector {
  public:
  Projector(Matx33f _warp, float _borderx, float _bordery,
	    float _cpi, float _rpi) {
    warp = _warp;
    borderx = _borderx;
    bordery = _bordery;
    cpi = _cpi;
    rpi = _rpi;
  }
  Point2f pt(float x, float y, float bx=0.0f, float by=0.0f) {
    Point2f pt = Point2f(x+bx*borderx*cpi,
    			 y+by*bordery*rpi);
    Point3f hi = warp.inv() * pt;
    Point2f po(hi.x, hi.y);
    return po;
  }
  Matx33f warp;
  float borderx;
  float bordery;
  float cpi;
  float rpi;
};

int main( int argc, char** argv ) {
  Config config;
  Markers markers(config);
  char key;
  namedWindow("Stitched Image", WINDOW_NORMAL);
  resizeWindow("Stitched Image", 800, 800);
  moveWindow("Stitched Image", 20, 20);
  namedWindow("Cell", WINDOW_AUTOSIZE);
  moveWindow("Cell", 600, 20);
  namedWindow("Camera", WINDOW_AUTOSIZE);
  moveWindow("Camera", 800, 20);

  string filename = "stitched.jpeg";

  Source *source;
  if (argc==1) {
    VideoCapture cap;
    if(!cap.open(config.video_source)) {
      LOG(ERROR) << "Could not open camera." << endl;
      return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, config.image_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, config.image_height);
    source = new VideoSource(cap);
    // TODO: Do in Source
    Mat junk;
    for (int i=0; i<30; i++) cap >> junk;
    config.setv4l(); // Exposure settings don't take unless we read some frames first.
  } else {
    VideoCapture cap;
    if(!cap.open(argv[1])) {
      LOG(ERROR) << "Could not open file." << endl;
      return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, config.image_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, config.image_height);
    source = new VideoSource(cap);
    // burn 5 seconds ... auto exposure...
    // TODO: probably unecessary now.
    /*
    Mat junk;
    for (int i=0; i<5*30; i++) {
      cap >> junk;
    }
    */
    /*
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
    */
  }
  
  UMat stitchedImg = imread(filename).getUMat(ACCESS_READ);
  imshow("Stitched Image", imscale(800, stitchedImg));

  float matchScale = 1.0;
  IncrementalStitcher stitcher(matchScale,
			       IncrementalStitcher::MatchMode::AGGREGATE,
			       IncrementalStitcher::DetectMethod::DETECT_SURF,
			       IncrementalStitcher::ExtractMethod::EXTRACT_FREAK);

  const float markerboard_width_actual = (config.markerboard_width -
					  config.markerboard_offset*2.0f);
  const float markerboard_height_actual = (config.markerboard_height -
					   config.markerboard_offset*2.0f);
  
  float cols_per_inch = 186.0;
  float rows_per_inch = 186.0;

  // Burn a frame to set cpi & rpi.
  UMat img_base;
  source->nextImage(markers, img_base);
  cols_per_inch = (float)img_base.cols / markerboard_width_actual;
  rows_per_inch = (float)img_base.rows / markerboard_height_actual;
  LOG(INFO) << "cpi: " << cols_per_inch << endl;
  LOG(INFO) << "rpi: " << rows_per_inch << endl;
  
  Grid grid(3, 3,
	    markerboard_width_actual,
	    markerboard_height_actual,
	    config.markerboard_project_width,
	    config.markerboard_project_height,
	    cols_per_inch, rows_per_inch);

  grid.gx = 100.0;
  grid.gy = img_base.rows-100;
  getOffsets(grid.gx, grid.gy); // Read saved offsets from file.
  grid.handleGridChange(stitchedImg);
  
  bool gridMode  = true;
  bool nudgeMode  = false;
  bool moveMode = false;

  while (!source->done()) {
    UMat img2, cell, stitchedCopy;
    stitchedImg.copyTo(stitchedCopy);
    
    if (gridMode) {
      // For grid mode, gray out base image and replace roi with color.
      UMat stitchedGray;
      cvtColor(stitchedCopy, stitchedGray, CV_RGB2GRAY);
      cvtColor(stitchedGray, stitchedCopy, CV_GRAY2RGB);
      Rect roi = grid.getRoi();
      stitchedImg(roi).copyTo(stitchedCopy(roi));
    }
    grid.drawGrid(stitchedCopy);
      
    // If we're in move mode, skip.
    if (!moveMode) {
      Markers::Status istatus = source->nextImage(markers, img2);

      if (istatus == 0) {
      	Mat R;
	IncrementalStitcher::Status status;
	if (!gridMode) {
	  status = stitcher.detectAndMatch(stitchedCopy, img2, R);
	} else {
	  status = stitcher.detectAndMatch(stitchedImg(grid.getRoi()), img2, R);
	}
	
	if (status == IncrementalStitcher::Status::OK) {
	  Matx33f warp = R;
	  Matx33f useWarp = warp;
	  if (gridMode) {
	    // Offset by grid offset.
	    warp(0,2) -= grid.getCell().x;
	    warp(1,2) -= grid.getCell().y;
	    grid.store(warp);
	    if (nudgeMode) {
	      useWarp = grid.avgWarp();
	    } else {
	      useWarp = warp;
	    }

	    float x = R.at<float>(0,2);
	    float y = R.at<float>(1,2);
	    float mx, my, mr, rx, ry, rr;
	    grid.avg(grid.ax, mx, rx);
	    grid.avg(grid.ay, my, ry);
	    grid.avg(grid.ar, mr, rr);

	    grid.drawMetrics(stitchedCopy, useWarp, x/cols_per_inch, y/cols_per_inch,
			     rx/cols_per_inch, ry/rows_per_inch, rr, nudgeMode);
	    imshow("Cell", imscale(600, stitchedCopy(grid.getRoi())));
	  }

	  imshow("Camera", imscale(600, img2));

	  // Draw markerboard projections.
	  const float borderx = (config.markerboard_project_width -
				 (markerboard_width_actual))/2.0f;
	  const float bordery = (config.markerboard_project_height -
				 (markerboard_height_actual))/2.0f;
	  Projector p(useWarp, borderx, bordery, cols_per_inch, rows_per_inch);
	  Point2f p0  = p.pt(0.0f, 0.0f);
	  Point2f p1  = p.pt(img2.cols, 0.0f);
	  Point2f p2  = p.pt(img2.cols, img2.rows);
	  Point2f p3  = p.pt(0.0f, img2.rows);
	  
	  Point2f p0b = p.pt(0.0f, 0.0f, -1.0, -1.0);
	  Point2f p1b = p.pt(img2.cols, 0.0f, 1.0, -1.0);
	  Point2f p2b = p.pt(img2.cols, img2.rows, 1.0, 1.0);
	  Point2f p3b = p.pt(0.0f, img2.rows, -1.0, 1.0);
	  
	  line(stitchedCopy, p0, p1, Scalar(255, 0, 0), 3, CV_AA);
	  line(stitchedCopy, p1, p2, Scalar(255, 0, 0), 3, CV_AA);
	  line(stitchedCopy, p2, p3, Scalar(255, 0, 0), 3, CV_AA);
	  line(stitchedCopy, p3, p0, Scalar(255, 0, 0), 3, CV_AA);
	  
	  line(stitchedCopy, p0b, p1b, Scalar(255, 0, 0), 3, CV_AA);
	  line(stitchedCopy, p1b, p2b, Scalar(255, 0, 0), 3, CV_AA);
	  line(stitchedCopy, p2b, p3b, Scalar(255, 0, 0), 3, CV_AA);
	  line(stitchedCopy, p3b, p0b, Scalar(255, 0, 0), 3, CV_AA);	
	} else {
	  showError(stitcher.getError(status), stitchedCopy);
	}
      } else {
	showError(markers.getError(istatus), stitchedCopy);
      }
    }

    UMat scaleCopy = imscale(800, stitchedCopy);
    if (stitchedCopy.cols > 0) {
      drawText(scaleCopy, "(G)rid Next  (N)udge  (M)ove Mode");
      imshow("Stitched Image", scaleCopy);
    }

    // Include a short pause for any drawing to catch up.
    key = waitKey(10);
    if (key == 27) break;
    if (key == 'g') grid.next();
    if (key == 'm') moveMode=!moveMode;
    if (key == 'n') nudgeMode = !nudgeMode;
    if (moveMode) {
      bool changed = false;
      if (key == 'Q') {
	grid.gx--;
	changed = true;
      }
      if (key == 'S') {
	grid.gx++;
	changed = true;
      }
      if (key == 'T') {
	grid.gy++;
	changed = true;
      }
      if (key == 'R') {
	grid.gy--;
	changed = true;
      }
      if (changed) {
	saveOffsets(grid.gx, grid.gy);
	grid.handleGridChange(stitchedImg);
      }
    }
  }
  key = (char) waitKey(0);
  return 0;
}

