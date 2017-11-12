#include "markers.hpp"

using namespace std;
using namespace cv;

Markers::Markers(Config& config, bool stabilizeMarkers) {
  cameraMatrix = config.cameraMatrix;
  distCoeffs = config.distCoeffs;
  image_width = config.image_width;
  markerboard_width = config.markerboard_width;
  markerboard_height = config.markerboard_height;
  ratio = markerboard_height / markerboard_width;
  markerboard_offset = config.markerboard_offset;

  dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

  params = aruco::DetectorParameters::create();
  params->cornerRefinementMethod=aruco::CORNER_REFINE_CONTOUR;
  params->cornerRefinementWinSize = 3;
  params->adaptiveThreshConstant = 11;
  //params->adaptiveThreshWinSizeStep = 20;
  //params->perspectiveRemovePixelPerCell = 10;

  if (!stabilizeMarkers) {
    TARGET_FRAMES = 1;
  }
}

UMat Markers::getPerspective(UMat img, Point2f srcQuad[]) {
  Point2f dstQuad[4];
  dstQuad[0] = Point2f(0, 0);
  dstQuad[1] = Point2f(image_width, 0);
  dstQuad[2] = Point2f(image_width, image_width*ratio);
  dstQuad[3] = Point2f(0, image_width*ratio);
  Mat pmat = getPerspectiveTransform(srcQuad, dstQuad);
  UMat dst = UMat::zeros(img.rows, img.cols, img.type());
  warpPerspective(img, dst, pmat, Size(image_width, image_width*ratio), INTER_AREA);
  return crop(dst);
}

void Markers::storeRect(Point2f a, Point2f b, Point2f c, Point2f d) {
  rects.insert(rects.begin(), d);
  rects.insert(rects.begin(), c);
  rects.insert(rects.begin(), b);
  rects.insert(rects.begin(), a);
  while (rects.size() > TARGET_FRAMES * 4) {
    rects.pop_back();
  }
}

void Markers::avgRect(Point2f r[]) {
  float ax, bx, cx, dx, ay, by, cy, dy;
  ax = bx = cx = dx = ay = by = cy = dy = 0;
  for (int i=0; i<rects.size(); i+=4) {
    ax += rects[i+0].x;
    bx += rects[i+1].x;
    cx += rects[i+2].x;
    dx += rects[i+3].x;

    ay += rects[i+0].y;
    by += rects[i+1].y;
    cy += rects[i+2].y;
    dy += rects[i+3].y;
  }
  r[0] = Point2f(ax/(float)rects.size()*4, ay/(float)rects.size()*4);
  r[1] = Point2f(bx/(float)rects.size()*4, by/(float)rects.size()*4);
  r[2] = Point2f(cx/(float)rects.size()*4, cy/(float)rects.size()*4);
  r[3] = Point2f(dx/(float)rects.size()*4, dy/(float)rects.size()*4);
}

Markers::Status Markers::getArucoOrientedImage(UMat& img, UMat& imgProj,
					       bool drawMarkers, bool doProjection) {
  // Undistort image according to camera profile.
  UMat undist_img;
  undistort(img, undist_img, cameraMatrix, distCoeffs);
  undist_img.copyTo(img);
  
  if (doProjection || drawMarkers) {
    // Detect markers and get locations.
    vector<int> ids;
    vector<vector<Point2f>> corners;
    aruco::detectMarkers(undist_img, dictionary, corners, ids, params);

    // If any markers detected.
    Point2f markers[4];
    if (ids.size() > 0) {
      vector< Vec3d > rvecs, tvecs;
      aruco::estimatePoseSingleMarkers(corners, 1.0, cameraMatrix, distCoeffs, rvecs, tvecs);

      if (drawMarkers) {
	aruco::drawDetectedMarkers(img, corners, ids);
      }

      for(int i=0; i<ids.size(); i++) {
	if (drawMarkers) {
	  line(img, corners[i][0], corners[i][1], Scalar(0, 255, 0), 1, CV_AA);
	  line(img, corners[i][1], corners[i][2], Scalar(0, 255, 0), 1, CV_AA);
	  line(img, corners[i][2], corners[i][3], Scalar(0, 255, 0), 1, CV_AA);
	  line(img, corners[i][3], corners[i][0], Scalar(0, 255, 0), 1, CV_AA);

	  line(img, corners[i][0], corners[i][2], Scalar(0, 255, 0), 1, CV_AA);
	  line(img, corners[i][1], corners[i][3], Scalar(0, 255, 0), 1, CV_AA);
	}
	markers[ids[i]] = corners[i][0];
      }

      // Need 4 markers for Perspective Transform.
      if (ids.size() == 4) {
	storeRect(markers[0], markers[1], markers[2], markers[3]);
	avgRect(markers);
	if (drawMarkers) {
	  line(img, markers[0], markers[1], Scalar(255, 0, 0), 1, CV_AA);
	  line(img, markers[1], markers[2], Scalar(255, 0, 0), 1, CV_AA);
	  line(img, markers[2], markers[3], Scalar(255, 0, 0), 1, CV_AA);
	  line(img, markers[3], markers[0], Scalar(255, 0, 0), 1, CV_AA);

	  line(img, markers[0], markers[2], Scalar(255, 0, 0), 1, CV_AA);
	  line(img, markers[1], markers[3], Scalar(255, 0, 0), 1, CV_AA);
	}
	if (doProjection) {
	  imgProj = getPerspective(undist_img, markers);
	  // !!
	  //getPerspective2(undist_img, markers);
	}
      } else {
	return ids.size() < 4 ? Status::ORIENT_TOO_FEW_MARKERS_ERR :
	  Status::ORIENT_TOO_MANY_MARKERS_ERR;
      }
    } else {
      return Status::ORIENT_NO_MARKERS_ERR;
    }
  }

  return Status::OK;
}

UMat Markers::crop(UMat img) {
  const float border = markerboard_offset;
  const float cols_per_inch = img.cols / markerboard_width;
  const float rows_per_inch = img.rows / markerboard_height;

  Rect roi;
  roi.x = (cols_per_inch*border);
  roi.width = (img.cols-cols_per_inch*border*2);
  roi.y = (rows_per_inch*border);
  roi.height = (img.rows-rows_per_inch*border*2);

  return img(roi);
}

string Markers::getError(Markers::Status status) {
  string error = "None";
  switch(status) {
  case Markers::Status::ERR:
    error = "Unknown Marker error.";
    break;
  case Markers::Status::ORIENT_NO_MARKERS_ERR:
    error = "Reoriented image was bad. No markers found. Skipping.";
    break;
  case Markers::Status::ORIENT_TOO_FEW_MARKERS_ERR:
    error = "Reoriented image was bad. Obstructed marker? Skipping.";
    break;
  case Markers::Status::ORIENT_TOO_MANY_MARKERS_ERR:
    error = "Reoriented image was bad. Too many markers. Skipping.";
    break;
  }
  return error;
}
