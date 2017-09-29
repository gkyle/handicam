#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "util.hpp"
#include "config.hpp"

#ifndef MARKERS
#define MARKERS

using namespace cv;

class CV_EXPORTS_W Markers {
  public:
    enum Status {
      OK = 0,
      ERR = 1,
      ORIENT_NO_MARKERS_ERR = 100,
      ORIENT_TOO_FEW_MARKERS_ERR = 101,
      ORIENT_TOO_MANY_MARKERS_ERR = 102,
    };

    /**
     * stabilizeMarkers: Enables average transform over TARGET_FRAMES.
     */
    Markers(Config& config, bool stabilizeMarkers=true);

    UMat getPerspective(UMat img, Point2f srcQuad[]);

    Status getArucoOrientedImage(UMat& img, UMat& imgProj, bool drawMarkers=false, bool doProjection=true);

    // Crop Aruco marker fragments out of oriented image.
    UMat crop(UMat img);

    string getError(Status status);
  private:
    Mat cameraMatrix;

    Mat distCoeffs;

    Ptr<aruco::Dictionary> dictionary;

    Ptr<aruco::DetectorParameters> params;
	
    int image_width;

    float markerboard_width;
    
    float markerboard_height;
    
    float markerboard_offset;
    
    float ratio;

    int TARGET_FRAMES = 25; // average corners over this many points for stability

    std::vector<Point2f> rects;

    void storeRect(Point2f a, Point2f b, Point2f c, Point2f d);

    void avgRect(Point2f r[]);
};

#endif
