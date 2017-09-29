#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <math.h>
#include "util.hpp"

#ifndef INCREMENTAL_STITCHER
#define INCREMENTAL_STITCHER

using namespace cv;

class CV_EXPORTS_W IncrementalStitcher {
  public:
    enum Status {
      OK = 0,
      MATCH_ERR = 100,
      TOO_FEW_MATCHES_ERR = 101,
      EXCEEDS_SCALE_THRESHOLD_ERR = 102,
      ESTIMATION_ERR = 200,
      COMPOSE_ERR = 300,
      COMPOSE_ERR_SCALE_IS_ZERO = 301,
    };

    enum MatchMode {
      PAIRWISE = 0,
      AGGREGATE = 1,
    };

    enum DetectMethod {
      DETECT_SURF = 0,
      DETECT_ORB = 1,
      DETECT_SIFT = 2,
    };

    enum ExtractMethod {
      EXTRACT_SURF = 0,
      EXTRACT_ORB = 1,
      EXTRACT_FREAK = 2,
      EXTRACT_BRISK = 3,
    };

    IncrementalStitcher(float scale=1.0, MatchMode matchMode=MatchMode::PAIRWISE,
			DetectMethod detectMethod=DetectMethod::DETECT_SURF,
			ExtractMethod extractMethod=ExtractMethod::EXTRACT_FREAK);

    /** Detect and matches features on 2 images. */
    Status detectAndMatch(UMat img1, UMat img2, Mat& R);

    /** Warp and compose 2 images based on the transform. */
    Status composeImages(UMat img1, UMat img2, Mat& R);

    /** Accessor for stiched image. */
    UMat getStitchedImage();

    /** Get next matching base image for current mode. */
    UMat getNextBaseImage();

    string getError(Status status);

  protected:
    Status matchImages(InputArrayOfArrays images, bool showMatches=false);

    /** Compose 2 images with image and coord system offsets. */
    UMat composeImagesWithOffset(UMat img1, UMat img2, UMat img2mask,
				 Point image_offset, Point coord_offset);

    /** Accumlate translations to zero position within coordinate system and to coordinate
	system itself. */
    void translate(float x, float y);

  private:
    Ptr<Feature2D> detector;

    Ptr<DescriptorExtractor> extractor;

    float matchScale;

    /** The aggregate stitched image. */
    UMat stitchedImage;

    /** The last image matched, post-warp. */
    UMat lastMatchedImage;

    /** Indicates pairwise vs. aggregate matching mode. In pairwise mode, we match the next
	input image to the previous input image (post-rotation). In aggregate mode, we match
	the next image to the aggregate stitched image.
     */
    MatchMode matchMode;

    DetectMethod detectMethod;
    
    ExtractMethod extractMethod;
    
    std::vector<cv::detail::ImageFeatures> features_;

    cv::detail::MatchesInfo matches_;

    /** Maximum +/- scale permitted in "good" affine matrix. Note that scale is removed
     * before compoisition. */
    float matchScaleThreshold = 0.01;
    
    /** Max detected points when using ORB matcher. */
    int maxDetectPoints = 1500;

    /** Min Hessian threshold when using SURF matcher. */
    int minHessian = 400;

    /** Represent zero point in coordinate system. */
    float dx = 0.0;
    float dy = 0.0;
    
    /** Represent changes to coordinate system. */
    float gx = 0.0;
    float gy = 0.0;
  };
 
#endif
