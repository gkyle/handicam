#include "stitcher.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace std;
using namespace cv;
using namespace detail;

IncrementalStitcher::IncrementalStitcher(float _matchScale, MatchMode _matchMode,
					 DetectMethod _detectMethod,
					 ExtractMethod _extractMethod) {
  matchScale = _matchScale;
  matchMode = _matchMode;
  detectMethod = _detectMethod;
  extractMethod = _extractMethod;

  if (detectMethod==DetectMethod::DETECT_ORB) {
    detector = ORB::create(maxDetectPoints, 1.5f, 5);
  } else if (detectMethod==DetectMethod::DETECT_SIFT) {
    detector = xfeatures2d::SIFT::create();
  } else {
    detector = xfeatures2d::SURF::create(minHessian, 3, 4, false, true);
  }

  if (extractMethod==ExtractMethod::EXTRACT_ORB) {
    extractor = ORB::create(maxDetectPoints, 1.5f, 5);
  } else if (extractMethod==ExtractMethod::EXTRACT_FREAK) {
    extractor = xfeatures2d::FREAK::create(true, false, 22.0f, 3);
  } else if (extractMethod==ExtractMethod::EXTRACT_BRISK) {
    extractor = BRISK::create();
  } else {
    extractor = xfeatures2d::SURF::create(minHessian, 3, 4, false, false);;
  }
}

IncrementalStitcher::Status IncrementalStitcher::detectAndMatch(UMat img1, UMat img2,
								Mat& R) {
  vector<UMat> imgs;
  imgs.push_back(img1);
  imgs.push_back(img2);

  Mat stitched;
  IncrementalStitcher::Status status;
  if ((status = matchImages(imgs)) != Status::OK) {
    return status;
  }

  matches_.H.convertTo(R, CV_32F);
  LOG(INFO) << R << endl;
  return Status::OK;
}

IncrementalStitcher::Status IncrementalStitcher::matchImages(InputArrayOfArrays images,
							     bool showMatches) {
  Status status = Status::OK;
  vector<UMat> imgs;
  images.getUMatVector(imgs);
  CV_Assert(imgs.size() == 2);

  if (matchScale != 1.0) {
    for (int i=0; i<imgs.size(); i++) {
      UMat tmpImg;
      resize(imgs[i], tmpImg, Size(imgs[i].cols*matchScale, imgs[i].rows*matchScale),
	     0, 0, INTER_AREA);
      imgs[i] = tmpImg;
    }
  }
  
  // Create masks for images.
  Mat gray_img0, mask0;
  cvtColor(imgs[0], gray_img0, CV_BGR2GRAY);
  threshold(gray_img0, mask0, 10, 255, THRESH_BINARY);

  Mat gray_img1, mask1;
  cvtColor(imgs[1], gray_img1, CV_BGR2GRAY);
  threshold(gray_img1, mask1, 10, 255, THRESH_BINARY);

  // Erode some of our image masks so that feature detection doesn't identify mask edges
  // as features.
  int erosion_size = 50;
  Mat elem = getStructuringElement(MORPH_RECT,
				   Size(2*erosion_size + 1, 2*erosion_size+1),
				   Point(erosion_size, erosion_size));
  Mat dmask0, dmask1;
  erode(mask0, dmask0, elem);
  erode(mask1, dmask1, elem);

  vector<KeyPoint> keypoints_1, keypoints_2;
  UMat descriptors_1, descriptors_2;

  detector->detect(imgs[0].getMat(ACCESS_READ), keypoints_1, dmask0);
  detector->detect(imgs[1].getMat(ACCESS_READ), keypoints_2, dmask1);

  if (true || extractMethod==ExtractMethod::EXTRACT_FREAK) {
    extractor->compute(gray_img0, keypoints_1, descriptors_1);
    extractor->compute(gray_img1, keypoints_2, descriptors_2);
  } else {
    extractor->compute(imgs[0], keypoints_1, descriptors_1);
    extractor->compute(imgs[1], keypoints_2, descriptors_2);
  }

  // Prepare structures for matcher.
  detail::ImageFeatures f0 = {
    0,                                // img_idx
    Size(imgs[0].cols, imgs[0].rows), // img_size
    keypoints_1,                      // keypoints
    descriptors_1                     // descriptors
  };

  detail::ImageFeatures f1 = {
    1,                                // img_idx
    Size(imgs[1].cols, imgs[1].rows), // img_size
    keypoints_2,                      // keypoints
    descriptors_2                     // descriptors
  };

  features_.clear();
  features_.push_back(f0);
  features_.push_back(f1);

  LOG(INFO) << "KeyPoints 1: " << keypoints_1.size() << "  2: " << keypoints_2.size() << endl;
  
  // Match.
  vector<cv::detail::MatchesInfo> pairwise_matches_;
  detail::AffineBestOf2NearestMatcher(false, true, 0.3f)(features_, pairwise_matches_);
  matches_ = pairwise_matches_[1];
  //affineMatch(f0, f1, matches_);

  LOG(INFO) << "Matches: " << matches_.num_inliers << endl;
  LOG(INFO) << "Confidence: " << matches_.confidence << endl;
  if (matches_.num_inliers < 1) {
    status = Status::TOO_FEW_MATCHES_ERR;
  }

  Mat R;
  if (matches_.H.rows > 0) {
    matches_.H.convertTo(R, CV_32F);
    if (abs(1-getScale(R)) > matchScaleThreshold) {
      LOG(INFO) << "Affine transform scale: " << getScale(R) << endl;
      status = Status::EXCEEDS_SCALE_THRESHOLD_ERR;
    }
  } else {
    status = Status::TOO_FEW_MATCHES_ERR;
  }

  // Optionally, draw matches.
  if (showMatches) {
    vector<uchar> umask = matches_.inliers_mask;
    vector<char> mask = std::vector<char>( umask.begin(), umask.end() );
    Mat drawing;
    drawMatches(imgs[0], keypoints_1, imgs[1], keypoints_2, matches_.matches, drawing,
		Scalar::all(-1), Scalar::all(-1), mask);
    imshow("Matches", imscale(1000, drawing));
  }
  return status;
}

void IncrementalStitcher::translate(float x, float y) {
  // In pairwise mode, track changes to cordinate system independently.
  if (matchMode == MatchMode::PAIRWISE) {
    gx = (x+dx) < 0 ? (x+dx)*-1.0 : 0;
    gy = (y+dy) < 0 ? (y+dy)*-1.0 : 0;
    dx = (x+dx) > 0 ? (x+dx) : 0;
    dy = (y+dy) > 0 ? (y+dy) : 0;
  } else {
    gx = (x) < 0 ? (x)*-1.0 : 0;
    gy = (y) < 0 ? (y)*-1.0 : 0;
    dx = (x) > 0 ? (x) : 0;
    dy = (y) > 0 ? (y) : 0;
  }
  LOG(INFO) << "gx: " << gx << "\t" << "dx: " << dx << endl;
  LOG(INFO) << "gy: " << gy << "\t" << "dy: " << dy << endl;
}

float scale(Mat H) {
  CV_Assert(H.type() == CV_32F);
  float a = H.at<float>(0,0);
  float b = H.at<float>(0,1);
  float s = sqrt(pow(a, 2) + pow(b, 2));
  return s;
}

void undoScale(Mat H, float scale) {
  H.at<float>(0,0) = H.at<float>(0,0) / scale;
  H.at<float>(0,1) = H.at<float>(0,1) / scale;
  H.at<float>(1,0) = H.at<float>(1,0) / scale;
  H.at<float>(1,1) = H.at<float>(1,1) / scale;
}

UMat IncrementalStitcher::getStitchedImage() {
  return stitchedImage;
}

UMat IncrementalStitcher::getNextBaseImage() {
  if (matchMode == MatchMode::PAIRWISE) {
    return lastMatchedImage;
  } else {
    return stitchedImage;
  }
}

IncrementalStitcher::Status IncrementalStitcher::composeImages(UMat img1, UMat img2,
							       Mat& R) {
  // Warp the current image mask.
  UMat mask;
  mask.create(img2.size(), CV_8U);
  mask.setTo(Scalar::all(255));

  if (matchScale != 1.0) {
    R.at<float>(0,2) /= matchScale;
    R.at<float>(1,2) /= matchScale;
  }
  
  Ptr<WarperCreator> wc = new cv::AffineWarper();
  Ptr<detail::RotationWarper> w = wc->create(1.0);

  UMat wimg2, wmask;
  Mat_<float> K = Mat::eye(3, 3, CV_32F);
  
  float s = scale(R);
  if (s != 0) { // TODO: Make this an assertion. Scale variance should have been handled earlier.
    undoScale(R, s);
    Point tl = w->warp(img2, K, R, INTER_AREA, BORDER_REFLECT, wimg2);
    w->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, wmask);

    if (stitchedImage.cols==0) img1.copyTo(stitchedImage);
    translate(tl.x, tl.y);
    UMat buff = composeImagesWithOffset(stitchedImage, wimg2, wmask,
					Point(dx, dy), Point(gx, gy));
    buff.copyTo(stitchedImage);

    lastMatchedImage = UMat::ones(wimg2.rows, wimg2.cols, CV_8UC3 );
    wimg2.copyTo(lastMatchedImage, wmask);
  } else {
    lastMatchedImage = img1;
    LOG(ERROR) << "Transform matrix scale: " << s << endl;
    return Status::COMPOSE_ERR_SCALE_IS_ZERO;
  }

  return Status::OK;
}

UMat IncrementalStitcher::composeImagesWithOffset(UMat img1, UMat img2, UMat img2mask,
						  Point img_offset, Point coord_offset) {
  vector<Point> corners;
  corners.push_back(Point(0,0));
  corners.push_back(img_offset-coord_offset);
  
  vector<Size> sizes;
  sizes.push_back(img1.size());
  sizes.push_back(img2.size());

  Rect bounds = detail::resultRoi(corners, sizes);
  UMat buff = UMat::ones(bounds.height, bounds.width, CV_8UC3 );
  Rect r1(img_offset.x, img_offset.y, img2.cols, img2.rows);
  Rect r2(coord_offset.x, coord_offset.y, img1.cols, img1.rows);
  img1.copyTo(buff(r2));           // Place base image with no mask.
  img2.copyTo(buff(r1), img2mask); // Place second image, masked.

  return buff;
}

string IncrementalStitcher::getError(IncrementalStitcher::Status status) {
  string error;
  switch(status) {
  case IncrementalStitcher::Status::COMPOSE_ERR_SCALE_IS_ZERO:
    error = "Bad (zero) scale in transform matrix. Can't stitch image.";
    break;
  case IncrementalStitcher::Status::MATCH_ERR:
    error = "Match error.";
    break;
  case IncrementalStitcher::Status::TOO_FEW_MATCHES_ERR:
    error = "Insufficient matching features.";
    break;
  case IncrementalStitcher::Status::EXCEEDS_SCALE_THRESHOLD_ERR:
    error = "Match scale exceeds threshold.";
    break;
  case IncrementalStitcher::Status::ESTIMATION_ERR:
    error = "Camera estimation error.";
    break;
  }
  return error;
}
