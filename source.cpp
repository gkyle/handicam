#include "source.hpp"

bool Source::done() {
  return false;
};


VideoSource::VideoSource(VideoCapture vc) {
  cap = vc;
}

Markers::Status VideoSource::nextImage(Markers markers, UMat& imgProj) {
  UMat img;
  // Blow away any buffered frames so we don't lag.
  for (int i=0; i<SKIP_FRAMES; i++) {
    cap >> img;
  }
  //TODO: make debug mode
  /*
    char buf [3];
    sprintf(buf, "%03d", sequence++);
    std::string filename = "stitchframe-" + to_string(0) + buf + ".jpeg";
    cout << filename << endl;
    imwrite(filename, img);
  */
  Markers::Status status;
  if (img.cols > 0) {
    status = markers.getArucoOrientedImage(img, imgProj);
    if (status == Markers::Status::OK && imgProj.cols == 0) {
      status = Markers::Status::ERR;
    }
  } else {
    isDone = true;
    status = Markers::Status::ERR;
  }
  return status;
}

bool VideoSource::done() {
  return isDone;
}


ImageSource::ImageSource(vector<UMat> i) {
  imgs = i;
}

Markers::Status ImageSource::nextImage(Markers markers, UMat& imgProj) {
  UMat img = imgs.front();
  imgs.erase(imgs.begin());
  Markers::Status status = markers.getArucoOrientedImage(img, imgProj);
  if (status == Markers::Status::OK && imgProj.cols == 0) {
    status = Markers::Status::ERR;
  }
  return status;
}

bool ImageSource::done() {
  return (imgs.size()==0);
}
