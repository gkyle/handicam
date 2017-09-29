#include <opencv2/opencv.hpp>
#include "markers.hpp"

#ifndef SOURCE
#define SOURCE

using namespace cv;
using namespace std;

class Source {
  public:
  virtual Markers::Status nextImage(Markers markers, UMat& imgProj) = 0;
  virtual bool done();
  virtual ~Source(){}
};

class VideoSource: public Source {
  public:
  VideoSource(VideoCapture vc);
  virtual Markers::Status nextImage(Markers markers, UMat& imgProj);
  virtual bool done();

  private:
  VideoCapture cap;
  int sequence = 0;
  bool isDone = false;
  const int SKIP_FRAMES = 5;
};

class ImageSource: public Source {
  public:
  ImageSource(vector<UMat> i);
  virtual Markers::Status nextImage(Markers markers, UMat& imgProj);
  virtual bool done();

  private:
  vector<UMat> imgs;
};

#endif
