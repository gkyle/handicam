#include "grid.hpp"

// TODO: Replace
void undoScale2(Matx33f H, float s) {
  H(0,0) = H(0,0) / s;
  H(0,1) = H(0,1) / s;
  H(1,0) = H(1,0) / s;
  H(1,1) = H(1,1) / s;
}

Grid::Grid(int _grid_cols, int _grid_rows,
	   float _cell_width, float _cell_height,
	   float _cell_project_width, float _cell_project_height,
	   float _cols_per_inch, float _rows_per_inch) {
  int n = 1;
  cols = _grid_cols;
  rows = _grid_rows;
  cell_width = _cell_width;
  cell_height = _cell_height;
  cell_project_width = _cell_project_width;
  cell_project_height = _cell_project_height;
  cols_per_inch = _cols_per_inch;
  rows_per_inch = _rows_per_inch;
  for (int r=0; r<_grid_rows; r++) {
    for (int c=0; c<_grid_cols; c++) {
      Point2f p(c*cols_per_inch*cell_project_width,
		r*-1*rows_per_inch*cell_project_height);
      cells.push_back(p);
      names.push_back("G" + to_string(n++));
    }
  }
}

Point2f Grid::getCell() {
  return getCell(selected);
}
  
Point2f Grid::getCell(int c) {
  return Point2f(cells[c].x+gx-(cell_width-cell_project_width)*cols_per_inch/2,
		 cells[c].y+gy-(cell_height-cell_project_height)*rows_per_inch/2);
}
  
Point2f Grid::getCellProject() {
  return getCellProject(selected);
}
  
Point2f Grid::getCellProject(int c) {
  return Point2f(cells[c].x+gx, cells[c].y+gy);
}
  
void Grid::drawRectProject(UMat img, int c) {
  Rect roi = getRoiProject(c);
  Mat rimg = img.getMat(ACCESS_RW);
  rectangle(rimg, roi, Scalar(128,128,128), 5);
  drawGridText(img, roi, names[c], 20.0);
}
  
void Grid::drawRect(UMat img, int c) {
  Rect roi = getRoi(c);
  Mat rimg = img.getMat(ACCESS_RW);
  rectangle(rimg, roi, Scalar(0,0,255), 5);
}

void Grid::drawGrid(UMat img) {
  for (int i=0; i<cells.size(); i++) {
    drawRectProject(img, i);
    drawRect(img, i);
  }
}

void Grid::drawMetrics(UMat img, Matx33f warp,
		       float x, float y, float rx, float ry, float rr, int mode) {
  double t = atan(warp(1,0) / warp(0,0));
  double deg = t * (180/3.1415926535897) * -1;
  char dbuf [7];
  sprintf(dbuf, "%.2f", deg);
  char xbuf [7];
  sprintf(xbuf, "%.3f", x);
  char ybuf [7];
  sprintf(ybuf, "%.3f", y);
  char rxbuf [7];
  sprintf(rxbuf, "%.3f", rx);
  char rybuf [7];
  sprintf(rybuf, "%.3f", ry);
  char rrbuf [7];
  sprintf(rrbuf, "%.3f", rr);

  Rect roi = getRoi();
  drawGridTextSmall(img, roi, "Y: " + string(ybuf) + "in +/- " +
		    string(rybuf) + "in", 1.0, mode);
  drawGridTextSmall(img, roi, "X: " + string(xbuf) + "in +/- " +
		    string(rxbuf) + "in", 3.0, mode);
  drawGridTextSmall(img, roi, "Rotation: " + string(dbuf) + "d +/- " +
		    string(rrbuf) + "d", 5.0, mode);
}

Rect Grid::getGridRoi() {
  return Rect(Point2f(gx, gy-(rows-1)*cell_project_height*rows_per_inch),
	      Size(cols*cell_project_width*cols_per_inch,
		   rows*cell_project_height*rows_per_inch));
}

Rect Grid::getRoi() {
  return getRoi(selected);
}

Rect Grid::getRoi(int c) {
  Point2f p = getCell(c);
  Rect r = Rect(p, Size(cell_width*cols_per_inch, cell_height*rows_per_inch));
  return r;
}

Rect Grid::getRoiProject(int c) {
  Point2f p = getCellProject(c);
  Rect r = Rect(p, Size(cell_project_width*cols_per_inch,
			cell_project_height*rows_per_inch));
  return r;
}
  
void Grid::prev() {
  selected--;
  if (selected < 0) {
    selected = cells.size() - 1;
  }
}

void Grid::next() {
  selected++;
  if (selected >= cells.size()) {
    selected = 0;
  }
}

void Grid::logStats(Matx33f aH) {
  ax.insert(ax.begin(), aH(0,2));
  ay.insert(ay.begin(), aH(1,2));
  ar.insert(ar.begin(), getAngle(aH));
  as.insert(as.begin(), getScale(aH));

  while (ax.size() > MAX) ax.pop_back();
  while (ay.size() > MAX) ay.pop_back();
  while (ar.size() > MAX) ar.pop_back();
  while (as.size() > MAX) as.pop_back();
}

void Grid::avg(vector<float> list, float& mean, float& range) {
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

void Grid::store(Matx33f warp) {
  logStats(warp);
  undoScale2(warp, (getScale(warp))); // FYI: destructive
  warps.insert(warps.begin(), warp);
  while (warps.size() > MAX) warps.pop_back();
}

Matx33f Grid::avgWarp() {
  Matx33f aWarp;
  aWarp(2,2) = 1.0f;
  int count = warps.size();
  for (int i=0; i<count; i++) {
    aWarp(0,0) += warps[i](0,0);
    aWarp(0,1) += warps[i](0,1);
    aWarp(0,2) += warps[i](0,2);
    aWarp(1,0) += warps[i](1,0);
    aWarp(1,1) += warps[i](1,1);
    aWarp(1,2) += warps[i](1,2);
  }
  aWarp(0,0) /= count;
  aWarp(0,1) /= count;
  aWarp(0,2) /= count;
  aWarp(1,0) /= count;
  aWarp(1,1) /= count;
  aWarp(1,2) /= count;
  return aWarp;
}

// In pixels...
void Grid::showStats(vector<float> list, string name) {
  float mean, range;
  avg(list, mean, range);
  cout << name << ": " << mean << "  " << range << endl;
}

void Grid::showStats() {
  showStats(ax, "ax");
  showStats(ay, "ay");
  showStats(ar, "ar");
  showStats(as, "as");
}

// Scale image to accomodate grid.
void Grid::handleGridChange(UMat& stitchedImg) {
  Rect roi = getGridRoi();

  // Handle <0 exceeds-bounds case.
  if (roi.x < 0 || roi.y < 0) {
    float dx = roi.x < 0 ? roi.x * -1 : 0;
    float dy = roi.y < 0 ? roi.y * -1 : 0;
    LOG(INFO) << "dx: " << dx << "  dy:" << dy << endl; 
    UMat bufImg = UMat::zeros(stitchedImg.rows+dy, stitchedImg.cols+dx,
			      stitchedImg.type());
    Rect croi(Point2f(dx, dy), Size(stitchedImg.cols, stitchedImg.rows));
    stitchedImg.copyTo(bufImg(croi));
    bufImg.copyTo(stitchedImg);
    
    gx += dx;
    gy += dy;
    roi = getGridRoi(); // update roi
    
    // Save corresponding changes to image if they impact relative offsets.
    imwrite("stitched.jpeg", stitchedImg);
    saveOffsets(gx, gy);
  }
    
  // Handle >cols or >rows exceeds case.
  if (roi.x + roi.width > stitchedImg.cols || roi.y + roi.height > stitchedImg.rows) {
    float w = roi.x + roi.width > stitchedImg.cols ? roi.x + roi.width : stitchedImg.cols;
    float h = roi.y + roi.height > stitchedImg.rows ? roi.y + roi.height : stitchedImg.rows;
    UMat bufImg = UMat::zeros(h, w, stitchedImg.type());
    Rect croi(Point2f(0,0), Size(stitchedImg.cols, stitchedImg.rows));
    stitchedImg.copyTo(bufImg(croi));
    bufImg.copyTo(stitchedImg);
  }
}
