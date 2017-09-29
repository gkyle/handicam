#include <opencv2/opencv.hpp>
#include "util.hpp"

#ifndef GRID
#define GRID

class Grid {
  public:
  vector<Point2f> cells;
  vector<string> names;
  int selected = 0;
  int cols = 0;
  int rows = 0;
  float gx = 0.0;
  float gy = 0.0;
  float cell_width = 0.0;
  float cell_height = 0.0;
  float cell_project_width = 0.0;
  float cell_project_height = 0.0;
  float cols_per_inch = 0.0;
  float rows_per_inch = 0.0;
  vector<Matx33f> warps;
  int MAX = 10;
  vector<float> ax, ay, ar, as;

  Grid(int _grid_cols, int _grid_rows,
       float _cell_width, float _cell_height,
       float _cell_project_width, float _cell_project_height,
       float _cols_per_inch, float _rows_per_inch);
  
  Point2f getCell();
  
  Point2f getCell(int c);
  
  Point2f getCellProject();
  
  Point2f getCellProject(int c);
  
  void drawRectProject(UMat img, int c);
  
  void drawRect(UMat img, int c);

  void drawGrid(UMat img);

  void drawMetrics(UMat img, Matx33f warp,
		   float x, float y, float rx, float ry, float rr, int mode);

  Rect getGridRoi();

  Rect getRoi();

  Rect getRoi(int c);

  Rect getRoiProject(int c);
  
  void prev();

  void next();

  void logStats(Matx33f aH);

  void avg(vector<float> list, float& mean, float& range);

  void store(Matx33f warp);

  Matx33f avgWarp();

  // In pixels...
  void showStats(vector<float> list, string name);

  void showStats();

  // Scale image to accomodate grid.
  void handleGridChange(UMat& stitchedImg);
};

#endif
