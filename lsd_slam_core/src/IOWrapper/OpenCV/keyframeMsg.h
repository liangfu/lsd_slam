#include <vector>

namespace lsd_slam_viewer{
struct keyframeMsg{
  int id;
  double time;
  bool isKeyframe;

  // camToWorld as serialization of sophus sim(3).
  // may change with keyframeGraph-updates.
  float camToWorld[7];

  // camera parameter (fx fy cx cy), width, height
  // will never change, but required for display.
  float fx;
  float fy;
  float cx;
  float cy;
  unsigned int height;
  unsigned int width;

  // data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height
  // may be empty, in that case no associated pointcloud is ever shown.
  std::vector<unsigned char> pointcloud;
};
};

