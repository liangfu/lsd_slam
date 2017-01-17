#include <vector>

namespace lsd_slam_viewer{
struct keyframeGraphMsg{
  // data as serialization of sim(3)'s: (int id, float[7] camToWorld)
  unsigned int numFrames;
  std::vector<unsigned char> frameData;

  // constraints (int from, int to, float err)
  unsigned int numConstraints;
  std::vector<unsigned char> constraintsData;
};
};
