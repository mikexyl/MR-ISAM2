#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <aria_viz/visualizer_rerun.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

using namespace gtsam;
using namespace aria;

ABSL_FLAG(std::string, filename, "data/toy_graph.g2o", "Input file name");

std::vector<std::string> getKeysAsString(const gtsam::Values &values) {
  std::vector<std::string> keys;
  for (const auto &key : values.keys()) {
    keys.push_back(gtsam::DefaultKeyFormatter(key));
  }
  return keys;
}

Eigen::Vector4f getColor(const gtsam::Key &key) {
  Symbol symbol(key);
  if (symbol.chr() == 'a') {
    return Eigen::Vector4f(0, 255, 0, 255); // green
  } else if (symbol.chr() == 'b') {
    return Eigen::Vector4f(255, 0, 0, 255); // red
  }
  return Eigen::Vector4f(0, 0, 0, 255); // default black
}

std::vector<Eigen::Vector4f> getColors(const gtsam::Values &values) {
  std::vector<Eigen::Vector4f> colors;
  for (const auto &key : values.keys()) {
    colors.push_back(getColor(key));
  }
  return colors;
}

std::vector<Eigen::Vector4f> getColors(const NonlinearFactorGraph &graph) {
  std::vector<Eigen::Vector4f> colors;
  for (const auto &factor : graph) {
    if (factor->keys().size() != 2) {
      colors.emplace_back(Eigen::Vector4f(0, 0, 0, 1)); // black
    } else {
      auto key1 = factor->keys()[0];
      auto key2 = factor->keys()[1];
      auto color1 = getColor(key1), color2 = getColor(key2);
      // mix color
      Eigen::Vector4f mixed_color = (color1 + color2) / 2;
      colors.push_back(mixed_color);
    }
  }
  return colors;
}

int main(int argc, char **argv) {
  // parse command line arguments
  absl::ParseCommandLine(argc, argv);

  viz::VisualizerRerun visualizer(viz::VisualizerRerun::Params("mr-isam2"));

  // read graph
  std::string filename = absl::GetFlag(FLAGS_filename);
  auto [graph, values] =
      gtsam::readG2o(filename, false, gtsam::KernelFunctionTypeHUBER);

  // visualize graph
  visualizer.setTimeNSec(0);
  visualizer.drawPoints("graph/vertices", *values, getColors(*values), {5.},
                        getKeysAsString(*values));
  visualizer.drawFactors("graph/factors", *graph, *values, getColors(*graph),
                         2.f);

  return 0;
}