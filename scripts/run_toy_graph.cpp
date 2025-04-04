#include "MRISAM2.h"
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <aria_viz/visualizer_rerun.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

using namespace gtsam;
using namespace aria;
using MRBT = MRISAM2::MRBT;

ABSL_FLAG(std::string, filename, "data/toy_graph.g2o", "Input file name");

template <> struct fmt::formatter<MRBT::Clique> {
  constexpr auto parse(fmt::format_parse_context &ctx)
      -> fmt::format_parse_context::iterator {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const MRBT::Clique &clique, FormatContext &ctx) ->
      typename FormatContext::iterator {
    std::ostringstream oss;
    for (auto key_it = clique.allKeys().begin();
         key_it != clique.allKeys().end(); key_it++) {
      oss << gtsam::DefaultKeyFormatter(*key_it) << " ";
    }

    return fmt::format_to(ctx.out(), "{}", oss.str());
  }
};

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

void drawBayesTree(rerun::RecordingStream *rec_, const std::string &entity_path,
                   const MRISAM2::MRBT &bayes_tree, const Eigen::Vector4f &rgba,
                   float line_width = 0.1f, bool is_static = false) {
  std::vector<std::string> cliques;
  std::vector<rerun::components::GraphEdge> edges;
  std::vector<rerun::components::Color> colors;
  for (auto const &[key, clique] : bayes_tree.nodes()) {
    if (!clique)
      continue;
    cliques.push_back(fmt::format("{}", *clique));
    for (auto const &child : clique->childCliques()) {
      if (!child)
        continue;
      edges.push_back({fmt::format("{}", *clique), fmt::format("{}", *child)});
    }

    // draw parent edges
    for (auto const &parent : clique->parentCliques()) {
      if (!parent)
        continue;
      edges.push_back({fmt::format("{}", *clique), fmt::format("{}", *parent)});
    }
  }

  rec_->log_with_static(
      entity_path, is_static,
      rerun::GraphNodes(cliques).with_labels(cliques).with_colors(colors));
  rec_->log_with_static(entity_path, is_static,
                        rerun::GraphEdges(edges).with_graph_type(
                            rerun::components::GraphType::Directed));
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
  auto now = std::chrono::system_clock::now();
  visualizer.setTime(now.time_since_epoch().count());
  visualizer.drawPoints("graph/vertices", *values, getColors(*values), {5.},
                        getKeysAsString(*values));
  visualizer.drawFactors("graph/factors", *graph, *values, getColors(*graph),
                         2.f);

  MRISAM2::RootKeySetMap other_root_keys_map;
  other_root_keys_map.emplace(Symbol('b', 0), KeySet{gtsam::Symbol('b', 0)});

  NonlinearFactorGraph prior_graph;
  Values prior_values;
  // add a prior for a0
  auto prior = gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('a', 0), gtsam::Pose2(0, 0, 0),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-3, 1e-3, 1e-3)));

  auto prior_b = BetweenFactor<Pose2>(
      gtsam::Symbol('b', 0), gtsam::Symbol('a', 0), Pose2(-5, 0, 0),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(100, 100, 100)));

  prior_graph.push_back(prior);
  prior_graph.push_back(prior_b);
  prior_values.insert(prior.key(), prior.prior());
  prior_values.insert(prior_b.key(), prior.prior());

  prior_graph.print("prior_graph");

  MRISAM2Params params;
  params.show_details = true;
  std::unique_ptr<MRISAM2> mrisam2 = std::make_unique<MRISAM2>(
      prior_graph, prior_values, gtsam::Ordering::Colamd(prior_graph),
      Symbol('a', 0), other_root_keys_map, params);

  // read the graph by a step size
  int n_factor_per_step = 1;
  CHECK(n_factor_per_step == 1, "step size not supported");
  for (int i = 0; i < graph->size(); i += n_factor_per_step) {
    NonlinearFactorGraph factors_step;
    Values theta_step;
    char robot;
    for (int j = 0; j < n_factor_per_step && i + j < graph->size(); ++j) {
      auto factor = graph->at(i + j);
      factors_step.push_back(factor);
      auto keys = factor->keys();
      for (const auto &key : keys) {
        if (not mrisam2->theta().exists(key)) {
          theta_step.insert_or_assign(key, Pose2(0, 0, 0));
        }
        robot = gtsam::Symbol(key).chr();
      }
    }

    std::cout << fmt::format(
                     "Iteration {}: Adding {}->{} factor for robot {}", i,
                     DefaultKeyFormatter(factors_step.at(0)->keys()[0]),
                     DefaultKeyFormatter(factors_step.at(0)->keys()[1]), robot)
              << std::endl;
    theta_step.print("theta_step");

    MRISAM2::RootID root_id = Symbol(robot, 0);

    try {
      mrisam2->updateRoot(root_id, factors_step, theta_step, true);
    } catch (const std::exception &e) {
      std::cerr << "Error updating MRISAM2: " << e.what() << std::endl;
      factors_step.print("factors_step");
      throw e;
    }

    auto estimates = mrisam2->calculateEstimate();
    auto factors = mrisam2->nonlinearFactors();

    auto now = std::chrono::system_clock::now();
    visualizer.setTime(now.time_since_epoch().count());
    visualizer.drawPoints("graph/vertices", estimates, getColors(estimates),
                          {5.}, getKeysAsString(estimates));
    visualizer.drawFactors("graph/factors", factors, estimates,
                           getColors(factors), 2.f);

    drawBayesTree(visualizer.rec(), "graph/bayes_tree", *mrisam2,
                  Eigen::Vector4f(0, 0, 255, 255), 0.1f, false);

    mrisam2->saveGraph("mrbt.dot");
    visualizer.drawDotFile("graph/bayes_tree_dot", "mrbt.dot", false);
    // sleep for 50ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  mrisam2->calculateEstimate().print("final estimate");

  return 0;
}