#include <kimera_dsg/graph_utilities.h>
#include <kimera_dsg/scene_graph_layer.h>

#include <gtest/gtest.h>

namespace kimera {

using NodeSet = std::set<NodeId>;
using ResultSet = std::vector<NodeSet>;
using graph_utilities::Components;
using graph_utilities::getConnectedComponents;

template <typename Expected, typename Result>
bool matchesExpected(const Expected& expected, const Result& result) {
  if (expected.size() != result.size()) {
    return false;
  }

  for (const NodeId id : result) {
    if (!expected.count(id)) {
      return false;
    }
  }

  return true;
}

template <typename Expected, typename Result>
bool matchesExpectedSet(const Expected& expected,
                        const std::vector<Result>& result_set) {
  for (const auto& result : result_set) {
    if (matchesExpected(expected, result)) {
      return true;
    }
  }

  return false;
}

struct ConnectedComponentTestConfig {
  NodeSet query;
  ResultSet expected;
  bool restrict_to_query;
  bool connected_graph;
};

struct ConnectedComponentFixture
    : public testing::TestWithParam<ConnectedComponentTestConfig> {
  ConnectedComponentFixture() : layer(1) {}

  virtual ~ConnectedComponentFixture() = default;

  void SetUp() override {
    for (size_t i = 0; i < 6; ++i) {
      layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    }

    layer.insertEdge(0, 1);
    layer.insertEdge(0, 2);
    layer.insertEdge(1, 2);
    layer.insertEdge(3, 4);
    layer.insertEdge(3, 5);
    layer.insertEdge(4, 5);
  }

  IsolatedSceneGraphLayer layer;
};

TEST_P(ConnectedComponentFixture, ResultCorrect) {
  ConnectedComponentTestConfig info = GetParam();
  if (info.connected_graph) {
    layer.insertEdge(2, 3);
  }

  Components result = getConnectedComponents<SceneGraphLayer>(
      layer, info.query, info.restrict_to_query);
  EXPECT_EQ(info.expected.size(), result.size());
  for (const auto& expected : info.expected) {
    EXPECT_TRUE(matchesExpectedSet(expected, result))
        << displayNodeSymbolContainer(expected);
  }
}

const ConnectedComponentTestConfig cc_test_cases[] = {
    {NodeSet{0}, ResultSet{{0, 1, 2}}, false, false},
    {NodeSet{0, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, false, false},
    {NodeSet{0, 3}, ResultSet{{0, 1, 2, 3, 4, 5}}, false, true},
    {NodeSet{0}, ResultSet{{0}}, true, false},
    {NodeSet{0, 3}, ResultSet{{0}, {3}}, true, false},
    {NodeSet{0, 1, 2, 3, 4, 5}, ResultSet{{0, 1, 2}, {3, 4, 5}}, true, false},
    {NodeSet{0, 2, 3, 5}, ResultSet{{0, 2, 3, 5}}, true, true},
    {NodeSet{0, 1, 4, 5}, ResultSet{{0, 1}, {4, 5}}, true, true},
    {NodeSet{0, 1, 2, 3, 4, 5}, ResultSet{{0, 1, 2, 3, 4, 5}}, true, true},
};

INSTANTIATE_TEST_CASE_P(GetConnectedComponent,
                        ConnectedComponentFixture,
                        testing::ValuesIn(cc_test_cases));

struct GraphConfig {
  size_t num_nodes;
  std::vector<std::pair<NodeId, NodeId>> edges;
};

struct DepthLimitedCCTestConfig {
  NodeSet query;
  ResultSet expected;
  size_t depth;
  GraphConfig graph;
};

struct DepthLimitedCCFixture : public testing::TestWithParam<DepthLimitedCCTestConfig> {
  DepthLimitedCCFixture() : layer(1) {}

  virtual ~DepthLimitedCCFixture() = default;

  void SetUp() override {
    DepthLimitedCCTestConfig config = GetParam();
    for (size_t i = 0; i < config.graph.num_nodes; ++i) {
      layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    }

    for (const auto& edge : config.graph.edges) {
      layer.insertEdge(edge.first, edge.second);
    }
  }

  IsolatedSceneGraphLayer layer;
};

TEST_P(DepthLimitedCCFixture, ResultCorrect) {
  DepthLimitedCCTestConfig info = GetParam();

  Components result =
      getConnectedComponents<SceneGraphLayer>(layer, info.depth, info.query);

  EXPECT_EQ(info.expected.size(), result.size());
  for (const auto& expected : info.expected) {
    EXPECT_TRUE(matchesExpectedSet(expected, result))
        << displayNodeSymbolContainer(expected);
  }
}

const GraphConfig dl_graphs[] = {
    {6, {{0, 1}, {1, 2}, {3, 4}, {3, 5}, {4, 5}}},
    {7, {{0, 1}, {1, 2}, {3, 4}, {3, 5}, {4, 5}}},
    {15,
     {{0, 7},
      {0, 2},
      {0, 5},
      {0, 14},
      {2, 14},
      {3, 13},
      {3, 8},
      {4, 10},
      {4, 13},
      {5, 9},
      {6, 9},
      {9, 12},
      {11, 14},
      {12, 13}}},
};

const DepthLimitedCCTestConfig dl_test_cases[] = {
    {NodeSet{0}, ResultSet{{0, 1}}, 1, dl_graphs[0]},
    {NodeSet{0, 3}, ResultSet{{0, 1}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{0, 3}, ResultSet{{0, 1}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{1, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{0, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 2, dl_graphs[0]},
    {NodeSet{0, 1, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{0, 2, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{6}, ResultSet{{6}}, 1, dl_graphs[1]},
    {NodeSet{0, 2, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[1]},
    {NodeSet{0, 1, 2, 3, 4, 5, 6},
     ResultSet{{0, 1, 2}, {3, 4, 5}, {6}},
     1,
     dl_graphs[1]},
    {NodeSet{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
     ResultSet{{0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}, {1}},
     3,
     dl_graphs[2]},
};

INSTANTIATE_TEST_CASE_P(GetDepthLimitedConnectedComponent,
                        DepthLimitedCCFixture,
                        testing::ValuesIn(dl_test_cases));

}  // namespace kimera