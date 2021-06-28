﻿#include "kimera_dsg/scene_graph.h"

#include <glog/logging.h>

namespace kimera {

using LayerRef = SceneGraph::LayerRef;
using EdgeRef = SceneGraph::EdgeRef;
using NodeRef = SceneGraph::NodeRef;
using Node = SceneGraph::Node;
using EdgeInfo = SceneGraph::EdgeInfo;

SceneGraph::SceneGraph() : SceneGraph(getDefaultLayerIds()) {}

SceneGraph::SceneGraph(const std::vector<LayerId>& layer_ids)
    : last_edge_idx_(0),
      layer_ids_(layer_ids),
      layers(layers_),
      inter_layer_edges(inter_layer_edges_) {
  if (layer_ids.empty()) {
    // TODO(nathan) custom exception
    throw std::runtime_error("Scene graph cannot be initialized with no layers");
  }

  initialize_();
}

void SceneGraph::initialize_() {
  for (const auto& id : layer_ids_) {
    layers_[id] = std::make_unique<SceneGraphLayer>(id);
  }

  // TODO(nathan) reconsider if we actually need this
  for (const auto& id_layer_pair : layers_) {
    CHECK(id_layer_pair.second) << "Layer " << id_layer_pair.first << " was null!";
  }
}

void SceneGraph::clear() {
  layers_.clear();
  inter_layer_edges_.clear();
  initialize_();
}

bool SceneGraph::emplaceNode(LayerId layer_id,
                             NodeId node_id,
                             NodeAttributes::Ptr&& attrs) {
  if (!hasLayer(layer_id)) {
    LOG(WARNING) << "Invalid layer: " << layer_id;
    return false;
  }

  if (node_layer_lookup_.count(node_id) != 0) {
    return false;
  }

  bool successful = layers_[layer_id]->emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_layer_lookup_[node_id] = layer_id;
  }

  return successful;
}

bool SceneGraph::insertNode(Node::Ptr&& node) {
  if (node == nullptr) {
    LOG(ERROR) << "Attempting to add an uninitialized node";
    return false;
  }

  // we grab these here to avoid problems with move
  const LayerId node_layer = node->layer;
  const NodeId node_id = node->id;

  if (!hasLayer(node_layer)) {
    return false;
  }

  if (node_layer_lookup_.count(node_id) != 0) {
    return false;
  }

  bool successful = layers_[node_layer]->insertNode(std::move(node));
  if (successful) {
    node_layer_lookup_[node_id] = node_layer;
  }

  return successful;
}

// TODO(nathan) this duplicates work with some other stuff. maybe rethink
Node* SceneGraph::getParentNode_(NodeId source, NodeId target) const {
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  return source_layer > target_layer
             ? layers_.at(source_layer)->nodes_.at(source).get()
             : layers_.at(target_layer)->nodes_.at(target).get();
}

// TODO(nathan) this duplicates work with some other stuff. maybe rethink
Node* SceneGraph::getChildNode_(NodeId source, NodeId target) const {
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  return source_layer < target_layer
             ? layers_.at(source_layer)->nodes_.at(source).get()
             : layers_.at(target_layer)->nodes_.at(target).get();
}

bool SceneGraph::insertEdge(NodeId source, NodeId target, EdgeInfo::Ptr&& edge_info) {
  if (hasEdge(source, target)) {
    return false;
  }

  if (!hasNode(source)) {
    // TODO(nathan) maybe consider logging here
    return false;
  }

  if (!hasNode(target)) {
    // TODO(nathan) maybe consider logging here
    return false;
  }

  // defer to layers if it is a intra-layer edge
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_[source_layer]->insertEdge(source, target, std::move(edge_info));
  }

  // TODO(nathan) consider warning for non-adjacent edges
  // non-adjacent edges have a good use case: coarse detections at building
  // level for lower layers

  Node* parent = getParentNode_(source, target);
  Node* child = getChildNode_(source, target);
  // TODO(nathan) asserts

  if (child->hasParent()) {
    return false;
  }

  child->setParent_(parent->id);
  parent->children_.insert(child->id);

  last_edge_idx_++;
  inter_layer_edges_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(last_edge_idx_),
      std::forward_as_tuple(source,
                            target,
                            (edge_info == nullptr) ? std::make_unique<EdgeInfo>()
                                                   : std::move(edge_info)));
  inter_layer_edges_info_[source][target] = last_edge_idx_;
  inter_layer_edges_info_[target][source] = last_edge_idx_;
  return true;
}

bool SceneGraph::hasLayer(LayerId layer_id) const {
  return layers_.count(layer_id) != 0;
}

bool SceneGraph::hasNode(NodeId node_id) const {
  return node_layer_lookup_.count(node_id) != 0;
}

bool SceneGraph::hasEdge(NodeId source, NodeId target) const {
  if (!(hasNode(source) && hasNode(target))) {
    return false;
  }

  // defer to layers if it is a intra-layer edge
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_.at(source_layer)->hasEdge(source, target);
  }

  return inter_layer_edges_info_.count(source) != 0 &&
         inter_layer_edges_info_.at(source).count(target) != 0;
}

std::optional<LayerRef> SceneGraph::getLayer(LayerId layer_id) const {
  if (!hasLayer(layer_id)) {
    return std::nullopt;
  }

  // TODO(nathan) consider assert here instead
  CHECK(layers_.at(layer_id)) << "Invalid layer";
  return std::cref(*layers_.at(layer_id));
}

// TODO(nathan) look at gtsam casting
std::optional<NodeRef> SceneGraph::getNode(NodeId node_id) const {
  if (!hasNode(node_id)) {
    return std::nullopt;
  }

  // TODO(nathan) consider assert
  LayerId layer = node_layer_lookup_.at(node_id);
  return layers_.at(layer)->getNode(node_id);
}

std::optional<EdgeRef> SceneGraph::getEdge(NodeId source, NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  // defer to layers if it is a intra-layer edge
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_.at(source_layer)->getEdge(source, target);
  }

  size_t edge_idx = inter_layer_edges_info_.at(source).at(target);
  return std::cref(inter_layer_edges_.at(edge_idx));
}

bool SceneGraph::removeNode(NodeId node_id) {
  if (!hasNode(node_id)) {
    return false;
  }

  LayerId layer = node_layer_lookup_.at(node_id);
  Node* node = layers_[layer]->nodes_.at(node_id).get();
  // TODO(nathan) consider asserts
  if (node->hasParent()) {
    removeInterLayerEdge_(node_id, node->parent_);
  }

  std::set<NodeId> targets_to_erase = node->children_;
  for (const auto& target : targets_to_erase) {
    removeInterLayerEdge_(node_id, target);
  }

  layers_[layer]->removeNode(node_id);
  return true;
}

void SceneGraph::removeInterLayerEdge_(NodeId source, NodeId target) {
  inter_layer_edges_.erase(inter_layer_edges_info_.at(source).at(target));
  inter_layer_edges_info_.at(source).erase(target);
  if (inter_layer_edges_info_.at(source).empty()) {
    inter_layer_edges_info_.erase(source);
  }

  inter_layer_edges_info_.at(target).erase(source);
  if (inter_layer_edges_info_.at(target).empty()) {
    inter_layer_edges_info_.erase(target);
  }

  Node* parent = getParentNode_(source, target);
  Node* child = getChildNode_(source, target);
  parent->children_.erase(child->id);
  child->clearParent_();
}

bool SceneGraph::removeEdge(NodeId source, NodeId target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_.at(source_layer)->removeEdge(source, target);
  }

  removeInterLayerEdge_(source, target);
  return true;
}

size_t SceneGraph::numLayers() const { return layers_.size(); }

size_t SceneGraph::numNodes() const {
  size_t total_nodes = 0u;
  for (const auto& id_layer_pair : layers_) {
    total_nodes += id_layer_pair.second->numNodes();
  }

  return total_nodes;
}

size_t SceneGraph::numEdges() const {
  size_t total_edges = inter_layer_edges_.size();
  for (const auto& id_layer_pair : layers_) {
    total_edges += id_layer_pair.second->numEdges();
  }

  return total_edges;
}

Eigen::Vector3d SceneGraph::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the layer");
  }

  LayerId layer = node_layer_lookup_.at(node);
  return layers_.at(layer)->getPosition(node);
}

SceneGraph::LayerIds getDefaultLayerIds() {
  SceneGraph::LayerIds default_ids{to_underlying(KimeraDsgLayers::OBJECTS),
                                   to_underlying(KimeraDsgLayers::PLACES),
                                   to_underlying(KimeraDsgLayers::ROOMS),
                                   to_underlying(KimeraDsgLayers::BUILDINGS)};
  return default_ids;
}

}  // namespace kimera