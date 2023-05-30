/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "spark_dsg/floor_graph.h"
#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"

#include <list>

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;
using NodeRef = FloorGraph::NodeRef;
using EdgeRef = FloorGraph::EdgeRef;

FloorGraph::FloorGraph()
    : FloorGraph(getDefaultFloorgraphLayerIds()) {}

FloorGraph::FloorGraph(const LayerIds& layer_ids)
    : layer_ids(layer_ids) {
  if (layer_ids.empty()) {
    throw std::domain_error("scene graph cannot be initialized without layers");
  }

  clear();
}

void FloorGraph::clear() {
  layers_.clear();

  node_lookup_.clear();

  interlayer_edges_.reset();

  for (const auto& id : layer_ids) {
    layers_[id] = std::make_unique<SceneGraphLayer>(id);
  }
}

bool FloorGraph::emplaceNode(LayerId layer_id,
                                    NodeId node_id,
                                    NodeAttributes::Ptr&& attrs) {
  if (node_lookup_.count(node_id)) {
    return false;
  }

  if (!layers_.count(layer_id)) {
    SG_LOG(WARNING) << "Invalid layer: " << layer_id << std::endl;
    return false;
  }

  const bool successful = layers_[layer_id]->emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_[node_id] = layer_id;
  }

  return successful;
}

bool FloorGraph::insertNode(Node::Ptr&& node) {
  if (!node) {
    return false;
  }

  if (node_lookup_.count(node->id)) {
    return false;
  }

  // we grab these here to avoid problems with move
  const LayerId node_layer = node->layer;
  const NodeId node_id = node->id;

  if (!hasLayer(node_layer)) {
    return false;
  }

  const bool successful = layers_[node_layer]->insertNode(std::move(node));
  if (successful) {
    node_lookup_[node_id] = node_layer;
  }

  return successful;
}

BaseLayer& FloorGraph::layerFromKey(const LayerKey& key) {
  const auto& layer = static_cast<const FloorGraph*>(this)->layerFromKey(key);
  return const_cast<BaseLayer&>(layer);
}

const BaseLayer& FloorGraph::layerFromKey(const LayerKey& key) const {
  return *layers_.at(key.layer);
}

bool FloorGraph::insertEdge(NodeId source,
                                   NodeId target,
                                   EdgeAttributes::Ptr&& edge_info) {
  LayerKey source_key, target_key;
  if (hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  if (!source_key || !target_key) {
    return false;
  }

  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  if (source_key == target_key) {
    return layerFromKey(source_key).insertEdge(source, target, std::move(attrs));
  }

  if (!addAncestry(source, target, source_key, target_key)) {
    return false;
  }

  interlayer_edges_.insert(source, target, std::move(attrs));

  return true;
}

bool FloorGraph::setNodeAttributes(NodeId node, NodeAttributes::Ptr&& attrs) {
  auto iter = node_lookup_.find(node);
  if (iter == node_lookup_.end()) {
    return false;
  }

  getNodePtr(node, iter->second)->attributes_ = std::move(attrs);
  return true;
}

bool FloorGraph::setEdgeAttributes(NodeId source,
                                          NodeId target,
                                          EdgeAttributes::Ptr&& attrs) {
  if (!hasEdge(source, target)) {
    return false;
  }

  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  if (source_key == target_key) {
    layerFromKey(source_key).edgeContainer().get(source, target).info =
        std::move(attrs);
    return true;
  }

  interlayer_edges_.get(source, target).info = std::move(attrs);
  return true;
}

bool FloorGraph::hasLayer(LayerId layer_id) const {
    return layers_.count(layer_id) != 0;
}

bool FloorGraph::hasNode(NodeId node_id) const {
  return node_lookup_.count(node_id) != 0;
}

NodeStatus FloorGraph::checkNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return NodeStatus::NONEXISTENT;
  }

  return layerFromKey(iter->second).checkNode(node_id);
}

const SceneGraphLayer& FloorGraph::getLayer(LayerId layer) const {
  if (!hasLayer(layer)) {
    std::stringstream ss;
    ss << "missing layer " << layer;
    throw std::out_of_range(ss.str());
  }

  return *layers_.at(layer);
}

std::optional<NodeRef> FloorGraph::getNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return std::nullopt;
  }

  return std::cref(*getNodePtr(node_id, iter->second));
}

std::optional<LayerKey> FloorGraph::getLayerForNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return std::nullopt;
  }

  return iter->second;
}

std::optional<EdgeRef> FloorGraph::getEdge(NodeId source, NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  if (source_key == target_key) {
    return layerFromKey(source_key).getEdge(source, target);
  }

  return std::cref(interlayer_edges_.get(source, target));

}

bool FloorGraph::removeNode(NodeId node_id) {
  if (!hasNode(node_id)) {
    return false;
  }

  const auto info = node_lookup_.at(node_id);

  auto node = getNodePtr(node_id, info);
  if (node->hasParent()) {
    removeInterlayerEdge(node_id, node->parent_);
  }

  std::set<NodeId> targets_to_erase = node->children_;
  for (const auto& target : targets_to_erase) {
    removeInterlayerEdge(node_id, target);
  }

  layerFromKey(info).removeNode(node_id);
  node_lookup_.erase(node_id);
  return true;
}

bool FloorGraph::hasEdge(NodeId source,
                                NodeId target,
                                LayerKey* source_key,
                                LayerKey* target_key) const {
  auto source_iter = node_lookup_.find(source);
  if (source_iter == node_lookup_.end()) {
    return false;
  }

  auto target_iter = node_lookup_.find(target);
  if (target_iter == node_lookup_.end()) {
    return false;
  }

  if (source_key != nullptr) {
    *source_key = source_iter->second;
  }

  if (target_key != nullptr) {
    *target_key = target_iter->second;
  }

  if (source_iter->second == target_iter->second) {
    return layerFromKey(source_iter->second).hasEdge(source, target);
  }

  return interlayer_edges_.contains(source, target);

}

bool FloorGraph::removeEdge(NodeId source, NodeId target) {
  LayerKey source_key, target_key;
  if (!hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  if (!source_key || !target_key) {
    return false;
  }

  if (source_key == target_key) {
    return layerFromKey(source_key).removeEdge(source, target);
  }

  removeInterlayerEdge(source, target, source_key, target_key);
  return true;
}

bool FloorGraph::mergeNodes(NodeId node_from, NodeId node_to) {
  if (!hasNode(node_from) || !hasNode(node_to)) {
    return false;
  }

  if (node_from == node_to) {
    return false;
  }

  const auto info = node_lookup_.at(node_from);

  if (info != node_lookup_.at(node_to)) {
    return false;  // Cannot merge nodes of different layers
  }

  Node* node = layers_[info.layer]->nodes_.at(node_from).get();

  // Remove parent
  if (node->hasParent()) {
    rewireInterlayerEdge(node_from, node_to, node->parent_);
  }

  // Reconnect children
  std::set<NodeId> targets_to_rewire = node->children_;
  for (const auto& target : targets_to_rewire) {
    rewireInterlayerEdge(node_from, node_to, target);
  }

  // TODO(nathan) dynamic merge
  layers_[info.layer]->mergeNodes(node_from, node_to);
  node_lookup_.erase(node_from);
  return true;
}

size_t FloorGraph::numLayers() const {

  return layers_.size();
}

size_t FloorGraph::numNodes() const {
  size_t total_nodes = 0u;
  for (const auto& id_layer_pair : layers_) {
    total_nodes += id_layer_pair.second->numNodes();
  }

  return total_nodes;
}

size_t FloorGraph::numEdges() const {
  size_t total_edges = interlayer_edges_.size();
  for (const auto& id_layer_pair : layers_) {
    total_edges += id_layer_pair.second->numEdges();
  }

  return total_edges;
}

bool FloorGraph::updateFromLayer(SceneGraphLayer& other_layer,
                                        std::unique_ptr<Edges>&& edges) {
  // TODO(nathan) consider condensing with mergeGraph
  if (!layers_.count(other_layer.id)) {
    SG_LOG(ERROR) << "Scene graph does not have layer: " << other_layer.id << std::endl;
    return false;
  }

  auto& internal_layer = *layers_.at(other_layer.id);
  for (auto& id_node_pair : other_layer.nodes_) {
    if (internal_layer.hasNode(id_node_pair.first)) {
      // just copy the attributes (prior edge information should be preserved)
      internal_layer.nodes_[id_node_pair.first]->attributes_ =
          std::move(id_node_pair.second->attributes_);
    } else {
      // we need to let the scene graph know about new nodes
      node_lookup_[id_node_pair.first] = internal_layer.id;
      internal_layer.nodes_[id_node_pair.first] = std::move(id_node_pair.second);
      internal_layer.nodes_status_[id_node_pair.first] = NodeStatus::NEW;
    }
  }

  // we just invalidated all the nodes in the other layer, so we better reset everything
  other_layer.reset();

  if (!edges) {
    return true;
  }

  for (auto& id_edge_pair : *edges) {
    auto& edge = id_edge_pair.second;
    if (internal_layer.hasEdge(edge.source, edge.target)) {
      internal_layer.edges_.edges.at(id_edge_pair.first).info = std::move(edge.info);
      continue;
    }

    internal_layer.insertEdge(edge.source, edge.target, std::move(edge.info));
  }

  // we just invalidated all the info for the new edges, so reset the edges
  edges.reset();
  return true;
}

inline NodeId getMergedId(NodeId original,
                          const std::map<NodeId, NodeId>& previous_merges) {
  return previous_merges.count(original) ? previous_merges.at(original) : original;
}

std::vector<NodeId> FloorGraph::getRemovedNodes(bool clear_removed) {
  std::vector<NodeId> to_return;
  visitLayers([&](LayerKey, BaseLayer* layer) {
    layer->getRemovedNodes(to_return, clear_removed);
  });
  return to_return;
}

std::vector<NodeId> FloorGraph::getNewNodes(bool clear_new) {
  std::vector<NodeId> to_return;
  visitLayers(
      [&](LayerKey, BaseLayer* layer) { layer->getNewNodes(to_return, clear_new); });
  return to_return;
}

std::vector<EdgeKey> FloorGraph::getRemovedEdges(bool clear_removed) {
  std::vector<EdgeKey> to_return;
  visitLayers([&](LayerKey, BaseLayer* layer) {
    layer->getRemovedEdges(to_return, clear_removed);
  });

  interlayer_edges_.getRemoved(to_return, clear_removed);
  return to_return;
}

std::vector<EdgeKey> FloorGraph::getNewEdges(bool clear_new) {
  std::vector<EdgeKey> to_return;
  visitLayers(
      [&](LayerKey, BaseLayer* layer) { layer->getNewEdges(to_return, clear_new); });

  interlayer_edges_.getNew(to_return, clear_new);
  return to_return;
}

Eigen::Vector3d FloorGraph::getPosition(NodeId node) const {
  auto iter = node_lookup_.find(node);
  if (iter == node_lookup_.end()) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the graph");
  }

  auto info = iter->second;

  return layers_.at(info.layer)->getPosition(node);
}

void FloorGraph::visitLayers(const LayerVisitor& cb) {
  for (auto& id_layer_pair : layers_) {
    cb(id_layer_pair.first, id_layer_pair.second.get());
  }
}

SceneGraphNode* FloorGraph::getNodePtr(NodeId node, const LayerKey& info) const {
    
  return layers_.at(info.layer)->nodes_.at(node).get();
}

bool FloorGraph::addAncestry(NodeId source,
                                    NodeId target,
                                    const LayerKey& source_key,
                                    const LayerKey& target_key) {
  SceneGraphNode* source_node = getNodePtr(source, source_key);
  SceneGraphNode* target_node = getNodePtr(target, target_key);
  if (source_key.isParent(target_key)) {
    if (target_node->hasParent()) {
      return false;
    }
    source_node->children_.insert(target);
    target_node->setParent(source);
  } else if (target_key.isParent(source_key)) {
    if (source_node->hasParent()) {
      return false;
    }
    target_node->children_.insert(source);
    source_node->setParent(target);
  } else {
    source_node->siblings_.insert(target);
    target_node->siblings_.insert(source);
  }

  return true;
}

void FloorGraph::removeAncestry(NodeId source,
                                       NodeId target,
                                       const LayerKey& source_key,
                                       const LayerKey& target_key) {
  SceneGraphNode* source_node = getNodePtr(source, source_key);
  SceneGraphNode* target_node = getNodePtr(target, target_key);

  if (source_key.isParent(target_key)) {
    source_node->children_.erase(target);
    target_node->clearParent();
  } else if (target_key.isParent(source_key)) {
    target_node->children_.erase(source);
    source_node->clearParent();
  } else {
    source_node->siblings_.erase(target);
    target_node->siblings_.erase(source);
  }
}

void FloorGraph::removeInterlayerEdge(NodeId source,
                                             NodeId target,
                                             const LayerKey& source_key,
                                             const LayerKey& target_key) {
  removeAncestry(source, target, source_key, target_key);

  interlayer_edges_.remove(source, target);
}

void FloorGraph::rewireInterlayerEdge(NodeId source,
                                             NodeId new_source,
                                             NodeId target) {
  if (source == new_source) {
    return;
  }

  const auto& source_key = node_lookup_.at(source);

  LayerKey new_source_key, target_key;
  if (hasEdge(new_source, target, &new_source_key, &target_key)) {
    removeInterlayerEdge(source, target, source_key, target_key);
    return;
  }

  removeAncestry(source, target, source_key, target_key);
  bool new_source_has_parent =
      !addAncestry(new_source, target, new_source_key, target_key);

  // TODO(nathan) edges can technically jump from dynamic to static, so problems with
  // index not being available in other container
  EdgeAttributes::Ptr attrs;

  attrs = interlayer_edges_.get(source, target).info->clone();
  interlayer_edges_.remove(source, target);

  if (new_source_has_parent) {
    // we silently drop edges when the new source node also has a parent
    return;
  }

  interlayer_edges_.insert(new_source, target, std::move(attrs));
}

FloorGraph::LayerIds getDefaultFloorgraphLayerIds() {
  return {
      DsgLayers::OBJECTS, DsgLayers::PLACES, DsgLayers::ROOMS, DsgLayers::BUILDINGS};
}

}  // namespace spark_dsg
