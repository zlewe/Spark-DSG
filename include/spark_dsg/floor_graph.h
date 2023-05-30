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
#pragma once
#include "spark_dsg/scene_graph_layer.h"

#include <map>
#include <memory>
#include <type_traits>

namespace spark_dsg {

/**
 * @brief Floorplan Graph Class
 * 
 * Layers: 
 *  - Floorplan Layer
 *  - 
 * 
 */
class FloorGraph {
 public:
  using NodeRef = BaseLayer::NodeRef;
  using EdgeRef = BaseLayer::EdgeRef;

  //! Desired pointer type of the scene graph
  using Ptr = std::shared_ptr<FloorGraph>;
  //! container type for the layer ids
  using LayerIds = std::vector<LayerId>;
  //! Edge container
  using Edges = EdgeContainer::Edges;
  //! Layer container
  using Layers = std::map<LayerId, SceneGraphLayer::Ptr>;
  //! Callback type
  using LayerVisitor = std::function<void(LayerKey, BaseLayer*)>;

  friend class SceneGraphLogger;
  friend class SceneGraphLayer;
  friend class BaseLayer;

  /**
   * @brief Construct the scene graph (with a default layer factory)
   */
  explicit FloorGraph();

  /**
   * @brief Construct the scene graph (with a provided layer factory)
   * @param factory List of layer ids
   */
  FloorGraph(const LayerIds& factory);

  /**
   * @brief Default destructor
   */
  virtual ~FloorGraph() = default;

  /**
   * @brief Delete all layers and edges
   */
  void clear();

  /**
   * @brief construct and add a node to the specified layer in the graph
   * @param layer_id layer to add to
   * @param node_id node to create
   * @param attrs node attributes
   * @return true if the node was added successfully
   */
  bool emplaceNode(LayerId layer_id, NodeId node_id, NodeAttributes::Ptr&& attrs);

  /**
   * @brief add a node to the graph
   *
   * Checks that the layer id matches a current layer, that the node
   * is not null and the node doesn't already exist
   *
   * @param node to add
   * @return true if the node was added successfully
   */
  bool insertNode(SceneGraphNode::Ptr&& node);

  /**
   * @brief Add an edge to the graph
   *
   * Checks that the edge doesn't already exist and
   * that the source and target already exist. Handles passing
   * the edge to the layer if the edge is a intra-layer edge,
   * and updates parents and children of the respective nodes
   *
   * @param source start node
   * @param target end node
   * @param edge_info optional edge attributes (will use
   *        default edge attributes if not supplied)
   * @return true if the edge was successfully added
   */
  bool insertEdge(NodeId source,
                  NodeId target,
                  EdgeAttributes::Ptr&& edge_info = nullptr);

  /**
   * @brief Set the attributes of an existing node
   * @param node Node ID to set the attributes for
   * @param attrs New attributes for the node
   * @return Returns true if update was successful
   */
  bool setNodeAttributes(NodeId node, NodeAttributes::Ptr&& attrs);

  /**
   * @brief Set the attributes of an existing edge
   * @param source Source ID to set the attributes for
   * @param target Target ID to set the attributes for
   * @param attrs New attributes for the edge
   * @return Returns true if update was successful
   */
  bool setEdgeAttributes(NodeId source, NodeId target, EdgeAttributes::Ptr&& attrs);

  /**
   * @brief Check whether the layer exists and is valid
   * @param layer_id Layer id to check
   * @returns Returns true if the layer exists and is valid
   */
  bool hasLayer(LayerId layer_id) const;

  /**
   * @brief check if a given node exists
   * @param node_id node to check for
   * @returns true if the given node exists
   */
  bool hasNode(NodeId node_id) const;

  /**
   * @brief Check the status of a node
   * @param node_id node to check for
   * @returns status of type NodeStatus
   */
  NodeStatus checkNode(NodeId node_id) const;

  /**
   * @brief check if a given edge exists
   *
   * This checks either the presence of an
   * edge from source to target or from target
   * to source
   *
   * @param source source id of edge to check for
   * @param target target id of edge to check for
   * @returns true if the given edge exists
   */
  inline bool hasEdge(NodeId source, NodeId target) const {
    return hasEdge(source, target, nullptr, nullptr);
  }

  /**
   * @brief Get a layer if the layer exists
   * @param layer_id layer to get
   * @returns a constant reference to the requested layer
   * @throws std::out_of_range if the layer doesn't exist
   */
  const SceneGraphLayer& getLayer(LayerId layer_id) const;

  /**
   * @brief Get a particular node in the graph
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns a potentially valid node constant reference
   */
  std::optional<NodeRef> getNode(NodeId node_id) const;

  std::optional<LayerKey> getLayerForNode(NodeId node_id) const;

  /**
   * @brief Get a particular edge in the graph
   *
   * This can be used to update the edge "info", though
   * information about the edge (i.e. source and target) cannot
   * be modified
   *
   * @param source source of edge to get
   * @param target target of edge to get
   * @returns a potentially valid edge constant reference
   */
  std::optional<EdgeRef> getEdge(NodeId source, NodeId target) const;

  /**
   * @brief Remove a node from the graph
   * @param Node node to remove
   * @returns Returns true if the removal was successful
   */
  bool removeNode(NodeId node);

  /**
   * @brief Remove an edge from the graph
   * @param source Source of edge to remove
   * @param target Target of edge to remove
   * @returns Returns true if the removal was successful
   */
  bool removeEdge(NodeId source, NodeId target);

  /**
   * @brief merge two nodes
   * @param node_from node to remove
   * @param node_to node to merge to
   * @returns true if operation succeeded
   */
  bool mergeNodes(NodeId node_from, NodeId node_to);

  /**
   * @brief Get the number of layers in the graph
   * @return number of layers in the graph
   */
  size_t numLayers() const;

  /**
   * @brief Get the total number of nodes in the graph
   * @return The number of nodes in the graph
   */
  size_t numNodes() const;

  /**
   * @brief Get number of edges in the graph
   * @return number of edges in the graph
   */
  size_t numEdges() const;

  /**
   * @brief Get whether or not the scene graph is empty
   * @note the scene graph invariants make it so only nodes have to be checked
   * @return true if the scene graph is empty
   */
  inline bool empty() const { return numNodes() == 0; }

  /**
   * @brief Get the position of a node in the layer with bounds checking
   */
  Eigen::Vector3d getPosition(NodeId node) const;

  /**
   * @brief Update graph from separate layer
   * @note Will invalidate the layer and edges passed in
   * @param other_layer Layer to update from
   * @param edges Optional edges to add to graph
   * @return Whether the update was successful or not
   */
  bool updateFromLayer(SceneGraphLayer& other_layer,
                       std::unique_ptr<Edges>&& edges = nullptr);

  std::vector<NodeId> getRemovedNodes(bool clear_removed = false);

  std::vector<NodeId> getNewNodes(bool clear_new = false);

  std::vector<EdgeKey> getRemovedEdges(bool clear_removed = false);

  std::vector<EdgeKey> getNewEdges(bool clear_new = false);

  void save(const std::string& filepath, bool include_mesh = true) const;

  std::string serialize(bool include_mesh = false) const;

  static Ptr load(const std::string& filepath);

  static Ptr deserialize(const std::string& contents);

  // current static layer ids in the graph
  const LayerIds layer_ids;

 protected:
  void visitLayers(const LayerVisitor& cb);

  bool hasEdge(NodeId source,
               NodeId target,
               LayerKey* source_key,
               LayerKey* target_key) const;

  BaseLayer& layerFromKey(const LayerKey& key);

  const BaseLayer& layerFromKey(const LayerKey& key) const;

  SceneGraphNode* getNodePtr(NodeId node, const LayerKey& key) const;

  void removeInterlayerEdge(NodeId source,
                            NodeId target,
                            const LayerKey& source_key,
                            const LayerKey& target_key);

  inline void removeInterlayerEdge(NodeId n1, NodeId n2) {
    removeInterlayerEdge(n1, n2, node_lookup_.at(n1), node_lookup_.at(n2));
  }

  void rewireInterlayerEdge(NodeId source, NodeId new_source, NodeId target);

  bool addAncestry(NodeId source,
                   NodeId target,
                   const LayerKey& source_key,
                   const LayerKey& target_key);

  void removeAncestry(NodeId source,
                      NodeId target,
                      const LayerKey& source_key,
                      const LayerKey& target_key);

 protected:
  Layers layers_;

  std::map<NodeId, LayerKey> node_lookup_;

  EdgeContainer interlayer_edges_;

 public:
  /**
   * @brief constant iterator around the layers
   */
  inline const Layers& layers() const { return layers_; };

  /**
   * @brief constant iterator around the inter-layer edges
   *
   * @note inter-layer edges are edges between nodes in different layers
   */
  inline const Edges& interlayer_edges() const { return interlayer_edges_.edges; };

};

// /**
//  * @brief Return a container of the layer hierarchy from #spark_dsg::DsgLayers
//  */
FloorGraph::LayerIds getDefaultFloorgraphLayerIds();

}  // namespace spark_dsg
