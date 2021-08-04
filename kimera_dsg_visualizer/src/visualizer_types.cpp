#include "kimera_dsg_visualizer/visualizer_types.h"

#include <ros/ros.h>

namespace kimera {

#define READ_PARAM(nh, c, name)                                            \
  if (!nh.param(#name, c.name, c.name)) {                                  \
    ROS_INFO_STREAM("failed to find param " #name " under namespace "      \
                    << nh.getNamespace() << ". defaulting to " << c.name); \
  }                                                                        \
  static_assert(true, "")

VisualizerConfig getVisualizerConfig(const std::string& visualizer_namespace) {
  ros::NodeHandle nh(visualizer_namespace);
  VisualizerConfig config;
  READ_PARAM(nh, config, layer_z_step);
  READ_PARAM(nh, config, mesh_edge_break_ratio);
  READ_PARAM(nh, config, mesh_layer_offset);
  READ_PARAM(nh, config, collapse_layers);
  READ_PARAM(nh, config, color_places_by_distance);
  READ_PARAM(nh, config, places_colormap_min_distance);
  READ_PARAM(nh, config, places_colormap_max_distance);
  return config;
}

LayerConfig getLayerConfig(const std::string& layer_namespace) {
  ros::NodeHandle nh(layer_namespace);
  LayerConfig config;
  READ_PARAM(nh, config, visualize);
  READ_PARAM(nh, config, z_offset_scale);
  READ_PARAM(nh, config, marker_scale);
  READ_PARAM(nh, config, marker_alpha);
  READ_PARAM(nh, config, use_sphere_marker);
  READ_PARAM(nh, config, use_label);
  READ_PARAM(nh, config, label_height);
  READ_PARAM(nh, config, label_scale);
  READ_PARAM(nh, config, use_bounding_box);
  READ_PARAM(nh, config, bounding_box_alpha);
  READ_PARAM(nh, config, use_edge_source);
  READ_PARAM(nh, config, interlayer_edge_scale);
  READ_PARAM(nh, config, interlayer_edge_alpha);
  READ_PARAM(nh, config, interlayer_edge_use_color);
  READ_PARAM(nh, config, interlayer_edge_insertion_skip);
  READ_PARAM(nh, config, intralayer_edge_scale);
  READ_PARAM(nh, config, intralayer_edge_alpha);
  READ_PARAM(nh, config, intralayer_edge_use_color);
  READ_PARAM(nh, config, intralayer_edge_insertion_skip);
  return config;
}

ColormapConfig getColormapConfig(const std::string& colormap_namespace) {
  ros::NodeHandle nh(colormap_namespace);
  ColormapConfig config;
  READ_PARAM(nh, config, min_hue);
  READ_PARAM(nh, config, max_hue);
  READ_PARAM(nh, config, min_saturation);
  READ_PARAM(nh, config, max_saturation);
  READ_PARAM(nh, config, min_luminance);
  READ_PARAM(nh, config, max_luminance);
  return config;
}

#undef READ_PARAM

}  // namespace kimera