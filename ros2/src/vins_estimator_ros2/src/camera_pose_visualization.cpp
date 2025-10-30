/*******************************************************
 * Copyright (C) 2025
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#include "vins_estimator_ros2/camera_pose_visualization.hpp"

#include <geometry_msgs/msg/point.hpp>

namespace vins_multi {

using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {
void EigenToPoint(const Eigen::Vector3d & v, Point & p) {
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}
}  // namespace

const Eigen::Vector3d CameraPoseVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrt = Eigen::Vector3d(1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imlb = Eigen::Vector3d(-1.0, 0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrb = Eigen::Vector3d(1.0, 0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);

CameraPoseVisualization::CameraPoseVisualization(float r, float g, float b, float a)
: m_marker_ns("CameraPoseVisualization"), m_scale(0.2), m_line_width(0.01) {
  m_image_boundary_color.r = r;
  m_image_boundary_color.g = g;
  m_image_boundary_color.b = b;
  m_image_boundary_color.a = a;
  m_optical_center_connector_color = m_image_boundary_color;
}

void CameraPoseVisualization::setImageBoundaryColor(float r, float g, float b, float a) {
  m_image_boundary_color.r = r;
  m_image_boundary_color.g = g;
  m_image_boundary_color.b = b;
  m_image_boundary_color.a = a;
}

void CameraPoseVisualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
  m_optical_center_connector_color.r = r;
  m_optical_center_connector_color.g = g;
  m_optical_center_connector_color.b = b;
  m_optical_center_connector_color.a = a;
}

void CameraPoseVisualization::setScale(double s) {
  m_scale = s;
}

void CameraPoseVisualization::setLineWidth(double width) {
  m_line_width = width;
}

void CameraPoseVisualization::add_edge(const Eigen::Vector3d & p0, const Eigen::Vector3d & p1) {
  Marker marker;
  marker.ns = m_marker_ns;
  marker.id = static_cast<int>(m_markers.size()) + 1;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.scale.x = 0.005;
  marker.color.g = 1.0f;
  marker.color.a = 1.0f;

  Point point0;
  Point point1;
  EigenToPoint(p0, point0);
  EigenToPoint(p1, point1);

  marker.points.push_back(point0);
  marker.points.push_back(point1);
  marker.colors.push_back(m_image_boundary_color);
  marker.colors.push_back(m_image_boundary_color);

  m_markers.push_back(marker);
}

void CameraPoseVisualization::add_loopedge(const Eigen::Vector3d & p0, const Eigen::Vector3d & p1) {
  Marker marker;
  marker.ns = m_marker_ns;
  marker.id = static_cast<int>(m_markers.size()) + 1;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.scale.x = 0.04;
  marker.color.r = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  Point point0;
  Point point1;
  EigenToPoint(p0, point0);
  EigenToPoint(p1, point1);

  marker.points.push_back(point0);
  marker.points.push_back(point1);
  marker.colors.push_back(m_image_boundary_color);
  marker.colors.push_back(m_image_boundary_color);

  m_markers.push_back(marker);
}

void CameraPoseVisualization::add_pose(const Eigen::Vector3d & p, const Eigen::Quaterniond & q) {
  Marker marker;
  marker.ns = m_marker_ns;
  marker.id = static_cast<int>(m_markers.size()) + 1;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.scale.x = m_line_width;
  marker.pose.orientation.w = 1.0;

  Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;
  EigenToPoint(q * (m_scale * imlt) + p, pt_lt);
  EigenToPoint(q * (m_scale * imlb) + p, pt_lb);
  EigenToPoint(q * (m_scale * imrt) + p, pt_rt);
  EigenToPoint(q * (m_scale * imrb) + p, pt_rb);
  EigenToPoint(q * (m_scale * lt0) + p, pt_lt0);
  EigenToPoint(q * (m_scale * lt1) + p, pt_lt1);
  EigenToPoint(q * (m_scale * lt2) + p, pt_lt2);
  EigenToPoint(q * (m_scale * oc) + p, pt_oc);

  auto push_edge = [&](const Point & a, const Point & b, const std_msgs::msg::ColorRGBA & color) {
    marker.points.push_back(a);
    marker.points.push_back(b);
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  };

  push_edge(pt_lt, pt_lb, m_image_boundary_color);
  push_edge(pt_lb, pt_rb, m_image_boundary_color);
  push_edge(pt_rb, pt_rt, m_image_boundary_color);
  push_edge(pt_rt, pt_lt, m_image_boundary_color);

  push_edge(pt_lt0, pt_lt1, m_image_boundary_color);
  push_edge(pt_lt1, pt_lt2, m_image_boundary_color);

  push_edge(pt_lt, pt_oc, m_optical_center_connector_color);
  push_edge(pt_lb, pt_oc, m_optical_center_connector_color);
  push_edge(pt_rt, pt_oc, m_optical_center_connector_color);
  push_edge(pt_rb, pt_oc, m_optical_center_connector_color);

  m_markers.push_back(marker);
}

void CameraPoseVisualization::reset() {
  m_markers.clear();
}

void CameraPoseVisualization::publish_by(
  const rclcpp::Publisher<MarkerArray>::SharedPtr & pub,
  const std_msgs::msg::Header & header) {
  if (!pub) {
    return;
  }

  MarkerArray marker_array;
  marker_array.markers.reserve(m_markers.size());
  for (auto marker : m_markers) {
    marker.header = header;
    marker_array.markers.push_back(marker);
  }
  pub->publish(marker_array);
}

void CameraPoseVisualization::publish_clear(
  const rclcpp::Publisher<MarkerArray>::SharedPtr & pub,
  const std_msgs::msg::Header & header) {
  if (!pub) {
    return;
  }

  MarkerArray marker_array;
  for (auto & marker : m_markers) {
    Marker clear_marker = marker;
    clear_marker.header = header;
    clear_marker.action = Marker::DELETE;
    marker_array.markers.push_back(clear_marker);
  }
  pub->publish(marker_array);
}

}  // namespace vins_multi
