// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// save_map_panel.hpp -- RViz2 panel plugin for TofSLAM map saving.
//
// Provides a GUI panel in RViz2 with:
//   - Editable service name field
//   - "Save Map" button
//   - Status display
// Calls std_srvs/srv/Trigger to request map save from tofslam_node.

#ifndef TOF_SLAM_RVIZ_SAVE_MAP_PANEL_HPP_
#define TOF_SLAM_RVIZ_SAVE_MAP_PANEL_HPP_

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rviz_common/panel.hpp>

namespace tof_slam {

class SaveMapPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit SaveMapPanel(QWidget* parent = nullptr);
  ~SaveMapPanel() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 protected Q_SLOTS:
  void onSaveMapClicked();
  void onServiceNameChanged();
  void checkServiceResponse();

 private:
  void updateServiceClient();

  // UI elements
  QLineEdit* service_name_edit_{nullptr};
  QPushButton* save_map_btn_{nullptr};
  QLabel* status_label_{nullptr};
  QLabel* last_save_label_{nullptr};

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture pending_future_;
  bool request_pending_{false};

  // Timer for checking async response
  QTimer* response_timer_{nullptr};

  // Config
  QString service_name_{"/tofslam_node/save_map"};
};

}  // namespace tof_slam

#endif  // TOF_SLAM_RVIZ_SAVE_MAP_PANEL_HPP_
