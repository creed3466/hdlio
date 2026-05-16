#include "tof_slam/rviz/save_map_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFont>
#include <QDateTime>
#include <rviz_common/display_context.hpp>

namespace tof_slam {

SaveMapPanel::SaveMapPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
  // Main layout
  auto* layout = new QVBoxLayout();

  // Title
  auto* title = new QLabel("TofSLAM Map Control");
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(12);
  title->setFont(title_font);
  layout->addWidget(title);

  // Service name group
  auto* service_group = new QGroupBox("Service");
  auto* service_layout = new QHBoxLayout();
  auto* service_label = new QLabel("Name:");
  service_name_edit_ = new QLineEdit(service_name_);
  service_layout->addWidget(service_label);
  service_layout->addWidget(service_name_edit_);
  service_group->setLayout(service_layout);
  layout->addWidget(service_group);

  // Save button
  save_map_btn_ = new QPushButton("Save Map");
  save_map_btn_->setMinimumHeight(40);
  save_map_btn_->setStyleSheet(
      "QPushButton { background-color: #4CAF50; color: white; font-size: 14px; font-weight: bold; border-radius: 5px; }"
      "QPushButton:hover { background-color: #45a049; }"
      "QPushButton:pressed { background-color: #3d8b40; }"
      "QPushButton:disabled { background-color: #cccccc; color: #666666; }");
  layout->addWidget(save_map_btn_);

  // Status group
  auto* status_group = new QGroupBox("Status");
  auto* status_layout = new QVBoxLayout();
  status_label_ = new QLabel("Ready");
  status_label_->setWordWrap(true);
  last_save_label_ = new QLabel("Last save: -");
  status_layout->addWidget(status_label_);
  status_layout->addWidget(last_save_label_);
  status_group->setLayout(status_layout);
  layout->addWidget(status_group);

  layout->addStretch();
  setLayout(layout);

  // Connections
  connect(save_map_btn_, &QPushButton::clicked, this, &SaveMapPanel::onSaveMapClicked);
  connect(service_name_edit_, &QLineEdit::editingFinished, this, &SaveMapPanel::onServiceNameChanged);

  // Response check timer
  response_timer_ = new QTimer(this);
  response_timer_->setInterval(100);  // 100ms polling
  connect(response_timer_, &QTimer::timeout, this, &SaveMapPanel::checkServiceResponse);
}

void SaveMapPanel::onInitialize() {
  // Get the ROS node from RViz context
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  updateServiceClient();
}

void SaveMapPanel::updateServiceClient() {
  if (!node_) return;
  client_ = node_->create_client<std_srvs::srv::Trigger>(
      service_name_.toStdString());
}

void SaveMapPanel::onServiceNameChanged() {
  QString new_name = service_name_edit_->text().trimmed();
  if (new_name != service_name_ && !new_name.isEmpty()) {
    service_name_ = new_name;
    updateServiceClient();
    status_label_->setText("Service updated: " + service_name_);
    Q_EMIT configChanged();
  }
}

void SaveMapPanel::onSaveMapClicked() {
  if (!client_) {
    status_label_->setText("Error: ROS node not initialized");
    status_label_->setStyleSheet("color: red;");
    return;
  }

  if (request_pending_) {
    status_label_->setText("Request already pending...");
    return;
  }

  if (!client_->service_is_ready()) {
    status_label_->setText("Service not available: " + service_name_);
    status_label_->setStyleSheet("color: orange;");
    return;
  }

  // Send async request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  pending_future_ = client_->async_send_request(request).future.share();
  request_pending_ = true;

  save_map_btn_->setEnabled(false);
  save_map_btn_->setText("Saving...");
  status_label_->setText("Saving map...");
  status_label_->setStyleSheet("color: blue;");

  response_timer_->start();
}

void SaveMapPanel::checkServiceResponse() {
  if (!request_pending_) {
    response_timer_->stop();
    return;
  }

  // Check if future is ready
  if (pending_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    response_timer_->stop();
    request_pending_ = false;
    save_map_btn_->setEnabled(true);
    save_map_btn_->setText("Save Map");

    try {
      auto response = pending_future_.get();
      if (response->success) {
        status_label_->setText(QString::fromStdString(response->message));
        status_label_->setStyleSheet("color: green;");
        last_save_label_->setText("Last save: " +
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
      } else {
        status_label_->setText("Failed: " + QString::fromStdString(response->message));
        status_label_->setStyleSheet("color: red;");
      }
    } catch (const std::exception& e) {
      status_label_->setText("Error: " + QString(e.what()));
      status_label_->setStyleSheet("color: red;");
    }
  }
}

void SaveMapPanel::load(const rviz_common::Config& config) {
  Panel::load(config);
  QString name;
  if (config.mapGetString("service_name", &name)) {
    service_name_ = name;
    if (service_name_edit_) {
      service_name_edit_->setText(service_name_);
    }
    updateServiceClient();
  }
}

void SaveMapPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  config.mapSetValue("service_name", service_name_);
}

}  // namespace tof_slam

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tof_slam::SaveMapPanel, rviz_common::Panel)
