#include "planner_status_text.h"

#include <chrono>
#include <mutex>
#include <QLabel>
#include <common_msgs_pkg/ComponentStatus.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/properties/float_property.h>

namespace visualization_pkg {
class PlannerStatusDisplay : public rviz::Display {
public:
  PlannerStatusDisplay() {
    timeout_ = new rviz::FloatProperty("Display timeout", 2.0,
        "Seconds without a new status before displaying STALE; display only.", this);
    timeout_->setMin(0.1);
  }
  ~PlannerStatusDisplay() override {
    subscriber_.shutdown();
    delete label_;
  }
  void onInitialize() override {
    label_ = new QLabel(context_->getViewManager()->getRenderPanel());
    label_->setAttribute(Qt::WA_TransparentForMouseEvents);
    label_->setStyleSheet("QLabel { color: #eaf4ff; background: rgba(19,27,38,220);"
                         " padding: 8px 12px; border: 1px solid #607d98;"
                         " border-radius: 4px; font-size: 17px; font-weight: bold; }");
    label_->move(12, 12);
    label_->hide();
  }
  void onEnable() override {
    { std::lock_guard<std::mutex> guard(mutex_); received_ = false; }
    subscriber_ = node_.subscribe("/molit/internal/visualization/planner_status", 1,
                                 &PlannerStatusDisplay::receive, this);
    label_->setText("Planner: WAITING");
    label_->adjustSize();
    label_->show();
  }
  void onDisable() override {
    subscriber_.shutdown();
    if (label_) label_->hide();
  }
  void update(float, float) override {
    if (!label_ || !isEnabled()) return;
    std::string text;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      text = received_ ? text_ : "Planner: WAITING";
      if (received_ && std::chrono::duration<double>(Clock::now() - arrival_).count() > timeout_->getFloat())
        text += " | STALE";
    }
    label_->setText(QString::fromStdString(text));
    label_->adjustSize();
    label_->raise();
  }
private:
  using Clock = std::chrono::steady_clock;
  void receive(const common_msgs_pkg::ComponentStatus::ConstPtr& message) {
    if (message->component != "path_planning_pkg") return;
    std::lock_guard<std::mutex> guard(mutex_);
    text_ = plannerStatusText(message->reason, message->stop_required);
    // A latched status from a stopped producer must not appear current.
    const double age = (ros::Time::now() - message->header.stamp).toSec();
    arrival_ = Clock::now() - std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(message->header.stamp.isZero() || age < 0 ? 1e6 : age));
    received_ = true;
  }
  ros::NodeHandle node_;
  ros::Subscriber subscriber_;
  QLabel* label_ = nullptr;
  rviz::FloatProperty* timeout_;
  std::mutex mutex_;
  bool received_ = false;
  std::string text_;
  Clock::time_point arrival_;
};
}  // namespace visualization_pkg

PLUGINLIB_EXPORT_CLASS(visualization_pkg::PlannerStatusDisplay, rviz::Display)
