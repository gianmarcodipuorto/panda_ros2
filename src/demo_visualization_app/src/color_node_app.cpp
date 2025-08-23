#include "std_msgs/msg/color_rgba.hpp"
#include <QApplication>
#include <QLabel>
#include <QPalette>
#include <QVBoxLayout>
#include <QWidget>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>

using std_msgs::msg::ColorRGBA;

class ColorWidget : public QWidget {
public:
  ColorWidget(std::string state_topic_name)
      : state_topic_name(state_topic_name) {
    QVBoxLayout *layout = new QVBoxLayout(this);
    label_ = new QLabel(std::string{"Listening on " + state_topic_name}.c_str(),
                        this);
    layout->addWidget(label_);
    setLayout(layout);
    setAutoFillBackground(true);
  }

  void updateColor(ColorRGBA value) {
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(value.r, value.g, value.b));
    setPalette(pal);
  }

  const char *get_topic_name() { return state_topic_name.c_str(); }

private:
  QLabel *label_;
  std::string state_topic_name;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("color_widget_node");

  std::string topic_name;
  if (argc != 2) {
    RCLCPP_ERROR(node->get_logger(), "Usage: <topic_name>");
    rclcpp::shutdown();
    return -1;
  } else {
    topic_name = std::string(argv[1]);
    RCLCPP_INFO(node->get_logger(), "Topic to listen: %s", topic_name.c_str());
  }

  QApplication app(argc, argv);
  ColorWidget widget(topic_name);
  widget.show();

  auto sub = node->create_subscription<std_msgs::msg::ColorRGBA>(
      widget.get_topic_name(), 10,
      [&widget](std_msgs::msg::ColorRGBA::SharedPtr msg) {
        widget.updateColor(*msg);
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread ros_thread([&exec]() { exec.spin(); });

  int ret = app.exec();
  rclcpp::shutdown();
  ros_thread.join();
  return ret;
}
