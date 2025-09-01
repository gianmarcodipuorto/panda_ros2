#include "std_msgs/msg/color_rgba.hpp"
#include <QApplication>
#include <QFont>
#include <QLabel>
#include <QPalette>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>
#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using std_msgs::msg::ColorRGBA;

class ColorWidget : public QWidget {
public:
  ColorWidget(std::string state_topic_name,
              std::string wrist_contact_topic_name)
      : state_topic_name(state_topic_name),
        wrist_contact_topic_name(wrist_contact_topic_name) {

    this->resize(QSize(400.0, 500.0));

    QVBoxLayout *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(0, 0, 0, 0); // Remove margins
    main_layout->setSpacing(0); // Remove spacing between widgets

    // --- Top Bar Widget for Topic Names ---
    top_bar_widget_ = new QWidget(this);
    QVBoxLayout *top_bar_layout = new QVBoxLayout(top_bar_widget_);
    top_bar_layout->setContentsMargins(5, 5, 5, 5);

    state_topic_label_ =
        new QLabel(QString::fromStdString("State Topic: " + state_topic_name),
                   top_bar_widget_);
    QFont font_topic = state_topic_label_->font();
    font_topic.setPointSize(16);
    state_topic_label_->setFont(font_topic);
    state_topic_label_->setAlignment(Qt::AlignCenter);
    top_bar_layout->addWidget(state_topic_label_);

    joint_topic_label_ =
        new QLabel(QString::fromStdString("Wrist Contact Topic: " +
                                          wrist_contact_topic_name),
                   top_bar_widget_);
    joint_topic_label_->setFont(font_topic); // Use same font as state topic
    joint_topic_label_->setAlignment(Qt::AlignCenter);
    top_bar_layout->addWidget(joint_topic_label_);

    // Set top bar background to white
    QPalette pal_top = top_bar_widget_->palette();
    pal_top.setColor(QPalette::Window, Qt::white);
    top_bar_widget_->setPalette(pal_top);
    top_bar_widget_->setAutoFillBackground(true);

    top_bar_widget_->setLayout(
        top_bar_layout); // Set layout for top_bar_widget_
    main_layout->addWidget(top_bar_widget_);

    // --- Color Area Widget for Joint Index ---
    color_area_widget_ = new QWidget(this);
    QVBoxLayout *color_area_layout = new QVBoxLayout(color_area_widget_);
    color_area_layout->setContentsMargins(0, 0, 0, 0);
    color_area_layout->setSpacing(0);
    // Ensure color area can expand
    color_area_widget_->setSizePolicy(QSizePolicy::Expanding,
                                      QSizePolicy::Expanding);

    // Label for joint index (big number in the middle)
    wrist_index_label_ = new QLabel("", this);
    QFont font_index = wrist_index_label_->font();
    font_index.setPointSize(72);
    wrist_index_label_->setFont(font_index);
    wrist_index_label_->setAlignment(Qt::AlignCenter);
    color_area_layout->addWidget(wrist_index_label_);
    wrist_index_label_->hide(); // Initially hidden

    color_area_widget_->setLayout(
        color_area_layout); // Set layout for color_area_widget_
    main_layout->addWidget(color_area_widget_);

    setLayout(main_layout);
    // autoFillBackground(true); // Handled by sub-widgets
  }

  void updateColor(ColorRGBA value) {
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(value.r, value.g, value.b));
    color_area_widget_->setPalette(pal); // Apply to color_area_widget_
    color_area_widget_->setAutoFillBackground(true); // Ensure it fills
  }

  const char *get_topic_name() { return state_topic_name.c_str(); }

  void updateWristContactIndex(std::string wrist) {
    // As per user request, -1 represents no contact.
    if (wrist.empty()) {
      wrist_index_label_->hide();
    } else {
      wrist_index_label_->setText(QString(wrist.c_str()));
      wrist_index_label_->show();
    }
  }

  const char *get_wrist_contact_topic_name() {
    return wrist_contact_topic_name.c_str();
  }

private:
  QLabel *state_topic_label_; // Renamed from 'label_'
  QLabel *joint_topic_label_; // New label for joint contact topic in top bar
  QLabel *wrist_index_label_; // Member for displaying joint index

  QWidget *top_bar_widget_;    // New widget for the top bar
  QWidget *color_area_widget_; // New widget for the color changing area

  std::string state_topic_name; // Keep these for constructor initialization
  std::string
      wrist_contact_topic_name; // Keep these for constructor initialization
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("color_widget_node");

  std::string state_topic_name, wrist_contact_topic_name;
  if (argc != 3) {
    RCLCPP_ERROR(node->get_logger(),
                 "Usage: <state_topic_name> <wrist_contact_topic_name>");
    rclcpp::shutdown();
    return -1;
  } else {
    state_topic_name = std::string(argv[1]);
    wrist_contact_topic_name = std::string(argv[2]);
    RCLCPP_INFO(node->get_logger(), "Topic to listen for state: %s",
                state_topic_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Topic to listen for wrist contact: %s",
                wrist_contact_topic_name.c_str());
  }

  QApplication app(argc, argv);
  ColorWidget widget(state_topic_name, wrist_contact_topic_name);
  widget.show();

  auto sub = node->create_subscription<std_msgs::msg::ColorRGBA>(
      widget.get_topic_name(), 10,
      [&widget](std_msgs::msg::ColorRGBA::SharedPtr msg) {
        widget.updateColor(*msg);
      });

  auto joint_sub = node->create_subscription<std_msgs::msg::String>(
      widget.get_wrist_contact_topic_name(), 10,
      [&widget](std_msgs::msg::String msg) {
        widget.updateWristContactIndex(std::string{msg.data});
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread ros_thread([&exec]() { exec.spin(); });

  int ret = app.exec();
  rclcpp::shutdown();
  ros_thread.join();
  return ret;
}
