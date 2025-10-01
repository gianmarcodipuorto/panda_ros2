#include "panda_interfaces/msg/human_contact.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <QApplication>
#include <QFont>
#include <QLabel>
#include <QPalette>
#include <QPushButton> // Added for the on/off button
#include <QString>
#include <QVBoxLayout>
#include <QWidget>
#include <cstdint>
#include <future>
#include <memory>
#include <qpalette.h>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <thread>

using std_msgs::msg::ColorRGBA;
using namespace std::chrono_literals;

class ColorWidget : public QWidget {
public:
  ColorWidget(std::string state_topic_name,
              std::string wrist_contact_topic_name,
              rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr demo_client)
      : state_topic_name(state_topic_name),
        wrist_contact_topic_name(wrist_contact_topic_name),
        demo_client(demo_client) {

    this->resize(QSize(400.0, 500.0));

    QVBoxLayout *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(0, 0, 0, 0); // Remove margins
    main_layout->setSpacing(0); // Remove spacing between widgets

    // --- Top Bar Widget for Topic Names ---
    top_bar_widget_ = new QWidget(this);
    top_bar_widget_->setAutoFillBackground(true);
    QVBoxLayout *top_bar_layout = new QVBoxLayout(top_bar_widget_);
    top_bar_layout->setContentsMargins(5, 5, 5, 5);

    // Add an on/off button
    on_off_button_ = new QPushButton("Demo not running", top_bar_widget_);
    QFont font_button = on_off_button_->font();
    font_button.setPointSize(16);
    on_off_button_->setFont(font_button);
    on_off_button_->setCheckable(true); // Make it a toggle button
    button_default_palette_ = on_off_button_->palette();
    top_bar_layout->addWidget(on_off_button_);
    connect(on_off_button_, &QPushButton::clicked, this,
            &ColorWidget::toggleButtonState);

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
    color_area_layout->setContentsMargins(5, 5, 5, 5);
    color_area_layout->setSpacing(0);
    // Ensure color area can expand
    color_area_widget_->setSizePolicy(QSizePolicy::Expanding,
                                      QSizePolicy::Expanding);

    // Label for joint index (big number in the middle)
    wrist_index_label_ = new QLabel("", this);
    joint_index_label_ = new QLabel("", this);
    QFont font_index = wrist_index_label_->font();
    font_index.setPointSize(48); // Slightly smaller to fit both
    wrist_index_label_->setFont(font_index);
    wrist_index_label_->setAlignment(Qt::AlignCenter);
    color_area_layout->addWidget(wrist_index_label_);
    wrist_index_label_->hide(); // Initially hidden

    // Apply the same font as wrist_index_label_
    joint_index_label_->setFont(font_index);
    joint_index_label_->setAlignment(Qt::AlignCenter);
    color_area_layout->addWidget(joint_index_label_);
    joint_index_label_->hide(); // Initially hidden

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

  void updateWristContact(panda_interfaces::msg::HumanContact contact_msg) {
    // As per user request, -1 represents no contact.
    if (contact_msg.joint_frame.data.empty()) {
      wrist_index_label_->hide();
      joint_index_label_->hide();
    } else {
      wrist_index_label_->setText(
          QString("Wrist: %1").arg(contact_msg.in_contact_wrist.data.c_str()));
      joint_index_label_->setText(
          QString("Joint: %1").arg(contact_msg.joint_frame.data.c_str()));
      wrist_index_label_->show();
      joint_index_label_->show();
    }
  }

  const char *get_wrist_contact_topic_name() {
    return wrist_contact_topic_name.c_str();
  }

private:
  QLabel *wrist_index_label_; // Member for displaying joint index
  QLabel *joint_index_label_; // Member for displaying joint index

  QWidget *top_bar_widget_;    // New widget for the top bar
  QWidget *color_area_widget_; // New widget for the color changing area
  QPushButton *on_off_button_; // New button for on/off functionality
  QPalette button_default_palette_;

  std::string state_topic_name; // Keep these for constructor initialization
  std::string
      wrist_contact_topic_name; // Keep these for constructor initialization
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr demo_client{};

private slots:
  void toggleButtonState();
};

// Slot implementation for the on/off button
void ColorWidget::toggleButtonState() {
  if (on_off_button_->isChecked()) {
    std::thread{[this]() {
      std_srvs::srv::SetBool_Request::SharedPtr demo_req =
          std::make_shared<std_srvs::srv::SetBool_Request>();
      demo_req->set__data(true);
      auto fut = demo_client->async_send_request(demo_req);
      auto status = fut.wait_for(10s);
      if (status == std::future_status::ready) {
        if (fut.get()->success) {
          on_off_button_->setText("Demo running");
          QPalette pal = palette();
          pal.setColor(QPalette::Button, QColor(0, 255, 0));
          on_off_button_->setPalette(pal);
          on_off_button_->setAutoFillBackground(true); // Ensure it fills
          RCLCPP_INFO(rclcpp::get_logger("color_widget_node"),
                      "Button State: On");
        } else {
          auto current_palette = on_off_button_->palette();
          QPalette pal = palette();
          pal.setColor(QPalette::Button, QColor(255, 0, 0));
          on_off_button_->setPalette(pal);
          on_off_button_->setAutoFillBackground(true); // Ensure it fills
          rclcpp::sleep_for(1s);
          on_off_button_->setPalette(current_palette);
          RCLCPP_INFO(rclcpp::get_logger("color_widget_node"),
                      "Button State: Off");
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("color_widget_node"),
                     "Demo request not accepted");
      }
    }}.detach();
  } else {
    std::thread{[this]() {
      std_srvs::srv::SetBool_Request::SharedPtr demo_req =
          std::make_shared<std_srvs::srv::SetBool_Request>();
      demo_req->set__data(false);
      auto fut = demo_client->async_send_request(demo_req);
      auto status = fut.wait_for(10s);
      if (status == std::future_status::ready) {
        if (fut.get()->success) {
          on_off_button_->setText("Demo not running");
          on_off_button_->setPalette(button_default_palette_);
          on_off_button_->setAutoFillBackground(true); // Ensure it fills
          RCLCPP_INFO(rclcpp::get_logger("color_widget_node"),
                      "Button State: Off");
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("color_widget_node"),
                     "Demo request not accepted");
      }
    }}.detach();
  }
}

int main(int argc, char **argv) { // Original main function context
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

  auto demo_client =
      node->create_client<std_srvs::srv::SetBool>("/enable_demo");
  RCLCPP_INFO(node->get_logger(), "Waiting for demo service");
  demo_client->wait_for_service();

  RCLCPP_INFO(node->get_logger(), "Demo service UP");

  ColorWidget widget(state_topic_name, wrist_contact_topic_name, demo_client);
  widget.show();

  auto sub = node->create_subscription<std_msgs::msg::ColorRGBA>(
      widget.get_topic_name(), 10,
      [&widget](std_msgs::msg::ColorRGBA::SharedPtr msg) {
        widget.updateColor(*msg);
      });

  auto joint_sub =
      node->create_subscription<panda_interfaces::msg::HumanContact>(
          widget.get_wrist_contact_topic_name(), 10,
          [&widget](panda_interfaces::msg::HumanContact msg) {
            widget.updateWristContact(msg);
          });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread ros_thread([&exec]() { exec.spin(); });

  int ret = app.exec();
  rclcpp::shutdown();
  ros_thread.join();
  return ret;
}
