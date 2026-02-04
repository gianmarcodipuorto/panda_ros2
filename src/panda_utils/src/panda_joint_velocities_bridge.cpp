#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/robot_state.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "multibody/fwd.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
#include "panda_interfaces/msg/human_contact.hpp"
#include "panda_interfaces/msg/joint_torque_measure_stamped.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/msg/joints_commanded_velocities.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/debug_publisher.hpp"
#include "panda_utils/robot_model.hpp"
#include "panda_utils/utils_func.hpp"
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <vector>


template <typename messageT> using Publisher = rclcpp::Publisher<messageT>;  
using geometry_msgs::msg::Accel;
using geometry_msgs::msg::AccelStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using panda_interfaces::msg::JointsCommand;
using panda_interfaces::msg::JointsEffort;
using panda_interfaces::msg::JointsPos;
using panda_interfaces::msg::JointTorqueMeasureStamped;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;

struct robot_state {
  std::optional<franka::RobotState> state;
  rclcpp::Time state_time;
  std::mutex mut;
};

Eigen::Matrix<double, 7, 6>
compute_jacob_pseudoinv(const Eigen::Matrix<double, 6, 7> &jacobian) {
  return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

void print_initial_franka_state(const franka::RobotState state,
                                const franka::Model &model,
                                rclcpp::Logger logger) {
  Eigen::Vector<double, 7> vec7;
  for (int i = 0; i < 7; i++) {
    vec7[i] = state.q[i];
  }

  for (size_t i = 0; i < state.q.size(); i++) {
    RCLCPP_INFO_STREAM(logger, "Joint " << i + 1 << ": " << state.q[i]);
  }
  Pose current_pose =
      geom_utils::get_pose(model.pose(franka::Frame::kFlange, state)); //chiamo il metodo pose della libreria franka che mi restituisce la posa del flange rispetto all'origine del robot e poi usa una funzione di utility per convertirla in un messaggio ROS
      RCLCPP_INFO_STREAM_ONCE(
      logger, "Current position: ["
                  << current_pose.position.x << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << "]; Orientation: [] "
                  << current_pose.orientation.w << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << "]");

  //a partire dallo stato del robot mi vado ad ottenere la posa dell'end effector rispetto al frame di origine del robot
  //usa la trasformata fornita dal robot, ossia quello calcolato internamente dal robot stesso
  current_pose = geom_utils::get_pose(state.O_T_EE);
  RCLCPP_INFO_STREAM_ONCE(
      logger, "Current position (with O_T_EE): ["
                  << current_pose.position.x << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << "]; Orientation: [] "
                  << current_pose.orientation.w << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << "]");
  auto jacobian = geom_utils::get_jacobian(model.zeroJacobian(franka::Frame::kFlange, state));
  RCLCPP_INFO_STREAM_ONCE(logger, "Current jacobian: [" << jacobian << "]");

  // B(q) letta dal modello
  std::array<double, 49> mass_matrix_raw = model.mass(state);
  Eigen::Matrix<double, 7, 7> mass_matrix;
  for (size_t i = 0; i < 7; i++) {
    for (size_t j = 0; j < 7; j++) {
      mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
    }
  }

  // C(q, qdot) calcolata dal modello
  std::array<double, 7> coriolis_raw = model.coriolis(state);
  Eigen::Vector<double, 7> coriolis;
  for (size_t i = 0; i < 7; i++) {
    coriolis(i) = coriolis_raw[i];
  }

  // Funzione per ottenere il Jacobiano dato l'attuale posizione dei giunti
  auto get_jacob = [&model](const Eigen::Vector<double, 7> &current_joint_pos) {
    auto state = franka::RobotState{};
    for (size_t i = 0; i < state.q.size(); i++) {
      state.q[i] = current_joint_pos[i];
    }

    return geom_utils::get_jacobian(
        model.zeroJacobian(franka::Frame::kFlange, state)); //kFlange è l'end effector
  };

  //Estrae il valore singolare più piccolo (σ_min) del jacobiano
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                      Eigen::ComputeThinV);
  double sigma_min = svd.singularValues().tail(1)(0);

  // Calcolo della pseudoinversa del Jacobiano con damping per proteggere da singolarità
  double k_max = 1.0;
  double eps = 0.1;
  double lambda = k_max * (1 - pow(sigma_min, 2) / pow(eps, 2));
  lambda = (sigma_min >= eps ? 0.0 : lambda);
  Eigen::Matrix<double, 7, 6> jacobian_pinv = compute_jacob_pseudoinv(jacobian);

  RCLCPP_INFO_STREAM_ONCE(logger,
                          "Current mass matrix: [" << mass_matrix << "]");

  RCLCPP_INFO_STREAM_ONCE(logger,
                          "Current coriolis vector: [" << coriolis << "]");

  RCLCPP_INFO_STREAM_ONCE(logger,
                          "Current pseudoinv: [" << jacobian_pinv << "]");

}

class JointVelocitiesBridge : public rclcpp_lifecycle::LifecycleNode {

    private:

    //Subscribers
    rclcpp::Subscription<panda_interfaces::msg::JointsCommandedVelocities>::SharedPtr velocity_cmd_sub;

    // Robot pose publisher and debug
    realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub{}; 
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub{}; 
    
    // Robot related variables
    std::optional<franka::Robot> panda_franka; //It is the object thath communicates with the robot through the FCI
    // Robot model given by the FCI
    std::optional<franka::Model> panda_franka_model; //Dynamic model of the robot provided by the FCI
    robot_state panda_franka_state;
    
    // Robot load variables
    double load = 0.0;
    std::array<double, 3> F_x_Cload{0.0, 0.0, 0.0};
    std::array<double, 9> load_inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();

    std::mutex joint_state_mutex;
    sensor_msgs::msg::JointState::SharedPtr current_joint_config{nullptr};
    std::vector<double> world_base_link;

    // Callback function for the FCI
    std::function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> velocities_bridge_callback; //Callback for sending velocities to the robot

    // Commanded velocities
    realtime_tools::RealtimeThreadSafeBox<std::array<double, 7>> commanded_velocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    // Debug
    Eigen::Vector<double, 7> current_joints_speed = Eigen::Vector<double, 7>::Zero(); //vettore 7d della velocità attuale dei giunti
    JointState joint_state_to_pub{};

    // Safe limits
    Eigen::VectorXd joint_min_limits{};
    Eigen::VectorXd joint_max_limits{};
    Eigen::VectorXd velocity_limits{};
    double joints_speed_cutoff_freq{30.0};
    double joint_speed_safe_limit{};
    bool clamp;
    Eigen::VectorXd last_control_input;

    //thread
    std::thread control_thread;
    std::atomic<bool> start_flag{false};

    // Utility functions
    void clamp_control(Eigen::Vector<double, 7> &control_input) {
      for (int i = 0; i < control_input.size(); i++) {
        if (control_input[i] > velocity_limits[i]) {
          control_input[i] = velocity_limits[i];
        } else if (control_input[i] < -velocity_limits[i]) {
          control_input[i] = -velocity_limits[i];
        }
      }
    }

    public:
    JointVelocitiesBridge() : rclcpp_lifecycle::LifecycleNode("panda_joint_velocities_bridge") {
        
        this->declare_parameter<double>("safe_joint_speed", 0.7);
        this->declare_parameter<bool>("clamp", true);
        this->declare_parameter<std::string>("robot_ip", "10.224.20.198");

        //Safe limits
        joint_speed_safe_limit = this->get_parameter("safe_joint_speed").as_double();
        clamp = this->get_parameter("clamp").as_bool();
        //Taking limits from constants.hpp
        joint_min_limits= panda_constants::joint_min_limits;
        joint_max_limits= panda_constants::joint_max_limits;
        velocity_limits= panda_constants::velocity_limits;
        last_control_input= Eigen::Vector<double, 7>::Zero();

        velocity_cmd_sub = this->create_subscription<panda_interfaces::msg::JointsCommandedVelocities>(
            "/panda/joint_velocity_command",panda_interface_names::DEFAULT_TOPIC_QOS(),
            [this](const panda_interfaces::msg::JointsCommandedVelocities::SharedPtr msg) {
              std::array<double, 7> vel_cmd;
              for (size_t i = 0; i < 7; i++) {
                vel_cmd[i] = msg->velocities[i];
              }
              commanded_velocities.set(vel_cmd);
            });
        
        robot_pose_pub = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            this->create_publisher<geometry_msgs::msg::PoseStamped>(panda_interface_names::panda_pose_state_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        joint_states_pub =std::make_shared<realtime_tools::RealtimePublisher<JointState>>(
            this->create_publisher<JointState>(panda_interface_names::joint_state_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

    }

    ~JointVelocitiesBridge() {
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
    }
    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Configuring Joint Velocities Bridge...");

      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "No real time kernel available for franka lib communication");
        return CallbackReturn::FAILURE;
      }

      // Initialize FCI robot
      try {
        panda_franka = franka::Robot(this->get_parameter("robot_ip").as_string());
        RCLCPP_INFO_STREAM(this->get_logger(),
                         "Connected to robot with ip "
                             << this->get_parameter("robot_ip").as_string());

        RCLCPP_INFO_STREAM(this->get_logger(), "Set collision behavior...");                   
        std::array<double, 7> temp{100.0, 100.0, 100.0, 100.0,100.0, 100.0, 100.0};
        std::array<double, 6> temp2{100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
        panda_franka->setCollisionBehavior(temp, temp, temp, temp, temp2, temp2,temp2, temp2);
        RCLCPP_INFO_STREAM(this->get_logger(), "Set collision behavior done.");
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting load...");
        panda_franka->setLoad(load, F_x_Cload, load_inertia);
        RCLCPP_INFO_STREAM(this->get_logger(), "Load set.");
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Loading robot model...");
        panda_franka_model = panda_franka->loadModel();
        RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded.");

        // Debug prints before activation
        print_initial_franka_state(panda_franka->readOnce(),panda_franka_model.value(),this->get_logger());
      
        joint_state_to_pub.position.resize(7);
        joint_state_to_pub.velocity.resize(7);
        joint_state_to_pub.effort.resize(7);
        joint_state_to_pub.name = std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
      
        //adesso viene definita la callback che viene eseguita ad ogni ciclo di controllo e mandata alla funzione control della libreria franka
        velocities_bridge_callback = [this](const franka::RobotState &robot_state,
                                           franka::Duration dt) -> franka::JointVelocities {
          
          std::array<double, 7> vel_cmd{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

          //Aggiorno lo stato del robot
          {
            std::lock_guard<std::mutex> lock(panda_franka_state.mut);
            panda_franka_state.state = robot_state;
            panda_franka_state.state_time = this->now();
          }

          //Get current joint speeds
          for (size_t i = 0; i < 7; i++) {
            current_joints_speed[i] = robot_state.dq[i];
          }

          //Get commanded velocities
          vel_cmd = commanded_velocities.get();

          //TODO: Safety check

          //Convert to Eigen vector for clamping
          Eigen::Vector<double, 7> vel_cmd_eigen;
          for (size_t i = 0; i < 7; i++) {
            vel_cmd_eigen[i] = vel_cmd[i];
          }

          //Clamp control input if enabled
          if (clamp) {
            clamp_control(vel_cmd_eigen);
          }

          //Convert back to std::array
          for (size_t i = 0; i < 7; i++) {
            vel_cmd[i] = vel_cmd_eigen[i];
          }

          //print control input for debug
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Control input: [" << vel_cmd_eigen.transpose() << "]");
          return franka::JointVelocities(vel_cmd);
        };

      } catch (const franka::Exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Failed to connect to robot: " << e.what());
        return CallbackReturn::FAILURE;
      }

      RCLCPP_INFO(this->get_logger(), "Joint Velocities Bridge configured.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Activating Joint Velocities Bridge...");
      using namespace std::chrono_literals;

      last_control_input.resize(7);
      for (int i = 0; i < 7; i++) {
        last_control_input[i] = 0.0;
      }

      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(this->get_logger(),
                     "The robot thread has no real time kernel, shutting down");
        start_flag.store(false);
        rclcpp::shutdown();
      }

      if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return CallbackReturn::FAILURE;
      }

      // Start control loop thread
      start_flag.store(true);
      control_thread = std::thread{[this]() {
        try {

          std::thread{[this](){
            // Debug thread to print current joint speeds
            using namespace std::chrono_literals;
            RCLCPP_INFO(this->get_logger(), "Starting debug thread...");
            while (rclcpp::ok() && start_flag.load()) {
              {
                std::lock_guard<std::mutex> lock(joint_state_mutex);
                if (panda_franka_state.state.has_value()) {
                  const auto &stat = panda_franka_state.state.value();
                  for (size_t i = 0; i < 7; i++) { 
                    joint_state_to_pub.position[i] = stat.q[i];
                    joint_state_to_pub.velocity[i] = stat.dq[i];
                    //joint_state_to_pub.effort[i] = stat.tau[i];
                  }
                }
              }
              if (joint_states_pub->trylock()) {
                joint_states_pub->msg_ = joint_state_to_pub;
                joint_states_pub->unlockAndPublish();
              }
              std::this_thread::sleep_for(5ms);
            }
          }}.detach();
          //Set real time priority
          if (realtime_tools::configure_sched_fifo(99)) {
            RCLCPP_INFO(this->get_logger(), "Set real time priority");
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Real time priority not set, shutting down");
            start_flag.store(false);
            rclcpp::shutdown();
          }
          // Start control
          //debug print
          RCLCPP_INFO(this->get_logger(), "Starting control with real time robot...");
          panda_franka->control(velocities_bridge_callback);
        } catch (const franka::Exception &ex) {
          start_flag.store(false);
          if (panda_franka.has_value()) {
            try {
              panda_franka->stop();
            } catch (const franka::Exception &ex) {
              RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
            }
          }
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
      }};

      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with real time robot");

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Deactivating Joint Velocities Bridge...");
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      RCLCPP_INFO(this->get_logger(), "Joint Velocities Bridge deactivated.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Cleaning up Joint Velocities Bridge...");
      panda_franka.reset();
      panda_franka_model.reset();
      RCLCPP_INFO(this->get_logger(), "Joint Velocities Bridge cleaned up.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Shutting down Joint Velocities Bridge...");
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      RCLCPP_INFO(this->get_logger(), "Joint Velocities Bridge shut down.");
      return CallbackReturn::SUCCESS;
    }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointVelocitiesBridge>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
