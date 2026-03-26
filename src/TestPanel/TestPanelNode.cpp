/*
 *  This file is part of ROS2ISOBUS
 *
 *  Test panel node that shows telemetry and lets the user send
 *  Class3-related commands via keyboard.
 *
 *  Controls:
 *   - Arrow Up/Down: increase/decrease cruise speed. When speed != 0, cruise
 *     commands are sent
 *   - Arrow Left/Right: adjust curvature and start continuous guidance commands.
 *   - Space: toggle PTO engage.
 *   - PageUp/PageDown: hitch up/down.
 *   - Valves (flow adjust): 1:+/- toggles sending per valve via number keys 1-0
 *     Valve1 Q/A, Valve2 W/S, Valve3 E/D, Valve4 R/F, Valve5 T/G,
 *     Valve6 Y/H, Valve7 U/J, Valve8 I/K, Valve9 O/L
 *
 */

#include <array>
#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros2_isobus/topics.hpp"
#include "ros2_isobus/msg/tecu_guidance_command.hpp"
#include "ros2_isobus/msg/tecu_cruise_command.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_command.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_command.hpp"
#include "ros2_isobus/msg/aux_valve_command.hpp"
#include "ros2_isobus/msg/tecu_guidance_status.hpp"
#include "ros2_isobus/msg/tecu_cruise_status.hpp"
#include "ros2_isobus/msg/tecu_steering_valve_status.hpp"
#include "ros2_isobus/msg/aux_valve_status.hpp"
#include "ros2_isobus/msg/tecu_wheel_speed.hpp"
#include "ros2_isobus/msg/tecu_ground_speed.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_status.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_status.hpp"
#include "ros2_isobus/msg/tecu_steering_wheel.hpp"
#include "ros2_isobus/msg/tim_curvature_command.hpp"
#include "ros2_isobus/msg/tim_cruise_command.hpp"
#include "ros2_isobus/msg/tim_rear_hitch_command.hpp"
#include "ros2_isobus/msg/tim_rear_pto_command.hpp"
#include "ros2_isobus/msg/tim_aux_valve_command.hpp"
#include "ros2_isobus/msg/tim_curvature_status.hpp"
#include "ros2_isobus/msg/tim_cruise_status.hpp"
#include "ros2_isobus/msg/tim_rear_hitch_status.hpp"
#include "ros2_isobus/msg/tim_rear_pto_status.hpp"
#include "ros2_isobus/msg/tim_aux_valve_status.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/msg/isobus_address_book.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace ros2_isobus
{
namespace
{
constexpr double kCruiseStep = 0.2;        // Speed step (m/s) per key press.
constexpr double kCurvatureStep = 0.005;   // Curvature step (1/m) per key press.
constexpr double kHitchStep = 2.0;         // Hitch adjustment percent per key press.
constexpr double kValveStep = 5.0;         // Valve adjustment percent per key press.
}  // namespace

class TerminalMode
{
public:
  TerminalMode()
  {
    tcgetattr(STDIN_FILENO, &old_);
    termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  }
  ~TerminalMode() { tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_); }

private:
  termios old_{};
};

class TestPanelNode : public rclcpp::Node
{
public:
  TestPanelNode()
  : Node("isobus_test_panel")
  {
    declare_parameter<double>("initial_curvature", 0.0);
    declare_parameter<double>("initial_cruise_speed", 0.0);
    declare_parameter<double>("initial_hitch", 0.0);
    declare_parameter<bool>("initial_pto_engaged", false);
    declare_parameter<std::string>("control_interface", "tecu");

    state_.curvature = get_parameter("initial_curvature").as_double();
    state_.cruise_speed = get_parameter("initial_cruise_speed").as_double();
    state_.hitch = get_parameter("initial_hitch").as_double();
    state_.pto_engaged = get_parameter("initial_pto_engaged").as_bool();
    {
      auto mode = get_parameter("control_interface").as_string();
      std::transform(mode.begin(), mode.end(), mode.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      use_tim_interface_ = (mode == "tim");
    }
    terminal_ui_enabled_ = (isatty(STDIN_FILENO) == 1) && (isatty(STDOUT_FILENO) == 1);

    // Publishers
    guidance_pub_ = create_publisher<msg::TecuGuidanceCommand>(kTECUCurvatureCommandTopic, 10);
    cruise_pub_ = create_publisher<msg::TecuCruiseCommand>(kTECUCruiseCommandTopic, 10);
    hitch_pub_ = create_publisher<msg::TecuRearHitchCommand>(kTECURearHitchCommandTopic, 10);
    pto_pub_ = create_publisher<msg::TecuRearPtoCommand>(kTECURearPtoCommandTopic, 10);
    valve_pub_ = create_publisher<msg::AuxValveCommand>(kAuxValveCommandTopic, 10);
    tim_guidance_pub_ = create_publisher<msg::TimCurvatureCommand>(kTIMCurvatureCommandTopic, 10);
    tim_cruise_pub_ = create_publisher<msg::TimCruiseCommand>(kTIMCruiseCommandTopic, 10);
    tim_hitch_pub_ = create_publisher<msg::TimRearHitchCommand>(kTIMRearHitchCommandTopic, 10);
    tim_pto_pub_ = create_publisher<msg::TimRearPtoCommand>(kTIMRearPtoCommandTopic, 10);
    tim_valve_pub_ = create_publisher<msg::TimAuxValveCommand>(kTIMAuxValveCommandTopic, 10);

    // Subscriptions (telemetry cached for rendering)
    guidance_sub_ = create_subscription<msg::TecuGuidanceStatus>(
      kTECUGuidanceStatusTopic, 10,
      [this](const msg::TecuGuidanceStatus::SharedPtr m) {
        if (use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.measured_curvature = m->measured_curvature;
        telem_.command_device = m->command_device;
      });

    cruise_sub_ = create_subscription<msg::TecuCruiseStatus>(
      kTECUCruiseStatusTopic, 10,
      [this](const msg::TecuCruiseStatus::SharedPtr m) {
        if (use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.cruise_speed = m->command_speed;
        telem_.cruise_status = m->command_status;
      });

    steering_valve_sub_ = create_subscription<msg::TecuSteeringValveStatus>(
      kTECUSteeringValveStatusTopic, 10,
      [this](const msg::TecuSteeringValveStatus::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.steer_mode = m->mode;
        telem_.steer_device = m->command_device;
      });

    valve_status_sub_ = create_subscription<msg::AuxValveStatus>(
      kAuxValveStatusTopic, 10,
      [this](const msg::AuxValveStatus::SharedPtr m) {
        if (use_tim_interface_) return;
        if (!m) return;
        if (m->valve_number < 1 || m->valve_number > state_.valve_flow.size()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        auto idx = m->valve_number - 1;
        telem_.valve_extend[idx] = m->extend_flow_percent;
        telem_.valve_retract[idx] = m->retract_flow_percent;
        telem_.valve_state[idx] = m->state;
        telem_.valve_failsafe[idx] = m->failsafe;
      });

    wheel_speed_sub_ = create_subscription<msg::TecuWheelSpeed>(
      kTECUWheelSpeedTopic, 10,
      [this](const msg::TecuWheelSpeed::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.wheel_speed = m->speed_ms;
      });

    ground_speed_sub_ = create_subscription<msg::TecuGroundSpeed>(
      kTECUGroundSpeedTopic, 10,
      [this](const msg::TecuGroundSpeed::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.ground_speed = m->speed_ms;
      });

    hitch_status_sub_ = create_subscription<msg::TecuRearHitchStatus>(
      kTECURearHitchTopic, 10,
      [this](const msg::TecuRearHitchStatus::SharedPtr m) {
        if (use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.hitch = m->position_percent;
      });

    pto_status_sub_ = create_subscription<msg::TecuRearPtoStatus>(
      kTECURearPtoTopic, 10,
      [this](const msg::TecuRearPtoStatus::SharedPtr m) {
        if (use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.pto_rpm = m->rpm;
        telem_.pto_engaged = (m->engagement == 0x01);
      });

    tim_guidance_sub_ = create_subscription<msg::TimCurvatureStatus>(
      kTIMCurvatureStatusTopic, 10,
      [this](const msg::TimCurvatureStatus::SharedPtr m) {
        if (!use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.measured_curvature = m->measured_curvature_km_inv / 1000.0;
        telem_.command_device = m->active ? 1 : 0;
      });

    tim_cruise_sub_ = create_subscription<msg::TimCruiseStatus>(
      kTIMCruiseStatusTopic, 10,
      [this](const msg::TimCruiseStatus::SharedPtr m) {
        if (!use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.cruise_speed = m->measured_speed;
        telem_.cruise_status = m->automation_status;
      });

    tim_hitch_status_sub_ = create_subscription<msg::TimRearHitchStatus>(
      kTIMRearHitchStatusTopic, 10,
      [this](const msg::TimRearHitchStatus::SharedPtr m) {
        if (!use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.hitch = m->position_percent;
      });

    tim_pto_status_sub_ = create_subscription<msg::TimRearPtoStatus>(
      kTIMRearPtoStatusTopic, 10,
      [this](const msg::TimRearPtoStatus::SharedPtr m) {
        if (!use_tim_interface_) return;
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.pto_rpm = m->rpm;
        telem_.pto_engaged = m->active;
      });

    tim_valve_status_sub_ = create_subscription<msg::TimAuxValveStatus>(
      kTIMAuxValveStatusTopic, 10,
      [this](const msg::TimAuxValveStatus::SharedPtr m) {
        if (!use_tim_interface_) return;
        if (!m) return;
        if (m->valve_number < 1 || m->valve_number > state_.valve_flow.size()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        const auto idx = m->valve_number - 1;
        // TIM AUX flow is signed: positive extend, negative retract.
        if (m->flow_percent >= 0.0f) {
          telem_.valve_extend[idx] = m->flow_percent;
          telem_.valve_retract[idx] = 0.0;
        } else {
          telem_.valve_extend[idx] = 0.0;
          telem_.valve_retract[idx] = -m->flow_percent;
        }
        telem_.valve_state[idx] = m->automation_status;
        telem_.valve_failsafe[idx] = m->fault;
      });

    steering_wheel_sub_ = create_subscription<msg::TecuSteeringWheel>(
      kTECUSteeringWheelTopic, 10,
      [this](const msg::TecuSteeringWheel::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.steering_angle = m->angle;
      });

    // Address manager
    addr_status_sub_ = create_subscription<msg::IsobusAddressStatus>(
      kAddressManagerStatus, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const msg::IsobusAddressStatus::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.my_sa = m->sa;
      });
    addr_book_sub_ = create_subscription<msg::IsobusAddressBook>(
      kAddressManagerAddressBook, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const msg::IsobusAddressBook::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.addr_book_entries = m->entries.size();
        telem_.addr_entries = m->entries;
      });

    // NMEA2000
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      kNmea2000GnssPositionTopic, 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.lat = m->latitude;
        telem_.lon = m->longitude;
        telem_.alt = m->altitude;
        telem_.nav_status = m->status.status;
        telem_.nav_service = m->status.service;
      });
    cog_sog_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      kNmea2000CogSogTopic, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.sog = m->twist.linear.x;
        telem_.cog = m->twist.angular.z;
      });
    attitude_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      kNmea2000AttitudeTopic, 10,
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr m) {
        if (!m) return;
        std::lock_guard<std::mutex> lk(mutex_);
        telem_.attitude = m->vector;
      });

    // Timers
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publishLoop(); });
    if (terminal_ui_enabled_)
    {
      render_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() { render(); });
      // Keyboard thread
      key_thread_ = std::thread([this]() { keyLoop(); });
      RCLCPP_INFO(get_logger(), "Test panel running. Use keyboard to send commands.");
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "TestPanel started without interactive TTY (e.g. via ros2 launch): terminal UI disabled.");
    }
  }

  ~TestPanelNode() override
  {
    running_.store(false);
    if (key_thread_.joinable()) key_thread_.join();
  }

private:
  struct Telemetry
  {
    // Address manager
    uint8_t my_sa{0xFF};
    std::size_t addr_book_entries{0};
    std::vector<msg::IsobusAddressEntry> addr_entries;

    double measured_curvature{0.0};
    uint8_t command_device{0};
    double cruise_speed{0.0};
    int cruise_status{0};
    uint8_t steer_mode{0};
    uint8_t steer_device{0};
    double wheel_speed{0.0};
    double ground_speed{0.0};
    double hitch{0.0};
    double pto_rpm{0.0};
    bool pto_engaged{false};
    double steering_angle{0.0};
    std::array<double, 9> valve_extend{};
    std::array<double, 9> valve_retract{};
    std::array<uint8_t, 9> valve_state{};
    std::array<bool, 9> valve_failsafe{};

    // NMEA2000
    double lat{0.0};
    double lon{0.0};
    double alt{0.0};
    int nav_status{0};
    uint16_t nav_service{0};
    double sog{0.0};
    double cog{0.0};
    geometry_msgs::msg::Vector3 attitude{};
  };

  struct CommandState
  {
    double curvature{0.0};
    bool curvature_active{false};
    double cruise_speed{0.0};
    bool cruise_active{false};
    rclcpp::Time last_cruise_input{};
    double hitch{0.0};
    bool pto_engaged{false};
    std::array<double, 9> valve_flow{};   // Extend (+)/retract (-).
    std::array<bool, 9> valve_enable{};
    std::array<bool, 9> valve_dirty{};
    bool hitch_dirty{false};
    bool pto_dirty{false};
  };

  void publishLoop()
  {
    std::lock_guard<std::mutex> lk(mutex_);

    if (state_.curvature_active && (use_tim_interface_ ? static_cast<bool>(tim_guidance_pub_) : static_cast<bool>(guidance_pub_)))
    {
      if (use_tim_interface_) {
        msg::TimCurvatureCommand msg;
        msg.curvature_km_inv = state_.curvature * 1000.0;
        msg.enable = state_.curvature_active;
        if (tim_guidance_pub_) tim_guidance_pub_->publish(msg);
      } else {
        msg::TecuGuidanceCommand msg;
        msg.curvature = state_.curvature;
        guidance_pub_->publish(msg);
      }
    }

    if (use_tim_interface_ ? static_cast<bool>(tim_cruise_pub_) : static_cast<bool>(cruise_pub_))
    {
      if (state_.cruise_active && std::abs(state_.cruise_speed) > 1e-4)
      {
        if (use_tim_interface_) {
          msg::TimCruiseCommand msg;
          msg.speed = state_.cruise_speed;
          msg.enable = state_.cruise_active;
          if (tim_cruise_pub_) tim_cruise_pub_->publish(msg);
        } else {
          msg::TecuCruiseCommand msg;
          msg.speed = state_.cruise_speed;
          msg.max_speed = std::abs(state_.cruise_speed);
          cruise_pub_->publish(msg);
        }
      }
    }

    const bool send_hitch = use_tim_interface_ ? true : state_.hitch_dirty;
    if (send_hitch && (use_tim_interface_ ? static_cast<bool>(tim_hitch_pub_) : static_cast<bool>(hitch_pub_)))
    {
      if (use_tim_interface_) {
        msg::TimRearHitchCommand msg;
        msg.position_percent = state_.hitch;
        msg.enable = true;
        if (tim_hitch_pub_) tim_hitch_pub_->publish(msg);
      } else {
        msg::TecuRearHitchCommand msg;
        msg.position_percent = state_.hitch;
        hitch_pub_->publish(msg);
      }
      if (!use_tim_interface_) state_.hitch_dirty = false;
    }

    const bool send_pto = use_tim_interface_ ? true : state_.pto_dirty;
    if (send_pto && (use_tim_interface_ ? static_cast<bool>(tim_pto_pub_) : static_cast<bool>(pto_pub_)))
    {
      if (use_tim_interface_) {
        msg::TimRearPtoCommand msg;
        msg.rpm = state_.pto_engaged ? 540.0 : 0.0;
        // In TIM test mode keep function enabled; Space toggles requested speed only.
        msg.engagement = true;
        if (tim_pto_pub_) tim_pto_pub_->publish(msg);
      } else {
        msg::TecuRearPtoCommand msg;
        msg.rpm = state_.pto_engaged ? 540.0 : 0.0;
        msg.engagement = state_.pto_engaged;
        pto_pub_->publish(msg);
      }
      if (!use_tim_interface_) state_.pto_dirty = false;
    }

    if (!state_.valve_enable.empty())
    {
      for (std::size_t i = 0; i < state_.valve_enable.size(); ++i)
      {
        // Publish while enabled, and also publish one-shot updates when disabling/changing.
        if (state_.valve_enable[i] || state_.valve_dirty[i])
        {
          if (use_tim_interface_) {
            msg::TimAuxValveCommand msg;
            msg.valve_number = static_cast<std::uint8_t>(i + 1);
            msg.flow_percent = static_cast<float>(std::clamp(state_.valve_flow[i], -100.0, 100.0));
            msg.enable = state_.valve_enable[i];
            if (tim_valve_pub_) tim_valve_pub_->publish(msg);
          } else if (valve_pub_) {
            msg::AuxValveCommand msg;
            msg.valve_number = static_cast<std::uint8_t>(i + 1);
            msg.flow_percent = static_cast<float>(state_.valve_flow[i]);
            msg.floating = false;  // Could add key toggle later.
            msg.failsafe = false;
            valve_pub_->publish(msg);
          }
          state_.valve_dirty[i] = false;
        }
      }
    }
  }

  void render()
  {
    Telemetry tcopy;
    CommandState scopy;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      tcopy = telem_;
      scopy = state_;
    }

    printf("\033[2J\033[H");  // Clear and home the terminal.
    printf("ROS2ISOBUS Test Panel\n");
    printf("Control interface: %s\n", use_tim_interface_ ? "TIM" : "TECUClass3");
    printf("---- AddressManager ---------------------------------------\n");
    printf("SA: %3u  Addr book entries: %zu\n", tcopy.my_sa, tcopy.addr_book_entries);
    if (!tcopy.addr_entries.empty())
    {
      printf("NAME / SA:\n");
      std::size_t shown = 0;
      for (const auto & entry : tcopy.addr_entries)
      {
        if (shown++ >= 10) { printf("... (%zu more)\n", tcopy.addr_entries.size() - shown + 1); break; }
        printf("  %s  -> 0x%02X\n", nameToHex(entry).c_str(), entry.sa);
      }
    }
    printf("---- NMEA200 Client ---------------------------------------\n");
    printf("GNSS: lat/lon %.6f / %.6f  alt: %.1f m  status:%d svc:0x%X\n",
           tcopy.lat, tcopy.lon, tcopy.alt, tcopy.nav_status, tcopy.nav_service);
    printf("COG/SOG: %8.3f rad     %8.3f m/s        Att: roll %.2f pitch %.2f yaw %.2f\n",
           tcopy.cog, tcopy.sog,
           tcopy.attitude.x, tcopy.attitude.y, tcopy.attitude.z);
    printf("----  TECU Client -------------------------------------------------\n");
    printf("Curvature meas: %8.4f  device:%u        Curvature cmd: %8.4f %s\n",
           tcopy.measured_curvature, tcopy.command_device,
           scopy.curvature,
           (scopy.curvature_active ? "[ACTIVE]" : "[idle]"));
    printf("Cruise speed:  %8.3f  status:%d         Cruise cmd:       %8.3f %s\n",
           tcopy.cruise_speed, tcopy.cruise_status,
           scopy.cruise_speed,
           (scopy.cruise_active ? "[ACTIVE]" : "[idle]"));
    printf("Wheel speed:   %8.3f\n", tcopy.wheel_speed);
    printf("Ground speed:  %8.3f\n", tcopy.ground_speed);
    printf("Hitch meas:    %8.1f%%                   Hitch cmd: %6.1f%% (PgUp/PgDn)\n",
           tcopy.hitch, scopy.hitch);
    printf("PTO meas:      %8.1f rpm [%s]            PTO: %s (Space)\n",
           tcopy.pto_rpm, tcopy.pto_engaged ? "engaged" : "off",
           (scopy.pto_engaged ? "ON " : "OFF"));
    printf("Steer angle:   %8.3f rad  mode:%u device:%u\n",
           tcopy.steering_angle, tcopy.steer_mode, tcopy.steer_device);
    printf("\nValve status (meas / cmd), enabled count: %zu\n", state_.valve_enable.size());
    for (std::size_t i = 0; i < state_.valve_enable.size(); ++i)
    {
      printf("V%02zu ext:%6.1f ret:%6.1f state:0x%02X fs:%d   cmd:%6.1f %s\n",
             i + 1,
             tcopy.valve_extend[i], tcopy.valve_retract[i],
             tcopy.valve_state[i], tcopy.valve_failsafe[i] ? 1 : 0,
             scopy.valve_flow[i],
             scopy.valve_enable[i] ? "[EN]" : "[--]");
    }
    printf("\nControls: Arrows (speed/curvature), Space PTO, PgUp/PgDn Hitch, valves per key (Q/A .. O/L), numbers toggle valve send\n");
    fflush(stdout);
  }

  void keyLoop()
  {
    TerminalMode term;
    std::array<char, 9> valve_up = {'q','w','e','r','t','y','u','i','o'};
    std::array<char, 9> valve_down = {'a','s','d','f','g','h','j','k','l'};
    while (running_.load())
    {
      char c = 0;
      if (read(STDIN_FILENO, &c, 1) <= 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      // Arrow sequences: ESC [ A/B/C/D
      if (c == '\033')
      {
        char seq[2];
        if (read(STDIN_FILENO, seq, 2) == 2 && seq[0] == '[')
        {
          if (seq[1] == 'A') onCruiseDelta(kCruiseStep);      // Up
          else if (seq[1] == 'B') onCruiseDelta(-kCruiseStep); // Down
          else if (seq[1] == 'C') onCurvatureDelta(kCurvatureStep); // Right
          else if (seq[1] == 'D') onCurvatureDelta(-kCurvatureStep); // Left
          else if (seq[1] == '5') { char tilde; read(STDIN_FILENO, &tilde, 1); onHitchDelta(kHitchStep); } // PgUp
          else if (seq[1] == '6') { char tilde; read(STDIN_FILENO, &tilde, 1); onHitchDelta(-kHitchStep); } // PgDn
        }
        continue;
      }

      switch (c)
      {
      case ' ':
        togglePto();
        break;
      case '\x0c': // Ctrl+L refresh
        render();
        break;
      default:
        break;
      }

      // Valve toggles number keys 1-9
      if (c >= '1' && c <= '9')
      {
        int idx = (c - '1');
        if (idx < static_cast<int>(state_.valve_enable.size())) toggleValve(idx);
      }

      for (std::size_t i = 0; i < valve_up.size(); ++i)
      {
        if (c == valve_up[i])
        {
          if (i < state_.valve_enable.size()) adjustValve(i, kValveStep);
        }
        else if (c == valve_down[i])
        {
          if (i < state_.valve_enable.size()) adjustValve(i, -kValveStep);
        }
      }
    }
  }

  void onCruiseDelta(double delta)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.cruise_speed += delta;
    state_.cruise_active = true;
    state_.last_cruise_input = now();
    if (std::abs(state_.cruise_speed) < 1e-3)
      state_.cruise_speed = 0.0;
  }

  void onCurvatureDelta(double delta)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.curvature += delta;
    state_.curvature_active = true;
  }

  void onHitchDelta(double delta)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.hitch = std::clamp(state_.hitch + delta, 0.0, 100.0);
    state_.hitch_dirty = true;
  }

  void togglePto()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.pto_engaged = !state_.pto_engaged;
    state_.pto_dirty = true;
  }

  void toggleValve(int idx)
  {
    if (idx < 0 || idx >= static_cast<int>(state_.valve_enable.size())) return;
    std::lock_guard<std::mutex> lk(mutex_);
    state_.valve_enable[idx] = !state_.valve_enable[idx];
    state_.valve_dirty[idx] = true;
  }

  void adjustValve(std::size_t idx, double delta)
  {
    if (idx >= state_.valve_enable.size()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    state_.valve_flow[idx] = std::clamp(state_.valve_flow[idx] + delta, -125.0, 125.0);
    state_.valve_dirty[idx] = true;
  }

  std::string nameToHex(const msg::IsobusAddressEntry & entry) const
  {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0');
    for (const auto b : entry.name)
    {
      oss << std::setw(2) << static_cast<int>(b);
    }
    return oss.str();
  }

  rclcpp::Publisher<msg::TecuGuidanceCommand>::SharedPtr guidance_pub_;
  rclcpp::Publisher<msg::TecuCruiseCommand>::SharedPtr cruise_pub_;
  rclcpp::Publisher<msg::TecuRearHitchCommand>::SharedPtr hitch_pub_;
  rclcpp::Publisher<msg::TecuRearPtoCommand>::SharedPtr pto_pub_;
  rclcpp::Publisher<msg::AuxValveCommand>::SharedPtr valve_pub_;
  rclcpp::Publisher<msg::TimCurvatureCommand>::SharedPtr tim_guidance_pub_;
  rclcpp::Publisher<msg::TimCruiseCommand>::SharedPtr tim_cruise_pub_;
  rclcpp::Publisher<msg::TimRearHitchCommand>::SharedPtr tim_hitch_pub_;
  rclcpp::Publisher<msg::TimRearPtoCommand>::SharedPtr tim_pto_pub_;
  rclcpp::Publisher<msg::TimAuxValveCommand>::SharedPtr tim_valve_pub_;

  rclcpp::Subscription<msg::TecuGuidanceStatus>::SharedPtr guidance_sub_;
  rclcpp::Subscription<msg::TecuCruiseStatus>::SharedPtr cruise_sub_;
  rclcpp::Subscription<msg::TecuSteeringValveStatus>::SharedPtr steering_valve_sub_;
  rclcpp::Subscription<msg::AuxValveStatus>::SharedPtr valve_status_sub_;
  rclcpp::Subscription<msg::TecuWheelSpeed>::SharedPtr wheel_speed_sub_;
  rclcpp::Subscription<msg::TecuGroundSpeed>::SharedPtr ground_speed_sub_;
  rclcpp::Subscription<msg::TecuRearHitchStatus>::SharedPtr hitch_status_sub_;
  rclcpp::Subscription<msg::TecuRearPtoStatus>::SharedPtr pto_status_sub_;
  rclcpp::Subscription<msg::TecuSteeringWheel>::SharedPtr steering_wheel_sub_;
  rclcpp::Subscription<msg::TimCurvatureStatus>::SharedPtr tim_guidance_sub_;
  rclcpp::Subscription<msg::TimCruiseStatus>::SharedPtr tim_cruise_sub_;
  rclcpp::Subscription<msg::TimRearHitchStatus>::SharedPtr tim_hitch_status_sub_;
  rclcpp::Subscription<msg::TimRearPtoStatus>::SharedPtr tim_pto_status_sub_;
  rclcpp::Subscription<msg::TimAuxValveStatus>::SharedPtr tim_valve_status_sub_;
  rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr addr_status_sub_;
  rclcpp::Subscription<msg::IsobusAddressBook>::SharedPtr addr_book_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cog_sog_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr render_timer_;

  std::thread key_thread_;
  std::atomic<bool> running_{true};
  std::mutex mutex_;
  bool use_tim_interface_{false};
  bool terminal_ui_enabled_{true};
  Telemetry telem_;
  CommandState state_;
};

}  // namespace ros2_isobus

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_isobus::TestPanelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
