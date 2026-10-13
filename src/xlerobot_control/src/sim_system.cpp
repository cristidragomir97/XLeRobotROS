#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace xlerobot_control {
constexpr uint64_t MAGIC = 0x584c45524f424f54;
constexpr size_t CAPACITY = 64;
// ABI shared with xlerobot_sim/control_io.py. Sequences guard concurrent copies.
struct Shared {
  uint64_t magic;
  uint32_t count, reserved;
  uint64_t state_sequence, command_sequence;
  double state_time, command_time;
  uint64_t enabled;
  double position[CAPACITY], velocity[CAPACITY], command[CAPACITY];
};
static_assert(sizeof(Shared) == 1592);
static double wall_time() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class SimulatorSystem : public hardware_interface::SystemInterface {
public:
  ~SimulatorSystem() override { close_shared(); }
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
    if (info_.joints.empty() || info_.joints.size() > CAPACITY) return hardware_interface::CallbackReturn::ERROR;
    path_ = info_.hardware_parameters.at("state_file");
    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 2) return hardware_interface::CallbackReturn::ERROR;
      const auto & type = joint.command_interfaces[0].name;
      if (type != hardware_interface::HW_IF_POSITION && type != hardware_interface::HW_IF_VELOCITY) return hardware_interface::CallbackReturn::ERROR;
      velocity_command_.push_back(type == hardware_interface::HW_IF_VELOCITY);
      minimum_.push_back(std::stod(joint.command_interfaces[0].min));
      maximum_.push_back(std::stod(joint.command_interfaces[0].max));
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    close_shared();
    const auto deadline = wall_time() + 15.;
    while (wall_time() < deadline) {
      fd_ = open(path_.c_str(), O_RDWR);
      struct stat status {};
      if (fd_ >= 0 && fstat(fd_, &status) == 0 && status.st_size == sizeof(Shared)) {
        auto memory = mmap(nullptr, sizeof(Shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (memory != MAP_FAILED) {
          shared_ = static_cast<Shared *>(memory);
          if (shared_->magic == MAGIC && shared_->count == info_.joints.size()) {
            if (read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0)) == hardware_interface::return_type::OK) {
              // MuJoCo's soft limits can leave measured positions slightly beyond
              // their bounds. Initialize controller hold commands within those bounds.
              for (size_t i = 0; i < info_.joints.size(); ++i) {
                command_[i] = velocity_command_[i] ? 0. : std::max(minimum_[i], std::min(maximum_[i], position_[i]));
              }
              return hardware_interface::CallbackReturn::SUCCESS;
            }
          }
        }
      }
      close_shared();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_ERROR(rclcpp::get_logger("xlerobot_control"), "Simulator joint state unavailable: %s", path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
      result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    }
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (size_t i = 0; i < info_.joints.size(); ++i) result.emplace_back(info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &command_[i]);
    return result;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    if (!shared_) return hardware_interface::CallbackReturn::ERROR;
    __atomic_store_n(&shared_->enabled, uint64_t(1), __ATOMIC_RELEASE);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    if (shared_) __atomic_store_n(&shared_->enabled, uint64_t(0), __ATOMIC_RELEASE);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    close_shared(); return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (!shared_) return hardware_interface::return_type::ERROR;
    std::array<double, CAPACITY> p{}, v{};
    for (int attempt = 0; attempt < 10; ++attempt) {
      const auto sequence = __atomic_load_n(&shared_->state_sequence, __ATOMIC_ACQUIRE);
      if (sequence & 1) continue;
      std::memcpy(p.data(), shared_->position, sizeof(shared_->position));
      std::memcpy(v.data(), shared_->velocity, sizeof(shared_->velocity));
      const double timestamp = shared_->state_time;
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      if (sequence != __atomic_load_n(&shared_->state_sequence, __ATOMIC_ACQUIRE)) continue;
      // Timestamp is part of the sequence-protected snapshot too: Python's
      // struct writer can otherwise expose a transient partially copied double.
      if (wall_time() - timestamp > 5.) {
        RCLCPP_ERROR(rclcpp::get_logger("xlerobot_control"), "Physics state stale for %.3f seconds", wall_time() - timestamp);
        return hardware_interface::return_type::ERROR;
      }
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (!std::isfinite(p[i]) || !std::isfinite(v[i])) {
          RCLCPP_ERROR(rclcpp::get_logger("xlerobot_control"), "Non-finite state for %s", info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        position_[i] = p[i]; velocity_[i] = v[i];
      }
      return hardware_interface::return_type::OK;
    }
    // Preserve the last complete snapshot if a camera/state update overlaps this read.
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (!shared_) return hardware_interface::return_type::ERROR;
    for (size_t i = 0; i < info_.joints.size(); ++i) if (!std::isfinite(command_[i])) return hardware_interface::return_type::ERROR;
    std::array<double, CAPACITY> bounded = command_;
    for (size_t i = 0; i < info_.joints.size(); ++i) bounded[i] = std::max(minimum_[i], std::min(maximum_[i], bounded[i]));
    __atomic_fetch_add(&shared_->command_sequence, uint64_t(1), __ATOMIC_ACQ_REL);
    std::memcpy(shared_->command, bounded.data(), sizeof(shared_->command));
    shared_->command_time = wall_time();
    __atomic_fetch_add(&shared_->command_sequence, uint64_t(1), __ATOMIC_RELEASE);
    return hardware_interface::return_type::OK;
  }
private:
  void close_shared() {
    if (shared_) { munmap(shared_, sizeof(Shared)); shared_ = nullptr; }
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
  }
  int fd_ = -1;
  Shared * shared_ = nullptr;
  std::string path_;
  std::vector<bool> velocity_command_;
  std::vector<double> minimum_, maximum_;
  std::array<double, CAPACITY> position_{}, velocity_{}, command_{};
};
}
PLUGINLIB_EXPORT_CLASS(xlerobot_control::SimulatorSystem, hardware_interface::SystemInterface)
