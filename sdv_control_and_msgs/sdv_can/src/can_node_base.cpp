#include "can_node_base.h"
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

CanNodeBase::CanNodeBase(const std::string &name)
    : Node(name), can_stream_(ios_), signals_(ios_, SIGINT, SIGTERM) {
  this->declare_parameter("can_interface", "can0");
  std::string can_interface = this->get_parameter("can_interface").as_string();
  RCLCPP_INFO(this->get_logger(), "Using %s interface", can_interface.c_str());

  fd_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ == -1) {
    throw std::runtime_error("Error opening socketcan.");
  }
  ifreq ifr{};
  strcpy(ifr.ifr_name, can_interface.c_str());
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0){
    throw std::runtime_error("Invalid interface name: " + can_interface);
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    throw std::runtime_error("Error in socket bind.");
  }

  can_stream_.assign(fd_);

  // Queue up first can frame.
  can_stream_.async_read_some(
      boost::asio::buffer(&tmp_frame_, sizeof(tmp_frame_)),
      std::bind(&CanNodeBase::can_listener, this, std::ref(tmp_frame_),
                std::ref(can_stream_)));

  signals_.async_wait(std::bind(&CanNodeBase::stop, this));
  std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
  std::thread bt(std::bind(run, &ios_));
  bt.detach();

  RCLCPP_INFO(this->get_logger(), "Finished initializing.");
}

void CanNodeBase::stop() {
  RCLCPP_INFO(this->get_logger(), "Stopping can node.");
  ios_.stop();
  signals_.clear();
}

void CanNodeBase::send_frame(uint32_t address, const vanttec::CANMessage &msg){
  struct can_frame frame;
  // TODO Should we add EFF/RTR/ERR flags?
  frame.can_id = address;
  frame.can_dlc = msg.len;
  memcpy(frame.data, msg.data.data(), sizeof(msg.data));
  send_frame(frame);
}

void CanNodeBase::send_frame(const struct can_frame frame) {
  can_stream_.async_write_some(
      boost::asio::buffer(&frame, sizeof(frame)),
      std::bind(&CanNodeBase::can_send_confirm_cb, this));
}

void CanNodeBase::can_send_confirm_cb() {
  // sent can msg.
}

void CanNodeBase::can_listener(
    struct can_frame &frame,
    boost::asio::posix::basic_stream_descriptor<> &stream) {
  parse_frame(frame);

  // Queue up next can read.
  stream.async_read_some(boost::asio::buffer(&frame, sizeof(frame)),
                         std::bind(&CanNodeBase::can_listener, this,
                                   std::ref(frame), std::ref(stream)));
}
