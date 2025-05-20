/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/network/udp_client.h"

namespace network {

UdpClient::UdpClient(std::string ip_address, unsigned int port) : IpClient(ip_address, port) {
  payload_buffer_.fill(0);
  server_endpoint_ = udp::endpoint(boost::asio::ip::address_v4::from_string(ip_address), port_);
  local_endpoint_  = udp::endpoint(boost::asio::ip::address_v4::any(), port_);
}

util::Status UdpClient::open() {
  socket_ = std::make_unique<udp::socket>(io_service_);
  boost::system::error_code ec;
  socket_->open(local_endpoint_.protocol(), ec);
  socket_->bind(local_endpoint_);

  if (!ec) {
    return util::Status();
  } else {
    return util::Status(util::ErrorCode::CONNECTION_ERROR, ec.message());
  }
}

int UdpClient::receive() {
  struct pollfd pfds[1];
  pfds[0].fd               = socket_->native_handle();
  pfds[0].events           = POLLIN;  // check ready to read
  constexpr size_t n_fds   = 1;
  constexpr int timeout_ms = 2'000;
  int n_ready              = poll(pfds, n_fds, timeout_ms);
  if (n_ready > 0) {
    return socket_->receive_from(boost::asio::buffer(payload_buffer_), server_endpoint_);
  } else {
    std::cout << "No data received for the last 2 seconds over UDP. Data is expected at 1 Hz." << std::endl;
    return 0;  // 0 bytes received
  }
}

}  // namespace network
