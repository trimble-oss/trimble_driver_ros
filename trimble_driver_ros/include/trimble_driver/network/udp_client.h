/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "trimble_driver/network/ip_client.h"
#include "trimble_driver/util/status.h"

namespace network {
class UdpClient : public IpClient {
 public:
  UdpClient(std::string ip_address, unsigned int port);
  util::Status open() override;
  int receive() override;

 private:
  using udp = boost::asio::ip::udp;

  boost::asio::io_service io_service_;
  std::unique_ptr<udp::socket> socket_;
  udp::endpoint server_endpoint_;
  udp::endpoint local_endpoint_;
};
}  // namespace network
