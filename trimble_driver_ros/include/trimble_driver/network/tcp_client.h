/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>

#include "trimble_driver/network/ip_client.h"

namespace network {

/**
 * A thin wrapper around a tcp socket
 */
class TcpClient : public IpClient {
 public:
  TcpClient() = delete;
  TcpClient(const std::string& ip_address, unsigned int port);

  util::Status open() override;
  util::Status close(boost::asio::socket_base::linger linger_option = boost::asio::socket_base::linger());
  int receive() override;
  util::Status send(std::vector<std::byte> data);
  uint peekBytes();

 private:
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
};

}  // namespace network
