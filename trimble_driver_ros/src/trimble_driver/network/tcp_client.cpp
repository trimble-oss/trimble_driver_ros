/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/network/tcp_client.h"

namespace network {

TcpClient::TcpClient(const std::string &ip_address, unsigned int port) : IpClient(ip_address, port), socket_(nullptr) {}

uint TcpClient::peekBytes() {
  uint bytes_available = 0;
  int base_sock        = socket_->lowest_layer().native_handle();

  timeval timeout;
  timeout.tv_sec  = 1;
  timeout.tv_usec = 0;

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(base_sock, &read_set);

  int ret = select(base_sock + 1, &read_set, NULL, NULL, &timeout);
  if (ret == -1) return 0;

  if (FD_ISSET(base_sock, &read_set)) {
    ret = ioctl(base_sock, FIONREAD, &bytes_available);
    if (ret == -1) return 0;
  }

  return bytes_available;
}

util::Status TcpClient::open() {
  using namespace boost::asio::ip;
  socket_ = std::make_unique<tcp::socket>(io_service_);

  boost::system::error_code ec;
  socket_->connect(tcp::endpoint(boost::asio::ip::address::from_string(ip_address_), port_), ec);

  boost::asio::ip::tcp::no_delay no_delay(true);
  socket_->set_option(no_delay);

  if (!ec) {
    return util::Status(util::ErrorCode::OK);
  }

  std::string error_info = "";
  if (ec.message() == "Connection refused") {
    error_info =
        ". Either the port is accessible but no applications are listening on that port or the the server is already "
        "servicing the maximum number of clients.";
  }

  return util::Status(util::ErrorCode::CONNECTION_ERROR, ec.message() + error_info);
}

int TcpClient::receive() {
  boost::system::error_code error;
  size_t bytes_read = socket_->read_some(boost::asio::buffer(payload_buffer_), error);

  if (error) {
    return -1;
  }

  return bytes_read;
}

util::Status TcpClient::send(std::vector<std::byte> data) {
  boost::system::error_code error;
  socket_->write_some(boost::asio::buffer(data), error);

  if (error) return util::Status(util::ErrorCode::CONNECTION_ERROR, error.message());

  return util::Status(util::ErrorCode::OK);
}

util::Status TcpClient::close(boost::asio::socket_base::linger linger_option) {
  boost::system::error_code error;
  if (socket_->is_open()) {
    socket_->set_option(linger_option);

    socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
    if (error) return util::Status(util::ErrorCode::CONNECTION_ERROR, error.message());
    socket_->close(error);
    if (error) return util::Status(util::ErrorCode::CONNECTION_ERROR, error.message());

    socket_ = nullptr;
  }
  return util::Status(util::ErrorCode::OK);
}

}  // namespace network
