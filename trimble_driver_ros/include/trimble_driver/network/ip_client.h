/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "trimble_driver/util/status.h"

namespace network {

class IpClient {
 public:
  static constexpr size_t MAX_BUF_SIZE = 1024 * 16;  // 16kB
  using Buffer                         = std::array<uint8_t, MAX_BUF_SIZE>;

  IpClient(std::string ip_address, unsigned int port) : ip_address_(std::move(ip_address)), port_(port) {}
  virtual ~IpClient()         = default;
  virtual util::Status open() = 0;
  virtual int receive()       = 0;
  [[nodiscard]] const Buffer &getBuffer() const { return payload_buffer_; }

 protected:
  std::string ip_address_;
  unsigned int port_;
  Buffer payload_buffer_{};

 private:
};

}  // namespace network
