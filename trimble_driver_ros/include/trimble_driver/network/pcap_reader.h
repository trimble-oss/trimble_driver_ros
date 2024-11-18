/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <pcap/pcap.h>

#include <boost/asio/buffer.hpp>
#include <list>
#include <optional>
#include <string>

#include "trimble_driver/network/network.h"

namespace network {

class PcapReader {
 public:
  PcapReader(const std::string &filename,
             const std::string &ip_address,
             unsigned int port,
             network::ProtocolType protocol_type);
  ~PcapReader();
  explicit operator bool() const;
  bool readSingle(network::NonOwningBuffer *payload);

  /**
   * Reads the whole pcap file into memory, stripping protocol data and keeping only the payload.
   * XXX Make sure you have enough RAM
   */
  std::list<std::vector<std::byte>> reallAll();

 private:
  network::ProtocolType protocol_type_;
  char err_buf_[512];
  int result_;
  pcap_t *handle_;
  bpf_program compiled_filter_;
  int data_link_type_;

  NonOwningBuffer getPcapPayload(const uint8_t *data, std::size_t length, ProtocolType pt);
  NonOwningBuffer getTcpPayload(const uint8_t *data, std::size_t length) const;
  NonOwningBuffer getUdpPayload(const uint8_t *data, std::size_t length);
};

}  // namespace network
