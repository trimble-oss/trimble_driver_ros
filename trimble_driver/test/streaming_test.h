/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <gtest/gtest.h>

#include <random>

#include "trimble_driver/network/pcap_reader.h"

class StreamingTest : public ::testing::Test {
 public:
  StreamingTest(const std::string &filename,
                const std::string &ip_address,
                unsigned int port,
                network::ProtocolType protocol_type)
      : pcap_reader_(filename, ip_address, port, protocol_type),
        tcp_packet_({nullptr, 0}),
        bytes_read_of_curr_packet_(0),
        rd_(),
        generator_(rd_()),
        distribution_(1, 25),
        buf_({}) {
    pcap_reader_.readSingle(&tcp_packet_);
  }

  std::optional<network::NonOwningBuffer> getData() {
    if (tcp_packet_.data == nullptr) {
      return std::nullopt;
    }

    int bytes_left_in_curr_pkt        = tcp_packet_.length - bytes_read_of_curr_packet_;
    int bytes_to_read                 = getRandom();
    const std::uint8_t *curr_data_ptr = tcp_packet_.data + bytes_read_of_curr_packet_;

    if (bytes_to_read < bytes_left_in_curr_pkt) {
      std::memcpy(buf_.data(), curr_data_ptr, bytes_to_read);
      bytes_read_of_curr_packet_ += bytes_to_read;
    } else if (bytes_to_read == bytes_left_in_curr_pkt) {
      std::memcpy(buf_.data(), curr_data_ptr, bytes_to_read);
      readNextPacket();
    } else if (bytes_to_read > bytes_left_in_curr_pkt) {
      std::memcpy(buf_.data(), curr_data_ptr, bytes_left_in_curr_pkt);
      if (readNextPacket()) {
        // Need to reset ptr because we read in a new packet
        curr_data_ptr = tcp_packet_.data + bytes_read_of_curr_packet_;
        int remainder = bytes_to_read - bytes_left_in_curr_pkt;
        std::memcpy(buf_.data() + bytes_left_in_curr_pkt, curr_data_ptr, remainder);
        bytes_read_of_curr_packet_ += remainder;
      }
    }

    return network::NonOwningBuffer(
        {reinterpret_cast<const std::uint8_t *>(buf_.data()), static_cast<std::size_t>(bytes_to_read)});
  }

 protected:
  network::PcapReader pcap_reader_;
  network::NonOwningBuffer tcp_packet_;
  std::size_t bytes_read_of_curr_packet_;

 private:
  std::random_device rd_;
  std::mt19937 generator_;
  std::uniform_int_distribution<> distribution_;
  std::array<std::byte, 32> buf_;

  int getRandom() { return distribution_(generator_); }

  bool readNextPacket() {
    bytes_read_of_curr_packet_ = 0;
    bool result                = pcap_reader_.readSingle(&tcp_packet_);
    if (!result) {
      tcp_packet_.data   = nullptr;
      tcp_packet_.length = 0;
    }
    return result;
  }
};
