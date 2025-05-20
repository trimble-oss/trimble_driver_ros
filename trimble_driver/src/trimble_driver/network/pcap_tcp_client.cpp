/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/network/pcap_tcp_client.h"

#include "trimble_driver/network/network.h"

namespace network {

PcapTcpClient::PcapTcpClient(const std::string &ip_address,
                             unsigned int port,
                             const std::string &filename,
                             std::vector<std::byte> start_delimiter,
                             std::vector<std::byte> end_delimiter)
    : IpClient(ip_address, port), pcap_reader_(filename, ip_address, port, ProtocolType::TCP), bytes_left_to_read_{0} {
  start_delimiter_.reserve(start_delimiter.size());
  end_delimiter_.reserve(end_delimiter.size());

  std::transform(start_delimiter.begin(),
                 start_delimiter.end(),
                 std::back_inserter(start_delimiter_),
                 [](std::byte b) -> char { return static_cast<char>(b); });
  std::transform(end_delimiter.begin(), end_delimiter.end(), std::back_inserter(end_delimiter_),
                 [](std::byte b) { return static_cast<char>(b); });

  payload_buffer_.fill(0U);

  if (pcap_reader_) {
    pcap_data_ = pcap_reader_.reallAll();

    // We store the data but we also need a view on it so that boost can iterate through the data.
    // We convert arrays underlying the vectors to pointers to create boost const_buffers
    for (auto &payload : pcap_data_) {
      pcap_view_.emplace_back(payload.data(), payload.size());
    }

    // The pcap_view_ provides an iterator over the sequence of buffers but we also need a way of
    // reading the sequence as a stream so we now need a SyncReadStreamAdapter on the view.
    pcap_view_stream_adapter_.emplace(pcap_view_);
  }
}

util::Status PcapTcpClient::open() {
  if (pcap_view_.empty()) {
    return util::Status(util::ErrorCode::DATA_EMPTY, "No packets were read from the pcap file");
  } else {
    return util::Status();
  }
}

int PcapTcpClient::receive() {
  boost::system::error_code ec;

  auto bytes_read = boost::asio::read_until(*pcap_view_stream_adapter_, streambuf_, start_delimiter_, ec);

  if (ec) {
    return -1;
  }

  auto it = std::copy(start_delimiter_.begin(), start_delimiter_.end(), payload_buffer_.begin());
  streambuf_.consume(bytes_read);

  // Read until we find the end delimiter and copy to our payload buffer
  bytes_read = boost::asio::read_until(*pcap_view_stream_adapter_, streambuf_, end_delimiter_, ec);

  if (ec) {
    return -1;
  }

  std::copy(boost::asio::buffers_begin(streambuf_.data()),
            boost::asio::buffers_begin(streambuf_.data()) + bytes_read,
            it);
  streambuf_.consume(bytes_read);

  return bytes_read + start_delimiter_.size();
}

}  // namespace network
