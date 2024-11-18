/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <boost/asio.hpp>
#include <cstddef>
#include <string>
#include <vector>

#include "trimble_driver/network/ip_client.h"
#include "trimble_driver/network/pcap_reader.h"
#include "trimble_driver/util/status.h"

namespace network {

class PcapTcpClient : public IpClient {
 public:
  PcapTcpClient(const std::string &ip_address,
                unsigned int port,
                const std::string &filename,
                std::vector<std::byte> start_delimiter,
                std::vector<std::byte> end_delimiter);
  util::Status open() override;
  int receive() override;

 private:
  PcapReader pcap_reader_;
  std::list<std::vector<std::byte>> pcap_data_;
  std::list<boost::asio::mutable_buffer> pcap_view_;
  boost::asio::basic_streambuf<std::allocator<std::uint8_t>> streambuf_;
  int bytes_left_to_read_;
  std::string start_delimiter_;
  std::string end_delimiter_;

  // Adapt a boost buffers_iterator to the SyncReadStream interface
  template <typename BufferSequence, typename MutableBufferSequence = boost::asio::mutable_buffer>
  struct SyncReadStreamAdapter {
    SyncReadStreamAdapter(BufferSequence &bs) {
      it  = boost::asio::buffers_iterator<BufferSequence>::begin(bs);
      end = boost::asio::buffers_iterator<BufferSequence>::end(bs);
    }
    boost::asio::buffers_iterator<BufferSequence> it;
    boost::asio::buffers_iterator<BufferSequence> end;

    // XXX This is very inefficient
    std::size_t read_some(MutableBufferSequence mb, boost::system::error_code &ec) {
      std::size_t bytes_read = 0;
      while (bytes_read < boost::asio::buffer_size(mb) && it != end) {
        auto data_ptr = boost::asio::buffer_cast<char *>(mb + bytes_read);
        *data_ptr     = *it;
        ++bytes_read;
        ++it;
      }

      ec = (it == end) ? boost::asio::error::eof : boost::system::error_code();

      return bytes_read;
    }

    std::size_t read_some(MutableBufferSequence mb) {
      boost::system::error_code ec;
      size_t s = read_some(mb, ec);
      if (ec) throw boost::system::system_error(ec);
      return s;
    }
  };

  std::optional<SyncReadStreamAdapter<std::list<boost::asio::mutable_buffer>>> pcap_view_stream_adapter_;
};

}  // namespace network
