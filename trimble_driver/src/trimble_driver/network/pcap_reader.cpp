/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/network/pcap_reader.h"

#include <iostream>
#include <sstream>

namespace network {

PcapReader::PcapReader(const std::string &filename,
                       const std::string &ip_address,
                       unsigned int port,
                       network::ProtocolType protocol_type)
    : protocol_type_(protocol_type), err_buf_{0}, result_(1), handle_(nullptr) {
  handle_ = pcap_open_offline(filename.c_str(), err_buf_);

  if (handle_ == nullptr) {
    return;
  }

  data_link_type_ = pcap_datalink(handle_);
  if (data_link_type_ != DLT_EN10MB && data_link_type_ != DLT_RAW) {
    result_ = -1;
    return;
  }

  // Filter all packets not comping from
  std::stringstream filter_expression;
  filter_expression << "src net " << ip_address << " and ";
  switch (protocol_type_) {
    case network::ProtocolType::TCP:
      filter_expression << "tcp port " << port;
      break;
    case network::ProtocolType::UDP:
      filter_expression << "udp port " << port;
      break;
    default:
      throw std::invalid_argument("Unhandled ip protocol type.");
  }

  result_ = pcap_compile(handle_,
                         &compiled_filter_,
                         filter_expression.str().c_str(),
                         1,  // 1 means do optimization
                         PCAP_NETMASK_UNKNOWN);

  if (result_ < 0) {
    return;
  }

  result_ = pcap_setfilter(handle_, &compiled_filter_);
  if (result_ < 0) {
    return;
  }
}

PcapReader::~PcapReader() {
  if (handle_ != nullptr) {
    pcap_close(handle_);
    pcap_freecode(&compiled_filter_);
    handle_ = nullptr;
  }
}

PcapReader::operator bool() const { return handle_ != nullptr && result_ >= 0; }

bool PcapReader::readSingle(network::NonOwningBuffer *payload) {
  struct pcap_pkthdr *header;
  const u_char *pkt_data;
  result_ = pcap_next_ex(handle_, &header, &pkt_data);

  if (result_ < 1) {
    return false;
  }

  *payload = getPcapPayload(pkt_data, header->caplen, protocol_type_);

  return true;
}

NonOwningBuffer PcapReader::getTcpPayload(const uint8_t *data, const std::size_t length) const {
  if (data_link_type_ == DLT_EN10MB) {
    data += EthernetHeader::SIZE;
  }

  IpHeader ip_header{};
  std::memcpy(&ip_header, data, sizeof ip_header);
  data += ip_header.getHeaderLength();

  // Invalid ip header
  if (ip_header.getHeaderLength() < IpHeader::MIN_LEN) {
    return {nullptr, 0};
  }

  TcpHeader tcp_header{};
  std::memcpy(&tcp_header, data, sizeof tcp_header);

  // Invalid tcp header
  auto data_offset = tcp_header.getDataOffset();
  if (data_offset < TcpHeader::MIN_SIZE || data_offset > TcpHeader::MAX_SIZE) {
    return {nullptr, 0};
  }
  data += data_offset;

  const size_t header_size = (data_link_type_ == DLT_EN10MB)
                                 ? EthernetHeader::SIZE + ip_header.getHeaderLength() + tcp_header.getDataOffset()
                                 : ip_header.getHeaderLength() + tcp_header.getDataOffset();
  const uint8_t *payload   = data;

  return {payload, length - header_size};
}

NonOwningBuffer PcapReader::getUdpPayload(const uint8_t *data, const std::size_t length) {
  IpHeader ip_header;
  std::memcpy(&ip_header, data + EthernetHeader::SIZE, sizeof ip_header);

  // Invalid header
  if (ip_header.getHeaderLength() < 20) return {nullptr, 0};

  UdpHeader udp_header;
  std::memcpy(&udp_header, data + EthernetHeader::SIZE + ip_header.getHeaderLength(), sizeof(udp_header));

  if (udp_header.length < UdpHeader::MIN_SIZE) return {nullptr, 0};

  // TODO(Andre) Make use of checksum

  const size_t header_size = EthernetHeader::SIZE + ip_header.getHeaderLength() + sizeof(udp_header);
  const uint8_t *payload   = data + header_size;

  return {payload, length - header_size};
}

NonOwningBuffer PcapReader::getPcapPayload(const uint8_t *data, const std::size_t length, const ProtocolType pt) {
  if (pt == ProtocolType::TCP) {
    return getTcpPayload(data, length);
  } else if (pt == ProtocolType::UDP) {
    return getUdpPayload(data, length);
  }

  return {nullptr, 0};
}

std::list<std::vector<std::byte>> PcapReader::reallAll() {
  std::list<std::vector<std::byte>> data_payloads;

  for (NonOwningBuffer buf{nullptr, 0}; readSingle(&buf);) {
    std::vector<std::byte> payload(buf.length);
    std::memcpy(payload.data(), buf.data, buf.length);
    data_payloads.emplace_back(std::move(payload));
  }

  return data_payloads;
}

}  // namespace network
