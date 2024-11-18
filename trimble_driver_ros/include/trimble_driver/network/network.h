#pragma once

#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <cstdint>
#include <cstring>

namespace network {
#pragma pack(push, 1)
struct EthernetHeader {
  static constexpr std::size_t SIZE      = 14;
  static constexpr std::size_t ADDR_SIZE = 6;

  uint8_t destination[ADDR_SIZE];
  uint8_t source[ADDR_SIZE];
  uint16_t ethernet_type;
};

struct IpHeader {
  uint8_t vhl;  /// Version and header length
  uint8_t tos;  /// Type of service
  uint16_t total_length;
  uint16_t id;
  uint16_t fragment_offset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t checksum;
  struct in_addr ip_source;
  struct in_addr ip_dst;

  static constexpr uint8_t VER_MASK = 0xF0;
  static constexpr uint8_t HL_MASK  = 0x0F;
  static constexpr uint8_t MIN_LEN  = 20;
  size_t getHeaderLength() const {
    unsigned int num_32bits = this->vhl & HL_MASK;
    return num_32bits * 4;  // Convert to number of bytes
  }

  uint8_t getVersion() const { return (this->vhl & VER_MASK) >> 4; }
};

struct TcpHeader {
  uint16_t src_port;
  uint16_t dst_port;
  tcp_seq seq_num;
  tcp_seq ack_num;
  uint8_t offset_rsv;  // 4 bits data offset, 3 bits reserved,
  uint8_t flags;
  uint16_t window;
  uint16_t checksum;
  uint16_t urgent_pointer;

  static constexpr uint8_t MIN_SIZE         = 20;
  static constexpr uint8_t MAX_SIZE         = 60;
  static constexpr uint8_t DATA_OFFSET_MASK = 0xF0;
  uint8_t getDataOffset() const { return ((offset_rsv & DATA_OFFSET_MASK) >> 4) * 4; }
};

struct UdpHeader {
  uint16_t src_port;
  uint16_t dst_port;
  uint16_t length;
  uint16_t checksum;

  static constexpr uint8_t MIN_SIZE = 8;
};

struct NonOwningBuffer {
  const uint8_t *data;
  std::size_t length;
};

enum class ProtocolType { TCP, UDP };
#pragma pack(pop)

NonOwningBuffer getPcapPayload(const uint8_t *data, const std::size_t length, const ProtocolType pt);

}  // namespace network
