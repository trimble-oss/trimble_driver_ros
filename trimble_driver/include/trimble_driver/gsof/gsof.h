/**
 * Byte definitions for Trimble GSOF protocol
 *
 * Glossary
 *   - GSOF: General Serial Output Format
 *   - DCOL packet: Data frames starting with START_TX (0x02) and ending with (0x03) In some documents these are
 *     referred to as "reports".
 *   - GSOF Record: DCOL packet where the type is GENOUT (0x40)
 *   - Page: Synonym of DCOL packet
 *   - Message or sub-record: In the payload of a GSOF record you can find one or multiple messages, sometimes
 *     referred to as sub-records. They all start with the bytes in Message::Header which define the type and length.
 *
 *  Therefore, a gsof::PacketParser parses the overall Trimcomm packet. The gsof::MessageParser parses the individual
 *  messages within a packet. The gsof::Stream parser takes in a sequence of bytes and returns a buffer containing a
 *  (mostly valid) packet ready for secondary checks by a gsof::PacketParser.
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace trmb::gsof {
// @formatter off
static constexpr uint8_t START_TX = 0x02;
static constexpr uint8_t END_TX   = 0x03;

static constexpr uint8_t GENOUT = 0x40;

static constexpr uint8_t GENOUT_BYTE_STX          = 0;
static constexpr uint8_t GENOUT_BYTE_STATUS       = 1;
static constexpr uint8_t GENOUT_BYTE_PACKET_TYPE  = 2;
static constexpr uint8_t GENOUT_BYTE_LENGTH       = 3;
static constexpr uint8_t GENOUT_BYTE_TRANS_NUM    = 4;
static constexpr uint8_t GENOUT_BYTE_PAGE_IDX     = 5;
static constexpr uint8_t GENOUT_BYTE_MAX_PAGE_IDX = 6;

// Number of bytes after the data length byte that still count as header bytes
static constexpr uint8_t NUM_HEADER_BYTES_IN_DATA_LENGTH = 3;
static constexpr uint8_t NUM_FOOTER_BYTES_IN_DATA_LENGTH = 1;
// @formatter on

namespace record {
#pragma pack(push, 1)
struct Header {
  uint8_t start_tx;
  uint8_t status;
  uint8_t type;
  uint8_t data_len;  // Length of the payload which starts immediately after the data_len byte
                     // This means it includes the last 3 bytes of the header but not the footer
  uint8_t tx_num;
  uint8_t page_idx;
  uint8_t max_page_idx;

  [[nodiscard]] std::size_t getSizeOfDataRecords() const {
    // Subtract 1 for the checksum
    // Subtract 3 because the transmission bytes aren't part of the actual records
    return data_len - NUM_HEADER_BYTES_IN_DATA_LENGTH;
  }
};

static_assert(sizeof(Header) == 7);

struct Footer {
  uint8_t checksum;
  uint8_t end_tx;
};
#pragma pack(pop)

/**
 * Gets the total number of bytes of a GSOF record including the Header and Footer
 */
inline std::size_t getTotalRecordLength(const Header& header) {
  constexpr size_t k_header_bytes_not_in_data_len = sizeof(Header) - NUM_HEADER_BYTES_IN_DATA_LENGTH;
  return header.data_len + k_header_bytes_not_in_data_len + sizeof(Footer);
}

/**
 * Get the position of the first footer byte (0-indexed) starting from the START_TX byte
 */
inline std::size_t getFooterByteOffset(const Header& header) {
  return sizeof(Header) - NUM_HEADER_BYTES_IN_DATA_LENGTH + header.data_len;
}

}  // namespace record
}  // namespace trmb::gsof
