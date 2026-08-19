/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "trimble_driver/gsof/gsof.h"

/**
 * Helpers for building GSOF byte streams in tests so that they do not depend on captured pcap files.
 */
namespace gsof_test {

/**
 * @brief Byteswaps a GSOF message into its on-the-wire (big endian) representation.
 *
 * Only usable for messages whose struct layout matches the wire layout, i.e. no optional or variable length fields.
 */
template <class MessageType>
std::vector<std::byte> serialize(MessageType message) {
  message.switchEndianess();
  std::vector<std::byte> bytes(sizeof(MessageType));
  std::memcpy(bytes.data(), &message, bytes.size());
  return bytes;
}

/**
 * @brief Wraps an already big-endian payload into a single GENOUT page.
 */
inline std::vector<std::byte> makeGenoutPage(const std::byte *payload, std::size_t payload_length, uint8_t tx_num,
                                             uint8_t page_idx, uint8_t max_page_idx) {
  using trmb::gsof::record::Footer;
  using trmb::gsof::record::Header;

  Header header{};
  header.start_tx     = trmb::gsof::START_TX;
  header.status       = 0;
  header.type         = trmb::gsof::GENOUT;
  header.data_len     = static_cast<uint8_t>(payload_length + trmb::gsof::NUM_HEADER_BYTES_IN_DATA_LENGTH);
  header.tx_num       = tx_num;
  header.page_idx     = page_idx;
  header.max_page_idx = max_page_idx;

  std::vector<std::byte> page(sizeof(Header) + payload_length + sizeof(Footer));
  std::memcpy(page.data(), &header, sizeof(Header));
  std::memcpy(page.data() + sizeof(Header), payload, payload_length);

  constexpr std::size_t k_checksum_data_start = 4;  // (1) stx + (1) status + (1) type + (1) data_len
  unsigned int checksum                       = header.status + header.type + header.data_len;
  for (std::size_t i = k_checksum_data_start; i < header.data_len + k_checksum_data_start; ++i) {
    checksum += static_cast<unsigned int>(page[i]);
  }

  const Footer footer{static_cast<uint8_t>(checksum % 256), trmb::gsof::END_TX};
  std::memcpy(page.data() + sizeof(Header) + payload_length, &footer, sizeof(Footer));

  return page;
}

/**
 * @brief Wraps one or more already big-endian GSOF messages into a single page GENOUT record.
 */
inline std::vector<std::byte> makeGenoutRecord(const std::byte *messages, std::size_t messages_length) {
  return makeGenoutPage(messages, messages_length, 0, 0, 0);
}

/**
 * @brief Concatenates messages into as few GENOUT pages as the single byte data length field allows.
 *
 * Messages are never split across a page boundary, which is how receivers actually page a transmission.
 */
inline std::vector<std::byte> makeGenoutTransmission(const std::vector<std::vector<std::byte>> &messages,
                                                     uint8_t tx_num) {
  constexpr std::size_t k_max_page_payload = 255 - trmb::gsof::NUM_HEADER_BYTES_IN_DATA_LENGTH;

  std::vector<std::vector<std::byte>> page_payloads(1);
  for (const auto &message : messages) {
    if (!page_payloads.back().empty() && page_payloads.back().size() + message.size() > k_max_page_payload) {
      page_payloads.emplace_back();
    }
    page_payloads.back().insert(page_payloads.back().end(), message.begin(), message.end());
  }

  const auto max_page_idx = static_cast<uint8_t>(page_payloads.size() - 1);

  std::vector<std::byte> transmission;
  for (std::size_t i = 0; i < page_payloads.size(); ++i) {
    const auto page =
        makeGenoutPage(page_payloads[i].data(), page_payloads[i].size(), tx_num, static_cast<uint8_t>(i), max_page_idx);
    transmission.insert(transmission.end(), page.begin(), page.end());
  }

  return transmission;
}

}  // namespace gsof_test
