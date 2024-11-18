/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/packet_parser.h"

#include <cstring>

#include "trimble_driver/gsof/gsof.h"

namespace trmb::gsof {
namespace detail {
PacketParserBase::PacketParserBase() : PacketParserBase(nullptr, 0) {}

PacketParserBase::PacketParserBase(const std::byte *data, const size_t length) {
  PacketParserBase::setData(data, length);
}

void PacketParserBase::setData(const std::byte *data, const size_t length) {
  data_     = data;
  messages_ = data_ + sizeof(gsof::record::Header);
  length_   = length;
}

bool PacketParserBase::isValid() const {
  gsof::record::Header header;
  std::memcpy(&header, data_, sizeof(header));

  if (header.start_tx != gsof::START_TX) return false;

  // Don't support packets other than the general report packet.
  if (header.type != gsof::GENOUT) return false;

  gsof::record::Footer footer;
  std::memcpy(&footer, data_ + length_ - sizeof(footer), sizeof(footer));

  if (footer.end_tx != gsof::END_TX) return false;

  unsigned int checksum                = header.status + header.type + header.data_len;
  constexpr size_t CHECKSUM_DATA_START = 4;  // (1) stx + (1) status + (1) type + (1) data_len
  for (size_t i = CHECKSUM_DATA_START; i < header.data_len + CHECKSUM_DATA_START; ++i) {
    checksum += static_cast<unsigned int>(*(data_ + i));
  }

  return checksum % 256 == footer.checksum;
}

}  // namespace detail

template <>
class PacketParser<MessageParser<SupportedPublicMessages>>;

}  // namespace trmb::gsof
