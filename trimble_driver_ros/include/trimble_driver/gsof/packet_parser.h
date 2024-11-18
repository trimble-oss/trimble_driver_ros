/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>

#include "trimble_driver/gsof/gsof.h"
#include "trimble_driver/gsof/message_parser.h"
#include "trimble_driver/parser_interface.h"

/**
 * See gsof.h for overall documentation
 */

namespace trmb::gsof {
namespace detail {

class PacketParserBase : public ParserInterface {
 public:
  PacketParserBase();
  /**
   * @brief The PacketParser class expects to be parsing a report containing one or more GSOF
   * messages.
   *
   * @param data Pointer to the start of the report (i.e. contains START_TX byte and page numbers)
   * @param length
   */
  PacketParserBase(const std::byte *data, std::size_t length);

  void setData(const std::byte *data, std::size_t length) override;
  [[nodiscard]] bool isValid() const override;

 protected:
  const std::byte *messages_ = nullptr;
};

}  // namespace detail

template <class MessageParserType = MessageParser<SupportedPublicMessages>>
class PacketParser : public detail::PacketParserBase {
 public:
  PacketParser() : detail::PacketParserBase() {}
  /**
   * @brief The PacketParser class expects to be parsing a report (DCOL transmission of type 0x40) containing one or
   * more GSOF messages.
   *
   * @param data Pointer to the start of the report (i.e. contains START_TX byte and page numbers)
   * @param length
   */
  PacketParser(const std::byte *data, std::size_t length) : detail::PacketParserBase(data, length) {}

  [[nodiscard]] bool isSupported() const override;
  [[nodiscard]] MessageParserType getMessageParser() const;

 private:
  MessageParserType message_parser_;
};

template <class MessageParserType>
bool PacketParser<MessageParserType>::isSupported() const {
  return getMessageParser().isSupported();
}

template <class MessageParserType>
MessageParserType PacketParser<MessageParserType>::getMessageParser() const {
  const size_t message_payload_length = length_ - sizeof(gsof::record::Header) - sizeof(gsof::record::Footer);
  return MessageParserType(messages_, message_payload_length);
}

using PublicPacketParser = PacketParser<MessageParser<SupportedPublicMessages>>;

}  // namespace trmb::gsof
