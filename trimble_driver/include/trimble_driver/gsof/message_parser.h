/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <array>
#include <cstdint>
#include <set>
#include <stdexcept>

#include "trimble_driver/gsof/message.h"
#include "trimble_driver/parser_interface.h"

namespace trmb::gsof {
namespace detail {
constexpr std::uint8_t k_supported_msgs_ids[] = {GSOF_ID_1_POS_TIME,
                                                 GSOF_ID_2_LLH,
                                                 GSOF_ID_3_ECEF,
                                                 GSOF_ID_6_ECEF_DELTA,
                                                 GSOF_ID_7_TPLANE_ENU,
                                                 GSOF_ID_8_VELOCITY,
                                                 GSOF_ID_9_DOP,
                                                 GSOF_ID_10_CLOCK_INFO,
                                                 GSOF_ID_11_POS_VCV_INFO,
                                                 GSOF_ID_12_POS_SIGMA,
                                                 GSOF_ID_15_REC_SERIAL_NUM,
                                                 GSOF_ID_16_CURR_TIME,
                                                 GSOF_ID_27_ATTITUDE,
                                                 GSOF_ID_33_ALL_SV_BRIEF,
                                                 GSOF_ID_34_ALL_SV_DETAIL,
                                                 GSOF_ID_35_RECEIVED_BASE_INFO,
                                                 GSOF_ID_37_BATTERY_MEM_INFO,
                                                 GSOF_ID_38_POSITION_TYPE_INFO,
                                                 GSOF_ID_40_LBAND_STATUS,
                                                 GSOF_ID_41_BASE_POSITION_QUALITY,
                                                 GSOF_ID_49_INS_FULL_NAV,
                                                 GSOF_ID_50_INS_RMS,
                                                 GSOF_ID_52_DMI_RAW_DATA,
                                                 GSOF_ID_63_INS_FULL_NAV_KRYPTON,
                                                 GSOF_ID_64_INS_RMS_KRYPTON};
}  // namespace detail

struct SupportedPublicMessages {
  static const std::set<std::uint8_t> ids;
};

template <class SupportedMessages = SupportedPublicMessages>
class MessageParser {
 public:
  MessageParser(const std::byte *data, const std::size_t length);
  MessageParser();

  void setData(const std::byte *data, std::size_t length);
  bool isValid() const;
  bool isSupported() const;

  class Iterator {
   public:
    friend class MessageParser<SupportedMessages>;
    // Declarations for std iterator compatibility
    using iterator_category = std::forward_iterator_tag;
    using value_type        = gsof::Message;
    using difference_type   = int;
    using pointer           = value_type *;
    using reference         = value_type &;

    Iterator(const std::byte *const data_begin, std::size_t length);

    reference operator*();
    pointer operator->();
    Iterator &operator++();
    bool operator==(const Iterator &rhs) const;
    bool operator!=(const Iterator &rhs) const;

   private:
    const std::byte *const data_begin_;
    const std::byte *data_;
    const std::size_t length_;
    std::size_t current_offset_;
    Message current_message_;

    Iterator &invalidate();
  };

  Iterator begin();
  Iterator end();

 private:
  const std::byte *data_;
  std::size_t length_;

  [[nodiscard]] inline bool isMessageSupported(std::uint8_t id) const;
};

template <class SupportedMessages>
MessageParser<SupportedMessages>::MessageParser(const std::byte *data, std::size_t length)
    : data_(data), length_(length) {}

template <class SupportedMessages>
MessageParser<SupportedMessages>::MessageParser() : MessageParser(nullptr, 0) {}

template <class SupportedMessages>
void MessageParser<SupportedMessages>::setData(const std::byte *data, std::size_t length) {
  data_   = data;
  length_ = length;
}

template <class SupportedMessages>
bool MessageParser<SupportedMessages>::isValid() const {
  Header header;
  std::memcpy(&header, data_, sizeof(header));
  return isMessageSupported(header.type);
}

template <class SupportedMessages>
bool MessageParser<SupportedMessages>::isSupported() const {
  // GSOF messages don't seem to have an end?
  return isValid();
}

template <class SupportedMessages>
bool MessageParser<SupportedMessages>::isMessageSupported(std::uint8_t id) const {
  return SupportedMessages::ids.count(id) > 0;
}

template <class SupportedMessages>
typename MessageParser<SupportedMessages>::Iterator MessageParser<SupportedMessages>::begin() {
  return Iterator(data_, length_);
}

template <class SupportedMessages>
typename MessageParser<SupportedMessages>::Iterator MessageParser<SupportedMessages>::end() {
  return this->begin().invalidate();
}

template <class SupportedMessages>
MessageParser<SupportedMessages>::Iterator::Iterator(const std::byte *const data_begin, const std::size_t length)
    : data_begin_(data_begin),
      data_(data_begin_),
      length_(length),
      current_offset_(0),
      current_message_(data_begin, length) {}

template <class SupportedMessages>
typename MessageParser<SupportedMessages>::Iterator::reference MessageParser<SupportedMessages>::Iterator::operator*() {
  if (current_offset_ >= length_) throw std::out_of_range("Tried to access element past end of buffer.");
  return current_message_;
}

template <class SupportedMessages>
typename MessageParser<SupportedMessages>::Iterator::pointer MessageParser<SupportedMessages>::Iterator::operator->() {
  if (current_offset_ >= length_) throw std::out_of_range("Tried to access element past end of buffer.");
  return &current_message_;
}

template <class SupportedMessages>
typename MessageParser<SupportedMessages>::Iterator &MessageParser<SupportedMessages>::Iterator::operator++() {
  if (current_offset_ >= length_) return *this;

  current_offset_ += current_message_.getHeader().length + sizeof(Header);
  if (current_offset_ >= length_) {
    invalidate();
  } else {
    data_            = data_begin_ + current_offset_;
    current_message_ = Message(data_, length_ - current_offset_);
  }

  return *this;
}

template <class SupportedMessages>
bool MessageParser<SupportedMessages>::Iterator::operator==(
    const MessageParser<SupportedMessages>::Iterator &rhs) const {
  return current_offset_ == rhs.current_offset_ && data_ == rhs.data_ && data_begin_ == rhs.data_begin_ &&
         length_ == rhs.length_;
}

template <class SupportedMessages>
bool MessageParser<SupportedMessages>::Iterator::operator!=(
    const MessageParser<SupportedMessages>::Iterator &rhs) const {
  // Clang will misdiagnose this line as being simplifiable however, doing so would call the incorrect comparison
  // function
  return !(*this == rhs);
}

template <class SupportedMessages>
typename MessageParser<SupportedMessages>::Iterator &MessageParser<SupportedMessages>::Iterator::invalidate() {
  current_offset_ = length_;
  data_           = nullptr;
  return *this;
}

}  // namespace trmb::gsof
