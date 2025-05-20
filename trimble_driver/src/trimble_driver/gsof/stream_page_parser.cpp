/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/stream_page_parser.h"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <utility>

namespace trmb::gsof {

StreamPageParser::StreamPageParser() : state_(State::k_find_start) {
  constexpr std::size_t k_buf_size = 4096;
  buf_.reserve(k_buf_size);
}

void StreamPageParser::registerGsofPageFoundCallback(StreamPageParser::GsofPageFoundCallback callback) {
  gsof_page_found_callback_ = std::move(callback);
}

void StreamPageParser::readSome(const uint8_t *const data, std::size_t length) {
  if (buf_.size() + length > buf_.max_size()) {
    reset();
    buf_.clear();
  }

  buf_.resize(buf_.size() + length);
  std::memcpy(buf_.data() + buf_.size() - length, data, length);

  while (isGsofPageFound()) {
    std::size_t total_data_length = getTotalRecordLength(current_header_);
    std::vector<std::byte> result;
    result.resize(total_data_length);
    std::memcpy(&result[0], &buf_[0], total_data_length);

    // Keep parts of the buffer we still need
    buf_.erase(buf_.begin(), buf_.begin() + total_data_length);

    if (gsof_page_found_callback_) {
      gsof_page_found_callback_(result);
    }
  }
}

bool StreamPageParser::isGsofPageFound() {
  // This switch acts as a very compact state machine
  bool packet_found   = false;
  auto previous_state = state_;
  switch (state_) {
    case State::k_find_start:
      state_ = !isStartTxFound() ? State::k_find_start : State::k_find_header;
      break;
    case State::k_find_header:
      state_ = !isHeaderFound() ? State::k_find_header : State::k_find_end;
      break;
    case State::k_find_end:
      packet_found = isEndTxFound();
      state_       = !packet_found ? State::k_find_end : State::k_find_start;
      break;
  }

  if (previous_state != state_ && !packet_found) {
    return isGsofPageFound();
  }

  return packet_found;
}

bool StreamPageParser::isStartTxFound() { return static_cast<std::uint8_t>(buf_[0]) == START_TX; }

bool StreamPageParser::isHeaderFound() {
  // Need more data
  if (buf_.size() < sizeof(record::Header)) {
    return false;
  }

  std::memcpy(&current_header_, buf_.data(), sizeof(current_header_));
  return current_header_.start_tx == START_TX;
}

bool StreamPageParser::isEndTxFound() {
  std::size_t footer_byte_offset = getFooterByteOffset(current_header_);

  if (buf_.size() < footer_byte_offset + sizeof(record::Footer)) {
    // Need more data
    return false;
  }

  record::Footer footer{};
  std::memcpy(&footer, buf_.data() + footer_byte_offset, sizeof(record::Footer));

  // We read in the required length, and it failed, reset stream parser
  return footer.end_tx == END_TX;
}

void StreamPageParser::reset() {
  buf_.clear();
  state_ = State::k_find_start;
}

}  // namespace trmb::gsof
