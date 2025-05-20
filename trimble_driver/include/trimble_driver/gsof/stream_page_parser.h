/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "trimble_driver/gsof/gsof.h"

namespace trmb::gsof {

/**
 * Given a sequence of bytes, accumulates bytes into a buffer until the buffer contains a GSOF
 * packet. Intended for use with streaming input.
 */
class StreamPageParser {
 public:
  StreamPageParser();

  using GsofPageFoundCallback = std::function<void(const std::vector<std::byte> &)>;
  void registerGsofPageFoundCallback(GsofPageFoundCallback callback);

  void readSome(const std::uint8_t *data, std::size_t length);

 private:
  enum class State { k_find_start, k_find_header, k_find_end };

  std::vector<std::byte> buf_;
  State state_;
  record::Header current_header_;
  GsofPageFoundCallback gsof_page_found_callback_;

  bool isGsofPageFound();

  // Reset the state machine
  void reset();

  bool isStartTxFound();
  bool isHeaderFound();
  bool isEndTxFound();
};

}  // namespace trmb::gsof
