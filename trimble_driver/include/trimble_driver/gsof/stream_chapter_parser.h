/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include "trimble_driver/gsof/stream_page_parser.h"

namespace trmb::gsof {

/**
 * The StreamMultiParser wraps a StreamParser and returns a chapter of pages once all the pages have been received.
 * This situation can happen if enough GSOF records have been enabled and become split over multiple transmissions.
 */
class StreamChapterParser {
 public:
  StreamChapterParser();

  using GsofChapterFoundCallback = std::function<void(const std::vector<std::byte> &)>;
  void registerGsofChapterFoundCallback(GsofChapterFoundCallback callback);

  void readSome(const std::uint8_t *data, std::size_t length);

 private:
  StreamPageParser stream_parser_;
  GsofChapterFoundCallback gsof_chapter_found_callback_;

  using Page    = std::vector<std::byte>;
  using Payload = std::vector<std::byte>;
  using Chapter = std::vector<Page>;

  Chapter chapter_buf_;

  /**
   * Go through all the pages in the chapter buffer and assemble the payload.
   * @param last_page The last page received in a tx
   * @return Series of GSOF records transmitted over a GENOUT chapter.
   */
  Payload assemblePayloadAndClearBuffer(const Payload &last_page);

  void gsofPageFoundCallback(const std::vector<std::byte> &page);
};

}  // namespace trmb::gsof
