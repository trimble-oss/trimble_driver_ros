/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/stream_chapter_parser.h"

#include <cassert>
#include <cstring>

#include "trimble_driver/gsof/gsof.h"

namespace trmb::gsof {

StreamChapterParser::StreamChapterParser() {
  stream_parser_.registerGsofPageFoundCallback(
      [this](const std::vector<std::byte> &page) { this->gsofPageFoundCallback(page); });
}

void StreamChapterParser::registerGsofChapterFoundCallback(GsofChapterFoundCallback callback) {
  gsof_chapter_found_callback_ = std::move(callback);
}

void StreamChapterParser::readSome(const std::uint8_t *data, std::size_t length) {
  stream_parser_.readSome(data, length);
}

StreamChapterParser::Payload StreamChapterParser::assemblePayloadAndClearBuffer(const Payload &last_page) {
  Payload payload;

  record::Header header{};
  std::memcpy(&header, last_page.data(), sizeof(header));
  std::size_t total_payload_size = header.getSizeOfDataRecords();
  for (const auto &page : chapter_buf_) {
    std::memcpy(&header, page.data(), sizeof(header));
    total_payload_size += header.getSizeOfDataRecords();
  }

#ifndef NDEBUG
  // In debug mode, assert that all pages have the same transmission number
  std::memcpy(&header, last_page.data(), sizeof(header));
  std::uint8_t tx_id = header.tx_num;
  for (const auto &page : chapter_buf_) {
    std::memcpy(&header, page.data(), sizeof(header));
    assert(tx_id == header.tx_num && "GENOUT Transmission id mismatch in chapter buffer.");
  }
#endif

  payload.resize(total_payload_size);

  std::size_t bytes_copied = 0;
  for (auto &page : chapter_buf_) {
    std::memcpy(&header, page.data(), sizeof(header));
    std::size_t data_length = header.getSizeOfDataRecords();
    std::memcpy(payload.data() + bytes_copied, page.data() + sizeof(record::Header), data_length);
    bytes_copied += data_length;
  }

  std::memcpy(&header, last_page.data(), sizeof(header));
  std::size_t data_length = header.getSizeOfDataRecords();
  std::memcpy(payload.data() + bytes_copied, last_page.data() + sizeof(record::Header), data_length);
  bytes_copied += data_length;

  assert(bytes_copied == total_payload_size);

  chapter_buf_.clear();

  return payload;
}

void StreamChapterParser::gsofPageFoundCallback(const std::vector<std::byte> &page) {
  record::Header header{};
  std::memcpy(&header, page.data(), sizeof(header));

  const bool is_single_page          = header.max_page_idx == 0;
  const bool is_last_page_in_chapter = header.max_page_idx == header.page_idx;
  if (is_single_page || is_last_page_in_chapter) {
    // This chapter only has 1 page therefore it is ready to be returned
    const auto &payload = assemblePayloadAndClearBuffer(page);
    if (gsof_chapter_found_callback_) gsof_chapter_found_callback_(payload);
    return;
  }

  // If we are here, this is the ith page of n, buffer it and return nullopt
  chapter_buf_.push_back(page);
}

}  // namespace trmb::gsof
