/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <atomic>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "trimble_driver/gsof/message_parser.h"
#include "trimble_driver/gsof/stream_chapter_parser.h"
#include "trimble_driver/gsof/stream_page_parser.h"
#include "trimble_driver/network/tcp_client.h"
#include "util/status.h"

namespace trmb {

/**
 * This client is meant to connect to products communicating using Trimble's Trimcomm/GSOF packets
 * over TCP.
 */
class GsofClient {
 public:
  using MessageCallback = std::function<void(const gsof::Message &)>;
  struct unsupported_callback_error : public std::runtime_error {
    unsupported_callback_error()
        : std::runtime_error(
              "Unable to dispatch callback, need to add "
              "dispatch in GsofClient::callbackDispatch") {}
  };

  GsofClient() = delete;
  GsofClient(const std::string &ip_address, unsigned int port);
  ~GsofClient();
  GsofClient(const GsofClient &)            = delete;
  GsofClient &operator=(const GsofClient &) = delete;

  util::Status start();
  void stop();

  std::vector<MessageCallback>::iterator registerCallback(gsof::Id id, const MessageCallback &callback);

 protected:
  gsof::StreamChapterParser gsof_stream_parser_;

  template <typename GsofMessageParser>
  void gsofChapterCallback(const std::vector<std::byte> &chapter);

 private:
  std::unique_ptr<std::thread> tcp_thread_;
  std::atomic_bool keep_running_;

  network::TcpClient tcp_client_;

  std::unordered_map<gsof::Id, std::vector<MessageCallback>> message_callbacks_;

  void runTcpConnection();
  void grabAndParseTcp();
};

template <typename GsofMessageParser>
void GsofClient::gsofChapterCallback(const std::vector<std::byte> &chapter) {
  GsofMessageParser message_parser(chapter.data(), chapter.size());

  for (const gsof::Message &message : message_parser) {
    auto message_callbacks = message_callbacks_.find(message.getHeader().type);
    if (message_callbacks == message_callbacks_.end()) {
      continue;
    }

    for (const MessageCallback &callback : message_callbacks->second) {
      callback(message);
    }
  }
}

}  // namespace trmb
