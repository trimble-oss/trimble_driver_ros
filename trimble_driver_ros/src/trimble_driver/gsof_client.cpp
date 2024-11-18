/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof_client.h"

#include "trimble_driver/gsof/gsof.h"
#include "trimble_driver/gsof/packet_parser.h"

namespace trmb {

GsofClient::GsofClient(const std::string &ip_address, unsigned int port)
    : keep_running_(true), tcp_client_(ip_address, port) {
  gsof_stream_parser_.registerGsofChapterFoundCallback([this](const std::vector<std::byte> &chapter) {
    this->gsofChapterCallback<gsof::MessageParser<gsof::SupportedPublicMessages>>(chapter);
  });
}

GsofClient::~GsofClient() {
  stop();
  if (tcp_thread_->joinable()) {
    tcp_thread_->join();
  }
}

util::Status GsofClient::start() {
  util::Status status = tcp_client_.open();
  if (!status) return status;

  tcp_thread_ = std::make_unique<std::thread>(&GsofClient::runTcpConnection, this);
  return status;
}

void GsofClient::stop() { keep_running_ = false; }

std::vector<GsofClient::MessageCallback>::iterator GsofClient::registerCallback(
    gsof::Id id,
    const GsofClient::MessageCallback &callback) {
  if (message_callbacks_.count(id) == 0) {
    message_callbacks_[id] = std::vector<MessageCallback>();
  }

  message_callbacks_[id].push_back(callback);

  return --message_callbacks_[id].end();
}

void GsofClient::runTcpConnection() {
  while (keep_running_.load()) {
    grabAndParseTcp();
  }
}

void GsofClient::grabAndParseTcp() {
  int bytes_rcvd = tcp_client_.receive();
  if (bytes_rcvd < 1) return;

  auto &&buffer = tcp_client_.getBuffer();
  gsof_stream_parser_.readSome(buffer.data(), bytes_rcvd);
}

}  // namespace trmb
