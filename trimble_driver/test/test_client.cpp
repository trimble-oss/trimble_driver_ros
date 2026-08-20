/*
 * Copyright (c) 2026. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>

#include "gsof_record_builder.h"
#include "trimble_driver/gsof/message.h"
#include "trimble_driver/gsof_client.h"

class TestableGsofClient : public trmb::GsofClient {
 public:
  using trmb::GsofClient::GsofClient;

  void dispatchChapter(const std::vector<std::byte> &chapter) {
    gsofChapterCallback<trmb::gsof::MessageParser<trmb::gsof::SupportedPublicMessages>>(chapter);
  }
};

class ClientTest : public ::testing::Test {
 protected:
  ClientTest() : acceptor_(io_context_) {}

  void SetUp() override {
    boost::system::error_code ec;
    acceptor_.open(boost::asio::ip::tcp::v4(), ec);
    ASSERT_FALSE(ec) << ec.message();

    acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true), ec);
    ASSERT_FALSE(ec) << ec.message();

    const boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::loopback(), 0);
    acceptor_.bind(endpoint, ec);
    ASSERT_FALSE(ec) << ec.message();

    acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
    ASSERT_FALSE(ec) << ec.message();

    port_ = acceptor_.local_endpoint(ec).port();
    ASSERT_FALSE(ec) << ec.message();
  }

  void TearDown() override {
    boost::system::error_code ec;
    acceptor_.close(ec);
  }

  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::uint16_t port_ = 0;
};

TEST_F(ClientTest, client) {
  EXPECT_GT(port_, 0);
  trmb::GsofClient client("127.0.0.1", port_);

  EXPECT_NO_THROW(client.start());
  EXPECT_NO_THROW(client.stop());
}

TEST_F(ClientTest, clientDestructorWithoutStarting) {
  // See bug https://github.com/trimble-oss/trimble_driver_ros/issues/32
  EXPECT_GT(port_, 0);
  trmb::GsofClient client("127.0.0.1", port_);
}

TEST_F(ClientTest, callbackExceptionHandlerReportsExceptionAndContinuesDispatch) {
  using namespace trmb::gsof;

  EXPECT_GT(port_, 0);
  TestableGsofClient client("127.0.0.1", port_);

  PositionTimeInfo position_time{};
  position_time.header.type   = GSOF_ID_1_POS_TIME;
  position_time.header.length = sizeof(PositionTimeInfo) - sizeof(Header);
  const auto chapter          = gsof_test::serialize(position_time);

  bool exception_handler_called = false;
  Id reported_id                = 0;
  std::string reported_message;
  client.setCallbackExceptionHandler([&](Id id, const std::string &message) {
    exception_handler_called = true;
    reported_id              = id;
    reported_message         = message;
  });

  bool second_callback_called = false;
  client.registerCallback(GSOF_ID_1_POS_TIME, [](const Message &) { throw std::runtime_error("bad callback"); });
  client.registerCallback(GSOF_ID_1_POS_TIME, [&](const Message &) { second_callback_called = true; });

  client.dispatchChapter(chapter);

  EXPECT_TRUE(exception_handler_called);
  EXPECT_EQ(reported_id, GSOF_ID_1_POS_TIME);
  EXPECT_EQ(reported_message, "bad callback");
  EXPECT_TRUE(second_callback_called);
}

TEST_F(ClientTest, callbackInvokedFromTcpMessage) {
  using namespace trmb::gsof;

  EXPECT_GT(port_, 0);

  PositionTimeInfo position_time{};
  position_time.header.type   = GSOF_ID_1_POS_TIME;
  position_time.header.length = sizeof(PositionTimeInfo) - sizeof(Header);
  const auto message          = gsof_test::serialize(position_time);
  const auto record           = gsof_test::makeGenoutRecord(message.data(), message.size());

  trmb::GsofClient client("127.0.0.1", port_);

  std::promise<Id> callback_promise;
  auto callback_future = callback_promise.get_future();
  std::atomic_bool callback_reported{false};

  client.registerCallback(GSOF_ID_1_POS_TIME, [&](const Message &msg) {
    if (!callback_reported.exchange(true)) {
      callback_promise.set_value(msg.getHeader().type);
    }
  });

  std::string server_error;
  std::thread server_thread([this, &record, &server_error] {
    boost::system::error_code ec;
    boost::asio::ip::tcp::socket socket(io_context_);

    acceptor_.accept(socket, ec);
    if (ec) {
      server_error = ec.message();
      return;
    }

    boost::asio::write(socket, boost::asio::buffer(record.data(), record.size()), ec);
    if (ec) {
      server_error = ec.message();
      return;
    }

    socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket.close(ec);
  });

  const auto status = client.start();
  ASSERT_TRUE(status) << status.error_msg();

  const auto callback_status = callback_future.wait_for(std::chrono::seconds(2));

  client.stop();
  if (server_thread.joinable()) {
    server_thread.join();
  }

  ASSERT_TRUE(server_error.empty()) << server_error;
  ASSERT_EQ(callback_status, std::future_status::ready);
  EXPECT_EQ(callback_future.get(), GSOF_ID_1_POS_TIME);
}

TEST_F(ClientTest, callbackExceptionHandlerReportsTcpCallbackExceptionAndContinuesDispatch) {
  using namespace trmb::gsof;

  EXPECT_GT(port_, 0);

  PositionTimeInfo position_time{};
  position_time.header.type   = GSOF_ID_1_POS_TIME;
  position_time.header.length = sizeof(PositionTimeInfo) - sizeof(Header);
  const auto message          = gsof_test::serialize(position_time);
  const auto record           = gsof_test::makeGenoutRecord(message.data(), message.size());

  trmb::GsofClient client("127.0.0.1", port_);

  struct CallbackExceptionReport {
    Id id;
    std::string message;
  };

  std::promise<CallbackExceptionReport> exception_promise;
  auto exception_future = exception_promise.get_future();
  std::atomic_bool exception_reported{false};

  client.setCallbackExceptionHandler([&](Id id, const std::string &msg) {
    if (!exception_reported.exchange(true)) {
      exception_promise.set_value({id, msg});
    }
  });

  std::promise<Id> callback_promise;
  auto callback_future = callback_promise.get_future();
  std::atomic_bool callback_reported{false};

  client.registerCallback(GSOF_ID_1_POS_TIME, [](const Message &) { throw std::runtime_error("bad tcp callback"); });
  client.registerCallback(GSOF_ID_1_POS_TIME, [&](const Message &msg) {
    if (!callback_reported.exchange(true)) {
      callback_promise.set_value(msg.getHeader().type);
    }
  });

  std::string server_error;
  std::thread server_thread([this, &record, &server_error] {
    boost::system::error_code ec;
    boost::asio::ip::tcp::socket socket(io_context_);

    acceptor_.accept(socket, ec);
    if (ec) {
      server_error = ec.message();
      return;
    }

    boost::asio::write(socket, boost::asio::buffer(record.data(), record.size()), ec);
    if (ec) {
      server_error = ec.message();
      return;
    }

    socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket.close(ec);
  });

  const auto status = client.start();
  ASSERT_TRUE(status) << status.error_msg();

  const auto exception_status = exception_future.wait_for(std::chrono::seconds(2));
  const auto callback_status  = callback_future.wait_for(std::chrono::seconds(2));

  client.stop();
  if (server_thread.joinable()) {
    server_thread.join();
  }

  ASSERT_TRUE(server_error.empty()) << server_error;
  ASSERT_EQ(exception_status, std::future_status::ready);
  const auto exception_report = exception_future.get();
  EXPECT_EQ(exception_report.id, GSOF_ID_1_POS_TIME);
  EXPECT_EQ(exception_report.message, "bad tcp callback");
  ASSERT_EQ(callback_status, std::future_status::ready);
  EXPECT_EQ(callback_future.get(), GSOF_ID_1_POS_TIME);
}
