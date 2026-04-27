/*
 * Copyright (c) 2026. Trimble Inc.
 * All rights reserved.
 */

#include <boost/asio.hpp>

#include <gtest/gtest.h>

#include "trimble_driver/gsof_client.h"

class ClientTest : public ::testing::Test
{
protected:
	ClientTest()
	  : acceptor_(io_context_)
	{
	}

	void SetUp() override
	{
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

	void TearDown() override
	{
		boost::system::error_code ec;
		acceptor_.close(ec);
	}

	boost::asio::io_context io_context_;
	boost::asio::ip::tcp::acceptor acceptor_;
	std::uint16_t port_ = 0;
};

TEST_F(ClientTest, client)
{
	EXPECT_GT(port_, 0);
    trmb::GsofClient client("127.0.0.1", port_);
    
    EXPECT_NO_THROW(client.start());
    EXPECT_NO_THROW(client.stop());
}

TEST_F(ClientTest, clientDestructorWithoutStarting)
{
    // See bug https://github.com/trimble-oss/trimble_driver_ros/issues/32
	EXPECT_GT(port_, 0);
    trmb::GsofClient client("127.0.0.1", port_);
}

