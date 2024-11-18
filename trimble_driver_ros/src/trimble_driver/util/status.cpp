/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/util/status.h"

namespace util {

Status::Status(ErrorCode code, const std::string &msg) : error_code_(code), error_msg_(msg) {}

Status::operator bool() const { return ok(); }

bool Status::ok() const { return error_code() == ErrorCode::OK; }

ErrorCode Status::error_code() const { return error_code_; }

const std::string &Status::error_msg() const { return error_msg_; }

std::string Status::toString() const {
  // XXX Very inefficient
  return std::string(wise_enum::to_string(error_code_)) + ": " + error_msg_;
}

}  // namespace util
