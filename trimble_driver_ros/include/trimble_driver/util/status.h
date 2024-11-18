/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <string>

#include "trimble_driver/util/wise_enum/wise_enum.h"

namespace util {

WISE_ENUM_CLASS(ErrorCode, OK, CANCELLED, INVALID_ARGUMENT, NOT_FOUND, UNIMPLEMENTED, CONNECTION_ERROR, DATA_EMPTY)

class Status {
 public:
  explicit Status(ErrorCode code = ErrorCode::OK, const std::string &msg = "");

  explicit operator bool() const;
  bool ok() const;
  ErrorCode error_code() const;
  const std::string &error_msg() const;

  std::string toString() const;

 private:
  ErrorCode error_code_;
  std::string error_msg_;
};

}  // namespace util
