/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace trmb {
class ParserInterface {
 public:
  virtual void setData(const std::byte *data, std::size_t length) = 0;
  [[nodiscard]] virtual bool isValid() const                      = 0;
  [[nodiscard]] virtual bool isSupported() const                  = 0;

 protected:
  const std::byte *data_;
  std::size_t length_;
};
}  // namespace trmb
