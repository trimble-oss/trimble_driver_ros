/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <byteswap.h>  // UNIX Specific

#include <cstring>

namespace trmb::utils {
/**
 * https://stackoverflow.com/questions/3529263/template-specialization-according-to-sizeof-type
 * Problem as old as the internet :-)
 * The Trimble GSOF documentation pages all show sample code to parse data types from big to little endian with
 * verbose pointer incrementation and reassignment. Modern processors usually have single instructions to swap the
 * bytes of a chunk of memory. Proof https://godbolt.org/z/EG7G5U
 */

namespace impl {
template <typename T, std::size_t n>
struct Byteswap;

template <typename T>
struct Byteswap<T, 2> {
  T operator()(T val) const {
    uint16_t bytes;
    memcpy(&bytes, &val, sizeof(bytes));
    bytes = bswap_16(bytes);
    T out_val;
    memcpy(&out_val, &bytes, sizeof(T));
    return out_val;
  }
};

template <typename T>
struct Byteswap<T, 4> {
  T operator()(T val) const {
    uint32_t bytes;
    memcpy(&bytes, &val, sizeof(bytes));
    bytes = bswap_32(bytes);
    T out_val;
    memcpy(&out_val, &bytes, sizeof(T));
    return out_val;
  }
};

template <typename T>
struct Byteswap<T, 8> {
  T operator()(T val) const {
    uint64_t bytes;
    memcpy(&bytes, &val, sizeof(bytes));
    bytes = bswap_64(bytes);
    T out_val;
    memcpy(&out_val, &bytes, sizeof(T));
    return out_val;
  }
};

template <typename T, std::size_t n>
struct ByteswapInPlace;

template <typename T>
struct ByteswapInPlace<T, 2> {
  void operator()(T *val) const {
    uint16_t bytes;
    memcpy(&bytes, val, sizeof(bytes));
    bytes = bswap_16(bytes);
    memcpy(val, &bytes, sizeof(T));
  }
};

template <typename T>
struct ByteswapInPlace<T, 4> {
  void operator()(T *val) const {
    uint32_t bytes;
    memcpy(&bytes, val, sizeof(bytes));
    bytes = bswap_32(bytes);
    memcpy(val, &bytes, sizeof(T));
  }
};

template <typename T>
struct ByteswapInPlace<T, 8> {
  void operator()(T *val) const {
    uint64_t bytes;
    memcpy(&bytes, val, sizeof(bytes));
    bytes = bswap_64(bytes);
    memcpy(val, &bytes, sizeof(T));
  }
};

}  // namespace impl

template <typename T>
T byteswap(T val) {
  return impl::Byteswap<T, sizeof(T)>()(val);
}

template <typename T>
void byteswapInPlace(T *val) {
  impl::ByteswapInPlace<T, sizeof(T)>()(val);
}

}  // namespace trmb::utils
