/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cmath>

#include "trimble_driver/utils/byteswap.h"

using trmb::utils::byteswapInPlace;

namespace trmb {

template <typename T>
static T deg2rad(const T val) {
  static constexpr double DEG2RAD = M_PI / 180.0;
  return static_cast<double>(val) * DEG2RAD;
}

template <typename T>
static T rad2deg(const T val) {
  static constexpr double RAD2DEG = 180.0 / M_PI;
  return static_cast<double>(val) * RAD2DEG;
}

#pragma pack(push, 1)

// Forward declaration so Ned can convert to Enu and vice-versa
template <typename T>
struct Enu;

template <typename T>
struct Ned {
  T north;
  T east;
  T down;

  Ned() = default;
  explicit Ned(const Enu<T>& enu) { fromEnu(enu); }
  void fromEnu(const Enu<T>& enu) {
    north = enu.north;
    east  = enu.east;
    down  = -enu.up;
  }
  void switchEndianess() {
    byteswapInPlace(&north);
    byteswapInPlace(&east);
    byteswapInPlace(&down);
  }
};

using Nedf = Ned<float>;
using Nedd = Ned<double>;

template <typename T>
struct Enu {
  T east;
  T north;
  T up;

  Enu() = default;
  explicit Enu(const Ned<T>& ned) { fromNed(ned); }
  void fromNed(const Ned<T>& ned) {
    east  = ned.east;
    north = ned.north;
    up    = -ned.down;
  }
  void switchEndianess() {
    byteswapInPlace(&east);
    byteswapInPlace(&north);
    byteswapInPlace(&up);
  }
};

using Enuf = Enu<float>;
using Enud = Enu<double>;

template <typename T>
struct Rph {
  T roll;
  T pitch;
  T heading;
  void switchEndianess() {
    byteswapInPlace(&roll);
    byteswapInPlace(&pitch);
    byteswapInPlace(&heading);
  }
};

using Rphf = Rph<float>;
using Rphd = Rph<double>;

template <typename T>
struct Pyr {
  T pitch;
  T yaw;
  T roll;
  void switchEndianess() {
    byteswapInPlace(&pitch);
    byteswapInPlace(&yaw);
    byteswapInPlace(&roll);
  }
};

using Pyrf = Pyr<float>;
using Pyrd = Pyr<double>;

template <typename T>
struct Xyz {
  T x;
  T y;
  T z;
  void switchEndianess() {
    byteswapInPlace(&x);
    byteswapInPlace(&y);
    byteswapInPlace(&z);
  }
};

using Xyzf = Xyz<float>;
using Xyzd = Xyz<double>;
using Xyzl = Xyz<long>;

template <typename T>
struct Lla {
  T latitude;
  T longitude;
  T altitude;
  void switchEndianess() {
    byteswapInPlace(&latitude);
    byteswapInPlace(&longitude);
    byteswapInPlace(&altitude);
  }
};

using Llaf = Lla<float>;
using Llad = Lla<double>;

#pragma pack(pop)

}  // namespace trmb
