/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstring>

#include "trimble_driver/utils/byteswap.h"

TEST(byteswap, unsignedIntegerSwaps) {
  using trmb::utils::byteswap;

  uint16_t test16 = 0x0200;
  ASSERT_EQ(byteswap(test16), 0x00'02U);

  uint32_t test32 = 0x01'02'03'04;
  ASSERT_EQ(byteswap(test32), 0x04'03'02'01U);

  uint64_t test64 = 0x01'02'03'04'05'06'07'08U;
  ASSERT_EQ(byteswap(test64), 0x08'07'06'05'04'03'02'01U);
}

TEST(byteswap, floatingPointSwaps) {
  using trmb::utils::byteswap;
  // @formatter off
  uint32_t test32 = (0x01 << 24) | (0x02 << 16) | (0x03 << 8) | (0x04);
  uint32_t res32  = (0x04 << 24) | (0x03 << 16) | (0x02 << 8) | (0x01);
  float testf, resf;
  std::memcpy(&testf, &test32, sizeof(float));
  std::memcpy(&resf, &res32, sizeof(float));
  // @formatter on

  ASSERT_EQ(resf, byteswap(testf));

  // @formatter off
  uint64_t test64 = (0x01UL << 56) | (0x02UL << 48) | (0x03UL << 40) | (0x04UL << 32) | (0x05UL << 24) |
                    (0x06UL << 16) | (0x07UL << 8) | (0x08UL);
  uint64_t res64 = (0x08UL << 56) | (0x07UL << 48) | (0x06UL << 40) | (0x05UL << 32) | (0x04UL << 24) | (0x03UL << 16) |
                   (0x02UL << 8) | (0x01UL);
  double testd, resd;
  std::memcpy(&testd, &test64, sizeof(double));
  std::memcpy(&resd, &res64, sizeof(double));
  // @formatter on

  ASSERT_EQ(resd, byteswap(testd));
}

TEST(byteswap, unsignedInplace) {
  using trmb::utils::byteswapInPlace;
  uint16_t test16 = 0x0200;
  byteswapInPlace(&test16);
  ASSERT_EQ(test16, 0x00'02U);

  uint32_t test32 = 0x01'02'03'04;
  byteswapInPlace(&test32);
  ASSERT_EQ(test32, 0x04'03'02'01U);

  uint64_t test64 = 0x01'02'03'04'05'06'07'08U;
  byteswapInPlace(&test64);
  ASSERT_EQ(test64, 0x08'07'06'05'04'03'02'01U);
}

TEST(byteswap, floatingPointInplace) {
  using trmb::utils::byteswapInPlace;
  // @formatter off
  uint32_t test32 = (0x01u << 24u) | (0x02u << 16u) | (0x03u << 8u) | (0x04u);
  uint32_t res32  = (0x04u << 24u) | (0x03u << 16u) | (0x02u << 8u) | (0x01u);
  float testf, resf;
  std::memcpy(&testf, &test32, sizeof(float));
  std::memcpy(&resf, &res32, sizeof(float));
  // @formatter on
  byteswapInPlace(&testf);
  ASSERT_EQ(resf, testf);

  // @formatter off
  uint64_t test64 = (0x01UL << 56U) | (0x02UL << 48U) | (0x03UL << 40U) | (0x04UL << 32U) | (0x05UL << 24U) |
                    (0x06UL << 16U) | (0x07UL << 8U) | (0x08UL);
  uint64_t res64 = (0x08UL << 56U) | (0x07UL << 48U) | (0x06UL << 40U) | (0x05UL << 32U) | (0x04UL << 24U) |
                   (0x03UL << 16U) | (0x02UL << 8U) | (0x01UL);
  double testd, resd;
  std::memcpy(&testd, &test64, sizeof(double));
  std::memcpy(&resd, &res64, sizeof(double));
  // @formatter on
  byteswapInPlace(&testd);
  ASSERT_EQ(resd, testd);
}
