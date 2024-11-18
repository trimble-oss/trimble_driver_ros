/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/message_parser.h"

#include <cstring>

#include "trimble_driver/gsof/message.h"

namespace trmb::gsof {

const std::set<std::uint8_t> SupportedPublicMessages::ids(std::begin(detail::k_supported_msgs_ids),
                                                          std::end(detail::k_supported_msgs_ids));

template <>
class MessageParser<SupportedPublicMessages>;

}  // namespace trmb::gsof
