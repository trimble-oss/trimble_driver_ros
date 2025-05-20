/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/message.h"

#include <cstring>

namespace trmb::gsof {

SatelliteType toSatelliteType(int sv_system) {
  switch (sv_system) {
    case 0:
      return SatelliteType::GPS;
    case 1:
      return SatelliteType::SBAS;
    case 2:
      return SatelliteType::GLONASS;
    case 3:
      return SatelliteType::GALILEO;
    case 4:
      return SatelliteType::QZSS;
    case 5:
      return SatelliteType::BEIDOU;
    case 6:
      return SatelliteType::IRNSS;
    case 10:
      return SatelliteType::OMNISTAR;
    default:
      return SatelliteType::UNKNOWN;
  }
}

SolutionIntegrity toSolutionIntegrity(uint8_t value) {
  value = (value & 0b0000'1100) >> 2;
  switch (value) {
    case 0:
      return SolutionIntegrity::k_not_checking;
    case 1:
      return SolutionIntegrity::k_checking_initialization;
    case 2:
      return SolutionIntegrity::k_initialization_passed;
    case 3:
      return SolutionIntegrity::k_initialization_failed;
    default:
      return SolutionIntegrity::k_unknown;
  }
}

RtkCondition toRtkCondition(uint8_t rtk_condition) {
  uint8_t value = rtk_condition & 0b0000'1111;
  switch (value) {
    case 0:
      return RtkCondition::k_new_position_computed;
    case 1:
      return RtkCondition::k_unable_to_obtain_synced_pair_from_both_stations;
    case 2:
      return RtkCondition::k_inssufficient_double_difference_measurements;
    case 3:
      return RtkCondition::k_reference_position_unavailable;
    case 4:
      return RtkCondition::k_failed_integer_verification_with_fixed_solution;
    case 5:
      return RtkCondition::k_solution_rms_over_limit;
    case 6:
      return RtkCondition::k_pdop_or_rdop_exceeds_mask;
    default:
      return RtkCondition::k_unknown;
  }
}

RtcmStatus toRtcmStatus(uint8_t network_flags) {
  uint8_t value = network_flags & 0b0000'0110;
  switch (value) {
    case 0:
      return RtcmStatus::k_not_available_or_unknown;
    case 1:
      return RtcmStatus::k_collecting_messages;
    case 2:
      return RtcmStatus::k_collection_complete;
    case 3:
      return RtcmStatus::k_working;
    default:
      return RtcmStatus::k_not_available_or_unknown;
  }
}

BaseQuality toBaseQuality(uint8_t quality) {
  if (quality <= 4) {
    return static_cast<BaseQuality>(quality);
  }

  return BaseQuality::k_not_available_or_invalid;
}

Message::Message(const std::byte* data, const std::size_t length) : data_(data), length_(length) {}

Header Message::getHeader() const {
  Header header;
  std::memcpy(&header, data_, sizeof(Header));
  return header;
}

template <>
Velocity Message::as<Velocity>() const {
  // Velocity requires special treatment because the local heading field is optional
  Velocity v{};
  auto data = data_;
  std::memcpy(&v.header, data, sizeof(Header));
  data += sizeof(Header);
  std::memcpy(&v.velocity_flags, data, sizeof(v.velocity_flags));
  data += sizeof(v.velocity_flags);

  std::memcpy(&v.velocity, data, sizeof(v.velocity));
  data += sizeof(v.velocity);
  std::memcpy(&v.heading, data, sizeof(v.heading));
  data += sizeof(v.heading);
  std::memcpy(&v.vertical_velocity, data, sizeof(v.vertical_velocity));
  data += sizeof(v.vertical_velocity);

  if (v.header.length == 17) {
    float f = 0;
    std::memcpy(&f, data, sizeof(float));
    v.local_heading = f;
  }

  v.switchEndianess();

  return v;
}

template <>
AttitudeInfo Message::as<AttitudeInfo>() const {
  // AttitudeInfo requires special treatment because the variance is optional
  AttitudeInfo att{};

  auto data = data_;
  std::memcpy(&att.header, data, sizeof(Header));
  data += sizeof(Header);

  std::memcpy(&att.gps_time, data, sizeof(att.gps_time));
  data += sizeof(att.gps_time);
  std::memcpy(&att.flags, data, sizeof(att.flags));
  data += sizeof(att.flags);
  std::memcpy(&att.num_svs, data, sizeof(att.num_svs));
  data += sizeof(att.num_svs);
  std::memcpy(&att.calc_mode, data, sizeof(att.calc_mode));
  data += sizeof(att.calc_mode);
  std::memcpy(&att.reserved, data, sizeof(att.reserved));
  data += sizeof(att.reserved);
  std::memcpy(&att.pyr, data, sizeof(att.pyr));
  data += sizeof(att.pyr);
  std::memcpy(&att.master_slave_range, data, sizeof(att.master_slave_range));
  data += sizeof(att.master_slave_range);
  std::memcpy(&att.pdop, data, sizeof(att.pdop));
  data += sizeof(att.pdop);

  if (att.header.length == 70) {
    att.variance.emplace();
    std::memcpy(&(att.variance.value()), data, sizeof(AttitudeInfo::Variance));
  }

  att.switchEndianess();
  return att;
}

template <>
[[nodiscard]] AllSvBrief Message::as<AllSvBrief>() const {
  AllSvBrief i{};
  auto data = data_;
  std::memcpy(&i.header, data, sizeof(Header));
  data = data + sizeof(Header);
  std::memcpy(&i.num_svs, data, sizeof(i.num_svs));
  data = data + sizeof(i.num_svs);

  i.sv_info.resize(i.num_svs);
  for (auto& info : i.sv_info) {
    std::memcpy(&info, data, sizeof(SVBriefInfo));
    data += sizeof(SVBriefInfo);
  }

  i.switchEndianess();
  return i;
}

template <>
[[nodiscard]] AllSvDetailed Message::as<AllSvDetailed>() const {
  AllSvDetailed i{};
  auto data = data_;
  std::memcpy(&i.header, data, sizeof(Header));
  data = data + sizeof(Header);
  std::memcpy(&i.num_svs, data, sizeof(i.num_svs));
  data = data + sizeof(i.num_svs);

  SVDetailedInfo info{};
  for (int n = 0; n < i.num_svs; n++) {
    std::memcpy(&info, data, sizeof(SVDetailedInfo));
    i.sv_info.push_back(info);
    data += sizeof(SVDetailedInfo);
  }

  i.switchEndianess();
  return i;
}

template <>
[[nodiscard]] PositionTypeInformation Message::as<PositionTypeInformation>() const {
  PositionTypeInformation p{};

  auto data = data_;
  std::memcpy(&p.header, data, sizeof(Header));
  data += sizeof(Header);

  std::memcpy(&p.error_scale, data, sizeof(p.error_scale));
  data += sizeof(p.error_scale);
  std::memcpy(&p.solution_flags, data, sizeof(p.solution_flags));
  data += sizeof(p.solution_flags);
  std::memcpy(&p.rtk_condition, data, sizeof(p.rtk_condition));
  data += sizeof(p.rtk_condition);
  std::memcpy(&p.correction_age, data, sizeof(p.correction_age));
  data += sizeof(p.correction_age);
  std::memcpy(&p.network_flags, data, sizeof(p.network_flags));
  data += sizeof(p.network_flags);
  std::memcpy(&p.network_flags2, data, sizeof(p.network_flags2));
  data += sizeof(p.network_flags2);
  std::memcpy(&p.frame_flag, data, sizeof(p.frame_flag));
  data += sizeof(p.frame_flag);
  std::memcpy(&p.itrf_epoch, data, sizeof(p.itrf_epoch));
  data += sizeof(p.itrf_epoch);
  std::memcpy(&p.tectonic_plate, data, sizeof(p.tectonic_plate));
  data += sizeof(p.tectonic_plate);
  std::memcpy(&p.rtx_ram_sub_minutes_left, data, sizeof(p.rtx_ram_sub_minutes_left));
  data += sizeof(p.rtx_ram_sub_minutes_left);
  std::memcpy(&p.pole_wobble_status_flag, data, sizeof(p.pole_wobble_status_flag));
  data += sizeof(p.pole_wobble_status_flag);
  std::memcpy(&p.pole_wobble_distance, data, sizeof(p.pole_wobble_distance));
  data += sizeof(p.pole_wobble_distance);
  std::memcpy(&p.position_fix_type, data, sizeof(p.position_fix_type));
  data += sizeof(p.position_fix_type);

  constexpr std::size_t k_fw4_94_msg_size = 26;
  if (p.header.length > k_fw4_94_msg_size) {
    const std::size_t size_of_remaining = p.header.length - k_fw4_94_msg_size - sizeof(Header);
    p.unparsed_bytes.resize(size_of_remaining);
    std::memcpy(p.unparsed_bytes.data(), data, size_of_remaining);
  }

  p.switchEndianess();

  return p;
}

}  // namespace trmb::gsof
