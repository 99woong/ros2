#pragma once
#include "time_sync_calib/types.hpp"
#include <string>

namespace time_sync_calib
{

// -------------------------------------------------------
// ROS2 bag 파일에서 두 IMU 데이터를 읽어 반환
// -------------------------------------------------------
class BagReader
{
public:
  explicit BagReader(const Config& cfg);

  // bag을 읽어 internal / external IMU 데이터 채움
  // 반환: 성공 여부
  bool read(ImuData& internal_out, ImuData& external_out);

private:
  const Config& cfg_;
};

}  // namespace time_sync_calib