#include "ManipulatorDriver.h"

#include <cstdint>
#include <memory>

#include "ManipulatorInterface.h"
#include "ManipulatorPlanner.h"
#include "canDriver.h"
#include "defines.h"

int32_t jointoffset[6] = {32000, 13930, 13156, 31861, 31692, 32000 + 29038 - 65536};
uint8_t control_frequency = 125;

// 这些对象存在依赖关系，定义顺序不能调整。
canDriver candriver(0, 500);
ManipulatorInterface armInterface(jointoffset, ManipulatorState.NumOfJoint, control_frequency, candriver);
ManipulatorPlanner armPlanner(armInterface);

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManipulatorDriver>());
  rclcpp::shutdown();
  return 0;
}
