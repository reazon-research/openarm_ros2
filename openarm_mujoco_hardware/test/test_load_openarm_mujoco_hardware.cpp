#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/system_interface.hpp"

static constexpr const char * kPluginName =
  "mujoco_openarm_hardware/MujocoHardware";

TEST(TestLoadMujocoOpenarmHardware, can_load_plugin)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "mujoco_openarm_hardware",
    "hardware_interface::SystemInterface");
  std::shared_ptr<hardware_interface::SystemInterface> instance;

  ASSERT_NO_THROW(
    instance = loader.createSharedInstance(kPluginName)
  );
  EXPECT_NE(instance, nullptr);
}
