#include <gtest/gtest.h>

#include <test_messages/msg/foo.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace {
TEST(TestMessages, MessageFieldsAreAccessible) {
  test_messages::msg::Foo foo;
  foo.bar.baz.value = 42;
  EXPECT_EQ(foo.bar.baz.value, 42);
}
TEST(TestMessages, MessageDefinitionsExist) {
  auto package_prefix =
      ament_index_cpp::get_package_share_directory("test_messages");
  EXPECT_TRUE(std::filesystem::exists(package_prefix + "/msg/Foo.msg"));
  EXPECT_TRUE(std::filesystem::exists(package_prefix + "/msg/Foo.idl"));
  EXPECT_FALSE(std::filesystem::exists(package_prefix + "/msg/Bar.msg"));
  EXPECT_TRUE(std::filesystem::exists(package_prefix + "/msg/Bar.idl"));
  EXPECT_TRUE(std::filesystem::exists(package_prefix + "/msg/Baz.msg"));
  EXPECT_TRUE(std::filesystem::exists(package_prefix + "/msg/Baz.idl"));
}

}  // namespace
