// Copyright 2026 Wouter van der Stoel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "test_messages/msg/foo.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace {

bool FileExists(const std::string& path) {
  std::ifstream file(path);
  return file.is_open();
}

TEST(TestMessages, MessageFieldsAreAccessible) {
  test_messages::msg::Foo foo;
  foo.bar.baz.value = 42;
  EXPECT_EQ(foo.bar.baz.value, 42);
}

TEST(TestMessages, MessageDefinitionsExist) {
  const auto package_prefix =
      ament_index_cpp::get_package_share_directory("test_messages");
  EXPECT_TRUE(FileExists(package_prefix + "/msg/Foo.msg"));
  EXPECT_TRUE(FileExists(package_prefix + "/msg/Baz.msg"));
  EXPECT_TRUE(FileExists(package_prefix + "/msg/Foo.idl"));
  EXPECT_TRUE(FileExists(package_prefix + "/msg/Bar.idl"));
  EXPECT_TRUE(FileExists(package_prefix + "/msg/Baz.idl"));
}

}  // namespace
