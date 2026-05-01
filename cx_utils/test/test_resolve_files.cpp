// Copyright (c) 2026 Carologistics
// SPDX-License-Identifier: Apache-2.0
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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "cx_utils/param_utils.hpp"

namespace fs = std::filesystem;

class ResolveFilesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // create a temp directory with a test file in it
    tmp_dir_ = fs::temp_directory_path() / "cx_utils_test";
    fs::create_directories(tmp_dir_);
    tmp_file_ = tmp_dir_ / "test_file.yaml";
    std::ofstream(tmp_file_).close();  // create empty file
  }

  void TearDown() override { fs::remove_all(tmp_dir_); }

  fs::path tmp_dir_;
  fs::path tmp_file_;
};

TEST_F(ResolveFilesTest, AbsolutePathExists)
{
  std::vector<std::string> in = {tmp_file_.string()};
  std::vector<std::string> out;
  cx::cx_utils::resolve_files(in, {}, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], tmp_file_.string());
}

TEST_F(ResolveFilesTest, AbsolutePathNotFound)
{
  std::vector<std::string> in = {(tmp_dir_ / "nonexistent.yaml").string()};
  std::vector<std::string> out;
  EXPECT_THROW(cx::cx_utils::resolve_files(in, {}, out), std::runtime_error);
}

TEST_F(ResolveFilesTest, RelativePathNoShareDirs)
{
  std::vector<std::string> in = {"some/relative/file.yaml"};
  std::vector<std::string> out;
  EXPECT_THROW(cx::cx_utils::resolve_files(in, {}, out), std::runtime_error);
}

TEST_F(ResolveFilesTest, EmptyInput)
{
  std::vector<std::string> in;
  std::vector<std::string> out;
  cx::cx_utils::resolve_files(in, {}, out);
  EXPECT_TRUE(out.empty());
}

TEST_F(ResolveFilesTest, MultipleAbsolutePaths)
{
  fs::path tmp_file2 = tmp_dir_ / "test_file2.yaml";
  std::ofstream(tmp_file2).close();

  std::vector<std::string> in = {tmp_file_.string(), tmp_file2.string()};
  std::vector<std::string> out;
  cx::cx_utils::resolve_files(in, {}, out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0], tmp_file_.string());
  EXPECT_EQ(out[1], tmp_file2.string());
}

TEST_F(ResolveFilesTest, PartialFailureThrows)
{
  // first file exists, second doesn't — should throw on second
  std::vector<std::string> in = {tmp_file_.string(), (tmp_dir_ / "missing.yaml").string()};
  std::vector<std::string> out;
  EXPECT_THROW(cx::cx_utils::resolve_files(in, {}, out), std::runtime_error);
}

TEST_F(ResolveFilesTest, RelativePathFoundInShareDir)
{
  std::vector<std::string> in = {"package.xml"};
  std::vector<std::string> out;
  cx::cx_utils::resolve_files(in, {"cx_utils"}, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(fs::exists(out[0]));
}

TEST_F(ResolveFilesTest, RelativePathFoundInSecondShareDir)
{
  // first share dir doesn't have it, second does
  std::vector<std::string> in = {"package.xml"};
  std::vector<std::string> out;
  cx::cx_utils::resolve_files(in, {"nonexistent_package", "cx_utils"}, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(fs::exists(out[0]));
}
