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
#include <rcutils/logging.h>

#include <rclcpp/rclcpp.hpp>

#include "cx_utils/clips_env_context.hpp"

// bring in CLIPS logical name constants
#include "clips_ns/clips.h"

namespace
{
struct CapturedLog
{
  int severity;
  std::string logger_name;
  std::string message;
};

// global capture buffer — reset before each test
std::vector<CapturedLog> g_captured;

void capture_handler(
  const rcutils_log_location_t *, int severity, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char buf[1024];
  vsnprintf(buf, sizeof(buf), format, *args);
  g_captured.push_back({severity, name ? name : "", buf});
}
}  // namespace

class CLIPSLoggerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    rcutils_logging_set_output_handler(capture_handler);
    // make sure all severity levels are enabled so nothing gets filtered
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
  }

  static void TearDownTestSuite()
  {
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    logger_ = std::make_unique<cx::CLIPSLogger>("test_clips", false, false);
    g_captured.clear();
  }
  std::unique_ptr<cx::CLIPSLogger> logger_;
};

// --- Buffering ---

TEST_F(CLIPSLoggerTest, EmptyStringProducesNoOutput)
{
  logger_->log(clips::STDOUT, "");
  EXPECT_TRUE(g_captured.empty());
}

TEST_F(CLIPSLoggerTest, PartialLineNotEmitted)
{
  logger_->log(clips::STDOUT, "no newline here");
  EXPECT_TRUE(g_captured.empty());
}

TEST_F(CLIPSLoggerTest, CompleteLineIsEmitted)
{
  logger_->log(clips::STDOUT, "hello\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_NE(g_captured[0].message.find("hello"), std::string::npos);
}

TEST_F(CLIPSLoggerTest, PartialThenCompletionEmitsOnce)
{
  logger_->log(clips::STDOUT, "hel");
  EXPECT_TRUE(g_captured.empty());
  logger_->log(clips::STDOUT, "lo\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_NE(g_captured[0].message.find("hello"), std::string::npos);
}

TEST_F(CLIPSLoggerTest, TwoLinesInOneCallEmitsTwice)
{
  logger_->log(clips::STDOUT, "line1\nline2\n");
  ASSERT_EQ(g_captured.size(), 2u);
  EXPECT_NE(g_captured[0].message.find("line1"), std::string::npos);
  EXPECT_NE(g_captured[1].message.find("line2"), std::string::npos);
}

TEST_F(CLIPSLoggerTest, StdinIsIgnored)
{
  logger_->log(clips::STDIN, "ignored\n");
  EXPECT_TRUE(g_captured.empty());
}

// --- Severity routing ---

TEST_F(CLIPSLoggerTest, DebugLogicalNameRoutesToDebug)
{
  logger_->log("debug", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_DEBUG);
}

TEST_F(CLIPSLoggerTest, LogDebugLogicalNameRoutesToDebug)
{
  logger_->log("logdebug", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_DEBUG);
}

TEST_F(CLIPSLoggerTest, WarnLogicalNameRoutesToWarn)
{
  logger_->log("warn", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_WARN);
}

TEST_F(CLIPSLoggerTest, LogWarnLogicalNameRoutesToWarn)
{
  logger_->log("logwarn", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_WARN);
}

TEST_F(CLIPSLoggerTest, StdWrnRoutesToWarn)
{
  logger_->log(clips::STDWRN, "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_WARN);
}

TEST_F(CLIPSLoggerTest, ErrorLogicalNameRoutesToError)
{
  logger_->log("error", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_ERROR);
}

TEST_F(CLIPSLoggerTest, LogErrorLogicalNameRoutesToError)
{
  logger_->log("logerror", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_ERROR);
}

TEST_F(CLIPSLoggerTest, StdErrRoutesToError)
{
  logger_->log(clips::STDERR, "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_ERROR);
}

TEST_F(CLIPSLoggerTest, UnknownLogicalNameRoutesToInfo)
{
  logger_->log("something_else", "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(CLIPSLoggerTest, StdoutRoutesToInfoByDefault)
{
  logger_->log(clips::STDOUT, "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_INFO);
}

// --- stdout_to_debug flag ---

TEST_F(CLIPSLoggerTest, StdoutRoutesToDebugWhenFlagSet)
{
  auto debug_logger = cx::CLIPSLogger("test_clips_debug", false, true);
  debug_logger.log(clips::STDOUT, "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_DEBUG);
}

// --- Logger name ---

TEST_F(CLIPSLoggerTest, LoggerNameIsCorrect)
{
  logger_->log(clips::STDOUT, "msg\n");
  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].logger_name, "test_clips");
}
