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

#pragma once
#include <string>
#include <utility>

#include "pqxx/pqxx"

namespace cx
{

template <typename... Args>
pqxx::result exec_params(pqxx::transaction_base & tx, const std::string & sql, Args &&... args)
{
#if defined(PQXX_EXEC_PARAMS_DEPRECATED)
  return tx.exec(sql, pqxx::params{std::forward<Args>(args)...});
#else
  return tx.exec_params(sql, std::forward<Args>(args)...);
#endif
}

template <typename... Args>
pqxx::row exec_params1(pqxx::transaction_base & tx, const std::string & sql, Args &&... args)
{
#if defined(PQXX_EXEC_PARAMS_DEPRECATED)
  return tx.exec(sql, pqxx::params{std::forward<Args>(args)...}).one_row();
#else
  return tx.exec_params1(sql, std::forward<Args>(args)...);
#endif
}

}  // namespace cx
