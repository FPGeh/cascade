// Copyright 2017-2019 VMware, Inc.
// SPDX-License-Identifier: BSD-2-Clause
//
// The BSD-2 license (the License) set forth below applies to all parts of the
// Cascade project.  You may not use this file except in compliance with the
// License.
//
// BSD-2 License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include "ext/cl/include/cl.h"
#include "gtest/gtest.h"
#include "test/harness.h"

using namespace cascade;
using namespace cl;
using namespace std;

auto& march = StrArg<string>::create("--march")
  .initial("minimal");

int main(int argc, char** argv) {
  Simple::read(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(benchmark, array) {
  run_code(march.value(), "data/test/benchmark/array/run_7.v", "268435457\n");
}
TEST(benchmark, bitcoin) {
  run_code(march.value(), "data/test/benchmark/bitcoin/run_20.v", "1ce5c0 1ce5c5\n");
}
TEST(benchmark, mips32) {
  run_code(march.value(), "data/test/benchmark/mips32/run_bubble_2048.v", "1");
}
TEST(benchmark, nw) {
  run_code(march.value(), "data/test/benchmark/nw/run_16.v", "TODO...");
}
TEST(benchmark, regex) {
  run_code(march.value(), "data/test/benchmark/regex/run_disjunct_16.v", "6784");
}
