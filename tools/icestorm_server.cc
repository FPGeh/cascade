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

#include <cstring>
#include <iostream>
#include <signal.h>
#include <string>
#include "ext/cl/include/cl.h"
#include "src/target/core/icebrk/icestorm_server.h"

using namespace cl;
using namespace cascade;
using namespace std;

namespace {

__attribute__((unused)) auto& g = Group::create("Icestorm Server Options");
auto& path = StrArg<string>::create("--path")
  .usage("<path/to/icestorm>")
  .description("Path to icestorm installation directory")
  .initial("/usr/local");
auto& usb = StrArg<string>::create("--usb")
  .usage("[x-y]")
  .description("USB interface providing JTAG connectivity")
  .initial("[3-11]");
auto& port = StrArg<uint32_t>::create("--port")
  .usage("<int>")
  .description("Port to run icestorm server on")
  .initial(9900);

IcestormServer* qs = nullptr;

void handler(int sig) {
  (void) sig;
  qs->request_stop();
}

} // namespace

int main(int argc, char** argv) {
  // Parse command line:
  Simple::read(argc, argv);

  struct sigaction action;
  memset(&action, 0, sizeof(action));
  action.sa_handler = ::handler;
  sigaction(SIGINT, &action, nullptr);

  ::qs = new IcestormServer();
  ::qs->path(::path.value());
  ::qs->usb(::usb.value());
  ::qs->port(::port.value());

  if (!::qs->check()) {
    cout << "Unable to locate core icestorm components!" << endl;
  } else {
    ::qs->run();
    ::qs->wait_for_stop();
  }
  delete ::qs;

  cout << "Goodbye!" << endl;
  return 0;
}
