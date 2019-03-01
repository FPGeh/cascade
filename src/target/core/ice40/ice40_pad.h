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

#ifndef CASCADE_SRC_TARGET_CORE_ICE40_ICE40_PAD_H
#define CASCADE_SRC_TARGET_CORE_ICE40_ICE40_PAD_H

#include <cassert>
#include "src/base/bits/bits.h"
#include "src/target/core.h"
#include "src/target/core/ice40/io.h"
#include "src/target/input.h"
#include "src/target/interface.h"

namespace cascade {

class Ice40Pad : public Pad {
  public:
    Ice40Pad(Interface* interface, VId out, size_t size, volatile uint8_t* pad_addr); 
    ~Ice40Pad() override = default;

    State* get_state() override;
    void set_state(const State* s) override;
    Input* get_input() override;
    void set_input(const Input* i) override;

    bool overrides_done_step() const override;
    void done_step() override;

    void read(VId id, const Bits* b) override;
    void evaluate() override;
    bool there_are_updates() const override;
    void update() override;

  private:
    VId out_;
    size_t size_;
    volatile uint8_t* pad_addr_;
    bool there_are_updates_;
};

inline Ice40Pad::Ice40Pad(Interface* interface, VId out, size_t size, volatile uint8_t* pad_addr) : Pad(interface) {
  out_ = out;
  size_ = size;
  pad_addr_ = pad_addr;
  there_are_updates_ = true;
}

inline State* Ice40Pad::get_state() {
  return new State();
} 

inline void Ice40Pad::set_state(const State* s) {
  // Stateless; does nothing
  (void) s;
}

inline Input* Ice40Pad::get_input() {
  // Outputs only; does nothing
  return new Input();
} 

inline void Ice40Pad::set_input(const Input* i) {
  // Outputs only; does nothing
  (void) i;
}

inline bool Ice40Pad::overrides_done_step() const {
  return true;
}

inline void Ice40Pad::done_step() {
  there_are_updates_ = true;
}

inline void Ice40Pad::read(VId id, const Bits* b) {
  // Invoking this method doesn't make sense
  (void) id;
  (void) b;
  assert(false);
}

inline void Ice40Pad::evaluate() {
  Bits bits(size_, ICE40_READ(pad_addr_));
  interface()->write(out_, &bits);
}

inline bool Ice40Pad::there_are_updates() const {
  return there_are_updates_;
}

inline void Ice40Pad::update() { 
  there_are_updates_ = false;
  evaluate();
}

} // namespace cascade

#endif
