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

#ifndef CASCADE_SRC_TARGET_CORE_ICE40_ICE40_LED_H
#define CASCADE_SRC_TARGET_CORE_ICE40_ICE40_LED_H

#include "src/base/bits/bits.h"
#include "src/target/core.h"
#include "src/target/core/icebrk/io.h"
#include "src/target/input.h"
#include "src/target/state.h"

namespace cascade {

// This file implements an LED engine for the Terasic ICE40-Nano board.  It
// supports LED0-LED7 from the FPGA side, and it requires a PIO core be
// available at 0x3000, which is where it is mapped tn the ICE40_NANO GHRD.
 
class IcebrkLed : public Led {
  public:
    IcebrkLed(Interface* interface, VId in, volatile uint8_t* led_addr); 
    ~IcebrkLed() override = default;

    State* get_state() override;
    void set_state(const State* s) override;
    Input* get_input() override;
    void set_input(const Input* i) override;

    void read(VId id, const Bits* b) override;
    void evaluate() override;
    bool there_are_updates() const override;
    void update() override;

  private:
    VId in_;
    volatile uint8_t* led_addr_;
};

inline IcebrkLed::IcebrkLed(Interface* interface, VId in, volatile uint8_t* led_addr) : Led(interface) {
  in_ = in;
  led_addr_ = led_addr;
}

inline State* IcebrkLed::get_state() {
  return new State();
} 

inline void IcebrkLed::set_state(const State* s) {
  // Stateless; does nothing
  (void) s;
}

inline Input* IcebrkLed::get_input() {
  auto* i = new Input();
  i->insert(in_, Bits(32, ICE40_READ(led_addr_)));
  return i;
} 

inline void IcebrkLed::set_input(const Input* i) {
  const auto itr = i->find(in_);
  if (itr != i->end()) {
    ICE40_WRITE(led_addr_, itr->second.to_int());
  }
}

inline void IcebrkLed::read(VId id, const Bits* b) {
  (void) id;
  ICE40_WRITE(led_addr_, b->to_int());
}

inline void IcebrkLed::evaluate() {
  // Does nothing.
}

inline bool IcebrkLed::there_are_updates() const {
  return false;
}

inline void IcebrkLed::update() { 
  // Does nothing.
}

} // namespace cascade

#endif