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

#ifndef CASCADE_SRC_TARGET_CORE_ICE40_ICE40_GPIO_H
#define CASCADE_SRC_TARGET_CORE_ICE40_ICE40_GPIO_H

#include "src/base/bits/bits.h"
#include "src/target/core.h"
#include "src/target/core/ice40/io.h"
#include "src/target/input.h"
#include "src/target/state.h"

namespace cascade {

// This file implements a GPIO engine for the Terasic ICE40-Nano board.  It
// supports GPI0-GPIO31 from the FPGA side, and it requires a PIO core be
// available at 0x5000, which is where it is mapped tn the ICE40_NANO GHRD.
 
class Ice40Gpio : public Gpio {
  public:
    Ice40Gpio(Interface* interface, VId in, volatile uint8_t* gpio_addr); 
    ~Ice40Gpio() override = default;

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
    volatile uint8_t* gpio_addr_;
};

inline Ice40Gpio::Ice40Gpio(Interface* interface, VId in, volatile uint8_t* gpio_addr) : Gpio(interface) {
  in_ = in;
  gpio_addr_ = gpio_addr;
}

inline State* Ice40Gpio::get_state() {
  return new State();
} 

inline void Ice40Gpio::set_state(const State* s) {
  // Stateless; does nothing
  (void) s;
}

inline Input* Ice40Gpio::get_input() {
  auto* i = new Input();
  i->insert(in_, Bits(32, ICE40_READ(gpio_addr_)));
  return i;
} 

inline void Ice40Gpio::set_input(const Input* i) {
  const auto itr = i->find(in_);
  if (itr != i->end()) {
    ICE40_WRITE(gpio_addr_, itr->second.to_int());
  }
}

inline void Ice40Gpio::read(VId id, const Bits* b) {
  (void) id;
  ICE40_WRITE(gpio_addr_, b->to_int());
}

inline void Ice40Gpio::evaluate() {
  // Does nothing.
}

inline bool Ice40Gpio::there_are_updates() const {
  return false;
}

inline void Ice40Gpio::update() { 
  // Does nothing.
}

} // namespace cascade

#endif

