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

#include "src/target/core/icebrk/icebrk_logic.h"

#include <cassert>
#include "src/target/core/icebrk/io.h"
#include "src/target/input.h"
#include "src/target/interface.h"
#include "src/target/state.h"
#include "src/verilog/ast/ast.h"
#include "src/verilog/analyze/evaluate.h"
#include "src/verilog/analyze/module_info.h"
#include "src/verilog/analyze/printf.h"
#include "src/verilog/print/text/text_printer.h"

using namespace std;

// Note that this is a plus, not a logical or! We can't guarantee that addr is aligned!
#define MANGLE(addr, idx) ((volatile uint8_t*)(((idx) << 2) + (size_t)addr))

namespace cascade {

IcebrkLogic::VarInfo::VarInfo(const Identifier* id, size_t idx, bool materialized) {
  id_ = id;
  idx_ = idx;
  materialized_ = materialized;
}

const Identifier* IcebrkLogic::VarInfo::id() const {
  return id_;
}

bool IcebrkLogic::VarInfo::materialized() const {
  return materialized_;
}

size_t IcebrkLogic::VarInfo::index() const {
  return idx_;
}

vector<size_t> IcebrkLogic::VarInfo::arity() const {
  return Evaluate().get_arity(id_);
}

size_t IcebrkLogic::VarInfo::elements() const {
  size_t res = 1;
  for (auto d : arity()) {
    res *= d;
  }
  return res;
}

size_t IcebrkLogic::VarInfo::bit_size() const {
  // Most of the time we'll be applying this function to variable declarations.
  // But sometimes we'll want to apply it to task arguments, which may have 
  // a different context-determined width
  const auto* r = Resolve().get_resolution(id_);
  assert(r != nullptr);
  return max(Evaluate().get_width(r), Evaluate().get_width(id_));
}

size_t IcebrkLogic::VarInfo::element_size() const {
  return (bit_size() + 31) / 32;
}

size_t IcebrkLogic::VarInfo::entry_size() const {
  return elements() * element_size();
}

IcebrkLogic::IcebrkLogic(Interface* interface, ModuleDeclaration* src, volatile uint8_t* addr) : Logic(interface), Visitor() { 
  src_ = src;
  addr_ = addr;
  next_index_ = 0;
  src_->accept(this);
}

IcebrkLogic::~IcebrkLogic() {
  delete src_;
}

IcebrkLogic& IcebrkLogic::set_input(const Identifier* id, VId vid) {
  // Insert inverse mapping into the variable map
  var_map_[id] = vid;
  // Insert a materialized version of id into the variable table
  if (var_table_.find(id) == var_table_.end()) {
    insert(id, true);
  }
  // Insert a pointer to this variable table entry into the input index
  if (vid >= inputs_.size()) {
    inputs_.resize(vid+1, nullptr);
  }
  inputs_[vid] = &var_table_.find(id)->second;

  return *this;
}

IcebrkLogic& IcebrkLogic::set_state(const Identifier* id, VId vid) {
  // Insert inverse mapping into the variable map
  var_map_[id] = vid;
  // Insert a materialized version of id into the variable table
  if (var_table_.find(id) == var_table_.end()) {
    insert(id, true);
  }

  return *this;
}

IcebrkLogic& IcebrkLogic::set_output(const Identifier* id, VId vid) {
  // Insert inverse mapping into the variable map
  var_map_[id] = vid;
  // Insert a materialized version of id (but only if it's stateful) into the
  // variable table
  if (var_table_.find(id) == var_table_.end()) {
    insert(id, ModuleInfo(src_).is_stateful(id));
  }
  // Insert a pointer to this variable into the output index
  outputs_.push_back(make_pair(vid, &var_table_.find(id)->second));

  return *this;
}

State* IcebrkLogic::get_state() {
  auto* s = new State();

  ModuleInfo info(src_);
  for (auto* v : info.stateful()) {
    const auto vid = var_map_.find(v);
    assert(vid != var_map_.end());
    const auto vinfo = var_table_.find(v);
    assert(vinfo != var_table_.end());

    read_array(vinfo->second);
    s->insert(vid->second, Evaluate().get_array_value(vinfo->second.id()));
  }

  return s;
}

void IcebrkLogic::set_state(const State* s) {
  ICE40_WRITE(MANGLE(addr_, live_idx()), 0);

  ModuleInfo info(src_);
  for (auto* v : info.stateful()) {
    const auto vid = var_map_.find(v);
    assert(vid != var_map_.end());
    const auto vinfo = var_table_.find(v);
    assert(vinfo != var_table_.end());

    const auto itr = s->find(vid->second);
    if (itr != s->end()) {
      write_array(vinfo->second, itr->second);
    }
  }

  ICE40_WRITE(MANGLE(addr_, sys_task_idx()), 0);
  ICE40_WRITE(MANGLE(addr_, live_idx()), 1);
}

Input* IcebrkLogic::get_input() {
  auto* i = new Input();

  ModuleInfo info(src_);
  for (auto* in : info.inputs()) {
    const auto vid = var_map_.find(in);
    assert(vid != var_map_.end());
    const auto vinfo = var_table_.find(in);
    assert(vinfo != var_table_.end());

    read_scalar(vinfo->second);
    i->insert(vid->second, Evaluate().get_value(vinfo->second.id()));
  }

  return i;
}

void IcebrkLogic::set_input(const Input* i) {
  ICE40_WRITE(MANGLE(addr_, live_idx()), 0);

  ModuleInfo info(src_);
  for (auto* in : info.inputs()) {
    const auto vid = var_map_.find(in);
    assert(vid != var_map_.end());
    const auto vinfo = var_table_.find(in);
    assert(vinfo != var_table_.end());

    const auto itr = i->find(vid->second);
    if (itr != i->end()) {
      write_scalar(vinfo->second, itr->second);
    }
  }

  ICE40_WRITE(MANGLE(addr_, sys_task_idx()), 0);
  ICE40_WRITE(MANGLE(addr_, live_idx()), 1);
}

void IcebrkLogic::read(VId id, const Bits* b) {
  assert(id < inputs_.size());
  assert(inputs_[id] != nullptr);
  write_scalar(*inputs_[id], *b);
}

void IcebrkLogic::evaluate() {
  // Read outputs and handle tasks
  handle_outputs();
  handle_tasks();
}

bool IcebrkLogic::there_are_updates() const {
  // Read there_are_updates flag
  return ICE40_READ(MANGLE(addr_, there_are_updates_idx()));
}

void IcebrkLogic::update() {
  // Throw the update trigger
  ICE40_WRITE(MANGLE(addr_, update_idx()), 1);
  // Read outputs and handle tasks
  handle_outputs();
  handle_tasks();
}

bool IcebrkLogic::there_were_tasks() const {
  return there_were_tasks_;
}

size_t IcebrkLogic::open_loop(VId clk, bool val, size_t itr) {
  // The fpga already knows the value of clk. We can ignore it.
  (void) clk;
  (void) val;

  // Go into open loop mode and handle tasks when we return. No need
  // to handle outputs. This methods assumes that we don't have any.
  ICE40_WRITE(MANGLE(addr_, open_loop_idx()), itr);
  handle_tasks();

  // Return the number of iterations that we ran for
  return ICE40_READ(MANGLE(addr_, open_loop_idx()));
}

IcebrkLogic::map_iterator IcebrkLogic::map_begin() const {
  return var_map_.begin();
}

IcebrkLogic::map_iterator IcebrkLogic::map_end() const {
  return var_map_.end();
}

IcebrkLogic::table_iterator IcebrkLogic::table_find(const Identifier* id) const {
  return var_table_.find(id);
}

IcebrkLogic::table_iterator IcebrkLogic::table_begin() const {
  return var_table_.begin();
}

IcebrkLogic::table_iterator IcebrkLogic::table_end() const {
  return var_table_.end();
}

size_t IcebrkLogic::table_size() const {
  return next_index_;
}

size_t IcebrkLogic::live_idx() const {
  return next_index_;
}

size_t IcebrkLogic::there_are_updates_idx() const {
  return next_index_ + 1;
}

size_t IcebrkLogic::update_idx() const {
  return next_index_ + 2;
}
  
size_t IcebrkLogic::sys_task_idx() const {
  return next_index_ + 3;
}

size_t IcebrkLogic::open_loop_idx() const {
  return next_index_ + 4;
}
  
bool IcebrkLogic::open_loop_enabled() const {
  ModuleInfo info(src_);
  if (info.inputs().size() != 1) {
    return false;
  }
  if (Evaluate().get_width(*info.inputs().begin()) != 1) {
    return false;
  }
  if (!info.outputs().empty()) {
    return false;
  }
  return true;
}

const Identifier* IcebrkLogic::open_loop_clock() const {
  return open_loop_enabled() ? *ModuleInfo(src_).inputs().begin() : nullptr;
}

void IcebrkLogic::visit(const DisplayStatement* ds) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(ds);
  Inserter i(this);
  ds->accept_args(&i);
}

void IcebrkLogic::visit(const ErrorStatement* es) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(es);
  Inserter i(this);
  es->accept_args(&i);
}

void IcebrkLogic::visit(const FatalStatement* fs) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(fs);
  Inserter i(this);
  fs->accept_arg(&i);
  fs->accept_args(&i);
}

void IcebrkLogic::visit(const FinishStatement* fs) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(fs);
  Inserter i(this);
  fs->accept_arg(&i);
}

void IcebrkLogic::visit(const InfoStatement* is) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(is);
  Inserter i(this);
  is->accept_args(&i);
}

void IcebrkLogic::visit(const RetargetStatement* rs) {
  // Record this task, but no need to descend on its argument (which is a constant)
  tasks_.push_back(rs);
}

void IcebrkLogic::visit(const WarningStatement* ws) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(ws);
  Inserter i(this);
  ws->accept_args(&i);
}

void IcebrkLogic::visit(const WriteStatement* ws) {
  // Record this task and insert materialized instances of the
  // variables in its arguments into the variable table.
  tasks_.push_back(ws);
  Inserter i(this);
  ws->accept_args(&i);
}

void IcebrkLogic::insert(const Identifier* id, bool materialized) {
  assert(var_table_.find(id) == var_table_.end());
  VarInfo vi(id, next_index_, materialized);
  var_table_.insert(make_pair(id, vi));

  next_index_ += vi.entry_size();
}

void IcebrkLogic::read_scalar(const VarInfo& vi) {
  // Make sure this is actually a scalar
  assert(vi.arity().empty());
  // But use the generic array implementation
  read_array(vi);
}

void IcebrkLogic::read_array(const VarInfo& vi) {
  // Move bits from fpga into local storage one word at a time.  Values are
  // stored in ascending order in memory, just as they are in the variable
  // table
  auto idx = vi.index();
  for (size_t i = 0, ie = vi.elements(); i < ie; ++i) {
    for (size_t j = 0, je = vi.element_size(); j < je; ++j) {
      const volatile auto word = ICE40_READ(MANGLE(addr_, idx));
      Evaluate().assign_word<uint32_t>(vi.id(), i, j, word);
      ++idx;
    } 
  }
}

void IcebrkLogic::write_scalar(const VarInfo& vi, const Bits& b) {
  // Make sure this is actually a scalar
  assert(vi.arity().empty());
  // Move bits to fpga one word at a time, skipping the local storage.
  // If an when we need a value we'll read it out of the fpga first.
  auto idx = vi.index();
  for (size_t i = 0, ie = vi.entry_size(); i < ie; ++i) {
    const volatile auto word = b.read_word<uint32_t>(i);
    ICE40_WRITE(MANGLE(addr_, idx), word);
    ++idx;
  }
}

void IcebrkLogic::write_array(const VarInfo& vi, const Vector<Bits>& bs) {
  // Move bits to fpga one word at a time, skipping the local storage.
  // If an when we need a value we'll read it out of the fpga first.
  auto idx = vi.index();
  for (size_t i = 0, ie = vi.elements(); i < ie; ++i) {
    for (size_t j = 0, je = vi.element_size(); j < je; ++j) {
      const volatile auto word = bs[i].read_word<uint32_t>(j);
      ICE40_WRITE(MANGLE(addr_, idx), word);
    }
  }
}

void IcebrkLogic::handle_outputs() {
  for (const auto& o : outputs_) {
    read_scalar(*o.second);
    interface()->write(o.first, &Evaluate().get_value(o.second->id()));
  }
}

void IcebrkLogic::handle_tasks() {
  // By default, we'll assume there were no tasks
  there_were_tasks_ = false;
  volatile auto task_queue = ICE40_READ(MANGLE(addr_, sys_task_idx()));
  if (task_queue == 0) {
    return;
  }

  // There were tasks after all; we need to empty the queue
  there_were_tasks_ = true;
  for (size_t i = 0; task_queue != 0; task_queue >>= 1, ++i) {
    if ((task_queue & 0x1) == 0) {
      continue;
    }
    if (tasks_[i]->is(Node::Tag::display_statement)) {
      const auto* ds = static_cast<const DisplayStatement*>(tasks_[i]);
      Sync sync(this);
      ds->accept_args(&sync);
      interface()->display(Printf().format(ds->begin_args(), ds->end_args()));
    } else if (tasks_[i]->is(Node::Tag::error_statement)) {
      const auto* es = static_cast<const ErrorStatement*>(tasks_[i]);
      Sync sync(this);
      es->accept_args(&sync);
      interface()->error(Printf().format(es->begin_args(), es->end_args()));
    } else if (tasks_[i]->is(Node::Tag::fatal_statement)) {
      const auto* fs = static_cast<const FatalStatement*>(tasks_[i]);
      Sync sync(this);
      fs->accept_arg(&sync);
      fs->accept_args(&sync);
      interface()->fatal(Evaluate().get_value(fs->get_arg()).to_int(), Printf().format(fs->begin_args(), fs->end_args()));
    } else if (tasks_[i]->is(Node::Tag::finish_statement)) {
      const auto* fs = static_cast<const FinishStatement*>(tasks_[i]);
      interface()->finish(Evaluate().get_value(fs->get_arg()).to_int());
    } else if (tasks_[i]->is(Node::Tag::info_statement)) {
      const auto* is = static_cast<const InfoStatement*>(tasks_[i]);
      Sync sync(this);
      is->accept_args(&sync);
      interface()->info(Printf().format(is->begin_args(), is->end_args()));
    } else if (tasks_[i]->is(Node::Tag::restart_statement)) {
      const auto* rs = static_cast<const RestartStatement*>(tasks_[i]);
      interface()->restart(rs->get_arg()->get_readable_val());
    } else if (tasks_[i]->is(Node::Tag::retarget_statement)) {
      const auto* rs = static_cast<const RetargetStatement*>(tasks_[i]);
      interface()->retarget(rs->get_arg()->get_readable_val());
    } else if (tasks_[i]->is(Node::Tag::save_statement)) {
      const auto* ss = static_cast<const SaveStatement*>(tasks_[i]);
      interface()->save(ss->get_arg()->get_readable_val());
    } else if (tasks_[i]->is(Node::Tag::warning_statement)) {
      const auto* ws = static_cast<const WarningStatement*>(tasks_[i]);
      Sync sync(this);
      ws->accept_args(&sync);
      interface()->warning(Printf().format(ws->begin_args(), ws->end_args()));
    } else if (tasks_[i]->is(Node::Tag::write_statement)) {
      const auto* ws = static_cast<const WriteStatement*>(tasks_[i]);
      Sync sync(this);
      ws->accept_args(&sync);
      interface()->write(Printf().format(ws->begin_args(), ws->end_args()));
    } else {
      assert(false);
    }
  }

  // Reset the task mask
  ICE40_WRITE(MANGLE(addr_, sys_task_idx()), 0);
}

IcebrkLogic::Inserter::Inserter(IcebrkLogic* de) : Visitor() {
  de_ = de;
}

void IcebrkLogic::Inserter::visit(const Identifier* id) {
  de_->insert(id, true);
}

IcebrkLogic::Sync::Sync(IcebrkLogic* de) : Visitor() {
  de_ = de;
}

void IcebrkLogic::Sync::visit(const Identifier* id) {
  const auto vinfo = de_->var_table_.find(id);
  assert(vinfo != de_->var_table_.end());
  de_->read_scalar(vinfo->second);
}

} // namespace cascade
