//===-- Timer.h -------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_TIMER_H
#define KLEE_TIMER_H

#include "llvm/Support/TimeValue.h"

#include <stdint.h>

namespace klee {
  class WallTimer {
    llvm::sys::TimeValue::SecondsType startSeconds;
    int_fast32_t startNanos;
    
  public:
    WallTimer();

    /// check - Return the delta since the timer was created, in microseconds.
    uint64_t check();
  };
}

#endif

