//===-- Timer.cpp ---------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Config/Version.h"
#include "klee/Internal/Support/Timer.h"

#include "klee/Internal/System/Time.h"

using namespace klee;
using namespace llvm;

WallTimer::WallTimer() {
  sys::TimeValue time = util::getWallTimeVal();
  startSeconds = time.seconds();
  startNanos = time.nanoseconds();    // time is normalized, so this will fit in 31 bits
}

uint64_t WallTimer::check() {
  sys::TimeValue time = util::getWallTimeVal();
  int_fast32_t nanos = time.nanoseconds();    // as above
  return (1000000 * (time.seconds() - startSeconds)) + ((nanos - startNanos) / 1000);
}
