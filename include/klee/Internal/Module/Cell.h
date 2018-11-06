//===-- Cell.h --------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_CELL_H
#define KLEE_CELL_H

#include <klee/Expr.h>

namespace klee {
  class MemoryObject;

  struct Cell {
    ref<Expr> value;

    // Concrete value; this should always have the same width as the
    // member `value` above, and should be a possible value of it
    // under the current constraints.
    llvm::APInt concreteValue;
  };
}

#endif
