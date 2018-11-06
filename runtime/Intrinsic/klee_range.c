//===-- klee_range.c ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <assert.h>
#include <klee/klee.h>

int klee_range(int start, int end, const char* name) {
  return klee_range_hinted(start, start, end, name);
}

int klee_range_hinted(int start, int hint, int end, const char* name) {
  int x;

  if (start > hint || hint >= end)
    klee_report_error(__FILE__, __LINE__, "invalid range", "user");

  if (start+1==end) {
    return start;
  } else {
    x = hint;
    klee_make_symbolic_range(&x, sizeof x, name, start, end);
    return x;
  }
}
