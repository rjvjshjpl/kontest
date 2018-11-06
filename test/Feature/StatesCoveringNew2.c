// Check that we properly detect tests not covering new instructions.
//
// RUN: %llvmgcc -I../../../include %s -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --optimize=false %t1.bc 2>&1 | FileCheck %s
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: test -f %t.klee-out/test000002.ktest
// RUN: not test -f %t.klee-out/test000003.ktest

// CHECK: completed paths = 3
// CHECK: generated tests = 2

#include <klee/klee.h>

int main() {
  int x = klee_range(0, 3, "x");

  while (x)
    --x;

  return 0;
}
