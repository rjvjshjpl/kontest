// RUN: %llvmgcc %s -emit-llvm -g -O0 -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: test -f %t.klee-out/test000002.ktest
// RUN: not test -f %t.klee-out/test000003.ktest

// Now try to replay with libkleeRuntest
// RUN: %cc %s %libkleeruntest -Wl,-rpath %libkleeruntestdir -o %t_runner
// RUN: env KTEST_FILE=%t.klee-out/test000001.ktest %t_runner | FileCheck -check-prefix=TESTONE %s
// RUN: env KTEST_FILE=%t.klee-out/test000002.ktest %t_runner | FileCheck -check-prefix=TESTTWO %s

#include "klee/klee.h"
#include <stdio.h>

int main() {
  char filter[] = { 1, 0, 1 };
  int x = klee_filtered_choice("x", 3, filter);
  switch (x) {
    // TESTONE: A
    case 0: printf("A\n"); break;
    case 1: klee_assert(0);
    // TESTTWO: B
    case 2: printf("B\n"); break;
    default: klee_assert(0);
  }
  return 0;
}

