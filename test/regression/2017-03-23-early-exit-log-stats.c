// RUN: %llvmgcc %s -emit-llvm -g -O0 -c -o %t.bc
// RUN: rm -rf %t.klee-out
// Delay writing instructions so that we ensure on exit that flush happens
// RUN: not %klee --output-dir=%t.klee-out -exit-on-error -output-stats -stats-write-interval=0 -stats-write-after-instructions=999999 %t.bc 2> %t.log
// RUN: FileCheck -check-prefix=CHECK-KLEE -input-file=%t.log %s
// RUN: FileCheck -check-prefix=CHECK-STATS -input-file=%t.klee-out/run.stats %s
#include "klee/klee.h"
#include <stdlib.h>
int main(){
  int a;
  klee_make_symbolic (&a, sizeof(int), "a");
  if (a) {
    // CHECK-KLEE: EXITING ON ERROR
    // CHECK-KLEE-NEXT: Error: abort failure
    abort();
  }
  return 0;
}
// First check we find a line with the expected format
// CHECK-STATS:{{^\('Instructions'}}
// Now check that we eventually get a line where a non zero amount of instructions were executed
// CHECK-STATS:{{^\([ ]*([1-9]|([1-9]+)[0-9])}}
