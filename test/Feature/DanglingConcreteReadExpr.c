// RUN: %llvmgcc %s -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --optimize=false --output-dir=%t.klee-out %t1.bc
// RUN: grep "total queries = 1" %t.klee-out/info

#include <assert.h>

int main() {
  unsigned char x, y;

  klee_make_symbolic(&x, sizeof x);
  
  y = x;

  // should be exactly one query (check feasibility of x ==/!= 10)
  // eventually should be 0 when we have fast solver
  if (x==10) {
    assert(y==10);
  }

  klee_silent_exit(0);
  return 0;
}
