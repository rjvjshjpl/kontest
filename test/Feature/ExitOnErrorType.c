// RUN: %llvmgcc %s -g -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -exit-on-error-type Assert %t1.bc 2>&1 > %t.log
// RUN: not grep "bad" %t.log

#include <assert.h>
#include <klee/klee.h>

int main() {
  int x = 0;
  klee_make_symbolic(&x, sizeof(x), "x");
  assert(x);

  printf("bad\n");

  return 0;
}
