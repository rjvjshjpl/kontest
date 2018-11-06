// RUN: %llvmgcc %s -g -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -max-test-instructions=100 %t1.bc 2>&1 > %t.log
// RUN: grep "good" %t.log
// RUN: not grep "bad" %t.log

#include <assert.h>
#include <klee/klee.h>

int main() {
  unsigned x = 1;
  klee_make_symbolic(&x, sizeof(x), "x");
  
  if (x) {
    for (int i = 0; i < 1000; i++)
      x = 3 * x;

    printf("bad\n");
  }

  printf("good\n");

  return 0;
}
