// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: grep A %t.log
// RUN: grep B %t.log

#include <stdio.h>
#include <klee/klee.h>

int main() {
  char filter[] = { 0, 1, 0, 1 };
  int x = klee_filtered_choice("x", 4, filter);
  switch (x) {
    case 0: klee_assert(0);
    case 1: printf("A\n"); break;
    case 2: klee_assert(0);
    case 3: printf("B\n"); break;
    default: klee_assert(0);
  }
  return 0;
}
