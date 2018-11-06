// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: grep A %t.log
// RUN: grep B %t.log
// RUN: grep C %t.log
// RUN: not grep X %t.log

#include <stdio.h>
#include <klee/klee.h>

int main() {
  int x;
  klee_pick(x, 12, 17, -42);
  if (x == 12)
    printf("A\n");
  else if (x == 17)
    printf("B\n");
  else if (x == -42)
    printf("C\n");
  else
    printf("X\n");

  return 0;
}
