// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --exit-on-divergence %t.bc > %t.log
// RUN: grep A %t.log
// RUN: grep B %t.log
// RUN: grep C %t.log
// RUN: grep D %t.log
// RUN: not grep X %t.log
// RUN: grep "abandoned paths = 0" %t.klee-out/info

#include <stdio.h>
#include <klee/klee.h>

int main() {
  int x = klee_range(-1, 3, "x");
  if (x == 2)
    printf("A\n");
  else if (x == 0)
    printf("B\n");
  else if (x == -1)
    printf("C\n");
  else if (x == 1)
    printf("D\n");
  else
    printf("X\n");

  return 0;
}
