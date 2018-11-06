// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --symbolic-pointers=fork %t.bc > %t.log
// RUN: grep AC %t.log
// RUN: grep AD %t.log
// RUN: not grep ^B %t.log

#include <stdio.h>

int main() {
  klee_set_forking(0);

  int x = klee_range(0, 2, "x");
  int y = 0;
  int a[2] = { 0, 1 };

  if (a[x] == 0) {
    printf("A");
  } else {
    printf("B");
  }

  klee_set_forking(1);

  y = klee_range(0, 2, "y");
  if (a[y] == 0) {
    printf("C");
  } else {
    printf("D");
  }

  printf("\n");

  return 0;
}
