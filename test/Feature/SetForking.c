// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t.bc > %t.log
// RUN: sort %t.log | uniq -c > %t.uniq.log
// RUN: grep "3 A" %t.uniq.log
// RUN: grep "2 B" %t.uniq.log
// RUN: grep "1 C" %t.uniq.log

#include <stdio.h>

int main() {
  klee_set_forking(0);
  
  int x = 0, y = 0;
  klee_make_symbolic(&x, sizeof(x), "x");
  if (x >= 0) {
    printf("A\n");
  } else {
    printf("A\n");
    y++;    // prevent this branch from getting merged with previous by optimizer
  }

  klee_set_forking(1);

  klee_make_symbolic(&y, sizeof(y), "y");
  if (y >= 0) {
    printf("B\n");
  } else {
    printf("C\n");
  }

  return 0;
}
