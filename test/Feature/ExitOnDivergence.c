// RUN: %llvmgcc %s -g -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -no-nonlinear-arithmetic -exit-on-divergence %t1.bc 2>&1 | FileCheck %s

#include <stdio.h>
#include <klee/klee.h>

int main() {
  int x = 0, y = 1;
  klee_make_symbolic(&x, sizeof(x), "x");
  klee_make_symbolic(&y, sizeof(y), "y");

  // The value z will be concretized to 0. KLEE will then wrongly think
  // the path conditions (z == 0, y == 1, x != 0) are feasible, generating
  // a test like (x,y)=(1,1), which, when executed, will diverge from the
  // first condition.
  int z = x * y;

// CHECK: ExitOnDivergence.c:[[@LINE+1]]: divergence detected
  if (z == 0)
    printf("z=0\n");
  if (y == 1)
    printf("y=1\n");
  if (x == 0)
    printf("x=0\n");

  return 0;
}
