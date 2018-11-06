// RUN: %llvmgcc %s -g -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -symbolic-pointers=partial -exit-on-divergence -optimize=false %t1.bc 2>&1 | FileCheck %s

#include <stdio.h>
#include <klee/klee.h>

int main() {
  int x = klee_range(0, 2, "x");

  int a[2] = { 0, 1 };

  // The value a[x] will be concretized to 0. KLEE will then wrongly think
  // the path conditions (a[x] == 0, x != 0) are feasible, generating the
  // test x=1, which, when executed, will diverge from the first condition.

// CHECK: ResolutionDivergence.c:[[@LINE+1]]: divergence detected
  if (a[x] == 0)
    printf("a[x]=0\n");
  if (x == 0)
    printf("x=0\n");

  return 0;
}
