// RUN: %llvmgcc %s -g -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --emit-all-errors --optimize=false %t1.bc 2>&1 | FileCheck %s
// RUN: ls %t.klee-out/ | grep .ktest | wc -l | grep 4
// RUN: ls %t.klee-out/ | grep .err | wc -l | grep 3

#include <stdlib.h>
#include <stdio.h>

int *make_int(int i) {
  int *x = malloc(sizeof(*x));
  *x = i;
  return x;
}

int main() {
  int *buf[4];
  int i,s;

  for (i=0; i<3; i++)
    buf[i] = make_int(i);
  buf[3] = 0;

  s = klee_range(0,4,"s");

  free(buf[s]);

  // CHECK: MultipleFreeResolution.c:[[@LINE+4]]: memory error: out of bound pointer
  // CHECK: MultipleFreeResolution.c:[[@LINE+3]]: memory error: out of bound pointer
  // CHECK: MultipleFreeResolution.c:[[@LINE+2]]: memory error: out of bound pointer
  for (i=0; i<3; i++) {
    printf("*buf[%d] = %d\n", i, *buf[i]);
  }

  return 0;
}
// CHECK: KLEE: done: generated tests = 4
