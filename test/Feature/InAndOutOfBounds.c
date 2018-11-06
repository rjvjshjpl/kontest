// RUN: %llvmgcc %s -g -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc 2>&1 | FileCheck %s
// RUN: test -f %t.klee-out/test000001.ptr.err -o -f %t.klee-out/test000002.ptr.err
// RUN: not test -f %t.klee-out/test000001.ptr.err -a -f %t.klee-out/test000002.ptr.err
// RUN: not test -f %t.klee-out/test000003.ktest

int main() {
  int *x = malloc(sizeof(int));
  // CHECK: InAndOutOfBounds.c:[[@LINE+1]]: memory error: out of bound pointer
  x[klee_range(0,2,"index")] = 1;
  free(x);
  return 0;
}
