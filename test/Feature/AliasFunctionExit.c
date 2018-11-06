// RUN: %llvmgcc %s -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -optimize=false %t1.bc > %t1.log
// RUN: grep -c START %t1.log | grep 4
// RUN: grep -c END %t1.log | grep 4

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void start(int x) {
  printf("START\n");
  if (x == 53)
    exit(1);
}

void __attribute__ ((noinline)) end(int status) {
  klee_alias_function("exit", "exit");
  printf("END: status = %d\n", status);
  exit(status);
}


int main() {
  int x = 0;
  klee_make_symbolic(&x, sizeof(x), "x");

  klee_alias_function("exit", "end");
  start(x);
  end(0);
}
