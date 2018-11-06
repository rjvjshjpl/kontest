// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: grep A %t.log
// RUN: grep B %t.log

#include <stdio.h>
#include <klee/kontest.h>

void actionA(KontestSchedulerState state) {
  printf("A\n");
}

void actionB(KontestSchedulerState state) {
  printf("B\n");
}

KontestRule rules[] = {
  { "rule A", NULL, actionA },
  { "rule B", NULL, actionB }
  };

int main() {
  KontestSchedulerID s = kontest_create_scheduler(NULL, 2, rules, 0, NULL);

  kontest_run_scheduler(s, 1, NULL);

  return 0;
}
