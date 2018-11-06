// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: head -1 %t.log | grep BAC

#include <stdio.h>
#include <klee/kontest.h>

void actionA(KontestSchedulerState state) { printf("A"); }
void actionB(KontestSchedulerState state) { printf("B"); }
void actionC(KontestSchedulerState state) { printf("C"); }

KontestRule rules[] = {
  { "rule A", NULL, actionA },
  { "rule B", NULL, actionB },
  { "rule C", NULL, actionC }
  };

int main() {
  KontestSchedulerID s = kontest_create_scheduler(NULL, 3, rules, 0, NULL);

  unsigned hint[] = { 1, 0, 2 };
  kontest_run_scheduler(s, 3, hint);

  printf("\n");

  return 0;
}
