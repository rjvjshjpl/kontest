// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: not grep ^B %t.log
// RUN: grep AA %t.log
// RUN: grep AB %t.log
// RUN: grep AC %t.log
// RUN: grep CA %t.log
// RUN: grep CB %t.log
// RUN: grep CC %t.log

#include <stdio.h>
#include <klee/kontest.h>

int preB(KontestSchedulerState state) { return (state.step != 0); }

void actionA(KontestSchedulerState state) { printf("A"); }
void actionB(KontestSchedulerState state) { printf("B"); }
void actionC(KontestSchedulerState state) { printf("C"); }

KontestRule rules[] = {
  { "rule A", NULL, actionA },
  { "rule B", preB, actionB },
  { "rule C", NULL, actionC }
  };

int main() {
  KontestSchedulerID s = kontest_create_scheduler(NULL, 3, rules, 0, NULL);

  kontest_run_scheduler(s, 2, NULL);

  printf("\n");

  return 0;
}
