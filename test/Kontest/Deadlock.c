// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --emit-all-errors %t.bc 2>&1 | FileCheck %s

#include <stdio.h>
#include <klee/kontest.h>

int pre(KontestSchedulerState state) { return (state.step == 0); }

void actionA(KontestSchedulerState state) { printf("firing rule A\n"); }
void actionB(KontestSchedulerState state) { printf("firing rule B\n"); }

KontestRule rules[] = {
  { "rule A", pre, actionA },
  { "rule B", pre, actionB }
  };

int main() {
  KontestSchedulerID s = kontest_create_scheduler(NULL, 2, rules, 0, NULL);

// CHECK-DAG: firing rule A
// CHECK-DAG: deadlock in scheduler
// CHECK-DAG: firing rule B
// CHECK-DAG: deadlock in scheduler
// CHECK: completed paths = 2
  kontest_run_scheduler(s, 2, NULL);

  return 0;
}
