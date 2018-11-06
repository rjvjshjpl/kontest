// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t.bc 2>&1 | FileCheck %s

#include <assert.h>
#include <stdio.h>
#include <klee/kontest.h>

typedef struct {
  int wugga;
} Blob;
Blob myBlob = { 0 };

void *initializer() { return &myBlob; }

void actionA(KontestSchedulerState state) { ((Blob *) state.userData)->wugga += 1; }
void actionB(KontestSchedulerState state) { ((Blob *) state.userData)->wugga -= 1; }

KontestRule rules[] = {
  { "rule A", NULL, actionA },
  { "rule B", NULL, actionB }
  };

// CHECK: Monitor.c:[[@LINE+1]]: ASSERTION FAIL
void monitor(KontestSchedulerState state) { assert(((Blob *) state.userData)->wugga < 3); }

KontestMonitor monitors[] = { &monitor };

int main() {
  KontestSchedulerID s = kontest_create_scheduler(initializer, 2, rules, 1, monitors);

  kontest_run_scheduler(s, 3, NULL);

  return 0;
}
