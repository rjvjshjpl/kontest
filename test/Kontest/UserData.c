// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log
// RUN: grep AAC %t.log
// RUN: not grep B*A?B*C %t.log

#include <stdio.h>
#include <klee/kontest.h>

typedef struct {
  int wugga;
} Blob;
Blob myBlob = { 0 };

void *initializer() { return &myBlob; }

int preC(KontestSchedulerState state) { return (((Blob *) state.userData)->wugga >= 2); }

void actionA(KontestSchedulerState state) { printf("A"); ((Blob *) state.userData)->wugga += 1; }
void actionB(KontestSchedulerState state) { printf("B"); ((Blob *) state.userData)->wugga -= 1; }
void actionC(KontestSchedulerState state) { printf("C"); }

KontestRule rules[] = {
  { "rule A", NULL, actionA },
  { "rule B", NULL, actionB },
  { "rule C", preC, actionC }
  };

int main() {
  KontestSchedulerID s = kontest_create_scheduler(initializer, 3, rules, 0, NULL);

  kontest_run_scheduler(s, 3, NULL);

  printf("\n");

  return 0;
}
