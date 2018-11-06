// RUN: %llvmgcc %s -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --optimize=false %t1.bc
// RUN: not grep "ERROR" %t.klee-out/messages.txt
// RUN: grep "KLEE: done: completed paths = 4" %t.klee-out/info

#include <stdio.h>

int *make_int(int i) {
  int *x = malloc(sizeof(*x));
  *x = i;
  return x;
}

int main() {
  int *buf[4];
  int i,s,t;

  for (i=0; i<4; i++)
    buf[i] = make_int((i+1)*2);

  s = klee_range(0, 4, "s");

  *buf[s] = 5;

  if ((*buf[0] + *buf[1] + *buf[2] + *buf[3]) == 17)
    if (s!=3)
      abort();

  return 0;
}
