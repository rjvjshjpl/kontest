// RUN: %llvmgcc %s -emit-llvm -O0 -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc

#include <assert.h>
#include <stdio.h>

int main() {  
  unsigned char x = klee_range(0, 2, "x");

  char delete[2] = {0,1};

  char tmp = delete[ x ];
  char tmp2 = delete[0];
  delete[ x ] = tmp2;

  if (x==1) {
    assert(delete[1] == 0);
    return 0;
  }

  return 0;
}
