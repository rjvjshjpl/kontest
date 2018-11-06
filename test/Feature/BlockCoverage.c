// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --optimize=false --output-istats=false %t.bc > %t.log 2> %t.err.log 
// RUN: cat %t.log | FileCheck %s
// RUN: grep "generated tests = 3" %t.err.log

// Make sure the block coverage heuristic still works even when
// IStats monitoring is disabled

// There are four possible tests:
//   (1) i=0: prints AD;
//   (2) i=1,j=0: prints ABD;
//   (3) i=1,j=1: prints ABCD;
//   (4) i=2: prints D.
// Test (1) is the concrete test and is expanded first, generating
// child test (4) and then (2). Test (4) covers no new blocks, but
// test (2) does. So test (2) should be expanded first, even though
// test (4) was generated first. We check this order:

// CHECK: 1AD
// CHECK: 1ABD
// CHECK: 1D

#include <stdio.h>

int main() {
  int i = klee_range(0, 3, "i");

  printf("%d", klee_is_symbolic(i));	// so we can identify the symbolic runs

  if (i <= 1)
    printf("A");
  if (i == 1) {
    printf("B");
    int j = klee_range(0, 2, "j");
    if (j == 1)		// ensures (i=1,j=0) is fertile
      printf("C");
  }

  printf("D\n");

  return 0;
}
