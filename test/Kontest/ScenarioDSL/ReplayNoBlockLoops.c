// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --output-all-tests=true %t.bc 2>&1 | FileCheck %s
// CHECK-NOT: unable to write output test case
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: test -f %t.klee-out/test000002.ktest

// Now try to replay with libkleeRuntest
// RUN: %cc %s %libkleeruntest -Wl,-rpath %libkleeruntestdir -o %t_runner
// RUN: env KTEST_FILE=%t.klee-out/test000001.ktest %t_runner > %t1.log
// RUN: env KTEST_FILE=%t.klee-out/test000002.ktest %t_runner > %t2.log
// RUN: grep ^A$ %t1.log %t2.log
// RUN: grep ^B$ %t1.log %t2.log

// This test makes sure that skeletons with no block loops are correctly
// written to the KTest file for replay.

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA)
	printf("A");
END_ACTION

BEGIN_ACTION(RB)
	printf("B");
END_ACTION

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(scene)
	ANY(RA(), RB());
	end();
END_SCENARIO

TEST_SCENARIO(scene)
