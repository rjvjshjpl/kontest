// RUN: %llvmgcc %s -emit-llvm -g -O0 -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --output-all-tests=true %t.bc
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: test -f %t.klee-out/test000002.ktest
// RUN: test -f %t.klee-out/test000003.ktest

// Now try to replay with libkleeRuntest
// RUN: %cc %s %libkleeruntest -Wl,-rpath %libkleeruntestdir -o %t_runner
// RUN: env KTEST_FILE=%t.klee-out/test000001.ktest %t_runner > %t1.log
// RUN: env KTEST_FILE=%t.klee-out/test000002.ktest %t_runner > %t2.log
// RUN: env KTEST_FILE=%t.klee-out/test000003.ktest %t_runner > %t3.log
// RUN: grep ^A$ %t1.log %t2.log %t3.log
// RUN: grep ^AB$ %t1.log %t2.log %t3.log
// RUN: grep ^ABB$ %t1.log %t2.log %t3.log

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
	RA();
	BEGIN_REPEAT(0, 2)
		RB();
	END_REPEAT
	end();
END_SCENARIO

TEST_SCENARIO(scene)