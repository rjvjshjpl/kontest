// RUN: %llvmgcc %s -emit-llvm -g -O0 -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --output-all-tests=true %t.bc
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: test -f %t.klee-out/test000002.ktest
// RUN: test -f %t.klee-out/test000003.ktest
// RUN: test -f %t.klee-out/test000004.ktest
// RUN: test -f %t.klee-out/test000005.ktest
// RUN: test -f %t.klee-out/test000006.ktest
// RUN: test -f %t.klee-out/test000007.ktest
// RUN: test -f %t.klee-out/test000008.ktest
// RUN: test -f %t.klee-out/test000009.ktest

// Now try to replay with libkleeRuntest
// RUN: %cc %s %libkleeruntest -Wl,-rpath %libkleeruntestdir -o %t_runner
// RUN: env KTEST_FILE=%t.klee-out/test000001.ktest %t_runner > %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000002.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000003.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000004.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000005.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000006.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000007.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000008.ktest %t_runner >> %t.log
// RUN: env KTEST_FILE=%t.klee-out/test000009.ktest %t_runner >> %t.log
// RUN: grep ^ACC0$ %t.log
// RUN: grep ^ACC1$ %t.log
// RUN: grep ^ACC2$ %t.log
// RUN: grep ^CAC0$ %t.log
// RUN: grep ^CAC1$ %t.log
// RUN: grep ^CAC2$ %t.log
// RUN: grep ^CCA0$ %t.log
// RUN: grep ^CCA1$ %t.log
// RUN: grep ^CCA2$ %t.log


#include <stdio.h>
#include <kontest/scenarios.h>

int gx = 42;

BEGIN_ACTION(RA, int x)
	printf("A");
	gx = x;
END_ACTION

BEGIN_SCENARIO(S1)
	int x = klee_range(0, 2, "x");
	RA(x);
	KT_ASSERT(gx == x);
END_SCENARIO

int zs[2] = { 0, 0 };
unsigned nextZ = 0;

BEGIN_ACTION(RC, int z)
	printf("C");
	zs[nextZ++] = z;
END_ACTION

BEGIN_ACTION(check)
	if (zs[0] == zs[1]) {
		printf("1\n");
	} else {		// should be possible in each interleaving
		if (klee_range(0, 2, "b"))
			printf("2\n");
		else
			printf("0\n");
	}
END_ACTION

BEGIN_SCENARIO(S2)
	BEGIN_REPEAT(2, 2)
		int z = klee_range(0, 2, "z");
		RC(z);
	END_REPEAT
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(S1(), S2());		// 3 possible interleavings
	check();
END_SCENARIO

TEST_SCENARIO(scene)
