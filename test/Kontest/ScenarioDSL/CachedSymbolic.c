// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc > %t.log 2> %t.log.err
// RUN: grep "skeletons = 6$" %t.log.err
// RUN: grep "completed paths = 12$" %t.log.err

#include <stdio.h>
#include <kontest/scenarios.h>

int gx = 42, gx2 = 42, gy = 42, gz = 42;

BEGIN_ACTION(RA, int x)
	printf("A");
	gx = x;
END_ACTION

BEGIN_ACTION(RB, int x, int y)
	printf("B");
	gx2 = x;
	gy = y;
END_ACTION

BEGIN_SCENARIO(S1)
	int x = klee_range(0, 2, "x");
	RA(x);
	KT_ASSERT(gx == x);
	int y = klee_range(0, 2, "y");
	RB(x, y);
	KT_ASSERT(gx == x && gx2 == x && gy == y);
END_SCENARIO

int zs[2] = { 0, 0 };
unsigned nextZ = 0;

BEGIN_ACTION(RC, int z)
	printf("C");
	zs[nextZ++] = z;
END_ACTION

BEGIN_ACTION(check)
	if (zs[0] == zs[1])
		printf("1\n");
	else
		printf("0\n");		// should be possible in each interleaving
END_ACTION

BEGIN_SCENARIO(S2)
	BEGIN_REPEAT(2, 2)
		int z = klee_range(0, 2, "z");
		RC(z);
	END_REPEAT
END_SCENARIO

BEGIN_SCENARIO(scene)
	INTERLEAVE(S1(), S2());		// 6 possible interleavings
	check();
END_SCENARIO

TEST_SCENARIO(scene)
