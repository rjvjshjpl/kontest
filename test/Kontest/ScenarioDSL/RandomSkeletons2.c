// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=2 --skeletons=random --random-seed=1 %t.bc > %t.log
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=2 --skeletons=random --random-seed=2 %t.bc > %t2.log
// RUN: not diff %t.log %t2.log
// RUN: grep ^0123456789$ %t.log
// RUN: grep -v ^0123456798$ %t.log
// RUN: grep ^0123456789$ %t2.log
// RUN: grep -v ^0123456798$ %t2.log

// Check that --skeletons=random in fact produces random skeletons (after the first).
// We can't really test this well but make a few simple checks that should succeed
// with very high probability if the skeletons are random.

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA, unsigned x)
	printf("%d", x);
END_ACTION

BEGIN_ACTION(end)
	printf("\n");
END_ACTION

BEGIN_SCENARIO(scene)
	INTERLEAVE(RA(0), RA(1), RA(2), RA(3), RA(4), RA(5), RA(6), RA(7), RA(8), RA(9));
  end();
END_SCENARIO

TEST_SCENARIO(scene)
