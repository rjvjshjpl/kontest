// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=2 --skeletons=random --random-seed=1 %t.bc > %t.log
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=2 --skeletons=random --random-seed=2 %t.bc > %t2.log
// RUN: not diff %t.log %t2.log
// RUN: grep A %t.log
// RUN: grep B %t.log
// RUN: grep A %t2.log
// RUN: grep B %t2.log

// Check that --skeletons=random in fact produces random skeletons (after the first).
// We can't really test this well but make a few simple checks that should succeed
// with very high probability if the skeletons are random.

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
	BEGIN_REPEAT(32, 48)
    ANY(RA(), RB());
	END_REPEAT
  end();
END_SCENARIO

TEST_SCENARIO(scene)
