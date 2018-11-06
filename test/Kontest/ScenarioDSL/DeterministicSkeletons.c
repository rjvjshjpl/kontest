// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=2 --skeletons=random %t.bc > %t.log
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=2 --skeletons=random %t.bc > %t2.log
// RUN: diff %t.log %t2.log

// This test checks that by default skeletons are generated deterministically,
// even with --skeletons=random. (This is for reproducibility. Of course using
// a non-default --random-seed will change the result).
// The value of --max-tests must be at least 2, since the very first skeleton
// is always the same.

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
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
	ANY(RA(), RB());
  end();
END_SCENARIO

TEST_SCENARIO(scene)
