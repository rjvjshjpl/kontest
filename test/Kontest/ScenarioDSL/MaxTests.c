// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --max-tests=1 --output-all-tests --stop-after-n-instructions=5000 %t.bc
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: not test -f %t.klee-out/test000002.ktest

// This test checks that --max-tests works even when no symbolics are created, so
// that every skeleton only generates one test. In case this fails, we depend on
// --stop-after-n-instructions to stop the test in a timely manner.

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
