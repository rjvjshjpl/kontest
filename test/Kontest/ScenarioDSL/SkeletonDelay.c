// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error --skeleton-delay=5 --stop-after-n-instructions=5000 %t.bc > %t.log
// RUN: test -f %t.klee-out/test000001.ktest
// RUN: test -f %t.klee-out/test000002.ktest

// RUN: %cc %s %libkleeruntest -Wl,-rpath %libkleeruntestdir -o %t_runner
// RUN: env KTEST_FILE=%t.klee-out/test000001.ktest %t_runner > %t1.log
// RUN: env KTEST_FILE=%t.klee-out/test000002.ktest %t_runner > %t2.log
// RUN: grep ^A$ %t1.log %t2.log
// RUN: grep ^B$ %t1.log %t2.log

#include <stdio.h>
#include <kontest/scenarios.h>

BEGIN_ACTION(RA)
	printf("A\n");
END_ACTION

BEGIN_ACTION(RB)
	printf("B\n");
END_ACTION

BEGIN_ACTION(RZ)
  // This loop can spawn infinitely many symbolic tests
	while (klee_range(0, 2, "loop"))
    ;
END_ACTION

BEGIN_SCENARIO(SA)
  RA();
  RZ();
END_SCENARIO

BEGIN_SCENARIO(scene)
  ANY(SA(), RB());
END_SCENARIO

TEST_SCENARIO(scene)
