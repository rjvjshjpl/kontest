// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t.bc 2> %t.log
// RUN: grep "completed paths = 3" %t.log

// Defining actions/scenarios (e.g. in a header file) should not break
// tests that don't use the scenario DSL.

#include <stdio.h>
#include <kontest/scenarios.h>

int foo = 0;

BEGIN_ACTION(inc, unsigned x)
	foo += x;
END_ACTION

BEGIN_ACTION(dec)
	--foo;
END_ACTION

BEGIN_SCENARIO(scene)
	BEGIN_REPEAT(1, 4)
		ANY(inc(2), inc(4), dec());
	END_REPEAT
END_SCENARIO

int main(void) {
	int x = klee_range(0, 2, "x");

	if (x) {
		int y = klee_range(0, 2, "y");
		if (y)
			printf("C\n");
		else
			printf("B\n");
	} else {
		printf("A\n");
	}

	return 0;
}
