// RUN: %llvmgcc -emit-llvm -g -c -o %t.bc %s
// RUN: rm -rf %t.klee-out
// We disable linking with the Kontest runtime to prevent any undefined reference warnings
// RUN: %klee --output-dir=%t.klee-out --optimize=false --exit-on-error --exit-on-divergence --kontest-runtime=false --symbolic-pointers=none --warn-on-concretization %t.bc > %t.warn.log 2>&1
// RUN: FileCheck -input-file=%t.warn.log -check-prefix=CHECK-WARN %s
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --optimize=false --exit-on-error --exit-on-divergence --kontest-runtime=false --symbolic-pointers=none --warn-on-concretization=false %t.bc > %t.default.log 2>&1
// RUN: FileCheck -input-file=%t.default.log -check-prefix=CHECK-DEFAULT %s

#include <assert.h>
#include <klee/klee.h>

int main() {
  int x = klee_range(0, 2, "x");
  int a[2] = { 0, 1 };

  // CHECK-WARN: KLEE: WARNING ONCE: {{.*}}:[[@LINE+2]]: silently concretizing
  // CHECK-DEFAULT-NOT: KLEE: WARNING
  if (a[x] == 1)
    assert(0);    // should not be reached because of concretization

  return 0;
}

