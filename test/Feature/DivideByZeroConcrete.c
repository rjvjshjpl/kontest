// RUN: %llvmgcc -emit-llvm -c -g -O0 %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -optimize=false %t.bc 2>&1 | FileCheck %s

// This test is to ward off a future in which check-div-zero is
// disabled by default but concrete divides by zero are unhandled.

int main()
{
  int x = klee_range(1, 10, "x");
  int y = klee_range(0, 10, "y");

// CHECK: DivideByZeroConcrete.c:[[@LINE+1]]: divide by zero
  int result1 = x / y;

  return 0;
}

// CHECK: completed paths = 2
