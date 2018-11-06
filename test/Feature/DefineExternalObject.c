// RUN: %llvmgcc -emit-llvm -c -o %t1.bc %s
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc

#include <stdio.h>
#include <time.h>
#include <assert.h>

int main() {
  time_t t = 60 * 60 * 24 * 7;    // one week after the epoch
  struct tm *lt = localtime(&t);
  klee_define_external_object(lt, sizeof(struct tm));
  int year = lt->tm_year;
  assert(year == 70);   // should be 1970

  return 0;
}
