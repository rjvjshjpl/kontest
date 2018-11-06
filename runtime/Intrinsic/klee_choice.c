//===-- klee_choice.c -----------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <stdarg.h>
#include <klee/klee.h>
#include <klee/Internal/intrinsics.h>

int klee_choice(const char *name, int n, int *choices) {
  if (klee_is_symbolic(n))
    klee_report_error(__FILE__, __LINE__, "symbolic number of choices", "user");
  if (n < 1)
    klee_report_error(__FILE__, __LINE__, "invalid number of choices", "user");
  if (klee_is_symbolic((uintptr_t) choices))
    klee_report_error(__FILE__, __LINE__, "symbolic list of choices", "user");

  int x = choices[0];   // deterministic initial concrete value
  klee_make_symbolic(&x, sizeof x, name);
  x = klee_require_in_list(x, n, choices);

  return x;
}

int klee_filtered_choice(const char *name, int n, char *filter) {
  if (klee_is_symbolic(n))
    klee_report_error(__FILE__, __LINE__, "symbolic number of choices", "user");
  if (n < 1)
    klee_report_error(__FILE__, __LINE__, "invalid number of choices", "user");
  if (klee_is_symbolic((uintptr_t) filter))
    klee_report_error(__FILE__, __LINE__, "symbolic list of choices", "user");

  int x = 0;    // deterministic initial concrete value (may be overwritten below)
  klee_obtain_true_index(&x, n, filter, name);

  return x;
}