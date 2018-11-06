//===-- memmove.c ---------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <stdlib.h>
#include <assert.h>
#include <stdint.h>

__attribute__((weak)) void *memmove(void *dst, const void *src, size_t count) {
  char *a = dst;
  const char *b = src;

  if (src == dst)
    return dst;

  // DJF: the src>dst comparison is a source of nondeterminism when using memmove
  // on objects allocated with malloc (unless the deterministic allocator is used
  // instead). Fortunately, the comparison is only needed for overlapping objects,
  // which cannot arise from malloc. So we only do it when necessary.
  uintptr_t ia = (uintptr_t) a;
  uintptr_t ib = (uintptr_t) b;
  int disjoint = ((count <= ib - ia) | (count <= ia - ib));    // detect disjointness without branching
  if (disjoint || src>dst) {
    while (count--) *a++ = *b++;
  } else {
    a+=count-1;
    b+=count-1;
    while (count--) *a-- = *b--;
  }

  return dst;
}
