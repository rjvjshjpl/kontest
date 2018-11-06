//===-- memalign.c --------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <stdlib.h>

// This is a simplification of the uClibc implementation
void *memalign(size_t alignment, size_t size) {
  void *result;
  unsigned long int adj;
  result = malloc(size + alignment - 1);
  if (result == NULL)
    return NULL;
  adj = (unsigned long int) ((unsigned long int) ((char *) result - (char *) NULL)) % alignment;
  if (adj != 0)
    result = (char *) result + alignment - adj;
  return result;
}
