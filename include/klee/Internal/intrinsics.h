/*===-- intrinsics.h --------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===*/
//
// This file declares intrinsics intended for use by higher-level public KLEE
// functions but not for public use themselves.
//
//===----------------------------------------------------------------------===*/

#ifndef __INTRINSICS_H__
#define __INTRINSICS_H__

#include "stdint.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif
  
int klee_require_in_list(int x, int n, int *choices);

void klee_obtain_true_index(int *x, int n, char *list, const char *name);

#ifdef __cplusplus
}
#endif

#endif /* __INTRINSICS_H__ */
