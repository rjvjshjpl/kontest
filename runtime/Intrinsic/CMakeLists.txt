#===------------------------------------------------------------------------===#
#
#                     The KLEE Symbolic Virtual Machine
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
#===------------------------------------------------------------------------===#
add_library(kleeIntrinsics OBJECT
  klee_div_zero_check.c
  klee_int.c
  klee_overshift_check.c
  klee_choice.c
  klee_range.c
  memalign.c
  memcpy.c
  memmove.c
  mempcpy.c
  memset.c
)

# need PIC since these are being compiled into libkleeRuntest
set_target_properties(kleeIntrinsics
  PROPERTIES
    POSITION_INDEPENDENT_CODE True
)

if (KLEE_RUNTIME_32BIT)
  set_target_properties(kleeIntrinsics
    PROPERTIES
      COMPILE_FLAGS "-m32"
      LINK_FLAGS "-m32"
  )
endif()
