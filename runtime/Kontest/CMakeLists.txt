#===------------------------------------------------------------------------===#
#
#                     The KLEE Symbolic Virtual Machine
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
#===------------------------------------------------------------------------===#

add_library(kontest OBJECT
  Scheduler.c
  Utils.c
)

# need PIC since these are being compiled into libkleeRuntest
set_target_properties(kontest
  PROPERTIES
    POSITION_INDEPENDENT_CODE True
)

target_compile_definitions(kontest PRIVATE KONTEST_FOR_RUNTEST=1)

if (KLEE_RUNTIME_32BIT)
  set_target_properties(kontest
    PROPERTIES
      COMPILE_FLAGS "-m32"
      LINK_FLAGS "-m32"
  )
endif()
