#===------------------------------------------------------------------------===#
#
#                     The KLEE Symbolic Virtual Machine
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
#===------------------------------------------------------------------------===#

add_library(kleeRuntest SHARED
  $<TARGET_OBJECTS:kleeIntrinsics>
  $<TARGET_OBJECTS:kontest>
  intrinsics.c
  # HACK:
  ${CMAKE_SOURCE_DIR}/lib/Basic/KTest.cpp
)
# Increment version appropriately if ABI/API changes, more details:
# http://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html#AEN135
set(KLEE_RUNTEST_VERSION 1.0)
set_target_properties(kleeRuntest
  PROPERTIES
    VERSION ${KLEE_RUNTEST_VERSION}
    SOVERSION ${KLEE_RUNTEST_VERSION}
)

if (KLEE_RUNTIME_32BIT)
  set_target_properties(kleeRuntest
    PROPERTIES
      COMPILE_FLAGS "-m32"
      LINK_FLAGS "-m32"
  )
endif()

install(TARGETS kleeRuntest
  DESTINATION "${CMAKE_INSTALL_FULL_LIBDIR}")
