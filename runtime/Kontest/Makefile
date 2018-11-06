#===-- runtime/Intrinsic/Kontest/Makefile ------------------*- Makefile -*--===#
#
#                     The KLEE Symbolic Virtual Machine
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
#===------------------------------------------------------------------------===#

LEVEL=../..

# Needed for LLVM version
include $(LEVEL)/Makefile.config

MODULE_NAME=kontest

DONT_BUILD_RELINKED=1
# Don't strip debug info from the module.
DEBUG_RUNTIME=1
NO_PEDANTIC=1
NO_BUILD_ARCHIVE=1

include $(LEVEL)/Makefile.common
