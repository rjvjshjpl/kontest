//===-- FileHandling.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#include "klee/Internal/Support/FileHandling.h"
#include "klee/Config/Version.h"
#include "klee/Config/config.h"
#include "klee/Internal/Support/ErrorHandling.h"

#include "llvm/Support/FileSystem.h"

namespace klee {

llvm::raw_fd_ostream *klee_open_output_file(std::string &path,
                                            std::string &error) {
  llvm::raw_fd_ostream *f;
  std::error_code ec;
  f = new llvm::raw_fd_ostream(path.c_str(), ec, llvm::sys::fs::F_None);
  if (ec)
    error = ec.message();
  if (!error.empty()) {
    if (f)
      delete f;
    f = NULL;
  }
  return f;
}
}
