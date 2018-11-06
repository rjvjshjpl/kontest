// Copyright 2016-2018 California Institute of Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by dfremont on 6/20/17.
//

#include "ConcolicTest.h"

#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace klee;

unsigned ConcolicTest::nextTestID = 0;

void ConcolicTest::dump() const {
  llvm::errs() << "gen " << generation << ", score " << newBlocks << ", rep " << isReplacement << ", " << objects.size()
               << " objects: \n";
  for (const ConcolicTestObject &obj : objects) {
    llvm::errs() << "  " << obj.name << ": ";
    for (unsigned i = 0; i < obj.bytes->size; i++)
      llvm::errs() << (int) obj.bytes->bytes[i] << " ";
    llvm::errs() << "\n";
  }
  llvm::errs() << " path: ";
  for (bool b : expectedPath)
    llvm::errs() << (int) b;
  llvm::errs() << "\n";
}

static void *interpretAsPointer(const ConcolicTestObject &obj) {
  assert(obj.bytes->size == sizeof(void *) && "object does not have pointer size!");
  return *reinterpret_cast<void **>(obj.bytes->bytes);
}

void ConcolicTest::dumpArgs() const {
  if (objects.size() < 2)
    return;
  if (*objects[0].name != "n_args")
    return;
  if (*objects[1].name != "arg0")
    return;

  unsigned *addr = static_cast<unsigned *>(interpretAsPointer(objects[0]));
  unsigned nargs = *addr;
  for (unsigned i = 0; i < nargs; i++) {
    char *saddr = static_cast<char *>(interpretAsPointer(objects[i + 1]));
    llvm::errs() << "\"" << std::string(saddr) << "\" ";
  }
}
