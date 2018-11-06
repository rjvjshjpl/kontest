//===-- MemoryManager.h -----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_MEMORYMANAGER_H
#define KLEE_MEMORYMANAGER_H

#include "klee/Expr.h"

#include <unordered_set>
#include <stdint.h>

namespace llvm {
class Value;
}

namespace klee {
class MemoryObject;
class ArrayCache;

class MemoryManager {
private:
  typedef std::unordered_set<MemoryObject *> objects_ty;
  objects_ty objects;
  ArrayCache *const arrayCache;

  char *deterministicSpace;
  char *nextFreeSlot;
  size_t spaceSize;

  const Expr::Width targetPointerWidth;
  const ref<ConstantExpr> nullPointer;

public:
  MemoryManager(ArrayCache *arrayCache);
  ~MemoryManager();

  /**
   * Returns memory object which contains a handle to real virtual process
   * memory.
   */
  MemoryObject *allocate(uint64_t size, bool isLocal, bool isGlobal,
                         const llvm::Value *allocSite, size_t alignment);
  MemoryObject *allocateFixed(uint64_t address, uint64_t size,
                              const llvm::Value *allocSite);
  void deallocate(const MemoryObject *mo);
  void markFreed(MemoryObject *mo);
  ArrayCache *getArrayCache() const { return arrayCache; }

  /*
   * Returns the size used by deterministic allocation in bytes
   */
  size_t getUsedDeterministicSize();

  typedef std::pair<char*, unsigned long> ResetToken;
  ResetToken getResetToken() const;
  void reset(ResetToken token);

  ref<Expr> createSExtToPointerWidth(ref<Expr> e) const {
    return SExtExpr::create(e, targetPointerWidth);
  }

  ref<Expr> createZExtToPointerWidth(ref<Expr> e) const {
    return ZExtExpr::create(e, targetPointerWidth);
  }

  ref<ConstantExpr> createPointer(uint64_t v) const {
    return ConstantExpr::create(v, targetPointerWidth);
  }

  ref<ConstantExpr> createNullPointer() const {
    return nullPointer;
  }
};

} // End klee namespace

#endif
