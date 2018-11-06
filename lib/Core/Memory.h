//===-- Memory.h ------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_MEMORY_H
#define KLEE_MEMORY_H

#include "Context.h"
#include "ConcolicTest.h"
#include "klee/Expr.h"

#include "llvm/ADT/StringExtras.h"

#include <vector>
#include <string>

namespace llvm {
  class Value;
}

namespace klee {

class BitArray;
class MemoryManager;
class Solver;
class ArrayCache;
class AddressSpace;
class ConstraintManager;

class MemoryObject {
  friend class STPBuilder;
  friend class ObjectState;
  friend class ExecutionState;
  friend class AddressSpace;

public:
  static int counter;

private:
  mutable unsigned refCount;

public:
  unsigned id;
  uint64_t address;

  /// size in bytes
  unsigned size;
  mutable std::string name;

  bool isLocal;
  mutable bool isGlobal;
  bool isFixed;

  bool isUserSpecified;

  MemoryManager *parent;

  /// "Location" for which this memory object was allocated. This
  /// should be either the allocating instruction or the global object
  /// it was allocated for (or whatever else makes sense).
  const llvm::Value *allocSite;
  
  /// A list of boolean expressions the user has requested be true of
  /// a counterexample. Mutable since we play a little fast and loose
  /// with allowing it to be added to during execution (although
  /// should sensibly be only at creation time).
  mutable std::vector< ref<Expr> > cexPreferences;

private:
  // cached ConstantExprs
  const ref<ConstantExpr> baseExpr, sizeExpr, endExpr;

public:
  MemoryObject(uint64_t _address, unsigned _size, 
               bool _isLocal, bool _isGlobal, bool _isFixed,
               const llvm::Value *_allocSite,
               MemoryManager *_parent)
    : refCount(0), 
      id(counter++),
      address(_address),
      size(_size),
      name("unnamed"),
      isLocal(_isLocal),
      isGlobal(_isGlobal),
      isFixed(_isFixed),
      isUserSpecified(false),
      parent(_parent), 
      allocSite(_allocSite),
      baseExpr(ConstantExpr::alloc(address, Context::get().getPointerWidth())),
      sizeExpr(ConstantExpr::alloc(size, Context::get().getPointerWidth())),
      endExpr(ConstantExpr::alloc(address + size, Context::get().getPointerWidth())) {}

  MemoryObject(const MemoryObject &b) = delete;
  MemoryObject &operator=(const MemoryObject &b) = delete;

  ~MemoryObject();

  /// Get an identifying string for this allocation.
  void getAllocInfo(std::string &result) const;

  void setName(std::string name) const {
    this->name = name;
  }

  ref<ConstantExpr> getBaseExpr() const { 
    return baseExpr;
  }
  ref<ConstantExpr> getSizeExpr() const { 
    return sizeExpr;
  }
  ref<ConstantExpr> getEndExpr() const {
    return endExpr;
  }
  unsigned getOffset(uint64_t pointer) const {
    return (pointer - address);
  }
  uint64_t getAddress(unsigned offset) const {
    return address + offset;
  }
  ref<Expr> getOffsetExpr(ref<Expr> pointer) const {
    return SubExpr::create(pointer, getBaseExpr());
  }
  ref<Expr> getBoundsCheckPointer(ref<Expr> pointer) const {
    if (size <= 1) {
      return EqExpr::create(pointer, getBaseExpr());
    } else {
      return AndExpr::create(UleExpr::create(getBaseExpr(), pointer), UltExpr::create(pointer, getEndExpr()));
    }
  }
  ref<Expr> getBoundsCheckPointer(ref<Expr> pointer, unsigned bytes) const {
    if (bytes<=size) {
      const ref<ConstantExpr> end = ConstantExpr::create(address + size - bytes + 1, Context::get().getPointerWidth());
      return AndExpr::create(UleExpr::create(getBaseExpr(), pointer), UltExpr::create(pointer, end));
    } else {
      return ConstantExpr::False;
    }
  }
  bool boundsCheckPointer(uint64_t pointer, unsigned bytes) const {
    return boundsCheckOffset(pointer - address, bytes);
  }
  bool boundsCheckOffset(uint64_t offset, unsigned bytes) const {
    return (bytes > size ? false : (offset <= size - bytes));
  }

  ref<Expr> getBoundsCheckOffset(ref<Expr> offset) const {
    if (size <= 1) {
      return EqExpr::create(offset, 
                            ConstantExpr::alloc(0, Context::get().getPointerWidth()));
    } else {
      return UltExpr::create(offset, getSizeExpr());
    }
  }
  ref<Expr> getBoundsCheckOffset(ref<Expr> offset, unsigned bytes) const {
    if (bytes<=size) {
      return UltExpr::create(offset, 
                             ConstantExpr::alloc(size - bytes + 1, 
                                                 Context::get().getPointerWidth()));
    } else {
      return ConstantExpr::False;
    }
  }
};

class ObjectState {
private:
  friend class AddressSpace;
  unsigned copyOnWriteOwner; // exclusively for AddressSpace
  AddressSpace &addressSpace;

  friend class ObjectHolder;
  mutable unsigned refCount;

  const MemoryObject *object;

  uint8_t *concreteStore;
  // XXX cleanup name of flushMask (its backwards or something)
  BitArray *concreteMask;

  // mutable because may need flushed during read of const
  mutable BitArray *flushMask;

  ref<Expr> *knownSymbolics;

  // mutable because we may need flush during read of const
  mutable UpdateList updates;

public:
  unsigned size;

  bool readOnly;

public:
  /// Create a new object state for the given memory object with concrete
  /// contents. The initial contents are undefined, it is the callers
  /// responsibility to initialize the object contents appropriately.
  ObjectState(const MemoryObject *mo, AddressSpace &addressSpace);

  /// Create a new object state for the given memory object with symbolic
  /// contents.
  ObjectState(const MemoryObject *mo, AddressSpace &addressSpace, const Array *array);

  ObjectState(const ObjectState &os, AddressSpace &addressSpace);
  ~ObjectState();

  const MemoryObject *getObject() const { return object; }

  void setReadOnly(bool ro) { readOnly = ro; }

  // make contents all concrete and zero
  void initializeToZero();
  // make contents all concrete and random
  void initializeToRandom();

  void claim() const { ++refCount; };
  void release() const {
    assert(refCount > 0 && "object state freed too many times!");
    refCount--;
    if (refCount == 0)
      delete this;
  }

  ref<Expr> read(ref<Expr> offset, unsigned concreteOffset, Expr::Width width, llvm::APInt &concreteResult,
                 const ConstraintManager *assumptions = 0) const;
  ref<Expr> read(unsigned offset, Expr::Width width, llvm::APInt &concreteResult,
                 const ConstraintManager *assumptions = 0) const;
  ref<Expr> read8(unsigned offset, uint8_t &concreteResult,
                  const ConstraintManager *assumptions = 0) const;

  // return bytes written.
  void write(unsigned offset, ref<Expr> value, const llvm::APInt &concreteValue);
  void write(unsigned offset, ref<ConstantExpr> value);
  void write(ref<Expr> offset, unsigned concreteOffset, ref<Expr> value, const llvm::APInt &concreteValue);

  void write8(unsigned offset, uint8_t value);
  void write16(unsigned offset, uint16_t value);
  void write32(unsigned offset, uint32_t value);
  void write64(unsigned offset, uint64_t value);

  void copyTo(ObjectState *os, unsigned count) const;


  // Functions directly reading the concrete value
  ref<ConstantExpr> readConcrete(unsigned offset, Expr::Width width, llvm::APInt &concreteResult);

  // Unsafe functions directly modifying the concrete value; these can
  // break the symbolic-concrete invariant and should be used with care.
  void writeConcrete(unsigned offset, const llvm::APInt &value);
  void writeConcrete8(unsigned offset, uint8_t value);
  void overwriteConcretesFrom(const ObjectBytes &bytes);
  void overwriteConcretesFrom(const uint8_t *bytes, unsigned length);
  const uint8_t *allConcretes() const;

private:
  const UpdateList &getUpdates(const ConstraintManager *assumptions = 0) const;
  bool shouldRecompressUpdates() const;

  void makeConcrete();

  void makeSymbolic();

  ref<Expr> read8(ref<Expr> offset, unsigned concreteOffset, uint8_t &concreteResult,
                  const ConstraintManager *assumptions) const;
  void write8(unsigned offset, ref<Expr> value, uint8_t concreteValue);
  void write8(ref<Expr> offset, unsigned concreteOffset, ref<Expr> value, uint8_t concreteValue);

  void fastRangeCheckOffset(ref<Expr> offset, unsigned *base_r, 
                            unsigned *size_r) const;
  void flushRangeForRead(unsigned rangeBase, unsigned rangeSize) const;
  void flushRangeForWrite(unsigned rangeBase, unsigned rangeSize);

  bool isByteConcrete(unsigned offset) const;
  bool isByteFlushed(unsigned offset) const;
  bool isByteKnownSymbolic(unsigned offset) const;

  void markByteConcrete(unsigned offset);
  void markByteSymbolic(unsigned offset);
  void markByteFlushed(unsigned offset);
  void markByteUnflushed(unsigned offset);
  void setKnownSymbolic(unsigned offset, Expr *value);

  void print();
  ArrayCache *getArrayCache() const;
};
  
} // End klee namespace

#endif
