//===-- Memory.cpp --------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Memory.h"

#include "Context.h"
#include "klee/Expr.h"
#include "klee/Constraints.h"
#include "klee/Solver.h"
#include "klee/util/BitArray.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/util/ArrayCache.h"

#include "AddressSpace.h"
#include "ObjectHolder.h"
#include "MemoryManager.h"

#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <cassert>
#include <sstream>

using namespace llvm;
using namespace klee;

namespace {
  cl::opt<bool>
  UseConstantArrays("use-constant-arrays",
                    cl::init(true));

  cl::opt<bool>
  SimplifyReads("simplify-reads",
                cl::init(true),
                cl::desc("Simplify indices and values during ReadExpr construction (default=on)"));
}

/***/

ObjectHolder::ObjectHolder(const ObjectHolder &b) : os(b.os) { 
  if (os) ++os->refCount; 
}

ObjectHolder::ObjectHolder(ObjectState *_os) : os(_os) { 
  if (os) ++os->refCount; 
}

ObjectHolder::~ObjectHolder() { 
  if (os && --os->refCount==0) delete os; 
}
  
ObjectHolder &ObjectHolder::operator=(const ObjectHolder &b) {
  if (b.os) ++b.os->refCount;
  if (os && --os->refCount==0) delete os;
  os = b.os;
  return *this;
}

/***/

int MemoryObject::counter = 0;

MemoryObject::~MemoryObject() {
  if (parent)
    parent->markFreed(this);
}

void MemoryObject::getAllocInfo(std::string &result) const {
  llvm::raw_string_ostream info(result);

  info << "MO" << id << "[" << size << "]";

  if (allocSite) {
    info << " allocated at ";
    if (const Instruction *i = dyn_cast<Instruction>(allocSite)) {
      info << i->getParent()->getParent()->getName() << "():";
      info << *i;
    } else if (const GlobalValue *gv = dyn_cast<GlobalValue>(allocSite)) {
      info << "global:" << gv->getName();
    } else {
      info << "value:" << *allocSite;
    }
  } else {
    info << " (no allocation info)";
  }
  
  info.flush();
}

/***/

ObjectState::ObjectState(const MemoryObject *mo, AddressSpace &addressSpace)
  : copyOnWriteOwner(0),
    addressSpace(addressSpace),
    refCount(0),
    object(mo),
    concreteStore(new uint8_t[mo->size]),
    concreteMask(0),
    flushMask(0),
    knownSymbolics(0),
    updates(0, 0),
    size(mo->size),
    readOnly(false) {
  if (!UseConstantArrays) {
    static unsigned id = 0;
    const Array *array =
        getArrayCache()->CreateArray("tmp_arr" + llvm::utostr(++id), size);
    updates = UpdateList(array, 0);
  }
  memset(concreteStore, 0, size);
}


ObjectState::ObjectState(const MemoryObject *mo, AddressSpace &addressSpace, const Array *array)
  : copyOnWriteOwner(0),
    addressSpace(addressSpace),
    refCount(0),
    object(mo),
    concreteStore(new uint8_t[mo->size]),
    concreteMask(0),
    flushMask(0),
    knownSymbolics(0),
    updates(array, 0),
    size(mo->size),
    readOnly(false) {
  makeSymbolic();
  memset(concreteStore, 0, size);
}

ObjectState::ObjectState(const ObjectState &os, AddressSpace &addressSpace)
  : copyOnWriteOwner(0),
    addressSpace(addressSpace),
    refCount(0),
    object(os.object),
    concreteStore(new uint8_t[os.size]),
    concreteMask(os.concreteMask ? new BitArray(*os.concreteMask, os.size) : 0),
    flushMask(os.flushMask ? new BitArray(*os.flushMask, os.size) : 0),
    knownSymbolics(0),
    updates(os.updates),
    size(os.size),
    readOnly(false) {
  assert(!os.readOnly && "no need to copy read only object?");
  if (os.knownSymbolics) {
    knownSymbolics = new ref<Expr>[size];
    for (unsigned i=0; i<size; i++)
      knownSymbolics[i] = os.knownSymbolics[i];
  }

  memcpy(concreteStore, os.concreteStore, size*sizeof(*concreteStore));
}

ObjectState::~ObjectState() {
  if (concreteMask) delete concreteMask;
  if (flushMask) delete flushMask;
  if (knownSymbolics) delete[] knownSymbolics;
  delete[] concreteStore;
}

ArrayCache *ObjectState::getArrayCache() const {
  assert(object && "object was NULL");
  return object->parent->getArrayCache();
}

/***/

bool ObjectState::shouldRecompressUpdates() const {
  if (!updates.head)
    return false;

  const UpdateNode *un = updates.head;
  return isa<ConstantExpr>(un->index);
}

const UpdateList &ObjectState::getUpdates(const ConstraintManager *assumptions) const {
  // Constant arrays are created lazily.
  if (!updates.root || shouldRecompressUpdates()) {
    // Collect the list of writes, with the oldest writes first.
    
    // FIXME: We should be able to do this more efficiently, we just need to be
    // careful to get the interaction with the cache right. In particular we
    // should avoid creating UpdateNode instances we never use.
    unsigned NumWrites = updates.head ? updates.head->getSize() : 0;
    std::vector< std::pair< ref<Expr>, ref<Expr> > > Writes(NumWrites);
    const UpdateNode *un = updates.head;
    for (unsigned i = NumWrites; i != 0; un = un->next) {
      --i;
      if (SimplifyReads && assumptions)
        Writes[i] = std::make_pair(assumptions->simplifyExpr(un->index), assumptions->simplifyExpr(un->value));
      else
        Writes[i] = std::make_pair(un->index, un->value);
    }

    std::vector< ref<ConstantExpr> > Contents(size);

    if (!updates.root) {
      // Initialize to zeros.
      for (unsigned i = 0, e = size; i != e; ++i)
        Contents[i] = ConstantExpr::create(0, Expr::Int8);
    } else {
      Contents = updates.root->constantValues;
    }

    // Pull off as many concrete writes as we can.
    bool reuseRoot = updates.root;
    unsigned Begin = 0, End = Writes.size();

    // First pass; continues to the first write with a symbolic index
    for (; Begin != End; ++Begin) {
      // Push concrete writes into the constant array.
      ConstantExpr *Index = dyn_cast<ConstantExpr>(Writes[Begin].first);
      if (!Index)
        break;

      ConstantExpr *Value = dyn_cast<ConstantExpr>(Writes[Begin].second);
      if (Value) {
        uint64_t ci = Index->getZExtValue();
        if (Contents[ci]->getAPValue() != Value->getAPValue())
          reuseRoot = false;
        Contents[ci] = Value;
      } else {
        // symbolic value: leave for second pass
      }
    }
    unsigned firstSymbolicIndex = Begin;

    static unsigned id = 0;
    const Array *array;
    const Array *oldArray = updates.root;
    if (reuseRoot) {    // if we didn't apply any concrete writes, reuse the old root if any
      array = oldArray;
    } else {
      array = getArrayCache()->CreateArray(
          "const_arr" + llvm::utostr(++id), size, &Contents[0],
          &Contents[0] + Contents.size());
    }
    updates = UpdateList(array, 0);
    addressSpace.backingArrayChanged(const_cast<ObjectState*>(this), oldArray);

    // Second pass: apply all symbolic-value writes we skipped above
    for (Begin = 0; Begin != firstSymbolicIndex; ++Begin) {
      if (!isa<ConstantExpr>(Writes[Begin].second)) {
        updates.extend(Writes[Begin].first, Writes[Begin].second);
      }
    }

    // Apply all writes starting from the first symbolic-index write
    for (; Begin != End; ++Begin)
      updates.extend(Writes[Begin].first, Writes[Begin].second);
  }

  return updates;
}

void ObjectState::makeConcrete() {
  if (concreteMask) delete concreteMask;
  if (flushMask) delete flushMask;
  if (knownSymbolics) delete[] knownSymbolics;
  concreteMask = 0;
  flushMask = 0;
  knownSymbolics = 0;
}

void ObjectState::makeSymbolic() {
  assert(!updates.head &&
         "XXX makeSymbolic of objects with symbolic values is unsupported");

  // XXX simplify this, can just delete various arrays I guess
  for (unsigned i=0; i<size; i++) {
    markByteSymbolic(i);
    setKnownSymbolic(i, 0);
    markByteFlushed(i);
  }
}

void ObjectState::initializeToZero() {
  makeConcrete();
  memset(concreteStore, 0, size);
}

void ObjectState::initializeToRandom() {  
  makeConcrete();
  // randomly selected by 256 sided die
  memset(concreteStore, 0xAB, size);
}

/*
Cache Invariants
--
isByteKnownSymbolic(i) => !isByteConcrete(i)
isByteConcrete(i) => !isByteKnownSymbolic(i)
!isByteFlushed(i) => (isByteConcrete(i) || isByteKnownSymbolic(i))
 */

void ObjectState::fastRangeCheckOffset(ref<Expr> offset,
                                       unsigned *base_r,
                                       unsigned *size_r) const {
  *base_r = 0;
  *size_r = size;
}

void ObjectState::flushRangeForRead(unsigned rangeBase, 
                                    unsigned rangeSize) const {
  if (!flushMask) flushMask = new BitArray(size, true);
 
  for (unsigned offset=rangeBase; offset<rangeBase+rangeSize; offset++) {
    if (!isByteFlushed(offset)) {
      if (isByteConcrete(offset)) {
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       ConstantExpr::create(concreteStore[offset], Expr::Int8));
      } else {
        assert(isByteKnownSymbolic(offset) && "invalid bit set in flushMask");
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       knownSymbolics[offset]);
      }

      flushMask->unset(offset);
    }
  } 
}

void ObjectState::flushRangeForWrite(unsigned rangeBase, 
                                     unsigned rangeSize) {
  if (!flushMask) flushMask = new BitArray(size, true);

  for (unsigned offset=rangeBase; offset<rangeBase+rangeSize; offset++) {
    if (!isByteFlushed(offset)) {
      if (isByteConcrete(offset)) {
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       ConstantExpr::create(concreteStore[offset], Expr::Int8));
        markByteSymbolic(offset);
      } else {
        assert(isByteKnownSymbolic(offset) && "invalid bit set in flushMask");
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       knownSymbolics[offset]);
        setKnownSymbolic(offset, 0);
      }

      flushMask->unset(offset);
    } else {
      // flushed bytes that are written over still need
      // to be marked out
      if (isByteConcrete(offset)) {
        markByteSymbolic(offset);
      } else if (isByteKnownSymbolic(offset)) {
        setKnownSymbolic(offset, 0);
      }
    }
  } 
}

bool ObjectState::isByteConcrete(unsigned offset) const {
  return !concreteMask || concreteMask->get(offset);
}

bool ObjectState::isByteFlushed(unsigned offset) const {
  return flushMask && !flushMask->get(offset);
}

bool ObjectState::isByteKnownSymbolic(unsigned offset) const {
  return knownSymbolics && knownSymbolics[offset].get();
}

void ObjectState::markByteConcrete(unsigned offset) {
  if (concreteMask)
    concreteMask->set(offset);
}

void ObjectState::markByteSymbolic(unsigned offset) {
  if (!concreteMask)
    concreteMask = new BitArray(size, true);
  concreteMask->unset(offset);
}

void ObjectState::markByteUnflushed(unsigned offset) {
  if (flushMask)
    flushMask->set(offset);
}

void ObjectState::markByteFlushed(unsigned offset) {
  if (!flushMask) {
    flushMask = new BitArray(size, false);
  } else {
    flushMask->unset(offset);
  }
}

void ObjectState::setKnownSymbolic(unsigned offset, 
                                   Expr *value /* can be null */) {
  if (knownSymbolics) {
    knownSymbolics[offset] = value;
  } else {
    if (value) {
      knownSymbolics = new ref<Expr>[size];
      knownSymbolics[offset] = value;
    }
  }
}

/***/

ref<Expr> ObjectState::read8(unsigned offset, uint8_t &concreteResult,
                             const ConstraintManager *assumptions) const {
  concreteResult = concreteStore[offset];
  if (isByteConcrete(offset)) {
    return ConstantExpr::create(concreteStore[offset], Expr::Int8);
  } else if (isByteKnownSymbolic(offset)) {
    return knownSymbolics[offset];
  } else {
    assert(isByteFlushed(offset) && "unflushed byte without cache value");
    
    return ReadExpr::create(getUpdates(assumptions),
                            ConstantExpr::create(offset, Expr::Int32));
  }
}

ref<Expr> ObjectState::read8(ref<Expr> offset, unsigned concreteOffset, uint8_t &concreteResult,
                             const ConstraintManager *assumptions) const {
  assert(!isa<ConstantExpr>(offset) && "constant offset passed to symbolic read8");
  unsigned base, size;
  fastRangeCheckOffset(offset, &base, &size);
  flushRangeForRead(base, size);

  if (size>4096) {
    std::string allocInfo;
    object->getAllocInfo(allocInfo);
    klee_warning_once(0, "flushing %d bytes on read, may be slow and/or crash: %s", 
                      size,
                      allocInfo.c_str());
  }

  concreteResult = concreteStore[concreteOffset];
  return ReadExpr::create(getUpdates(assumptions), ZExtExpr::create(offset, Expr::Int32));
}

void ObjectState::writeConcrete8(unsigned offset, uint8_t value) {
  concreteStore[offset] = value;
}

void ObjectState::write8(unsigned offset, uint8_t value) {
  //assert(read_only == false && "writing to read-only object!");
  concreteStore[offset] = value;
  setKnownSymbolic(offset, 0);

  markByteConcrete(offset);
  markByteUnflushed(offset);
}

void ObjectState::write8(unsigned offset, ref<Expr> value, uint8_t concreteValue) {
  // can happen when ExtractExpr special cases
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(value)) {
    assert(CE->getAPValue() == concreteValue && "concreteValue does not match value!");
    write8(offset, (uint8_t) CE->getZExtValue(8));
  } else {
    concreteStore[offset] = concreteValue;
    setKnownSymbolic(offset, value.get());
      
    markByteSymbolic(offset);
    markByteUnflushed(offset);
  }
}

void ObjectState::write8(ref<Expr> offset, unsigned concreteOffset, ref<Expr> value, uint8_t concreteValue) {
  assert(!isa<ConstantExpr>(offset) && "constant offset passed to symbolic write8");
  unsigned base, size;
  fastRangeCheckOffset(offset, &base, &size);
  flushRangeForWrite(base, size);

  if (size>4096) {
    std::string allocInfo;
    object->getAllocInfo(allocInfo);
    klee_warning_once(0, "flushing %d bytes on write, may be slow and/or crash: %s",
                      size,
                      allocInfo.c_str());
  }

  concreteStore[concreteOffset] = concreteValue;
  updates.extend(ZExtExpr::create(offset, Expr::Int32), value);
}

/***/

ref<Expr> ObjectState::read(ref<Expr> offset, unsigned concreteOffset, Expr::Width width, llvm::APInt &concreteResult,
                            const ConstraintManager *assumptions) const {
  // Truncate offset to 32-bits.
  offset = ZExtExpr::create(offset, Expr::Int32);

  // Check for reads at constant offsets.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(offset)) {
    assert(CE->getZExtValue() == concreteOffset && "concreteOffset does not match offset!");
    return read(CE->getZExtValue(32), width, concreteResult, assumptions);
  }

  // Treat bool specially, it is the only non-byte sized write we allow.
  if (width == Expr::Bool) {
    uint8_t cres;
    ref<Expr> res = ExtractExpr::create(read8(offset, concreteOffset, cres, assumptions), 0, Expr::Bool);
    concreteResult = cres & 1;
    return res;
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = width / 8;
  assert(width == NumBytes * 8 && "Invalid read size!");
  ref<Expr> Res(0);
  concreteResult = APInt(width, 0);
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    unsigned caddr = concreteOffset + idx;
    uint8_t cbyte;
    ref<Expr> Byte = read8(AddExpr::create(offset, 
                                           ConstantExpr::create(idx, 
                                                                Expr::Int32)),
                           caddr,
                           cbyte,
                           assumptions);
    Res = i ? ConcatExpr::create(Byte, Res) : Byte;
    concreteResult |= APInt(width, cbyte).shl(8 * i);
  }

  return Res;
}

ref<Expr> ObjectState::read(unsigned offset, Expr::Width width, llvm::APInt &concreteResult,
                            const ConstraintManager *assumptions) const {
  // Treat bool specially, it is the only non-byte sized write we allow.
  if (width == Expr::Bool) {
    uint8_t cres;
    ref<Expr> res = ExtractExpr::create(read8(offset, cres, assumptions), 0, Expr::Bool);
    concreteResult = cres & 1;
    return res;
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = width / 8;
  assert(width == NumBytes * 8 && "Invalid width for read size!");
  ref<Expr> Res(0);
  concreteResult = APInt(width, 0);
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    uint8_t cbyte;
    ref<Expr> Byte = read8(offset + idx, cbyte, assumptions);
    Res = i ? ConcatExpr::create(Byte, Res) : Byte;
    concreteResult |= APInt(width, cbyte).shl(8 * i);
  }

  return Res;
}

void ObjectState::write(ref<Expr> offset, unsigned concreteOffset, ref<Expr> value, const llvm::APInt &concreteValue) {
  // Truncate offset to 32-bits.
  offset = ZExtExpr::create(offset, Expr::Int32);

  // Check for writes at constant offsets.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(offset)) {
    write(CE->getZExtValue(32), value, concreteValue);
    return;
  }

  // Treat bool specially, it is the only non-byte sized write we allow.
  Expr::Width w = value->getWidth();
  if (w == Expr::Bool) {
    write8(offset, concreteOffset, ZExtExpr::create(value, Expr::Int8), concreteValue.getZExtValue());
    return;
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = w / 8;
  assert(w == NumBytes * 8 && "Invalid write size!");
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    unsigned caddr = concreteOffset + idx;
    write8(AddExpr::create(offset, ConstantExpr::create(idx, Expr::Int32)),
           caddr,
           ExtractExpr::create(value, 8 * i, Expr::Int8),
           concreteValue.lshr(8 * i).zextOrTrunc(8).getZExtValue());
  }
}

void ObjectState::write(unsigned offset, ref<ConstantExpr> value) {
  write(offset, value, value->getAPValue());
}

void ObjectState::write(unsigned offset, ref<Expr> value, const llvm::APInt &concreteValue) {
  // Check for writes of constant values.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(value)) {
    assert(concreteValue == CE->getAPValue() && "concreteValue does not match value!");
    Expr::Width w = CE->getWidth();
    if (w <= 64 && klee::bits64::isPowerOfTwo(w)) {
      uint64_t val = CE->getZExtValue();
      switch (w) {
      default: llvm_unreachable("Invalid write size!");
      case  Expr::Bool:
      case  Expr::Int8:  write8(offset, val); return;
      case Expr::Int16: write16(offset, val); return;
      case Expr::Int32: write32(offset, val); return;
      case Expr::Int64: write64(offset, val); return;
      }
    }
  }

  // Treat bool specially, it is the only non-byte sized write we allow.
  Expr::Width w = value->getWidth();
  if (w == Expr::Bool) {
    write8(offset, ZExtExpr::create(value, Expr::Int8), concreteValue.getZExtValue());
    return;
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = w / 8;
  assert(w == NumBytes * 8 && "Invalid write size!");
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx,
           ExtractExpr::create(value, 8 * i, Expr::Int8),
           concreteValue.lshr(8 * i).zextOrTrunc(8).getZExtValue());
  }
}

void ObjectState::write16(unsigned offset, uint16_t value) {
  unsigned NumBytes = 2;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, (uint8_t) (value >> (8 * i)));
  }
}

void ObjectState::write32(unsigned offset, uint32_t value) {
  unsigned NumBytes = 4;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, (uint8_t) (value >> (8 * i)));
  }
}

void ObjectState::write64(unsigned offset, uint64_t value) {
  unsigned NumBytes = 8;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, (uint8_t) (value >> (8 * i)));
  }
}

void ObjectState::copyTo(ObjectState *os, unsigned count) const {
  for (unsigned i = 0; i < count; i++) {
    uint8_t cbyte;
    ref<Expr> byte = read8(i, cbyte);
    os->write8(i, byte, cbyte);
  }
}

ref<ConstantExpr> ObjectState::readConcrete(unsigned offset, Expr::Width width, llvm::APInt &concreteResult) {
  // Treat bool specially, it is the only non-byte sized write we allow.
  if (width == Expr::Bool) {
    uint64_t val = concreteStore[offset] & 1;
    concreteResult = APInt(width, val);
    return ConstantExpr::alloc(concreteResult);
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = width / 8;
  assert(width == NumBytes * 8 && "Invalid width for read size!");
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    concreteResult |= APInt(width, concreteStore[offset+idx]).shl(8 * i);
  }

  return ConstantExpr::alloc(concreteResult);
}

void ObjectState::writeConcrete(unsigned offset, const llvm::APInt &value) {
  Expr::Width w = value.getBitWidth();
  if (w <= 64 && klee::bits64::isPowerOfTwo(w)) {
    uint64_t val = value.getZExtValue();
    switch (w) {
      default: llvm_unreachable("Invalid write size!");
      case  Expr::Bool:
      case  Expr::Int8:  write8(offset, val); return;
      case Expr::Int16: write16(offset, val); return;
      case Expr::Int32: write32(offset, val); return;
      case Expr::Int64: write64(offset, val); return;
    }
  }

  unsigned NumBytes = w / 8;
  assert(w == NumBytes * 8 && "Invalid write size!");
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    writeConcrete8(offset + idx, value.lshr(8 * i).zextOrTrunc(8).getZExtValue());
  }
}

void ObjectState::overwriteConcretesFrom(const ObjectBytes &bytes) {
  assert(bytes.size == size && "attempted to overwrite concretes with wrong length!");
  memcpy(concreteStore, bytes.bytes, size);
}

void ObjectState::overwriteConcretesFrom(const uint8_t *bytes, unsigned length) {
  assert(length == size && "attempted to overwrite concretes with wrong length!");
  memcpy(concreteStore, bytes, length);
}

const uint8_t *ObjectState::allConcretes() const {
  return concreteStore;
}

void ObjectState::print() {
  llvm::errs() << "-- ObjectState --\n";
  llvm::errs() << "\tMemoryObject ID: " << object->id << "\n";
  llvm::errs() << "\tRoot Object: " << updates.root << "\n";
  llvm::errs() << "\tSize: " << size << "\n";

  llvm::errs() << "\tBytes:\n";
  for (unsigned i=0; i<size; i++) {
    llvm::errs() << "\t\t["<<i<<"]"
               << " concrete? " << isByteConcrete(i)
               << " known-sym? " << isByteKnownSymbolic(i)
               << " flushed? " << isByteFlushed(i) << " = ";
    uint8_t ce;
    ref<Expr> e = read8(i, ce);
    llvm::errs() << e << " (concrete = " << ce << ")\n";
  }

  llvm::errs() << "\tUpdates:\n";
  for (const UpdateNode *un=updates.head; un; un=un->next) {
    llvm::errs() << "\t\t[" << un->index << "] = " << un->value << "\n";
  }
}
