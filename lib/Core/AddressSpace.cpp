//===-- AddressSpace.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "AddressSpace.h"
#include "CoreStats.h"
#include "Memory.h"
#include "TimingSolver.h"

#include "klee/Expr.h"
#include "klee/TimerStatIncrementer.h"

#include <iostream>

using namespace klee;

AddressSpace::AddressSpace(const AddressSpace &b) : objects(b.objects), states(), arrayMap(b.arrayMap) {
  for (const ObjectPair &op : b.states) {
    const MemoryObject *mo = op.first;
    ObjectState *os = op.second;
    mo->refCount++;
    if (!os->readOnly)
      os = new ObjectState(*os, *this);
    os->claim();
    states[mo] = os;
  }
}

AddressSpace::~AddressSpace()  {
  for (const ObjectPair &op : states) {
    const MemoryObject *mo = op.first;
    const ObjectState *os = op.second;
    os->release();
    assert(mo->refCount > 0 && "memory object freed too many times!");
    mo->refCount--;
    if (mo->refCount == 0)
      delete mo;
  }
}

///

void AddressSpace::bindObject(const MemoryObject *mo, ObjectState *os) {
  assert(states.count(mo) == 0 && "memory object is already bound!");
  assert(objects.count(mo->address) == 0 && "memory object already at this address!");
  mo->refCount++;
  os->claim();
  objects[mo->address] = mo;
  states[mo] = os;
  const Array *array = os->updates.root;
  if (array)
    arrayMap[array] = os;
}

void AddressSpace::rebindObject(const MemoryObject *mo, ObjectState *os) {
  ObjectState *old = states[mo];
  if (old) {
    const Array *oldArray = old->updates.root;
    if (oldArray)
      arrayMap.erase(oldArray);
    old->release();
  }
  os->claim();
  states[mo] = os;
  const Array *array = os->updates.root;
  if (array)
    arrayMap[array] = os;
}

void AddressSpace::unbindObject(const MemoryObject *mo) {
  assert(objects.count(mo->address) > 0 && states.count(mo) > 0 && "memory object is not bound!");
  ObjectState *os = states[mo];
  states.erase(mo);
  const Array *array = os->updates.root;
  if (array)
    arrayMap.erase(array);
  os->release();

  objects.erase(mo->address);
  assert(mo->refCount > 0 && "memory object freed too many times!");
  mo->refCount--;
  if (mo->refCount == 0)
    delete mo;
}

void AddressSpace::backingArrayChanged(ObjectState *os, const Array *oldArray) {
  if (oldArray) {
    assert (arrayMap[oldArray] == os && "attempted to change Array for wrong ObjectState!");
    arrayMap.erase(oldArray);
  }
  assert (os->updates.root && "attempted to use NULL backing Array!");
  arrayMap[os->updates.root] = os;
}

ObjectState *AddressSpace::findObject(const MemoryObject *mo) {
  return states[mo];
}

ObjectState *AddressSpace::findBackedObject(const Array *array) {
  ArrayMap::iterator it = arrayMap.find(array);
  if (it == arrayMap.end())
    return 0;
  return it->second;
}

/// 


bool AddressSpace::resolveOne(const ref<ConstantExpr> &addr, 
                              ObjectPair &result) {
  uint64_t address = addr->getZExtValue();
  return resolveOne(address, result);
}

bool AddressSpace::resolveOne(uint64_t address,
                              ObjectPair &result) {
  AddressMap::const_iterator it = objects.lower_bound(address);    // find MO with largest address <= the given one
  if (it != objects.end()) {
    const MemoryObject *mo = it->second;
    // Check if the provided address is between start and end of the object
    // [mo->address, mo->address + mo->size) or the object is a 0-sized object.
    if ((mo->size==0 && address==mo->address) ||
        (address - mo->address < mo->size)) {
      result.first = mo;
      result.second = states[mo];
      return true;
    }
  }

  return false;
}

bool AddressSpace::resolve(ExecutionState &state,
                           TimingSolver *solver, 
                           const ref<Expr> &address,
                           uint64_t concreteAddress,
                           int bytes,
                           ResolutionList &rl,
                           ref<ConstantExpr> &outOfBoundsAddr,
                           unsigned maxResolutions,
                           double timeout) {
  if (ConstantExpr *CE __attribute__ ((unused)) = dyn_cast<ConstantExpr>(address)) {
    assert(CE->getZExtValue() == concreteAddress && "concreteAddress does not match address!");
    ObjectPair res;
    if (resolveOne(concreteAddress, res))
      rl.push_back(res);
    return false;
  } else {
    WallTimer timer;
    uint64_t timeout_us = (uint64_t) (timeout*1000000.);

    // We use a different resolution technique than KLEE: repeatedly finding an MO
    // the address could lie in and blocking it. This has the advantage of never
    // taking more queries than the number of objects the address can lie in. The
    // disadvantage is that we can't handle out-of-bounds pointers the same way:
    // it could take arbitrarily many iterations to find all valid resolutions.
    // So we terminate as soon as we discover a single out-of-bounds resolution.
    // This means we could miss further bugs arising from valid resolutions. But
    // since users will probably re-run the tool after fixing the out-of-bounds
    // bug anyway, this isn't much of a problem.
    ref<ConstantExpr> badAddr;
    auto handler = [&](const ref<Expr> &target, const ref<ConstantExpr> &value) {
      ref<Expr> blocker;
      ObjectPair op;
      if (resolveOne(value, op)) {    // the found value is inside an MO
        const MemoryObject *mo = op.first;
        uint64_t cvalue = value->getZExtValue(64);
        ref<Expr> inBounds;
        bool cinBounds;
        // check whether the value points to at least the required number of valid bytes
        if (bytes == -1) {
          inBounds = EqExpr::create(target, mo->getBaseExpr());
          cinBounds = (cvalue == mo->address);
        } else {
          inBounds = mo->getBoundsCheckPointer(target, bytes);
          cinBounds = mo->boundsCheckPointer(cvalue, bytes);
        }
        if (cinBounds) {    // found an example proving the pointer can validly resolve to this MO
          rl.push_back(op);
          if (!maxResolutions || rl.size() < maxResolutions)    // don't continue searching if we've hit maxResolutions
            blocker = Expr::createIsZero(inBounds);
        } else {    // found an example proving the pointer can resolve to this MO but out-of-bounds
          outOfBoundsAddr = value;
        }
      } else {    // found an example proving the pointer can resolve outside of any MO
        outOfBoundsAddr = value;
      }
      if (timeout_us && timeout_us < timer.check())   // don't continue searching if we've hit the timeout
        return ref<Expr>();
      return blocker;
    };

    bool complete = solver->getAllValues(state, address, handler);

    // To make forks deterministic, sort resolutions
    std::sort(rl.begin(), rl.end(),
              [](const ObjectPair &a, const ObjectPair &b) {
                return a.first->id < b.first->id;
              });

    return !complete;
  }
}

// These two are pretty big hack so we can sort of pass memory back
// and forth to externals. They work by abusing the concrete cache
// store inside of the object states, which allows them to
// transparently avoid screwing up symbolics (if the byte is symbolic
// then its concrete cache byte isn't being used) but is just a hack.

void AddressSpace::copyOutConcretes() {
  for (const ObjectPair &op : states) {
    const MemoryObject *mo = op.first;

    if (!mo->isUserSpecified) {
      ObjectState *os = op.second;
      uint8_t *address = reinterpret_cast<uint8_t*>(mo->address);

      if (!os->readOnly)
        memcpy(address, os->concreteStore, mo->size);
    }
  }
}

bool AddressSpace::copyInConcretes() {
  for (const ObjectPair &op : states) {
    const MemoryObject *mo = op.first;

    if (!mo->isUserSpecified) {
      ObjectState *os = op.second;
      uint8_t *address = reinterpret_cast<uint8_t*>(mo->address);

      if (memcmp(address, os->concreteStore, mo->size)!=0) {
        if (os->readOnly) {
          return false;
        } else {
          memcpy(os->concreteStore, address, mo->size);
        }
      }
    }
  }

  return true;
}

/***/

bool MemoryObjectLT::operator()(const MemoryObject *a, const MemoryObject *b) const {
  return a->address < b->address;
}

