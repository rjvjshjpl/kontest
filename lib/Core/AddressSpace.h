//===-- AddressSpace.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_ADDRESSSPACE_H
#define KLEE_ADDRESSSPACE_H

#include "ObjectHolder.h"

#include "klee/Expr.h"

#include <map>
#include <unordered_map>
#include <functional>

namespace klee {
  class ExecutionState;
  class MemoryObject;
  class ObjectState;
  class TimingSolver;

  template<class T> class ref;

  typedef std::pair<const MemoryObject*, ObjectState*> ObjectPair;
  typedef std::vector<ObjectPair> ResolutionList;  

  /// Function object ordering MemoryObject's by address.
  struct MemoryObjectLT {
    bool operator()(const MemoryObject *a, const MemoryObject *b) const;
  };

  typedef std::map< uint64_t, const MemoryObject*, std::greater<uint64_t> > AddressMap;
  typedef std::unordered_map<const MemoryObject*, ObjectState*> MemoryMap;
  typedef std::unordered_map<const Array*, ObjectState*> ArrayMap;
  
  class AddressSpace {
  private:
    /// Unsupported, use copy constructor
    AddressSpace &operator=(const AddressSpace&); 
    
  public:
    AddressMap objects;
    MemoryMap states;
    ArrayMap arrayMap;
    
  public:
    AddressSpace() {}
    AddressSpace(const AddressSpace &b);
    ~AddressSpace();


    /// Resolve address to an ObjectPair in result.
    /// \return true iff an object was found.
    bool resolveOne(const ref<ConstantExpr> &address, 
                    ObjectPair &result);
    bool resolveOne(uint64_t address,
                    ObjectPair &result);

    /// Resolve address to a list of ObjectPairs it can point within
    /// yielding at least \ref bytes of valid memory (with -1 indicating
    /// that we require a pointer to the start of the object). If
    /// maxResolutions is non-zero then no more than that many pairs will
    /// be returned.
    ///
    /// \return true iff the resolution is incomplete (maxResolutions
    /// is non-zero and the search terminated early, or a query timed out).
    bool resolve(ExecutionState &state,
                 TimingSolver *solver,
                 const ref<Expr> &address,
                 uint64_t concreteAddress,
                 int bytes,
                 ResolutionList &rl,
                 ref<ConstantExpr> &outOfBoundsAddr,
                 unsigned maxResolutions=0,
                 double timeout=0.);

    /***/

    /// Add a binding to the address space.
    void bindObject(const MemoryObject *mo, ObjectState *os);

    /// Change a binding in the address space.
    void rebindObject(const MemoryObject *mo, ObjectState *os);

    /// Remove a binding from the address space.
    void unbindObject(const MemoryObject *mo);

    /// Update the Array associated with an ObjectState
    void backingArrayChanged(ObjectState *os, const Array *oldArray);

    /// Look up a binding from a MemoryObject.
    ObjectState *findObject(const MemoryObject *mo);

    /// Look up a binding from an Array
    ObjectState *findBackedObject(const Array *array);

    /// Copy the concrete values of all managed ObjectStates into the
    /// actual system memory location they were allocated at.
    void copyOutConcretes();

    /// Copy the concrete values of all managed ObjectStates back from
    /// the actual system memory location they were allocated
    /// at. ObjectStates will only be written to (and thus,
    /// potentially copied) if the memory values are different from
    /// the current concrete values.
    ///
    /// \retval true The copy succeeded. 
    /// \retval false The copy failed because a read-only object was modified.
    bool copyInConcretes();
  };
} // End klee namespace

#endif
