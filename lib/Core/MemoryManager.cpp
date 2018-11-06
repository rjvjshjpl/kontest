//===-- MemoryManager.cpp -------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "CoreStats.h"
#include "Memory.h"
#include "MemoryManager.h"

#include "klee/Internal/Support/ErrorHandling.h"

#include "llvm/Support/CommandLine.h"
#include "llvm/Support/MathExtras.h"

#include <inttypes.h>
#include <sys/mman.h>

using namespace klee;

namespace {
llvm::cl::opt<bool> DeterministicAllocation(
    "allocate-determ",
    llvm::cl::desc("Allocate memory deterministically(default=off)"),
    llvm::cl::init(false));

llvm::cl::opt<unsigned> DeterministicAllocationSize(
    "allocate-determ-size",
    llvm::cl::desc(
        "Preallocated memory for deterministic allocation in MB (default=100)"),
    llvm::cl::init(100));

llvm::cl::opt<bool>
    NullOnZeroMalloc("return-null-on-zero-malloc",
                     llvm::cl::desc("Returns NULL in case malloc(size) was "
                                    "called with size 0 (default=off)."),
                     llvm::cl::init(false));

llvm::cl::opt<unsigned> RedZoneSpace(
    "red-zone-space",
    llvm::cl::desc("Set the amount of free space between allocations. This is "
                   "important to detect out-of-bound accesses (default=10)."),
    llvm::cl::init(10));

llvm::cl::opt<unsigned long long> DeterministicStartAddress(
    "allocate-determ-start-address",
    llvm::cl::desc("Start address for deterministic allocation. Has to be page "
                   "aligned (default=0x7ff30000000)."),
    llvm::cl::init(0x7ff30000000));
}

/***/
MemoryManager::MemoryManager(ArrayCache *_arrayCache)
    : arrayCache(_arrayCache), deterministicSpace(0), nextFreeSlot(0),
      spaceSize(DeterministicAllocationSize.getValue() * 1024 * 1024),
      targetPointerWidth(Context::get().getPointerWidth()),
      nullPointer(ConstantExpr::alloc(0, targetPointerWidth)) {
  if (targetPointerWidth != Expr::Int32 && targetPointerWidth != Expr::Int64)
    klee_error("Target program uses neither 32-bit nor 64-bit pointers");

  if (DeterministicAllocation) {
    // Page boundary
    void *expectedAddress = (void *)DeterministicStartAddress.getValue();

    // Ensure everything fits if we need to be in 32-bit mode
    int flags = MAP_ANONYMOUS | MAP_PRIVATE;
    if (targetPointerWidth == Expr::Int32) {
      uint64_t maxSize = 0x40000000;   // MAP_32BIT can only map this much
      uint64_t requiredSize = spaceSize + RedZoneSpace;
      if (requiredSize >= maxSize)
        klee_error("Preallocated memory too large for 32-bit mode (must be <1G)");
      uint64_t tooHigh = 0xFFFFFFFF - requiredSize;
      if (reinterpret_cast<uint64_t>(expectedAddress) > tooHigh) {
        expectedAddress = 0;   // let the OS pick the address
        if (DeterministicStartAddress.getNumOccurrences() > 0)
          klee_warning("Ignoring allocate-determ-start-address too large for 32-bit mode");
      }
#ifdef MAP_32BIT
      flags |= MAP_32BIT;   // ensure pointers to the memory fit in 32 bits
#elif defined(ENABLE_DARWIN_32BIT)
      if (expectedAddress == 0)
        expectedAddress = (void *) 0x40000000;
#else
      klee_error("32-bit deterministic allocation not supported on this platform");
#endif
    }

    char *newSpace =
        (char *)mmap(expectedAddress, spaceSize, PROT_READ | PROT_WRITE,
                     flags, -1, 0);

    if (newSpace == MAP_FAILED) {
      klee_error("Couldn't mmap() memory for deterministic allocations");
    }
    if (expectedAddress != newSpace && expectedAddress != 0) {
      klee_error("Could not allocate memory deterministically");
    }
    if (targetPointerWidth == Expr::Int32) {
      // check memory actually can be addressed with 32 bits, in case someone
      // builds this on a system that ignores MAP_32BIT (this would otherwise
      // cause mystifying assertion failures or spurious errors later)
      uint64_t tooHigh = 0xFFFFFFFF - spaceSize;
      if (reinterpret_cast<uint64_t>(newSpace) > tooHigh)
        klee_error("unable to mmap() memory in the 32-bit address space");
    }

    klee_message("Deterministic memory allocation starting from %p", newSpace);
    deterministicSpace = newSpace;
    nextFreeSlot = newSpace;
  } else {
    // using system malloc to allocate memory
    if (targetPointerWidth == Expr::Int32)
      klee_error("Testing 32-bit programs requires allocate-determ");
  }
}

MemoryManager::~MemoryManager() {
  while (!objects.empty()) {
    MemoryObject *mo = *objects.begin();
    if (!mo->isFixed && !DeterministicAllocation)
      free((void *)mo->address);
    objects.erase(mo);
    delete mo;
  }

  if (DeterministicAllocation)
    munmap(deterministicSpace, spaceSize);
}

MemoryObject *MemoryManager::allocate(uint64_t size, bool isLocal,
                                      bool isGlobal,
                                      const llvm::Value *allocSite,
                                      size_t alignment) {
  if (size > 10 * 1024 * 1024)
    klee_warning_once(0, "Large alloc: %" PRIu64
                         " bytes.  KLEE may run out of memory.",
                      size);

  // Return NULL if size is zero, this is equal to error during allocation
  if (NullOnZeroMalloc && size == 0)
    return 0;

  if (!llvm::isPowerOf2_64(alignment)) {
    klee_warning("Only alignment of power of two is supported");
    return 0;
  }

  uint64_t address = 0;
  if (DeterministicAllocation) {

    address = llvm::RoundUpToAlignment((uint64_t)nextFreeSlot + alignment - 1,
                                       alignment);

    // Handle the case of 0-sized allocations as 1-byte allocations.
    // This way, we make sure we have this allocation between its own red zones
    size_t alloc_size = std::max(size, (uint64_t)1);
    if ((char *)address + alloc_size < deterministicSpace + spaceSize) {
      nextFreeSlot = (char *)address + alloc_size + RedZoneSpace;
    } else {
      klee_warning_once(0, "Couldn't allocate %" PRIu64
                           " bytes. Not enough deterministic space left.",
                        size);
      address = 0;
    }
  } else {
    // Use malloc for the standard case
    if (alignment <= 8)
      address = (uint64_t)malloc(size);
    else {
      int res = posix_memalign((void **)&address, alignment, size);
      if (res < 0) {
        klee_warning("Allocating aligned memory failed.");
        address = 0;
      }
    }
  }

  if (!address)
    return 0;

  ++stats::allocations;
  MemoryObject *res = new MemoryObject(address, size, isLocal, isGlobal, false,
                                       allocSite, this);
  objects.insert(res);
  return res;
}

MemoryObject *MemoryManager::allocateFixed(uint64_t address, uint64_t size,
                                           const llvm::Value *allocSite) {
#ifndef NDEBUG
  for (const MemoryObject *mo : objects) {
    if (address + size > mo->address && address < mo->address + mo->size)
      klee_error("Trying to allocate an overlapping object");
  }
#endif

  ++stats::allocations;
  MemoryObject *res =
      new MemoryObject(address, size, false, true, true, allocSite, this);
  objects.insert(res);
  return res;
}

void MemoryManager::deallocate(const MemoryObject *mo) { assert(0); }

void MemoryManager::markFreed(MemoryObject *mo) {
  if (objects.erase(mo)) {    // object has not yet been freed
    if (!mo->isFixed && !DeterministicAllocation)
      free((void *)mo->address);
  }
}

size_t MemoryManager::getUsedDeterministicSize() {
  return nextFreeSlot - deterministicSpace;
}

MemoryManager::ResetToken MemoryManager::getResetToken() const {
  return std::make_pair(nextFreeSlot, objects.size());
}

void MemoryManager::reset(ResetToken token) {
  nextFreeSlot = token.first;
  assert(objects.size() == token.second && "a MemoryObject was not freed correctly!");
}