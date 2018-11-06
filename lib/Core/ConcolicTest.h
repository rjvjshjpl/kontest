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

#ifndef KLEE_CONCOLICTEST_H
#define KLEE_CONCOLICTEST_H

#include "klee/Internal/ADT/KTest.h"
#include "klee/Internal/ADT/ReverseTrie.h"
#include "klee/ExecutionState.h"

namespace klee {
  struct ObjectBytes {
    unsigned size;
    unsigned char *bytes;

    ObjectBytes(const unsigned char *source, unsigned length) : size(length) {
      bytes = new unsigned char[size];
      memcpy(bytes, source, size);
    }

    ObjectBytes(const std::vector<unsigned char> &source) : size(source.size()) {
      bytes = new unsigned char[size];
      std::copy(source.begin(), source.end(), bytes);
    }

    bool operator==(const ObjectBytes &other) const {
      if (size != other.size)
        return false;
      return (memcmp(bytes, other.bytes, size) == 0);
    }

    ~ObjectBytes() {
      delete[] bytes;
    }
  };

  // Version of KTestObject that admits caching
  struct ConcolicTestObject {
    const std::string *name;
    const ObjectBytes *bytes;
  };

  struct ConcolicTest {
    std::vector<ConcolicTestObject> objects;

    std::vector<bool> expectedPath;   // expected branches to take
    ReverseTrie<unsigned>::Element expectedResolutions;   // expected resolutions of pointers; UINT_MAX indicates no resolution
    ReverseTrie<unsigned>::Element expectedOffsets;    // expected offsets of pointers; only used if symbolic addresses are disabled
    ReverseTrie<uint64_t>::Element expectedFunctionResolutions;    // expected resolutions of function pointers
    ReverseTrie<unsigned>::Element expectedAllocations;    // expected allocation sizes

    // Depths at which this test was created or diverged from expectations;
    // should be equal to the lengths of the corresponding expectations until
    // the latter are cleared after concrete execution
    unsigned branchDepth;
    unsigned resolutionDepth, offsetDepth;
    unsigned functionResolutionDepth;
    unsigned allocationDepth;

    //int memoryCounter;
    //ExecutionState *startState;

    int newBlocks;
    float predictionScore;
    unsigned generation;
    bool isReplacement, endedInError, alreadyRun, needsProcessing;
    unsigned testID, kTestID;

    static unsigned nextTestID;

    ConcolicTest(const ReverseTrie<unsigned>::Element &resolutions,
                 const ReverseTrie<unsigned>::Element &offsets,
                 const ReverseTrie<uint64_t>::Element &functionResolutions,
                 const ReverseTrie<unsigned>::Element &allocations) :
        expectedResolutions(resolutions),
        expectedOffsets(offsets),
        expectedFunctionResolutions(functionResolutions),
        expectedAllocations(allocations),
        branchDepth(expectedPath.size()),
        resolutionDepth(expectedResolutions.size()),
        offsetDepth(expectedOffsets.size()),
        functionResolutionDepth(expectedFunctionResolutions.size()),
        allocationDepth(expectedAllocations.size()),
        newBlocks(0),
        predictionScore(-1),
        generation(0),
        isReplacement(false), endedInError(false), alreadyRun(false), needsProcessing(false),
        testID(nextTestID++),
        kTestID(0) {}

    bool operator<(const ConcolicTest &other) const {
      // Lexicographic order on:
      // 1. having ended in error (such a test is automatically interesting;
      // also the user may want us to dump queries for it)
      if (endedInError != other.endedInError)
        return !endedInError;

      // 2. being a replacement
      if (isReplacement != other.isReplacement)
        return !isReplacement;

      // 3. main score: number of new blocks covered
      if (newBlocks != other.newBlocks)
        return newBlocks < other.newBlocks;

      // 4. prediction score (currently unused)
      if (predictionScore != other.predictionScore)
        return predictionScore < other.predictionScore;

      // 5. ID (ensures determinism; oldest first)
      assert(testID != other.testID && "two tests have the same ID!");
      return testID > other.testID;
    }

    void pushBranch(bool branch) {
      assert(expectedPath.size() == branchDepth && "tried to grow erased expectedPath!");
      expectedPath.push_back(branch);
      ++branchDepth;
    }

    void pushResolution(unsigned ID) {
      assert(expectedResolutions.size() == resolutionDepth && "tried to grow erased expectedResolutions!");
      expectedResolutions.push_back(ID);
      ++resolutionDepth;
    }

    void pushOffset(unsigned offset) {
      assert(expectedOffsets.size() == offsetDepth && "tried to grow erased expectedOffsets!");
      expectedOffsets.push_back(offset);
      ++offsetDepth;
    }

    void pushFunctionResolution(uint64_t ID) {
      assert(expectedFunctionResolutions.size() == functionResolutionDepth
             && "tried to grow erased expectedFunctionResolutions!");
      expectedFunctionResolutions.push_back(ID);
      ++functionResolutionDepth;
    }

    void pushAllocation(unsigned size) {
      assert(expectedAllocations.size() == allocationDepth && "tried to grow erased expectedAllocations!");
      expectedAllocations.push_back(size);
      ++allocationDepth;
    }

    void truncateExpectationsTo(const ExecutionState &state,
                                const ReverseTrie<unsigned>::Path &er,
                                const ReverseTrie<unsigned>::Path &eo,
                                const ReverseTrie<uint64_t>::Path &efr,
                                const ReverseTrie<unsigned>::Path &ea) {
      unsigned objectDepth = state.symbolics.size();
      objects.resize(objectDepth);

      branchDepth = state.forkHistory.size();
      resolutionDepth = state.resolutionHistory.size();
      offsetDepth = state.offsetHistory.size();
      functionResolutionDepth = state.functionResolutionHistory.size();
      allocationDepth = state.allocationHistory.size();

      expectedPath.resize(branchDepth);
      expectedResolutions = er.prefix(resolutionDepth);
      expectedOffsets = eo.prefix(offsetDepth);
      expectedFunctionResolutions = efr.prefix(functionResolutionDepth);
      expectedAllocations = ea.prefix(allocationDepth);
    }

    void dump() const;
    void dumpArgs() const;
  };
}

namespace std
{
  // hash function for ObjectBytes
  template<>
  struct hash< klee::ObjectBytes >
  {
    std::size_t operator()(const klee::ObjectBytes &p) const {
      std::size_t val = 0;
      for (unsigned i = 0; i < p.size; i++)
        val ^= p.bytes[i] + 0x9e3779b9 + (val << 6) + (val >> 2);   // magic numbers a la Boost
      return val;
    }
  };
}

#endif //KLEE_CONCOLICTEST_H
