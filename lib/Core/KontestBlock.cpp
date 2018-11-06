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
// Created by dfremont on 8/14/17.
//

#include <klee/ExecutionState.h>
#include "KontestBlock.h"
#include "ScenarioHandler.h"
#include <llvm/Support/CommandLine.h>
#include <klee/Internal/ADT/RNG.h>

#include <cassert>
#include <vector>
#include <algorithm>

using namespace llvm;

namespace {
  enum SkeletonGenerationMethod {
    eSkeletonRandom,
    eSkeletonSystematic
  };

  cl::opt<SkeletonGenerationMethod>
  SkeletonMode("skeletons", cl::desc("Select how skeletons are generated from a scenario:"),
               cl::values(clEnumValN(eSkeletonRandom, "random", "randomly (based on --random-seed)"),
                          clEnumValN(eSkeletonSystematic, "systematic",
                                     "systematically enumerating all possibilities (default)"),
                          clEnumValEnd),
               cl::init(eSkeletonSystematic));
};

namespace klee {
  extern RNG theRNG;

  /**
   * General
   */

  bool KontestBlock::generate(bool firstSkeleton) {
    if (firstSkeleton || (SkeletonMode == eSkeletonSystematic && !ready)) {
      generateFirst();
      return false;
    } else if (SkeletonMode == eSkeletonSystematic) {
      return generateNext();
    } else {
      generateRandom();
      return false;
    }
  }

  void KontestBlock::generateFirst() {
    for (KontestBlock *child : children)
      child->generateFirst();
    ready = true;
  }

  bool KontestBlock::generateNext() {
    for (KontestBlock *child : children) {
      bool cycled = child->generate(false);
      if (!cycled)
        return false;
    }
    return true;
  }

  void KontestBlock::generateRandom() {
    for (KontestBlock *child : children)
      child->generateRandom();
  }

  void KontestBlock::reset() {
    clearSymbolics();
    SCENARIO_DEBUG_MSG("reset");
  }

  /**
    * SCENARIO blocks
    */

  void ScenarioBlock::finishParsing(ExecutionState &state) {
    if (children.size() == 0) {
      ScenarioHandler::locatedParseError("scenario contains no actions");
      return;
    }
    // sequential composition of children
  }

  void ScenarioBlock::validate(BlockType type, uint64_t id,
                               const ref<Expr> &data1, const ref<Expr> &data2,
                               const std::string &name, ExecutionState &state) const {
    if (type != BlockType::SCENARIO)
      ScenarioHandler::locatedParseError("unexpected type of block; did you put an if inside a scenario?");
    if (name != scenario->name)
      ScenarioHandler::locatedParseError("unexpected scenario; did you put an if inside a scenario?");

    assert(id == scenario->identifier && "a scenario has two different identifiers!");
  }

/**
 * ACTION blocks
 */

  void ActionBlock::finishParsing(ExecutionState &state) {
    // sequential composition of children
  }

  void ActionBlock::validate(BlockType type, uint64_t id,
                           const ref<Expr> &data1, const ref<Expr> &data2,
                           const std::string &name, ExecutionState &state) const {
    if (type != BlockType::ACTION)
      ScenarioHandler::locatedParseError("unexpected type of block; did you put an if inside a scenario?");
    if (name != action->name)
      ScenarioHandler::locatedParseError("unexpected action; did you put an if inside a scenario?");

    assert(id == action->identifier && "an action has two different identifiers!");
  }

  void ActionBlock::finish() {
    stepping = false;   // actions always complete in one step
  }

  unsigned ActionBlock::actionsNeeded() const {
    return 1;
  }

/**
 * REPEAT blocks
 */

  bool RepeatBlock::loopParse() {
    nextChild = 0;
    return (++loopCount <= 1);    // single loop to parse children
  }

  void RepeatBlock::finishParsing(ExecutionState &state) {
    if (children.size() == 0) {
      ScenarioHandler::locatedParseError("empty REPEAT block");
      return;
    }
    std::swap(children, originalChildren);

    // repetition of sequential composition of children
  }

  void RepeatBlock::validate(BlockType type, uint64_t id,
                             const ref<Expr> &data1, const ref<Expr> &data2,
                             const std::string &name, ExecutionState &state) const {
    if (type != BlockType::REPEAT)
      ScenarioHandler::locatedParseError("unexpected type of block; did you put an if inside a scenario?");
    ConstantExpr *cmin = dyn_cast<ConstantExpr>(data1);
    ConstantExpr *cmax = dyn_cast<ConstantExpr>(data2);
    if (!cmin || !cmax || cmin->getZExtValue() != min || cmax->getZExtValue() != max)
      ScenarioHandler::locatedParseError("non-constant range used in REPEAT block");
  }

  void RepeatBlock::regenerateChildren() {
    for (KontestBlock *child : children)
      delete child;
    children.clear();
    children.reserve(selectedRepetitions * originalChildren.size());
    for (unsigned i = 0; i < selectedRepetitions; i++) {
      for (KontestBlock *child : originalChildren)
        children.push_back(child->clone());
    }
  }

  void RepeatBlock::generateFirst() {
    selectedRepetitions = min;
    regenerateChildren();
    KontestBlock::generateFirst();
  }

  bool RepeatBlock::generateNext() {
    if (!KontestBlock::generateNext())    // try generating new sequence for children
      return false;

    // Children have cycled; pick new number of repetitions
    if (min == max)
      return true;
    bool cycled = false;
    if (selectedRepetitions == min) {
      selectedRepetitions = max;
    } else {
      --selectedRepetitions;
      assert(selectedRepetitions >= min && "REPEAT generator broke!");
      if (selectedRepetitions == min)
        cycled = true;
    }

    regenerateChildren();
    for (KontestBlock *child : children)    // generate all newly-cloned children
      child->generate(false);

    return cycled;
  }

  void RepeatBlock::generateRandom() {
    unsigned oldReps = selectedRepetitions;
    selectedRepetitions = min + (theRNG.getInt32() % (max - min + 1));    // close enough to uniform
    if (selectedRepetitions != oldReps)
      regenerateChildren();
    KontestBlock::generateRandom();
  }

  bool RepeatBlock::run(bool stepping) {
    KontestBlock::run(stepping);
    haveLooped = false;
    return true;
  }

  bool RepeatBlock::loop() {
    if (stepping) {
      if (haveLooped)   // if stepping, terminate after one iteration
        return false;
      haveLooped = true;
      return true;
    }
    return (++loopCount <= selectedRepetitions);
  }

  void RepeatBlock::finish() {
    if (stepping) {
      if (activeChildFinished) {
        // when block is next run, trigger the next child
        ++activeChild;
        if (activeChild == originalChildren.size()) {
          ++loopCount;
          activeChild = 0;
          resetDescendants();
          clearSymbolics();
          if (loopCount == selectedRepetitions) {
            stepping = false;
            reset();
          }
        }
      }
    } else {
      reset();
    }
  }

  void RepeatBlock::reset() {
    KontestBlock::reset();
    loopCount = 0;
  }

/**
 * ANY blocks
 */

  void AnyBlock::finishParsing(ExecutionState &state) {
    if (children.size() == 0) {
      ScenarioHandler::locatedParseError("empty ANY block");
      return;
    }
    // union of children
  }

  void AnyBlock::validate(BlockType type, uint64_t id,
                          const ref<Expr> &data1, const ref<Expr> &data2,
                          const std::string &name, ExecutionState &state) const {
    if (type != BlockType::ANY)
      ScenarioHandler::locatedParseError("unexpected type of block; did you put an if inside a scenario?");
  }

  void AnyBlock::generateFirst() {
    selectedChild = 0;
    children[selectedChild]->generateFirst();
    ready = true;
  }

  bool AnyBlock::generateNext() {
    if (!children[selectedChild]->generateNext())    // try generating new sequence for child
      return false;

    bool cycled = (++selectedChild == children.size());
    if (cycled)
      selectedChild = 0;
    children[selectedChild]->generateFirst();

    return cycled;
  }

  void AnyBlock::generateRandom() {
    selectedChild = theRNG.getInt32() % children.size();    // close enough to uniform
    children[selectedChild]->generateRandom();    // generate new sequence for child
  }

  bool AnyBlock::runNextChild() {
    KontestBlock *child = getNextChild();
    bool retval;
    if (nextChild != selectedChild) {
      assert(selectedChild < children.size() && "invalid selected child!");
      retval = child->skip();
    } else {
      retval = child->run(stepping);
    }
    return retval;
  }

  unsigned AnyBlock::actionsNeeded() const {
    return children[selectedChild]->actionsNeeded();
  }

/**
 * INTERLEAVE blocks
 */

  bool InterleaveBlock::loopParse() {
    nextChild = 0;
    return (++loopCount <= 1);    // single loop to parse children
  }

  void InterleaveBlock::finishParsing(ExecutionState &state) {
    if (children.size() < 2) {
      ScenarioHandler::locatedParseError("INTERLEAVE block with fewer than 2 children");
      return;
    }

    // parallel composition of children
  }

  void InterleaveBlock::validate(BlockType type, uint64_t id,
                                 const ref<Expr> &data1, const ref<Expr> &data2,
                                 const std::string &name, ExecutionState &state) const {
    if (type != BlockType::INTERLEAVE)
      ScenarioHandler::locatedParseError("unexpected type of block; did you put an if inside a scenario?");
  }

  void InterleaveBlock::generateFirst() {
    KontestBlock::generateFirst();    // generate first sequence for children
    assert(selectedInterleaving.size() == 0 && "INTERLEAVE block initially generated twice!");
    // initial interleaving is sequential in child order
    for (unsigned i = 0; i < children.size(); i++) {
      const unsigned steps = children[i]->actionsNeeded();
      for (unsigned j = 0; j < steps; j++)
        selectedInterleaving.push_back(i);
    }
  }

  bool InterleaveBlock::generateNext() {
    // The usual order is inverted here: we try all possible interleavings
    // before asking our children to generate new sequences (since changing
    // the children could change the number of steps needed)

    bool cycled = nextPermutation();
    if (!cycled)
      return false;

    // finished all interleavings; generate new sequence for children
    bool retval = KontestBlock::generateNext();

    // set up initial interleaving for new sequence
    selectedInterleaving.clear();
    for (unsigned i = 0; i < children.size(); i++) {
      const unsigned steps = children[i]->actionsNeeded();
      for (unsigned j = 0; j < steps; j++)
        selectedInterleaving.push_back(i);
    }

    return retval;
  }

  void InterleaveBlock::generateRandom() {
    // generate new sequence for children
    KontestBlock::generateRandom();

    // set up initial interleaving for new sequence
    selectedInterleaving.clear();
    for (unsigned i = 0; i < children.size(); i++) {
      const unsigned steps = children[i]->actionsNeeded();
      for (unsigned j = 0; j < steps; j++)
        selectedInterleaving.push_back(i);
    }

    // generate random interleaving
    randomPermutation();
  }

  bool InterleaveBlock::nextPermutation() {
    std::vector<unsigned> &perm = selectedInterleaving;
    assert(perm.size() > 0 && "tried to find next permutation of empty sequence!");
    const unsigned li = perm.size() - 1;
    unsigned k = li;
    for (unsigned i = 0; i < li; i++) {
      if (perm[i] < perm[i + 1])
        k = i;
    }
    if (k == li)    // already at last permutation
      return true;
    unsigned l = 0;
    for (unsigned i = k + 1; i <= li; i++) {
      if (perm[k] < perm[i])
        l = i;
    }
    assert(l > k && "nextPermutation failed!");
    std::swap(perm[k], perm[l]);
    std::reverse(perm.begin() + k + 1, perm.end());
    return false;
  }

  void InterleaveBlock::randomPermutation() {
    std::vector<unsigned> &perm = selectedInterleaving;
    for (unsigned i = perm.size(); i > 1; i--) {
      const unsigned choice = theRNG.getInt32() % i;    // close enough to uniform
      std::swap(perm[choice], perm[i - 1]);
    }
  }

  bool InterleaveBlock::run(bool stepping) {
    KontestBlock::run(stepping);
    haveLooped = false;
    return true;
  }

  bool InterleaveBlock::runNextChild() {
    KontestBlock *child = getNextChild();
    bool retval;
    if (nextChild != activeChild) {
      assert(activeChild < children.size() && "invalid active child!");
      retval = child->skip();
    } else {
      retval = child->run(true);
    }
    return retval;
  }

  bool InterleaveBlock::loop() {
    nextChild = 0;
    if (stepping && haveLooped)   // if stepping, terminate after one iteration
      return false;
    haveLooped = true;
    assert(loopCount <= selectedInterleaving.size() && "INTERLEAVE block loop failed!");
    if (loopCount == selectedInterleaving.size()) {
      return false;
    } else {
      if (!stepping)
        activeChild = selectedInterleaving[loopCount];
      ++loopCount;
      return true;
    }
  }

  void InterleaveBlock::finish() {
    if (stepping) {
      // when block is next run, trigger the next child
      if (loopCount == selectedInterleaving.size()) {
        stepping = false;
        reset();
      } else {
        activeChild = selectedInterleaving[loopCount];
      }
    } else {
      reset();
    }
  }

  void InterleaveBlock::reset() {
    KontestBlock::reset();
    loopCount = 0;
    activeChild = selectedInterleaving[0];
  }

};
