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

#ifndef KLEE_KONTESTBLOCK_H
#define KLEE_KONTESTBLOCK_H

#include "Memory.h"

#include "llvm/Support/ErrorHandling.h"

namespace klee {
  class Action;
  class Scenario;

  enum class BlockType : unsigned {
    ACTION,
    SCENARIO,
    REPEAT,
    ANY,
    INTERLEAVE,
    INVALID_TYPE
  };

  class KontestBlock {
  protected:
    std::vector<KontestBlock*> children;
    unsigned nextChild, activeChild;
    bool ready, active, stepping, activeChildFinished;
    std::vector<ObjectState*> symbolics;
    unsigned nextSymbolic;

  public:
    mutable KontestBlock *cloned;

  public:
    KontestBlock() : children(), nextChild(0), activeChild(0), ready(false), active(true), stepping(false),
                     symbolics(), nextSymbolic(0) {};

    KontestBlock(const KontestBlock &b) : children(), nextChild(b.nextChild), activeChild(b.activeChild),
                                          ready(b.ready), active(b.active), stepping(b.stepping),
                                          activeChildFinished(b.activeChildFinished), symbolics(), nextSymbolic(0) {
      assert(b.symbolics.empty() && "cloning block with cached symbolics!");
      children.reserve(b.children.size());
      for (const KontestBlock *child : b.children)
        children.push_back(child->clone());
    }

    virtual KontestBlock* clone() const = 0;

    virtual void addChild(KontestBlock *child) {
      children.push_back(child);
    }

    virtual bool loopParse() { llvm_unreachable("unexpected loop in block!"); return false; }

    /// Called once all children have been added
    virtual void finishParsing(ExecutionState &state) = 0;

    virtual void validate(BlockType type, uint64_t id,
                          const ref<Expr> &data1, const ref<Expr> &data2,
                          const std::string &name, ExecutionState &state) const = 0;

    /// Pick a new concrete sequence of actions.
    /// @returns whether we have cycled through all possible sequences.
    bool generate(bool firstSkeleton);

    virtual void generateFirst();
    virtual bool generateNext();
    virtual void generateRandom();

    void resetAll() {
      resetDescendants();
      reset();
    }

    void resetDescendants() {
      for (KontestBlock *child : children) {
        child->resetDescendants();
        child->reset();
      }
    }

    void addSymbolic(const ObjectState *os, AddressSpace &as) {
      assert(symbolics.size() == nextSymbolic && "tried to add symbolic while others pending!");
      ObjectState *copy = new ObjectState(*os, as);
      copy->claim();
      symbolics.push_back(copy);
      ++nextSymbolic;
    }

    ObjectState* nextCachedSymbolic() {
      if (nextSymbolic < symbolics.size())
        return symbolics[nextSymbolic++];
      else
        return nullptr;
    }

    void clearSymbolics() {
      for (const ObjectState *os : symbolics)
        os->release();
      symbolics.clear();
    }

    virtual bool run(bool s) {
      nextChild = 0;
      nextSymbolic = 0;
      active = true;
      stepping = s;
      activeChildFinished = false;
      return true;
    }

    virtual bool skip() {
      active = false;
      activeChildFinished = false;
      return false;
    }

    virtual KontestBlock *getNextChild() const {
      assert(nextChild < children.size() && "nextChild corrupted in KontestBlock!");
      return children[nextChild];
    }

    virtual bool runNextChild() {
      KontestBlock *child = getNextChild();
      bool retval;
      if (stepping && nextChild != activeChild) {
        assert(activeChild < children.size() && "invalid active child!");
        retval = child->skip();
      } else {
        retval = child->run(stepping);
      }
      return retval;
    }

    virtual void childFinished(KontestBlock *child) {
      if (stepping && nextChild == activeChild && !child->stepping)
        activeChildFinished = true;
      ++nextChild;
    }

    virtual void finish() {
      if (stepping) {
        if (activeChildFinished) {
          // when block is next run, trigger the next child
          ++activeChild;
          activeChildFinished = false;
          if (activeChild == children.size()) {
            activeChild = 0;
            stepping = false;
            reset();
          }
        }
      } else {
        reset();
      }
    }

    virtual void reset();

    /// Number of times the block needs to be run to obtain a complete execution
    /// allowing only one action to fire each time
    virtual unsigned actionsNeeded() const {
      unsigned s = 0;
      for (const KontestBlock *child : children)
        s += child->actionsNeeded();
      return s;
    }

    virtual bool loop() { llvm_unreachable("unexpected loop in block!"); return false; }

    bool isActive() const { return active; }

    unsigned getActiveChild() const { return activeChild; }

    virtual BlockType type() const = 0;

    virtual ~KontestBlock() {
      for (KontestBlock *child : children)
        delete child;
      for (const ObjectState *os : symbolics)
        os->release();
    }

    static const char* nameOfBlockType(BlockType type) {
      switch(type) {
        case BlockType::ANY: return "ANY";
        case BlockType::INTERLEAVE: return "INTERLEAVE";
        case BlockType::REPEAT: return "REPEAT";
        case BlockType::ACTION: return "ACTION";
        case BlockType::SCENARIO: return "SCENARIO";
        default: return "<invalid>";
      }
    }
  };

  template<class BlockType>
  class ClonableBlock : public KontestBlock {
  public:
    virtual KontestBlock* clone() const {
      cloned = new BlockType(static_cast<const BlockType &>(*this));
      return cloned;
    }
  };

  class ActionBlock : public ClonableBlock<ActionBlock> {
  public:
    const Action* const action;

    ActionBlock(const Action *action) : action(action) {};

    void finishParsing(ExecutionState &state) override;

    void validate(BlockType type, uint64_t id,
                  const ref<Expr> &data1, const ref<Expr> &data2,
                  const std::string &name, ExecutionState &state) const override;

    void finish() override;

    unsigned actionsNeeded() const override;

    BlockType type() const override { return BlockType::ACTION; }

    ~ActionBlock() {};
  };

  class ScenarioBlock : public ClonableBlock<ScenarioBlock> {
  public:
    const Scenario* const scenario;

    ScenarioBlock(const Scenario *scenario) : scenario(scenario) {};

    void finishParsing(ExecutionState &state) override;

    void validate(BlockType type, uint64_t id,
                  const ref<Expr> &data1, const ref<Expr> &data2,
                  const std::string &name, ExecutionState &state) const override;

    BlockType type() const override { return BlockType::SCENARIO; }

    ~ScenarioBlock() {};
  };

  class RepeatBlock : public ClonableBlock<RepeatBlock> {
  private:
    unsigned min, max;
    unsigned selectedRepetitions, loopCount;
    bool haveLooped;
    std::vector<KontestBlock*> originalChildren;

  public:
    RepeatBlock(unsigned min, unsigned max) : min(min), max(max), selectedRepetitions(min), loopCount(0) {};
    RepeatBlock(const RepeatBlock &rb) : ClonableBlock<RepeatBlock>(rb), min(rb.min), max(rb.max),
                                         selectedRepetitions(rb.selectedRepetitions),
                                         loopCount(rb.loopCount), haveLooped(rb.haveLooped), originalChildren() {
      originalChildren.reserve(rb.originalChildren.size());
      for (const KontestBlock *child : rb.originalChildren)
        originalChildren.push_back(child->clone());
    }

    bool loopParse() override;
    void finishParsing(ExecutionState &state) override;

    void validate(BlockType type, uint64_t id,
                  const ref<Expr> &data1, const ref<Expr> &data2,
                  const std::string &name, ExecutionState &state) const override;

    void generateFirst() override;
    bool generateNext() override;
    void generateRandom() override;

    bool run(bool stepping) override;
    bool loop() override;
    void finish() override;
    void reset() override;

    BlockType type() const override { return BlockType::REPEAT; }

    ~RepeatBlock() {};

  private:
    void regenerateChildren();
  };

  class AnyBlock : public ClonableBlock<AnyBlock> {
  private:
    unsigned selectedChild = 0;

  public:
    void finishParsing(ExecutionState &state) override;

    void validate(BlockType type, uint64_t id,
                  const ref<Expr> &data1, const ref<Expr> &data2,
                  const std::string &name, ExecutionState &state) const override;

    void generateFirst() override;
    bool generateNext() override;
    void generateRandom() override;

    bool runNextChild() override;

    unsigned actionsNeeded() const override;

    BlockType type() const override { return BlockType::ANY; }

    ~AnyBlock() {};
  };

  class InterleaveBlock : public ClonableBlock<InterleaveBlock> {
  private:
    std::vector<unsigned> selectedInterleaving;
    unsigned loopCount;
    bool haveLooped;

  public:
    InterleaveBlock() : loopCount(0) {};

    bool loopParse() override;
    void finishParsing(ExecutionState &state) override;

    void validate(BlockType type, uint64_t id,
                  const ref<Expr> &data1, const ref<Expr> &data2,
                  const std::string &name, ExecutionState &state) const override;

    void generateFirst() override;
    bool generateNext() override;
    void generateRandom() override;

    bool run(bool stepping) override;
    bool runNextChild() override;
    bool loop() override;
    void finish() override;
    void reset() override;

    BlockType type() const override { return BlockType::INTERLEAVE; }

    ~InterleaveBlock() {};

  private:
    bool nextPermutation();
    void randomPermutation();
  };
};

#endif //KLEE_KONTESTBLOCK_H
