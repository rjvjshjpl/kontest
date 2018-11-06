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
// Created by dfremont on 8/10/17.
//

#ifndef KLEE_SCENARIOHANDLER_H
#define KLEE_SCENARIOHANDLER_H

#include <klee/Expr.h>
#include "Executor.h"
#include "KontestBlock.h"
#include "Memory.h"

#include <unordered_map>
#include <vector>
#include <memory>
#include <cassert>

// Debugging macro, normally not compiled in
#ifdef DEBUG_SCENARIOS
#define SCENARIO_DEBUG_MSG(fmt, ...) do { \
  fprintf(stderr, "KT: "); \
  for (unsigned i = ScenarioHandler::blockDepth(); i > 0; i--) \
    fprintf(stderr, "  "); \
  fprintf(stderr, fmt "\n", ##__VA_ARGS__); \
} while(0)
#else
#define SCENARIO_DEBUG_MSG(...)
#endif

namespace klee {
  class Executor;
  class ExecutionState;
  struct SavedState;

  enum class ScenarioMode : unsigned {
    INIT,
    PARSING,
    RUNNING,
    DONE,
    INVALID_MODE
  };

  class Action {
  public:
    uint64_t identifier;
    std::string name;

    // dumb, but allows nice syntax when inserting into maps
    // TODO if we switch to C++17: abjure this; make members const; use try_emplace
    Action() = default;

    Action(uint64_t id, const std::string &name) : identifier(id), name(name) {}
  };

  class Scenario : public Action {
  public:
    Scenario() = default;

    Scenario(uint64_t id, const std::string &name) : Action(id, name) {}
  };

  class ScenarioHandler {
  public:
    static ScenarioHandler handler;

  private:
    Executor *executor;
    SavedState parsedState;

    ScenarioMode mode, savedMode;
    using BlockStack = std::vector<KontestBlock*>;
    BlockStack blockStack, savedStack;
    ScenarioBlock *rootBlock, *savedRoot;
    std::vector<bool> beginBlockActive, endBlockActive, loopBlockActive;
    std::vector<bool> savedBeginBlockActive, savedEndBlockActive, savedLoopBlockActive;
    mutable unsigned char *beginBlockBytes, *endBlockBytes, *loopBlockBytes;

    std::unordered_map<uint64_t, Action> actions;
    std::unordered_map<uint64_t, Scenario> scenarios;

  public:
    ScenarioHandler() : executor(nullptr), parsedState(), mode(ScenarioMode::INIT), rootBlock(nullptr) {}

    // Interface to Executor
    void setExecutor(Executor *e) { executor = e; }
    SavedState getStartingState() const { return parsedState; }
    bool usedScenarioDSL() const { return (mode != ScenarioMode::INIT); }
    bool generateNewSkeleton();
    bool allowSymbolic(const MemoryObject *mo, ObjectState* &cached);
    void addSymbolic(ObjectState *os, AddressSpace &as);
    void saveState();
    void restoreState();
    void saveSkeleton(KTest *test) const;
    void testTerminated();
    void cleanup() { parsedState = SavedState(); }

    // Intrinsics called from scenario DSL macros
    void setMode(uint64_t mode, ExecutionState &state);
    bool beginBlock(unsigned type, uint64_t id, const ref<Expr> &data1, const ref<Expr> &data2,
                    const std::string &name, ExecutionState &state);
    bool endBlock(ExecutionState &state);
    bool loopBlock(ExecutionState &state);

    // Errors, debugging
    static void parseError(const char *error);
    static void locatedParseError(const char *error);
    static unsigned blockDepth() { return ScenarioHandler::handler.blockStack.size(); }

    ~ScenarioHandler() {
      delete rootBlock;
      delete savedRoot;
      delete[] beginBlockBytes;
      delete[] endBlockBytes;
      delete[] loopBlockBytes;
    }

  private:
    ScenarioBlock* copyState(ScenarioBlock *root, const BlockStack &src, BlockStack &dst) const;

    void encodeActiveVector(const char *name, const std::vector<bool> &vec,
                            KTestObject &kobj, unsigned char * &mem) const;

    KontestBlock *parseScenario(uint64_t id, const std::string &name, ExecutionState &state);
    KontestBlock *parseAction(uint64_t id, const std::string &name, const ref<Expr> &precondition,
                              ExecutionState &state, int &retval);
    KontestBlock *parseRepeat(const ref <Expr> &min, const ref <Expr> &max, ExecutionState &state);
  };
};

#endif //KLEE_SCENARIOHANDLER_H
