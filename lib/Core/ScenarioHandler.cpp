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

#include "ScenarioHandler.h"

#include "Executor.h"
#include <klee/ExecutionState.h>
#include <klee/Internal/Support/ErrorHandling.h>

using namespace klee;
using namespace llvm;

ScenarioHandler ScenarioHandler::handler;

// Convenience function
void ScenarioHandler::parseError(const char *error) {
  klee_error("SCENARIO PARSE ERROR: %s", error);
}

void ScenarioHandler::locatedParseError(const char *error) {
  ScenarioHandler::handler.executor->error("SCENARIO PARSE ERROR: %s", error);
}

/**
 * Interface to Executor
 */

bool ScenarioHandler::generateNewSkeleton() {
  if (!savedRoot)   // not using scenario DSL
    return false;

  bool exhausted = savedRoot->generate(false);
  savedRoot->resetAll();    // reset running state of blocks
  savedStack.clear();   // the first test will start from the beginning
  savedBeginBlockActive.clear();
  savedEndBlockActive.clear();
  savedLoopBlockActive.clear();
  SCENARIO_DEBUG_MSG("generated new skeleton");

  return !exhausted;
}

bool ScenarioHandler::allowSymbolic(const MemoryObject *mo, ObjectState* &cached) {
  if (mode == ScenarioMode::DONE)
    locatedParseError("created symbolic value after scenario");
  if (mode == ScenarioMode::PARSING)
    return false;   // dry run for parsing; do not create symbolic
  // We would also like to give an error here if mode == INIT, but only if
  // the program is actually using the scenario DSL. We can't detect that
  // until setMode is called, so we proceed for now.
  if (mode == ScenarioMode::INIT) {
    assert(!rootBlock && blockStack.empty() && "somehow reentered INIT after parsing!");
    cached = nullptr;
    return true;
  }

  // We are running the scenario, so symbolics are allowed; if the current
  // block has a cached symbolic pending, use that
  assert(!blockStack.empty() && "scenario running with empty stack!");
  cached = blockStack.back()->nextCachedSymbolic();
  SCENARIO_DEBUG_MSG("adding symbolic (%d)", cached != nullptr);
  if (cached && cached->getObject() != mo)
    locatedParseError("symbolic value creation order changed; did you put control flow in a scenario?");
  return true;
}

void ScenarioHandler::addSymbolic(ObjectState *os, AddressSpace &as) {
  if (blockStack.empty())   // not using scenario DSL
    return;

  blockStack.back()->addSymbolic(os, as);
}

void ScenarioHandler::saveState() {
  if (mode == ScenarioMode::INIT) {   // not using scenario DSL (or created a symbolic early: caught in setMode)
    assert(!rootBlock && "somehow reentered INIT after parsing!");
    return;
  }

  assert(mode != ScenarioMode::PARSING && "tried to save scenario state while parsing!");
  assert(savedStack.empty() && "tried to save scenario running state twice!");
  if (savedRoot)    // we may have saved state from the previous skeleton
    delete savedRoot;

  savedMode = mode;
  savedRoot = copyState(rootBlock, blockStack, savedStack);
  savedBeginBlockActive = beginBlockActive;
  savedEndBlockActive = endBlockActive;
  savedLoopBlockActive = loopBlockActive;
  SCENARIO_DEBUG_MSG("saved state");
}

void ScenarioHandler::restoreState() {
  if (mode == ScenarioMode::INIT) {   // not using scenario DSL
    assert(!rootBlock && "somehow reentered INIT after parsing!");
    return;
  }

  assert(blockStack.empty() && "blocks not cleaned up properly!");
  assert(!rootBlock && "root block not cleaned up properly!");

  mode = savedMode;
  rootBlock = copyState(savedRoot, savedStack, blockStack);
  beginBlockActive = savedBeginBlockActive;
  endBlockActive = savedEndBlockActive;
  loopBlockActive = savedLoopBlockActive;
  SCENARIO_DEBUG_MSG("restored state");
}

ScenarioBlock* ScenarioHandler::copyState(ScenarioBlock *root, const ScenarioHandler::BlockStack &src,
                                          ScenarioHandler::BlockStack &dst) const {
  // Clone all blocks
  ScenarioBlock *newRoot = new ScenarioBlock(*root);
  root->cloned = newRoot;

  // Make isomorphic copy of stack
  dst.reserve(src.size());
  for (const KontestBlock *block : src)
    dst.push_back(block->cloned);

  return newRoot;
}

void ScenarioHandler::saveSkeleton(KTest *test) const {
  if (mode == ScenarioMode::INIT)   // not using the scenario DSL
    return;
  if (mode == ScenarioMode::PARSING)
    parseError("program crashed during parsing; did you put side effects in a scenario?");

  KTestObject *objects = new KTestObject[test->numObjects + 3];
  encodeActiveVector("kontest_beginBlockActive", beginBlockActive, objects[0], beginBlockBytes);
  encodeActiveVector("kontest_endBlockActive", endBlockActive, objects[1], endBlockBytes);
  encodeActiveVector("kontest_loopBlockActive", loopBlockActive, objects[2], loopBlockBytes);
  std::copy(&test->objects[0], &test->objects[test->numObjects], &objects[3]);

  delete[] test->objects;
  test->objects = objects;
  test->numObjects += 3;
}

void ScenarioHandler::encodeActiveVector(const char *name, const std::vector<bool> &vec,
                                         KTestObject &kobj, unsigned char * &mem) const {
  kobj.name = const_cast<char *>(name);   // work around C type of KTestObject
  unsigned objSize = vec.size();
  if (objSize == 0)
    objSize = 1;    // hack around KTestObjects requiring nonzero size
  kobj.numBytes = objSize;
  delete[] mem;
  mem = new unsigned char[kobj.numBytes];
  std::copy(vec.cbegin(), vec.cend(), mem);
  kobj.bytes = mem;
}

void ScenarioHandler::testTerminated() {
  if (mode == ScenarioMode::INIT) {   // not using the scenario DSL, or something went wrong
    assert(blockStack.empty() && "scenario processing sequence broken!");
    if (rootBlock)
      parseError("did not run specified scenario; did you put TEST_SCENARIO inside an if?");
    return;   // not using the scenario DSL
  }
  if (mode == ScenarioMode::PARSING)
    parseError("program crashed during parsing; did you put side effects in a scenario?");

  blockStack.clear();   // will be nonempty if program did not terminate normally
  delete rootBlock;
  rootBlock = nullptr;
}

/**
 * Intrinsics invoked by scenario DSL macros
 */

void ScenarioHandler::setMode(uint64_t newMode, ExecutionState &state) {
  assert(newMode < static_cast<uint64_t>(ScenarioMode::INVALID_MODE) && "invalid scenario mode!");
  ScenarioMode nm = static_cast<ScenarioMode>(newMode);

  switch (nm) {
    case ScenarioMode::PARSING:
      assert(executor->currentTest->generation == 0 && "parsing after first test!");
      assert(!executor->currentTest->alreadyRun && "parsing during second pass!");
      if (mode == ScenarioMode::DONE)
        parseError("multiple scenarios specified to test");
      assert(mode == ScenarioMode::INIT && "scenario processing sequence broken!");
      assert(!rootBlock && "somehow reentered INIT after parsing!");
      if (!executor->currentTestState->symbolics.empty())
        parseError("created symbolic value before scenario");
      assert(!executor->startingState && "startingState set before parsing!");

      executor->inhibitForking = true;    // all branches are concrete during parsing, so don't bother forking
      break;

    case ScenarioMode::RUNNING:
      assert(mode == ScenarioMode::PARSING && "scenario processing sequence broken!");
      assert(rootBlock && "did not detect test scenario!");

      rootBlock->generate(true);   // generate initial concrete sequence of actions
      rootBlock->resetAll();    // reset running state of blocks
      parsedState = executor->saveCurrentState();   // save state to start new skeletons from
      executor->inhibitForking = false;
      break;

    case ScenarioMode::DONE:
      assert(mode == ScenarioMode::RUNNING && "scenario processing sequence broken!");

      // special case: if test used no symbolics, we haven't yet saved our state;
      // we'll overwrite the run-specific part anyway but need the rest in order
      // to generate further skeletons
      if (!savedRoot)
        saveState();
      break;

    default:
      llvm_unreachable("unexpected scenario mode!");
  }

  mode = nm;

  SCENARIO_DEBUG_MSG("switched to mode %d", mode);
}

bool ScenarioHandler::beginBlock(unsigned typeNum, uint64_t id,
                                 const ref<Expr> &data1, const ref<Expr> &data2,
                                 const std::string &name, ExecutionState &state) {
  if (mode != ScenarioMode::PARSING && mode != ScenarioMode::RUNNING) {
    locatedParseError("invoked scenario or action outside of TEST_SCENARIO");
    return false;
  }
  assert(typeNum < static_cast<uint64_t>(BlockType::INVALID_TYPE) && "invalid block type!");
  BlockType type = static_cast<BlockType>(typeNum);

  KontestBlock *block;
  int retval = true;    // blocks active by default

  if (mode == ScenarioMode::RUNNING) {
    if (blockStack.empty()) {   // just starting the run
      assert(type == BlockType::SCENARIO && "starting run outside of a scenario!");   // caught during parsing
      if (id != rootBlock->scenario->identifier) {
        parseError("unparsed test scenario; did you put TEST_SCENARIO inside an if?");
        return false;
      }
      block = rootBlock;
      block->validate(type, id, data1, data2, name, state);
      block->run(false);
    } else {    // run in progress
      KontestBlock *parent = blockStack.back();
      if (parent->type() == BlockType::ACTION) {    // can't detect this earlier since actions don't run when parsing
        locatedParseError("actions cannot invoke another action or use REPEAT, etc.");
        return false;
      }
      block = parent->getNextChild();
      block->validate(type, id, data1, data2, name, state);
      retval = parent->runNextChild();
    }
    beginBlockActive.push_back(retval);
  } else {    // parsing
    switch (type) {
      case BlockType::SCENARIO: block = parseScenario(id, name, state); break;
      case BlockType::ACTION: block = parseAction(id, name, data1, state, retval); break;
      case BlockType::REPEAT: block = parseRepeat(data1, data2, state); break;
      case BlockType::ANY: block = new AnyBlock(); break;
      case BlockType::INTERLEAVE: block = new InterleaveBlock(); break;
      default: llvm_unreachable("invalid BlockType somehow got through");
    }
  }

  SCENARIO_DEBUG_MSG("%s %s begin (%d)", KontestBlock::nameOfBlockType(type), name.c_str(), block->getActiveChild());
  blockStack.push_back(block);

  return retval;
}

bool ScenarioHandler::endBlock(ExecutionState &state) {
  assert((mode == ScenarioMode::PARSING || mode == ScenarioMode::RUNNING) && "block processing sequence broken!");

  KontestBlock *child = blockStack.back();
  std::size_t size = blockStack.size();
  if (mode == ScenarioMode::PARSING) {
    if (size > 1) {
      KontestBlock *parent = blockStack[size - 2];
      parent->addChild(child);
    } else if (child != rootBlock) {
      parseError("malformed TEST_SCENARIO (not a scenario?)");
    }
    child->finishParsing(state);
  } else {
    child->finish();
    if (size > 1) {
      KontestBlock *parent = blockStack[size - 2];
      parent->childFinished(child);
    }
  }

  blockStack.pop_back();
  SCENARIO_DEBUG_MSG("end (%d)", child->isActive());

  bool retval = child->isActive();
  if (mode == ScenarioMode::RUNNING)
    endBlockActive.push_back(retval);
  return retval;
}

bool ScenarioHandler::loopBlock(ExecutionState &state) {
  assert((mode == ScenarioMode::PARSING || mode == ScenarioMode::RUNNING) && "block processing sequence broken!");
  assert(!blockStack.empty() && "loopBlock called outside of block!");

  SCENARIO_DEBUG_MSG("loop");
  bool retval;
  if (mode == ScenarioMode::PARSING) {
    retval = blockStack.back()->loopParse();
  } else {
    retval = blockStack.back()->loop();
    loopBlockActive.push_back(retval);
  }
  return retval;
}

/**
 * Helper functions for parsing blocks
 */

KontestBlock* ScenarioHandler::parseScenario(uint64_t id, const std::string &name,
                                             ExecutionState &state) {
  const Scenario *scenario;
  bool found = scenarios.count(id);
  if (found) {
    scenario = &scenarios.at(id);
  } else {
    scenarios[id] = Scenario(id, name);
    scenario = &scenarios[id];
  }

  ScenarioBlock *block = new ScenarioBlock(scenario);
  if (!rootBlock) {
    assert(!found && scenarios.size() == 1 && "did not assign rootBlock correctly!");
    rootBlock = block;
  }
  return block;
}

KontestBlock *ScenarioHandler::parseAction(uint64_t id, const std::string &name,
                                           const ref<Expr> &precondition,
                                           ExecutionState &state, int &retval) {
  if (!rootBlock) {
    parseError("gave action instead of scenario in TEST_SCENARIO");
    return nullptr;
  }

  const Action *action;
  bool found = actions.count(id);
  if (found) {
    action = &actions.at(id);
  } else {
    actions[id] = Action(id, name);
    action = &actions[id];
  }
  retval = false;   // deactivate block
  return new ActionBlock(action);
}

KontestBlock *ScenarioHandler::parseRepeat(const ref<Expr> &emin, const ref<Expr> &emax, ExecutionState &state) {
  ConstantExpr *cmin = dyn_cast<ConstantExpr>(emin);
  ConstantExpr *cmax = dyn_cast<ConstantExpr>(emax);
  if (!cmin || !cmax)
    locatedParseError("REPEAT block has symbolic range");

  int min = cmin->getZExtValue();
  int max = cmax->getZExtValue();
  if (min < 0 || max < min) {
    locatedParseError("invalid range for REPEAT block");
    return nullptr;
  }
  return new RepeatBlock(min, max);
}
