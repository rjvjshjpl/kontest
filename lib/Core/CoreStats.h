//===-- CoreStats.h ---------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_CORESTATS_H
#define KLEE_CORESTATS_H

#include "klee/Statistic.h"

namespace klee {
namespace stats {

  extern Statistic allocations;
  extern Statistic resolveTime;
  extern Statistic instructions;
  extern Statistic instructionTime;
  extern Statistic instructionRealTime;
  extern Statistic coveredInstructions;
  extern Statistic uncoveredInstructions;  
  extern Statistic trueBranches;
  extern Statistic falseBranches;
  extern Statistic forkTime;
  extern Statistic solverTime;

  /// The number of divergences.
  extern Statistic divergences;

  /// The number of scenario skeletons used to generate tests.
  extern Statistic skeletons;

  /// The number of tests executed concretely.
  extern Statistic concreteTests;

  /// The number of tests executed symbolically.
  extern Statistic symbolicTests;

  /// The number of tests abandoned due to failing a klee_assume.
  extern Statistic abandonedTests;

  /// The amount of time spent in concrete execution
  extern Statistic concreteTime;

  /// The amount of time spent in symbolic execution
  extern Statistic symbolicTime;

  /// The number of process forks.
  extern Statistic forks;

  /// Number of states, this is a "fake" statistic used by istats, it
  /// isn't normally up-to-date.
  extern Statistic states;

  /// Instruction level statistic for tracking number of reachable
  /// uncovered instructions.
  extern Statistic reachableUncovered;

  /// Instruction level statistic tracking the minimum intraprocedural
  /// distance to an uncovered instruction; this is only periodically
  /// updated.
  extern Statistic minDistToUncovered;

  /// Instruction level statistic tracking the minimum intraprocedural
  /// distance to a function return.
  extern Statistic minDistToReturn;

}
}

#endif
