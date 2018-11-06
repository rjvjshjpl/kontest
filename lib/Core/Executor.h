//===-- Executor.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Class to perform actual execution, hides implementation details from external
// interpreter.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXECUTOR_H
#define KLEE_EXECUTOR_H

#include "klee/ExecutionState.h"
#include "klee/Interpreter.h"
#include "klee/Internal/ADT/EqualObjectCache.h"
#include "klee/Internal/ADT/IterablePriorityQueue.h"
#include "klee/Internal/ADT/KTest.h"
#include "klee/Internal/ADT/ReverseTrie.h"
#include "klee/Internal/Module/Cell.h"
#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/util/ArrayCache.h"
#include "ConcolicTest.h"
#include "MemoryManager.h"
#include "ScenarioHandler.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/ADT/Twine.h"

#include <vector>
#include <string>
#include <map>
#include <set>
#include <unordered_set>

namespace llvm {
  class BasicBlock;
  class BranchInst;
  class CallInst;
  class Constant;
  class ConstantExpr;
  class Function;
  class GlobalValue;
  class Instruction;
  class LLVMContext;
  class DataLayout;
  class Twine;
  class Value;
}

namespace klee {
  class Array;
  struct Cell;
  class ExecutionState;
  class ExternalDispatcher;
  class Expr;
  class InstructionInfoTable;
  struct KFunction;
  struct KInstruction;
  class KInstIterator;
  class KModule;
  class MemoryManager;
  class MemoryObject;
  class ObjectState;
  class PTree;
  class SpecialFunctionHandler;
  struct StackFrame;
  class StatsTracker;
  class TimingSolver;
  class TreeStreamWriter;
  template<class T> class ref;



  /// \todo Add a context object to keep track of data only live
  /// during an instruction step. Should contain addedStates,
  /// removedStates, and haltExecution, among others.

class Executor : public Interpreter {
  friend class ScenarioHandler;
  friend class SpecialFunctionHandler;
  friend class StatsTracker;

public:
  class Timer {
  public:
    Timer();
    virtual ~Timer();

    /// The event callback.
    virtual void run() = 0;
  };

  typedef std::pair<ExecutionState*,ExecutionState*> StatePair;

  enum TerminateReason {
    Abort,
    Assert,
    BadVectorAccess,
    Divergence,
    Exec,
    External,
    Free,
    Model,
    Overflow,
    Ptr,
    ReadOnly,
    ReportError,
    User,
    Unhandled
  };

private:
  static struct TestComparatorType {
    bool operator()(const std::unique_ptr<ConcolicTest> &a, const std::unique_ptr<ConcolicTest> &b) const {
      return *a < *b;
    }
  } testComparator;

  IterablePriorityQueue<std::unique_ptr<ConcolicTest>, TestComparatorType> tests;
  std::unordered_set<const llvm::BasicBlock*> blocksCovered;

  using ObjectNameCache = EqualObjectCache<std::string>;
  ObjectNameCache objectNames;
  using ObjectBytesCache = EqualObjectCache<ObjectBytes>;
  ObjectBytesCache objectBytes;

  // We store the expected resolutions, etc. of all pending tests in
  // common tries to save memory
  ReverseTrie<unsigned> resolutionTrie;
  ReverseTrie<unsigned> offsetTrie;
  ReverseTrie<uint64_t> functionResolutionTrie;
  ReverseTrie<unsigned> allocationTrie;

  // The expectations for the current test are extracted into these
  ReverseTrie<unsigned>::Path expectedResolutions;
  ReverseTrie<unsigned>::Path expectedOffsets;
  ReverseTrie<uint64_t>::Path expectedFunctionResolutions;
  ReverseTrie<unsigned>::Path expectedAllocations;

  ConcolicTest *currentTest;
  ExecutionState *currentTestState;
  std::vector< std::unique_ptr<ConcolicTest> > childTests;
  bool testTerminated, runningConcretely;
  bool testIsFertile;   // whether it is possible for this test to generate child tests when executed symbolically
  unsigned newBlocksCovered, testInstructions, symbolicCount;

  SavedState startingState;
  bool startingStateFinalized;    // whether we have set startingState to the latest possible moment

  std::unique_ptr<ConcolicTest> generateConcolicTest(const ExecutionState &state, const ConstraintManager &constraints);
  bool computeInitialValues(const std::vector<std::pair<const MemoryObject *, const Array *> > &symbolics,
                            const ConstraintManager &constraints, std::vector< std::vector<unsigned char> > &vals);
  void handleDivergence(ExecutionState &state, const char *reason);
  void recordEnteringBlock(const llvm::BasicBlock *block);
  SavedState saveCurrentState();
  void restoreState(const SavedState &ss);

  static struct {
    bool operator()(const ref<ConstantExpr> &a, const ref<ConstantExpr> &b) const {
      return a->getZExtValue(64) < b->getZExtValue(64);
    }
  } constantExprComparator;

  static const char *TerminateReasonNames[];

  class TimerInfo;

  KModule *kmodule;
  InterpreterHandler *interpreterHandler;

  ExternalDispatcher *externalDispatcher;
  TimingSolver *solver;
  MemoryManager *memory;
  std::set<ExecutionState*> states;
  StatsTracker *statsTracker;
  TreeStreamWriter *pathWriter, *symPathWriter;
  SpecialFunctionHandler *specialFunctionHandler;
  std::vector<TimerInfo*> timers;
  PTree *processTree;

  /// Used to track states that have been added during the current
  /// instructions step.
  /// \invariant \ref addedStates is a subset of \ref states.
  /// \invariant \ref addedStates and \ref removedStates are disjoint.
  std::vector<ExecutionState *> addedStates;
  /// Used to track states that have been removed during the current
  /// instructions step.
  /// \invariant \ref removedStates is a subset of \ref states.
  /// \invariant \ref addedStates and \ref removedStates are disjoint.
  std::vector<ExecutionState *> removedStates;

  /// Map of globals to their representative memory object.
  std::map<const llvm::GlobalValue*, MemoryObject*> globalObjects;

  /// Map of globals to their bound address. This also includes
  /// globals that have no representative object (i.e. functions).
  std::map<const llvm::GlobalValue*, ref<ConstantExpr> > globalAddresses;

  /// The set of legal function addresses, used to validate function
  /// pointers. We use the actual Function* address as the function address.
  std::set<uint64_t> legalFunctions;

  /// When non-null the bindings that will be used for calls to
  /// klee_make_symbolic in order replay.
  const struct KTest *replayKTest;
  /// The index into the current \ref replayKTest object.
  unsigned replayPosition;

  /// Disables forking, instead a random path is chosen. Enabled as
  /// needed to control memory usage. \see fork()
  bool atMemoryLimit;

  /// Disables forking, set by client. \see setInhibitForking()
  bool inhibitForking;

  /// Signals the executor to halt execution at the next instruction
  /// step.
  bool haltExecution;

  /// The maximum time to allow for a single core solver query.
  /// (e.g. for a single STP query)
  double coreSolverTimeout;

  /// Assumes ownership of the created array objects
  ArrayCache arrayCache;

  /// File to print executed instructions to
  llvm::raw_ostream *debugInstFile;

  // @brief Buffer used by logBuffer
  std::string debugBufferString;

  // @brief buffer to store logs before flushing to file
  llvm::raw_string_ostream debugLogBuffer;

  llvm::Function* getTargetFunction(llvm::Value *calledVal,
                                    ExecutionState &state);

  void executeInstruction(ExecutionState &state, KInstruction *ki);

  void printFileLine(ExecutionState &state, KInstruction *ki,
                     llvm::raw_ostream &file);

  void run(ExecutionState &initialState);

  bool runTest(const ExecutionState &initialState, ConcolicTest *test, bool concretely);

  // Given a concrete object in our [klee's] address space, add it to
  // objects checked code can reference.
  MemoryObject *addExternalObject(ExecutionState &state, void *addr,
                                  unsigned size, bool isReadOnly);

  void initializeGlobalObject(ExecutionState &state, ObjectState *os,
			      const llvm::Constant *c,
			      unsigned offset);
  void initializeGlobals(ExecutionState &state);

  bool stepInstruction(ExecutionState &state);
  void updateStates(ExecutionState *current);
  void transferToBasicBlock(llvm::BasicBlock *dst,
			    llvm::BasicBlock *src,
			    ExecutionState &state);

  void callExternalFunction(ExecutionState &state,
                            KInstruction *target,
                            llvm::Function *function,
                            std::vector< ref<Expr> > &arguments,
                            const llvm::APInt *concreteArguments);

  ObjectState *bindObjectInState(ExecutionState &state, const MemoryObject *mo,
                                 bool isLocal, const Array *array = 0);

  /// Resolve a pointer to the memory object it points inside,
  /// provided that it points to at least the given number of
  /// \ref bytes of valid memory. This method forks execution for
  /// all alternative resolutions, returning false there are none at
  /// all. Note that this method may simplify or concretize an address.
  ///
  /// \param op[out] An (MemoryObject,ObjectState) pair for the object
  /// the given address points into.
  bool resolve(ExecutionState &state,
               ref<Expr> &address,
               uint64_t concreteAddress,
               int bytes,
               ObjectPair &op);

  /// Helper function for resolve()
  bool resolveToObject(ExecutionState &state,
                       ref<Expr> &address,
                       const ref<ConstantExpr> &concreteAddress,
                       int bytes,
                       ObjectPair &op);

  ref<Expr> createResolutionConstraint(const MemoryObject *mo, int bytes, const ref<Expr> &address, const ref<ConstantExpr> &concreteAddress);

  /// Refine a resolution to a concrete offset. This method forks
  /// execution for all possible concrete offsets.
  void resolveToConcrete(ExecutionState &state,
                         ref<Expr> &address,
                         const ref<ConstantExpr> &concreteAddress,
                         const MemoryObject *mo);

  /// Resolve a pointer to the memory object it points to the start
  /// of, forking execution for all alternative resolutions. Prints an
  /// error if the pointer does not point to the start of any object.
  /// The same notes apply as to \ref resolve above.
  ///
  /// \param op[out] An (MemoryObject,ObjectState) pair for the object
  /// the given address points to the beginning of.
  bool resolveExact(ExecutionState &state,
                    ref<Expr> p,
                    uint64_t concreteP,
                    const std::string &name,
                    ObjectPair &op);

  /// Allocate and bind a new object in a particular state. NOTE: This
  /// function may fork.
  ///
  /// \param isLocal Flag to indicate if the object should be
  /// automatically deallocated on function return (this also makes it
  /// illegal to free directly).
  ///
  /// \param target Value at which to bind the base address of the new
  /// object.
  ///
  /// \param reallocFrom If non-zero and the allocation succeeds,
  /// initialize the new object from the given one and unbind it when
  /// done (realloc semantics). The initialized bytes will be the
  /// minimum of the size of the old and new objects, with remaining
  /// bytes initialized as specified by zeroMemory.
  void executeAlloc(ExecutionState &state,
                    ref<Expr> size,
                    uint64_t concreteSize,
                    bool isLocal,
                    KInstruction *target,
                    bool zeroMemory=false,
                    const ObjectState *reallocFrom=0,
                    bool sizeStaticallyConstant = false);

  /// Free the given address with checking for errors. If target is
  /// given it will be bound to 0 in the resulting states (this is a
  /// convenience for realloc). Note that this function can cause the
  /// state to fork and that \ref state cannot be safely accessed
  /// afterwards.
  void executeFree(ExecutionState &state,
                   ref<Expr> address,
                   uint64_t concreteAddress,
                   KInstruction *target = 0);
  
  void executeCall(ExecutionState &state, 
                   KInstruction *ki,
                   llvm::Function *f,
                   std::vector< ref<Expr> > &arguments,
                   const llvm::APInt *concreteArguments);
                   
  // do address resolution / object binding / out of bounds checking
  // and perform the operation
  void executeMemoryOperation(ExecutionState &state,
                              bool isWrite,
                              ref<Expr> address,
                              uint64_t concreteAddress,
                              ref<Expr> value /* undef if read */,
                              const llvm::APInt &concreteValue,
                              KInstruction *target /* undef if write */);
  void executeMemoryOperation(ExecutionState &state,
                              bool isWrite,
                              ref<Expr> address,
                              uint64_t concreteAddress,
                              ref<ConstantExpr> value /* undef if read */,
                              KInstruction *target /* undef if write */);

  void executeAssume(ExecutionState &state, const ref<Expr> &condition, bool concreteCondition, bool silent);

  typedef std::function< bool(Executor *, ExecutionState &, ObjectState *, bool &) > ConstraintEnforcer;
  void executeMakeSymbolic(ExecutionState &state, const MemoryObject *mo, const std::string &name,
                           ConstraintEnforcer enforcer = ConstraintEnforcer());

  /// Create a new state where each input condition has been added as
  /// a constraint and return the results. The input state is included
  /// as one of the results. Note that the output vector may included
  /// NULL pointers for states which were unable to be created.
  void branch(ExecutionState &state, 
              const std::vector< ref<Expr> > &conditions,
              std::vector<ExecutionState*> &result);

  // Fork current and return states in which condition holds / does
  // not hold, respectively. One of the states is necessarily the
  // current state, and one of the states may be null.
  StatePair fork(ExecutionState &current, ref<Expr> condition, bool concreteCondition, bool isInternal);

  // Fork new states for each of the given resolutions, assuming it is possible
  // for each to point to at least the given number of valid bytes. Putting
  // bytes == -1 indicates that the address must be an exact match to an object.
  // If an out-of-bounds address is given, also fork a state for that.
  void forkResolutions(ExecutionState &current, ResolutionList &rl, ref<Expr> address, int bytes, const ref <ConstantExpr> &outOfBoundsAddr);

  /// Add the given (boolean) condition as a constraint on state. This
  /// function is a wrapper around the state's addConstraint function
  /// which also manages propagation of implied values,
  /// validity checks, and seed patching.
  void addConstraint(ExecutionState &state, ref<Expr> condition);

  // Called on [for now] concrete reads, replaces constant with a symbolic
  // Used for testing.
  ref<Expr> replaceReadWithSymbolic(ExecutionState &state, ref<Expr> e);

  const Cell& eval(KInstruction *ki, unsigned index,
                   ExecutionState &state) const;

  Cell& getArgumentCell(ExecutionState &state,
                        KFunction *kf,
                        unsigned index) {
    return state.stack.back().locals[kf->getArgRegister(index)];
  }

  Cell& getDestCell(ExecutionState &state,
                    KInstruction *target) {
    return state.stack.back().locals[target->dest];
  }

  void bindLocal(KInstruction *target, 
                 ExecutionState &state, 
                 ref<Expr> value,
                 llvm::APInt concreteValue);
  void bindLocal(KInstruction *target,
                 ExecutionState &state,
                 llvm::APInt concreteValue);
  void bindArgument(KFunction *kf, 
                    unsigned index,
                    ExecutionState &state,
                    ref<Expr> value,
                    llvm::APInt concreteValue);

  // Helper versions for ConstantExprs
  void bindLocal(KInstruction *target,
                 ExecutionState &state,
                 ref<ConstantExpr> value);
  void bindArgument(KFunction *kf,
                    unsigned index,
                    ExecutionState &state,
                    ref<ConstantExpr> value);

  ref<klee::ConstantExpr> evalConstantExpr(const llvm::ConstantExpr *ce);

  /// Return a unique constant value for the given expression in the
  /// given state, if it has one (i.e. it provably only has a single
  /// value). Otherwise return the original expression.
  ref<Expr> toUnique(const ExecutionState &state, ref<Expr> &e);

  /// Return a constant value for the given expression, forcing it to
  /// be constant in the given state by adding a constraint if
  /// necessary. Note that this function breaks completeness and
  /// should generally be avoided.
  ///
  /// \param purpose An identify string to printed in case of concretization.
  ref<klee::ConstantExpr> toConstant(ExecutionState &state, const ref<Expr> &e, const llvm::APInt &concrete,
                                     const char *purpose);
  ref<klee::ConstantExpr> toConstant(ExecutionState &state, const Cell &cell,
                                     const char *purpose);
  ref<klee::ConstantExpr> toConstant(ExecutionState &state, const ref<Expr> &e, const ref<ConstantExpr> &concrete,
                                     const char *purpose);

  /// Bind a constant value for e to the given target. NOTE: This
  /// function may fork state if the state has multiple seeds.
  void executeGetValue(ExecutionState &state, ref<Expr> e, const llvm::APInt &concrete, KInstruction *target);

  void executeMakeConcrete(ExecutionState &state, ref<Expr> e, const llvm::APInt &concrete, KInstruction *target);

  /// Get textual information regarding a memory address.
  std::string getAddressInfo(ExecutionState &state, ref<Expr> address) const;

  // Determines the \param lastInstruction of the \param state which is not KLEE
  // internal and returns its InstructionInfo
  const InstructionInfo & getLastNonKleeInternalInstruction(const ExecutionState &state,
      llvm::Instruction** lastInstruction);

  bool shouldExitOn(enum TerminateReason termReason);

  // remove state from queue and delete
  void terminateState(ExecutionState &state);
  void terminateStateUnchecked(ExecutionState &state, bool abandonPath = false);
  // call exit handler and terminate state
  void terminateStateEarly(ExecutionState &state, const llvm::Twine &message, bool suppressExitOnError = false);
  // call exit handler and terminate state
  void terminateStateOnExit(ExecutionState &state);
  // call error handler and terminate state
  void terminateStateOnError(ExecutionState &state, const llvm::Twine &message,
                             enum TerminateReason termReason,
                             const char *suffix = NULL,
                             const llvm::Twine &longMessage = "");

  // call error handler and terminate state, for execution errors
  // (things that should not be possible, like illegal instruction or
  // unlowered instrinsic, or are unsupported, like inline assembly)
  void terminateStateOnExecError(ExecutionState &state,
                                 const llvm::Twine &message,
                                 const llvm::Twine &info="") {
    terminateStateOnError(state, message, Exec, NULL, info);
  }

  /// bindModuleConstants - Initialize the module constant table.
  void bindModuleConstants();

  template <typename TypeIt>
  void computeOffsets(KGEPInstruction *kgepi, TypeIt ib, TypeIt ie);

  /// bindInstructionConstants - Initialize any necessary per instruction
  /// constant values.
  void bindInstructionConstants(KInstruction *KI);

  void handlePointsToObj(ExecutionState &state,
                         KInstruction *target,
                         const std::vector<ref<Expr> > &arguments);

  void doImpliedValueConcretization(ExecutionState &state,
                                    ref<Expr> e,
                                    ref<ConstantExpr> value);

  /// Add a timer to be executed periodically.
  ///
  /// \param timer The timer object to run on firings.
  /// \param rate The approximate delay (in seconds) between firings.
  void addTimer(Timer *timer, double rate);

  void initTimers();
  void processTimers(ExecutionState *current,
                     double maxInstTime);
  void checkMemoryUsage();
  void printDebugInstructions(ExecutionState &state);
  void doDumpStates();
  void dumpConstraints(const ConstraintManager &constraints);
  void dumpState(const ExecutionState &state);

public:
  Executor(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
      InterpreterHandler *ie);
  virtual ~Executor();

  const InterpreterHandler& getHandler() {
    return *interpreterHandler;
  }

  // XXX should just be moved out to utility module
  ref<klee::ConstantExpr> evalConstant(const llvm::Constant *c);

  virtual void setPathWriter(TreeStreamWriter *tsw) {
    pathWriter = tsw;
  }
  virtual void setSymbolicPathWriter(TreeStreamWriter *tsw) {
    symPathWriter = tsw;
  }

  virtual void setReplayKTest(const struct KTest *out) {
    replayKTest = out;
    replayPosition = 0;
  }

  virtual const llvm::Module *
  setModule(llvm::Module *module, const ModuleOptions &opts);

  virtual void runFunctionAsMain(llvm::Function *f,
                                 int argc,
                                 char **argv,
                                 char **envp);

  /*** Runtime options ***/

  virtual void setHaltExecution(bool value) {
    haltExecution = value;
    if (haltExecution)
      testTerminated = true;    // abort the current test
  }

  virtual void setInhibitForking(bool value) {
    inhibitForking = value;
  }

  void prepareForEarlyExit();

  /*** State accessor methods ***/

  virtual unsigned getPathStreamID(const ExecutionState &state);

  virtual unsigned getSymbolicPathStreamID(const ExecutionState &state);

  virtual void getConstraintLog(const ExecutionState &state,
                                std::string &res,
                                Interpreter::LogType logFormat = Interpreter::STP);

  virtual KTest *getSymbolicSolution(const ExecutionState &state, unsigned testID, bool &ranSymbolically);

  virtual void getCoveredLines(const ExecutionState &state,
                               std::map<const std::string*, std::set<unsigned> > &res);

  Expr::Width getWidthForLLVMType(llvm::Type *type) const;
  size_t getAllocationAlignment(const llvm::Value *allocSite) const;

  /*** Error/warning reporting methods ***/

  void message(const char *msg, ...) const
    __attribute__((format(printf, 2, 3)));
  void note(const char *msg, ...) const
    __attribute__((format(printf, 2, 3)));
  void note_at(const InstructionInfo &ii, const char *msg, ...) const
    __attribute__((format(printf, 3, 4)));
  void error(const char *msg, ...) const
    __attribute__((format(printf, 2, 3), noreturn));
  void nonfatal_error_at(const InstructionInfo &ii, const char *msg, ...) const
  __attribute__((format(printf, 3, 4)));
  void warning(const char *msg, ...) const
    __attribute__((format(printf, 2, 3)));
  void warning_once(const void *id, const char *msg, ...) const
    __attribute__((format(printf, 3, 4)));

};

} // End klee namespace

#endif
