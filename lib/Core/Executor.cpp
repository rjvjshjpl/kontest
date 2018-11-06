//===-- Executor.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Executor.h"
#include "Context.h"
#include "CoreStats.h"
#include "ExternalDispatcher.h"
#include "ImpliedValue.h"
#include "Memory.h"
#include "MemoryManager.h"
#include "PTree.h"
#include "SpecialFunctionHandler.h"
#include "StatsTracker.h"
#include "TimingSolver.h"
#include "ExecutorTimerInfo.h"


#include "klee/ExecutionState.h"
#include "klee/Expr.h"
#include "klee/Interpreter.h"
#include "klee/TimerStatIncrementer.h"
#include "klee/CommandLine.h"
#include "klee/Common.h"
#include "klee/util/Assignment.h"
#include "klee/util/ExprPPrinter.h"
#include "klee/util/ExprSMTLIBPrinter.h"
#include "klee/util/ExprUtil.h"
#include "klee/util/GetElementPtrTypeIterator.h"
#include "klee/Config/Version.h"
#include "klee/Internal/ADT/KTest.h"
#include "klee/Internal/ADT/RNG.h"
#include "klee/Internal/Module/Cell.h"
#include "klee/Internal/Module/InstructionInfoTable.h"
#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/Internal/Support/FloatEvaluation.h"
#include "klee/Internal/Support/ModuleUtil.h"
#include "klee/Internal/System/Time.h"
#include "klee/Internal/System/MemoryUsage.h"
#include "klee/SolverStats.h"

#include "llvm/IR/Function.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/IR/CallSite.h"

#ifdef HAVE_ZLIB_H
#include "klee/Internal/Support/CompressionStream.h"
#endif

#include <cassert>
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cfloat>
#include <random>

#include <sys/mman.h>

#include <errno.h>
#include <cxxabi.h>
#include <llvm/ExecutionEngine/GenericValue.h>

using namespace llvm;
using namespace klee;





namespace {
  cl::opt<bool>
  DumpStatesOnHalt("dump-states-on-halt",
                   cl::init(true),
		   cl::desc("Dump test cases for all active states on exit (default=on)"));

  cl::opt<bool>
  AllowExternalSymCalls("allow-external-sym-calls",
                        cl::init(true),
			cl::desc("Allow calls with symbolic arguments to external functions.  This concretizes the symbolic arguments.  (default=on)"));

  /// The different query logging solvers that can switched on/off
  enum PrintDebugInstructionsType {
    STDERR_ALL, ///
    STDERR_SRC,
    STDERR_COMPACT,
    FILE_ALL,    ///
    FILE_SRC,    ///
    FILE_COMPACT ///
  };

  llvm::cl::list<PrintDebugInstructionsType> DebugPrintInstructions(
      "debug-print-instructions",
      llvm::cl::desc("Log instructions during execution."),
      llvm::cl::values(
          clEnumValN(STDERR_ALL, "all:stderr", "Log all instructions to stderr "
                                               "in format [src, inst_id, "
                                               "llvm_inst]"),
          clEnumValN(STDERR_SRC, "src:stderr",
                     "Log all instructions to stderr in format [src, inst_id]"),
          clEnumValN(STDERR_COMPACT, "compact:stderr",
                     "Log all instructions to stderr in format [inst_id]"),
          clEnumValN(FILE_ALL, "all:file", "Log all instructions to file "
                                           "instructions.txt in format [src, "
                                           "inst_id, llvm_inst]"),
          clEnumValN(FILE_SRC, "src:file", "Log all instructions to file "
                                           "instructions.txt in format [src, "
                                           "inst_id]"),
          clEnumValN(FILE_COMPACT, "compact:file",
                     "Log all instructions to file instructions.txt in format "
                     "[inst_id]"),
          clEnumValEnd),
      llvm::cl::CommaSeparated);
#ifdef HAVE_ZLIB_H
  cl::opt<bool> DebugCompressInstructions(
      "debug-compress-instructions", cl::init(false),
      cl::desc("Compress the logged instructions in gzip format."));
#endif

  cl::opt<bool>
  UseIVC("implied-values",
         cl::init(true),
         cl::desc("Use Implied Value Concretization (default=on)"));

  cl::opt<bool>
  DebugCheckForImpliedValues("debug-check-for-implied-values");


  cl::opt<bool>
  SimplifySymIndices("simplify-sym-indices",
                     cl::init(true),
		     cl::desc("Simplify symbolic accesses using equalities from other constraints (default=on)"));

  cl::opt<bool>
  EqualitySubstitution("equality-substitution",
		       cl::init(true),
		       cl::desc("Simplify equality expressions before querying the solver (default=on)."));

  cl::opt<unsigned>
  MaxSymArraySize("max-sym-array-size",
                  cl::init(0));

  cl::opt<bool>
  SuppressExternalWarnings("suppress-external-warnings",
			   cl::init(false),
			   cl::desc("Supress warnings about calling external functions."));

  cl::opt<bool>
  AllExternalWarnings("all-external-warnings",
		      cl::init(false),
		      cl::desc("Issue an warning everytime an external call is made,"
			       "as opposed to once per function (default=off)"));

  cl::opt<bool>
  WarnOnConcretization("warn-on-concretization",
                       cl::init(true),
                       cl::desc("Issue a warning when a symbolic value is concretized, "
                                "losing possible values (default=on). "
                                "May slightly hurt performance if concretizations are frequent."));

  cl::opt<bool>
  ConcretizeVerbosely("verbose-concretization",
                      cl::init(false),
                      cl::desc("Pretty-print all concretized expressions (default=off). "
                               "Likely to hurt performance if concretizations are frequent."));

  cl::opt<bool>
  OutputAllTests("output-all-tests",
                 cl::init(false),
                 cl::desc("Output all tests, not just those covering new code (default=off)."));

  cl::opt<bool>
  EmitAllErrors("emit-all-errors",
                cl::init(false),
                cl::desc("Generate test cases for all errors "
                         "(default=off, i.e. one per (error,instruction) pair)"));

  cl::opt<bool>
  NoExternals("no-externals",
           cl::desc("Do not allow external function calls (default=off)"));

  cl::opt<double>
  MaxInstructionTime("max-instruction-time",
                     cl::desc("Only allow a single instruction to take this much time (default=0s (off)). Enables --use-forked-solver"),
                     cl::init(0));

  cl::list<Executor::TerminateReason>
  ExitOnErrorType("exit-on-error-type",
		  cl::desc("Stop execution after reaching a specified condition.  (default=off)"),
		  cl::values(
		    clEnumValN(Executor::Abort, "Abort", "The program crashed"),
		    clEnumValN(Executor::Assert, "Assert", "An assertion was hit"),
        clEnumValN(Executor::BadVectorAccess, "BadVectorAccess", "Vector accessed out of bounds"),
		    clEnumValN(Executor::Exec, "Exec", "Trying to execute an unexpected instruction"),
		    clEnumValN(Executor::External, "External", "External objects referenced"),
		    clEnumValN(Executor::Free, "Free", "Freeing invalid memory"),
		    clEnumValN(Executor::Model, "Model", "Memory model limit hit"),
		    clEnumValN(Executor::Overflow, "Overflow", "An overflow occurred"),
		    clEnumValN(Executor::Ptr, "Ptr", "Pointer error"),
		    clEnumValN(Executor::ReadOnly, "ReadOnly", "Write to read-only memory"),
		    clEnumValN(Executor::ReportError, "ReportError", "klee_report_error called"),
		    clEnumValN(Executor::User, "User", "Wrong klee_* functions invocation"),
		    clEnumValN(Executor::Unhandled, "Unhandled", "Unhandled instruction hit"),
		    clEnumValEnd),
		  cl::ZeroOrMore);

  cl::opt<bool>
  ExitOnAllocFailure("exit-on-alloc-failure",
                     cl::desc("Stop execution if one of the tested program's memory allocations fails"),
                     cl::init(false));

  cl::opt<unsigned long long>
  StopAfterNInstructions("stop-after-n-instructions",
                         cl::desc("Stop testing after specified number of instructions (default=0 (off))"),
                         cl::init(0));

  cl::opt<unsigned long long>
  MaxTestInstructions("max-test-instructions",
                      cl::desc("Terminate each test after specified number of instructions (default=0 (off))"),
                      cl::init(0));

  cl::opt<unsigned>
  MaxTests("max-tests",
           cl::desc("Stop after running the specified number of tests (default=0 (off))"),
           cl::init(0));

  cl::opt<unsigned>
  MaxSymbolicTests("max-symbolic-tests",
                   cl::desc("Stop after symbolically executing specified number of tests (default=0 (off))"),
                   cl::init(0));

  cl::opt<unsigned>
  SkeletonDelay("skeleton-delay",
                cl::desc("Generate a new skeleton after this many tests that didn't cover new code (default=0 (off))"),
                cl::init(0));

  cl::opt<unsigned>
  MaxMemory("max-memory",
            cl::desc("Refuse to fork when above this amount of memory (in MB, default=2000)"),
            cl::init(2000));

  cl::opt<bool>
  MaxMemoryInhibit("max-memory-inhibit",
            cl::desc("Inhibit forking at memory cap (vs. random terminate) (default=on)"),
            cl::init(true));

  enum SymbolicPointerMethod {
    eSymbolicPointerSymbolic,
    eSymbolicPointerUnchecked,
    eSymbolicPointerFork,
    eSymbolicPointerPartial,
    eSymbolicPointerNone
  };

  cl::opt<SymbolicPointerMethod>
  SymbolicPointerMode("symbolic-pointers", cl::desc("Select how symbolic pointers are handled:"),
                      cl::values(clEnumValN(eSymbolicPointerSymbolic, "symbolic",
                                            "use symbolic addresses (default)"),
                                 clEnumValN(eSymbolicPointerUnchecked, "unchecked",
                                            "use symbolic addresses without checking for "
                                                "out-of-bounds or multiple resolutions"),
                                 clEnumValN(eSymbolicPointerFork, "fork",
                                            "fork on possible concrete addresses"),
                                 clEnumValN(eSymbolicPointerPartial, "partial",
                                            "concretize addresses after object resolution"),
                                 clEnumValN(eSymbolicPointerNone, "none",
                                            "concretize all addresses"),
                                 clEnumValEnd),
                      cl::init(eSymbolicPointerSymbolic));

  cl::opt<unsigned>
  MaxConcreteOffsets("max-concrete-offsets",
                     cl::desc("Maximum number of concrete offsets to fork when -symbolic-pointers=fork (default=8)"),
                     cl::init(8));

  cl::opt<bool>
  NoSymbolicFunctionPointers("no-symbolic-function-pointers",
                             cl::desc("Do not model symbolic function pointers (default=off)"),
                             cl::init(false));

  cl::opt<bool>
  NoSymbolicAllocations("no-symbolic-allocations",
                        cl::desc("Do not model allocations with symbolic sizes (default=off)"),
                        cl::init(false));

  cl::opt<unsigned>
  MaxSizesPerAllocation("max-sizes-per-allocation",
                            cl::desc("Maximum number of concrete sizes to explore "
                                         "from a symbolic allocation (default=3)"),
                            cl::init(3));

  cl::opt<bool>
  NoNonlinearArithmetic("no-nonlinear-arithmetic",
                        cl::desc("Do not model multiplication, division, or remainder (default=off)"),
                        cl::init(false));

  cl::opt<bool>
  ModelConcretization("model-concretization",
                      cl::desc("When concretizing values, add corresponding constraints (default=off). "
                                   "This prevents divergences from concretization but can unnecessarily "
                                   "restrict the search."),
                      cl::init(false));

  cl::opt<bool>
  ExitOnDivergence("exit-on-divergence",
                   cl::desc("Exit if concrete execution diverges from symbolic prediction (default=off)"),
                   cl::init(false));

  cl::opt<bool>
  GenerationalSearch("generational-search",
                     cl::desc("Use generational search as in SAGE (default=on)"),
                     cl::init(true));

  cl::opt<bool>
  CheckSymbolicDivergences("check-symbolic-divergences",
                           cl::desc("Check for divergences during symbolic execution (default=off). "
                                        "These can only happen if the environment is not fixed (e.g. the program "
                                        "accesses files) or there is a bug in Kontest. Checking for them can require "
                                        "a substantial amount of memory."),
                           cl::init(false));

  cl::opt<unsigned>
  RandomSeed("random-seed",
             cl::desc("Seed for random number generation, or 0 to "
                      "use a nondeterministic seed (default=1)."),
             cl::init(1));
}

///

namespace klee {
  RNG theRNG;
}

///

const char *Executor::TerminateReasonNames[] = {
  [ Abort ] = "abort",
  [ Assert ] = "assert",
  [ BadVectorAccess ] = "bad_vector_access",
  [ Divergence ] = "divergence",
  [ Exec ] = "exec",
  [ External ] = "external",
  [ Free ] = "free",
  [ Model ] = "model",
  [ Overflow ] = "overflow",
  [ Ptr ] = "ptr",
  [ ReadOnly ] = "readonly",
  [ ReportError ] = "reporterror",
  [ User ] = "user",
  [ Unhandled ] = "xxx",
};

Executor::TestComparatorType Executor::testComparator;

Executor::Executor(LLVMContext &ctx, const InterpreterOptions &opts,
    InterpreterHandler *ih)
    : Interpreter(opts),
      tests(testComparator),
      kmodule(0), interpreterHandler(ih),
      externalDispatcher(new ExternalDispatcher(ctx)), statsTracker(0),
      pathWriter(0), symPathWriter(0), specialFunctionHandler(0),
      processTree(0), replayKTest(0),
      atMemoryLimit(false), inhibitForking(false), haltExecution(false),
      coreSolverTimeout(MaxCoreSolverTime != 0 && MaxInstructionTime != 0
                            ? std::min(MaxCoreSolverTime, MaxInstructionTime)
                            : std::max(MaxCoreSolverTime, MaxInstructionTime)),
      debugInstFile(0), debugLogBuffer(debugBufferString) {

  if (coreSolverTimeout) UseForkedCoreSolver = true;
  Solver *coreSolver = klee::createCoreSolver(CoreSolverToUse);
  if (!coreSolver) {
    klee_error("Failed to create core solver\n");
  }

  Solver *solver = constructSolverChain(
      coreSolver,
      interpreterHandler->getOutputFilename(ALL_QUERIES_SMT2_FILE_NAME),
      interpreterHandler->getOutputFilename(SOLVER_QUERIES_SMT2_FILE_NAME),
      interpreterHandler->getOutputFilename(ALL_QUERIES_KQUERY_FILE_NAME),
      interpreterHandler->getOutputFilename(SOLVER_QUERIES_KQUERY_FILE_NAME));

  this->solver = new TimingSolver(solver, EqualitySubstitution);

  if (optionIsSet(DebugPrintInstructions, FILE_ALL) ||
      optionIsSet(DebugPrintInstructions, FILE_COMPACT) ||
      optionIsSet(DebugPrintInstructions, FILE_SRC)) {
    std::string debug_file_name =
        interpreterHandler->getOutputFilename("instructions.txt");
    std::string ErrorInfo;
#ifdef HAVE_ZLIB_H
    if (!DebugCompressInstructions) {
#endif

    std::error_code ec;
    debugInstFile = new llvm::raw_fd_ostream(debug_file_name.c_str(), ec,
                                             llvm::sys::fs::OpenFlags::F_Text);
    if (ec)
	    ErrorInfo = ec.message();
#ifdef HAVE_ZLIB_H
    } else {
      debugInstFile = new compressed_fd_ostream(
          (debug_file_name + ".gz").c_str(), ErrorInfo);
    }
#endif
    if (ErrorInfo != "") {
      klee_error("Could not open file %s : %s", debug_file_name.c_str(),
                 ErrorInfo.c_str());
    }
  }

  ScenarioHandler::handler.setExecutor(this);
}


const Module *Executor::setModule(llvm::Module *module,
                                  const ModuleOptions &opts) {
  assert(!kmodule && module && "can only register one module"); // XXX gross

  kmodule = new KModule(module);

  // Initialize the context.
  DataLayout *TD = kmodule->targetData;
  Context::initialize(TD->isLittleEndian(),
                      (Expr::Width) TD->getPointerSizeInBits());

  // set up MemoryManager; cannot do this earlier since we need to know
  // if the target program is 32-bit (so we can simulate 32-bit malloc)
  memory = new MemoryManager(&arrayCache);

  specialFunctionHandler = new SpecialFunctionHandler(*this);

  specialFunctionHandler->prepare();
  kmodule->prepare(opts, interpreterHandler);
  specialFunctionHandler->bind();

  if (StatsTracker::useStatistics()) {
    statsTracker =
      new StatsTracker(*this,
                       interpreterHandler->getOutputFilename("assembly.ll"),
                       false);
  }

  return module;
}

Executor::~Executor() {
  delete memory;
  delete externalDispatcher;
  if (processTree)
    delete processTree;
  if (specialFunctionHandler)
    delete specialFunctionHandler;
  if (statsTracker)
    delete statsTracker;
  delete solver;
  delete kmodule;
  while(!timers.empty()) {
    delete timers.back();
    timers.pop_back();
  }
  if (debugInstFile) {
    delete debugInstFile;
  }
}

/***/

void Executor::initializeGlobalObject(ExecutionState &state, ObjectState *os,
                                      const Constant *c,
                                      unsigned offset) {
  DataLayout *targetData = kmodule->targetData;
  if (const ConstantVector *cp = dyn_cast<ConstantVector>(c)) {
    unsigned elementSize =
      targetData->getTypeStoreSize(cp->getType()->getElementType());
    for (unsigned i=0, e=cp->getNumOperands(); i != e; ++i)
      initializeGlobalObject(state, os, cp->getOperand(i),
			     offset + i*elementSize);
  } else if (isa<ConstantAggregateZero>(c)) {
    unsigned i, size = targetData->getTypeStoreSize(c->getType());
    for (i=0; i<size; i++)
      os->write8(offset+i, (uint8_t) 0);
  } else if (const ConstantArray *ca = dyn_cast<ConstantArray>(c)) {
    unsigned elementSize =
      targetData->getTypeStoreSize(ca->getType()->getElementType());
    for (unsigned i=0, e=ca->getNumOperands(); i != e; ++i)
      initializeGlobalObject(state, os, ca->getOperand(i),
			     offset + i*elementSize);
  } else if (const ConstantStruct *cs = dyn_cast<ConstantStruct>(c)) {
    const StructLayout *sl =
      targetData->getStructLayout(cast<StructType>(cs->getType()));
    for (unsigned i=0, e=cs->getNumOperands(); i != e; ++i)
      initializeGlobalObject(state, os, cs->getOperand(i),
			     offset + sl->getElementOffset(i));
  } else if (const ConstantDataSequential *cds =
               dyn_cast<ConstantDataSequential>(c)) {
    unsigned elementSize =
      targetData->getTypeStoreSize(cds->getElementType());
    for (unsigned i=0, e=cds->getNumElements(); i != e; ++i)
      initializeGlobalObject(state, os, cds->getElementAsConstant(i),
                             offset + i*elementSize);
  } else if (!isa<UndefValue>(c)) {
    unsigned StoreBits = targetData->getTypeStoreSizeInBits(c->getType());
    ref<ConstantExpr> C = evalConstant(c);

    // Extend the constant if necessary;
    assert(StoreBits >= C->getWidth() && "Invalid store size!");
    if (StoreBits > C->getWidth())
      C = C->ZExt(StoreBits);

    os->write(offset, C);
  }
}

MemoryObject * Executor::addExternalObject(ExecutionState &state,
                                           void *addr, unsigned size,
                                           bool isReadOnly) {
  MemoryObject *mo = memory->allocateFixed((uint64_t) (unsigned long) addr,
                                           size, 0);
  ObjectState *os = bindObjectInState(state, mo, false);
  for(unsigned i = 0; i < size; i++)
    os->write8(i, ((uint8_t*)addr)[i]);
  if(isReadOnly)
    os->setReadOnly(true);
  return mo;
}


extern void *__dso_handle __attribute__ ((__weak__));

void Executor::initializeGlobals(ExecutionState &state) {
  Module *m = kmodule->module;

  if (m->getModuleInlineAsm() != "")
    klee_warning("executable has module level assembly (ignoring)");
  // represent function globals using the address of the actual llvm function
  // object. given that we use malloc to allocate memory in states this also
  // ensures that we won't conflict. we don't need to allocate a memory object
  // since reading/writing via a function pointer is unsupported anyway.
  for (Module::iterator i = m->begin(), ie = m->end(); i != ie; ++i) {
    Function *f = static_cast<Function *>(i);
    ref<ConstantExpr> addr(0);

    // If the symbol has external weak linkage then it is implicitly
    // not defined in this module; if it isn't resolvable then it
    // should be null.
    if (f->hasExternalWeakLinkage() &&
        !externalDispatcher->resolveSymbol(f->getName())) {
      addr = memory->createNullPointer();
    } else {
      addr = memory->createPointer((unsigned long) (void*) f);
      legalFunctions.insert((uint64_t) (unsigned long) (void*) f);
    }

    globalAddresses.insert(std::make_pair(f, addr));
  }

  // Disabled, we don't want to promote use of live externals.
#ifdef HAVE_CTYPE_EXTERNALS
#ifndef WINDOWS
#ifndef DARWIN
  /* From /usr/include/errno.h: it [errno] is a per-thread variable. */
  int *errno_addr = __errno_location();
  addExternalObject(state, (void *)errno_addr, sizeof *errno_addr, false);

  /* from /usr/include/ctype.h:
       These point into arrays of 384, so they can be indexed by any `unsigned
       char' value [0,255]; by EOF (-1); or by any `signed char' value
       [-128,-1).  ISO C requires that the ctype functions work for `unsigned */
  const uint16_t **addr = __ctype_b_loc();
  addExternalObject(state, const_cast<uint16_t*>(*addr-128),
                    384 * sizeof **addr, true);
  addExternalObject(state, addr, sizeof(*addr), true);

  const int32_t **lower_addr = __ctype_tolower_loc();
  addExternalObject(state, const_cast<int32_t*>(*lower_addr-128),
                    384 * sizeof **lower_addr, true);
  addExternalObject(state, lower_addr, sizeof(*lower_addr), true);

  const int32_t **upper_addr = __ctype_toupper_loc();
  addExternalObject(state, const_cast<int32_t*>(*upper_addr-128),
                    384 * sizeof **upper_addr, true);
  addExternalObject(state, upper_addr, sizeof(*upper_addr), true);
#endif
#endif
#endif

  // allocate and initialize globals, done in two passes since we may
  // need address of a global in order to initialize some other one.

  // allocate memory objects for all globals
  for (Module::const_global_iterator i = m->global_begin(),
         e = m->global_end();
       i != e; ++i) {
    const GlobalVariable *v = static_cast<const GlobalVariable *>(i);
    size_t globalObjectAlignment = getAllocationAlignment(v);
    if (i->isDeclaration()) {
      // FIXME: We have no general way of handling unknown external
      // symbols. If we really cared about making external stuff work
      // better we could support user definition, or use the EXE style
      // hack where we check the object file information.

      Type *ty = i->getType()->getElementType();
      uint64_t size = 0;
      if (ty->isSized()) {
	size = kmodule->targetData->getTypeStoreSize(ty);
      } else {
        klee_warning("Type for %.*s is not sized", (int)i->getName().size(),
			i->getName().data());
      }

      // XXX - DWD - hardcode some things until we decide how to fix.
#ifndef WINDOWS
      if (i->getName() == "_ZTVN10__cxxabiv117__class_type_infoE") {
        size = 0x2C;
      } else if (i->getName() == "_ZTVN10__cxxabiv120__si_class_type_infoE") {
        size = 0x2C;
      } else if (i->getName() == "_ZTVN10__cxxabiv121__vmi_class_type_infoE") {
        size = 0x2C;
      }
#endif

      if (size == 0) {
        klee_warning("Unable to find size for global variable: %.*s (use will result in out of bounds access)",
			(int)i->getName().size(), i->getName().data());
      }

      MemoryObject *mo = memory->allocate(size, /*isLocal=*/false,
                                          /*isGlobal=*/true, /*allocSite=*/v,
                                          /*alignment=*/globalObjectAlignment);
      if (!mo)
        klee_error("Unable to allocate memory for globals");
      ObjectState *os = bindObjectInState(state, mo, false);
      globalObjects.insert(std::make_pair(v, mo));
      globalAddresses.insert(std::make_pair(v, mo->getBaseExpr()));

      // Program already running = object already initialized.  Read
      // concrete value and write it to our copy.
      if (size) {
        void *addr;
        if (i->getName() == "__dso_handle") {
          addr = &__dso_handle; // wtf ?
        } else {
          addr = externalDispatcher->resolveSymbol(i->getName());
        }
        if (!addr)
          klee_error("unable to load symbol(%s) while initializing globals.",
                     i->getName().data());

        for (unsigned offset=0; offset<mo->size; offset++)
          os->write8(offset, ((unsigned char*)addr)[offset]);
      }
    } else {
      Type *ty = i->getType()->getElementType();
      uint64_t size = kmodule->targetData->getTypeStoreSize(ty);
      MemoryObject *mo = memory->allocate(size, /*isLocal=*/false,
                                          /*isGlobal=*/true, /*allocSite=*/v,
                                          /*alignment=*/globalObjectAlignment);
      if (!mo)
        klee_error("Unable to allocate memory for globals");
      ObjectState *os = bindObjectInState(state, mo, false);
      globalObjects.insert(std::make_pair(v, mo));
      globalAddresses.insert(std::make_pair(v, mo->getBaseExpr()));

      if (!i->hasInitializer())
          os->initializeToRandom();
    }
  }

  // link aliases to their definitions (if bound)
  for (Module::alias_iterator i = m->alias_begin(), ie = m->alias_end();
       i != ie; ++i) {
    // Map the alias to its aliasee's address. This works because we have
    // addresses for everything, even undefined functions.
    globalAddresses.insert(std::make_pair(static_cast<GlobalAlias *>(i),
	  evalConstant(i->getAliasee())));
  }

  // once all objects are allocated, do the actual initialization
  for (Module::const_global_iterator i = m->global_begin(),
         e = m->global_end();
       i != e; ++i) {
    if (i->hasInitializer()) {
      const GlobalVariable *v = static_cast<const GlobalVariable *>(i);
      MemoryObject *mo = globalObjects.find(v)->second;
      ObjectState *os = state.addressSpace.findObject(mo);
      assert(os);

      initializeGlobalObject(state, os, i->getInitializer(), 0);
      // if(i->isConstant()) os->setReadOnly(true);
    }
  }
}

void Executor::branch(ExecutionState &state,
                      const std::vector< ref<Expr> > &conditions,
                      std::vector<ExecutionState*> &result) {
  TimerStatIncrementer timer(stats::forkTime);
  unsigned N = conditions.size();
  assert(N);

  stats::forks += N-1;

  // XXX do proper balance or keep random?
  result.push_back(&state);
  for (unsigned i=1; i<N; ++i) {
    ExecutionState *es = result[theRNG.getInt32() % i];
    ExecutionState *ns = es->branch();
    addedStates.push_back(ns);
    result.push_back(ns);
  }

  for (unsigned i=0; i<N; ++i)
    if (result[i])
      addConstraint(*result[i], conditions[i]);
}

Executor::StatePair 
Executor::fork(ExecutionState &current, ref<Expr> condition, bool concreteCondition, bool isInternal) {
  bool branchingEnabled = !(current.forkDisabled || inhibitForking);
  bool testBranch = branchingEnabled && !runningConcretely;

  if (branchingEnabled) {
    unsigned forkDepth = current.forkHistory.size();
    if (forkDepth < currentTest->branchDepth) {   // branch should occur according to expectedPath
      bool checkForDivergence = runningConcretely || CheckSymbolicDivergences;
      bool divergence = false;
      if (checkForDivergence) {
        assert(currentTest->expectedPath.size() > forkDepth && "missing expected fork!");
        if (concreteCondition != currentTest->expectedPath[forkDepth]) {
          divergence = true;
          handleDivergence(current, "unexpected branch");
          if (testTerminated)
            return StatePair(0, 0);
        }
      }
      if (!divergence) {
        testBranch = false;   // this branch was already generated by a previous test
      }
    } else {
      testIsFertile = true;
    }
  }

  bool doBranch = false;
  if (testBranch) {
    // Check feasibility of branch
    solver->setTimeout(coreSolverTimeout);
    bool success;
    if (concreteCondition) {
      success = solver->mayBeFalse(current, condition, doBranch);
    } else {
      success = solver->mayBeTrue(current, condition, doBranch);
    }
    solver->setTimeout(0);
    if (!success) {
      // Continue with concrete branch of the fork
      doBranch = false;
    }
  }

  if (!doBranch) {    // branch disabled, infeasible, uncheckable due to timeout, or already generated
    current.forkHistory.push_back(concreteCondition);
    if (!isInternal && pathWriter)
      current.pathOS << (concreteCondition ? "1" : "0");

    if (concreteCondition) {
      if (!runningConcretely)
        addConstraint(current, condition);
      return StatePair(&current, 0);
    } else {
      if (!runningConcretely)
        addConstraint(current, Expr::createIsZero(condition));
      return StatePair(0, &current);
    }
  } else {    // branch enabled and feasible
    assert(!replayKTest && "in replay mode, only one branch can be true.");

    TimerStatIncrementer timer(stats::forkTime);
    ++stats::forks;

    ConstraintManager newConditions = current.constraints;
    ref<Expr> negatedCondition = Expr::createIsZero(condition);
    if (concreteCondition) {
      addConstraint(current, condition);
      newConditions.addConstraint(negatedCondition);
    } else {
      addConstraint(current, negatedCondition);
      newConditions.addConstraint(condition);
    }

    // Generate test leading to the new state (hopefully)
    std::unique_ptr<ConcolicTest> test = generateConcolicTest(current, newConditions);
    test->pushBranch(!concreteCondition);
    childTests.push_back(std::move(test));

    current.forkHistory.push_back(concreteCondition);
    if (!isInternal && pathWriter)
      current.pathOS << (concreteCondition ? "1" : "0");

    if (concreteCondition)
      return StatePair(&current, 0);
    else
      return StatePair(0, &current);
  }
}

void Executor::forkResolutions(ExecutionState &current, ResolutionList &rl, ref<Expr> address, int bytes, const ref <ConstantExpr> &outOfBoundsAddr) {
  assert(!replayKTest && !runningConcretely && "multiple resolutions while running concretely!");
  for (const ObjectPair &op : rl) {
    TimerStatIncrementer timer(stats::forkTime);
    ++stats::forks;

    const MemoryObject *mo = op.first;
    ref<Expr> inBounds = createResolutionConstraint(mo, bytes, address, ref<ConstantExpr>());
    ConstraintManager newConditions = current.constraints;
    newConditions.addConstraint(inBounds);

    std::unique_ptr<ConcolicTest> test = generateConcolicTest(current, newConditions);
    assert(mo->id < UINT_MAX && "incredibly large number of MemoryObjects allocated!");
    test->pushResolution(mo->id);
    childTests.push_back(std::move(test));
  }

  if (!outOfBoundsAddr.isNull()) {    // Fork a new state with the out-of-bounds address
    TimerStatIncrementer timer(stats::forkTime);
    ++stats::forks;

    ConstraintManager newConditions = current.constraints;
    newConditions.addConstraint(createResolutionConstraint(0, bytes, address, outOfBoundsAddr));

    std::unique_ptr<ConcolicTest> test = generateConcolicTest(current, newConditions);
    test->pushResolution(UINT_MAX);
    childTests.push_back(std::move(test));
  }
}

void Executor::addConstraint(ExecutionState &state, ref<Expr> condition) {
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(condition)) {
    if (!CE->isTrue())
      llvm::report_fatal_error("attempt to add invalid constraint");
    return;
  }

  state.addConstraint(condition);
  if (UseIVC)
    doImpliedValueConcretization(state, condition, ConstantExpr::True);
}

const Cell& Executor::eval(KInstruction *ki, unsigned index,
                           ExecutionState &state) const {
  assert(index < ki->inst->getNumOperands());
  int vnumber = ki->operands[index];

  assert(vnumber != -1 &&
         "Invalid operand to eval(), not a value or constant!");

  // Determine if this is a constant or not.
  if (vnumber < 0) {
    unsigned index = -vnumber - 2;
    return kmodule->constantTable[index];
  } else {
    unsigned index = vnumber;
    StackFrame &sf = state.stack.back();
    return sf.locals[index];
  }
}

void Executor::bindLocal(KInstruction *target, ExecutionState &state,
                         ref<Expr> value, llvm::APInt concreteValue) {
  if (ConstantExpr *ce __attribute__ ((unused)) = dyn_cast<ConstantExpr>(value))
    assert(ce->getAPValue() == concreteValue && "concreteValue does not match value!");

  Cell &cell = getDestCell(state, target);
  cell.value = value;
  cell.concreteValue = concreteValue;
}

void Executor::bindArgument(KFunction *kf, unsigned index,
                            ExecutionState &state, ref<Expr> value, llvm::APInt concreteValue) {
  if (ConstantExpr *ce __attribute__ ((unused)) = dyn_cast<ConstantExpr>(value))
    assert(ce->getAPValue() == concreteValue && "concreteValue does not match value!");

  Cell &cell = getArgumentCell(state, kf, index);
  cell.value = value;
  cell.concreteValue = concreteValue;
}

void Executor::bindLocal(KInstruction *target, ExecutionState &state,
                         ref<ConstantExpr> value) {
  bindLocal(target, state, value, value->getAPValue());
}

void Executor::bindLocal(KInstruction *target, ExecutionState &state,
                         llvm::APInt value) {
  bindLocal(target, state, ConstantExpr::alloc(value), value);
}

void Executor::bindArgument(KFunction *kf, unsigned index,
                            ExecutionState &state, ref<ConstantExpr> value) {
  bindArgument(kf, index, state, value, value->getAPValue());
}

ref<Expr> Executor::toUnique(const ExecutionState &state,
                             ref<Expr> &e) {
  ref<Expr> result = e;

  if (!isa<ConstantExpr>(e)) {
    assert(!runningConcretely && "non-constant toUnique expression in concrete execution!");
    ref<ConstantExpr> value;
    bool isTrue = false;

    solver->setTimeout(coreSolverTimeout);
    if (solver->getValue(state, e, value) &&
        solver->mustBeTrue(state, EqExpr::create(e, value), isTrue) &&
        isTrue)
      result = value;
    solver->setTimeout(0);
  }

  return result;
}


/* Concretize the given expression, and return a possible constant value. 
   'reason' is just a documentation string stating the reason for concretization. */
ref<klee::ConstantExpr>
Executor::toConstant(ExecutionState &state,
                     const Cell &cell,
                     const char *reason) {
  return toConstant(state, cell.value, cell.concreteValue, reason);
}

ref<klee::ConstantExpr>
Executor::toConstant(ExecutionState &state,
                     const ref<Expr> &e,
                     const llvm::APInt &concrete,
                     const char *reason) {
  return toConstant(state, e, ConstantExpr::alloc(concrete), reason);
}

ref<klee::ConstantExpr>
Executor::toConstant(ExecutionState &state,
                     const ref<Expr> &e,
                     const ref<ConstantExpr> &concrete,
                     const char *reason) {
  if (ConstantExpr *CE __attribute__ ((unused)) = dyn_cast<ConstantExpr>(e)) {
    assert(*CE == *concrete && "concreteValue does not match ConstantExpr!");
    return concrete;
  }

  if (WarnOnConcretization) {
    const InstructionInfo *ii = state.prevPC->info;
    if (ConcretizeVerbosely) {    // old KLEE warning, pretty-printing the expression in question
      std::string str;
      llvm::raw_string_ostream os(str);
      os << "silently concretizing (reason: " << reason << ") expression " << e
         << " to value " << concrete << " (" << ii->file << ":"
         << ii->line << ")";

      if (AllExternalWarnings)
        klee_warning(reason, os.str().c_str());
      else
        klee_warning_once(reason, "%s", os.str().c_str());
    } else {    // Kontest warning, reporting once per line and without the expression
      warning_once(ii, "silently concretizing (reason: %s)", reason);
    }
  }

  if (ModelConcretization)
    addConstraint(state, EqExpr::create(e, concrete));
    
  return concrete;
}

void Executor::executeGetValue(ExecutionState &state,
                               ref<Expr> e,
                               const llvm::APInt &concrete,
                               KInstruction *target) {
#ifndef NDEBUG
  e = state.constraints.simplifyExpr(e);
  if (ConstantExpr *ce = dyn_cast<ConstantExpr>(e))
    assert(ce->getAPValue() == concrete && "concrete does not match ConstantExpr!");
#endif    // NDEBUG
  bindLocal(target, state, concrete);
}

void Executor::executeMakeConcrete(ExecutionState &state,
                                   ref<Expr> e,
                                   const llvm::APInt &concrete,
                                   KInstruction *target) {
  e = state.constraints.simplifyExpr(e);
  ref<ConstantExpr> cval;
  if (ConstantExpr *ce = dyn_cast<ConstantExpr>(e)) {
    assert(ce->getAPValue() == concrete && "concrete does not match ConstantExpr!");
    cval = ce;
  } else {
    cval = ConstantExpr::alloc(concrete);
    state.addConstraint(EqExpr::create(e, cval));
  }
  bindLocal(target, state, cval, concrete);
}

void Executor::printDebugInstructions(ExecutionState &state) {
  // check do not print
  if (DebugPrintInstructions.size() == 0)
	  return;

  llvm::raw_ostream *stream = 0;
  if (optionIsSet(DebugPrintInstructions, STDERR_ALL) ||
      optionIsSet(DebugPrintInstructions, STDERR_SRC) ||
      optionIsSet(DebugPrintInstructions, STDERR_COMPACT))
    stream = &llvm::errs();
  else
    stream = &debugLogBuffer;

  if (!optionIsSet(DebugPrintInstructions, STDERR_COMPACT) &&
      !optionIsSet(DebugPrintInstructions, FILE_COMPACT)) {
    (*stream) << "     ";
    state.pc->printFileLine(*stream);
    (*stream) << ":";
  }

  (*stream) << state.pc->info->id;

  if (optionIsSet(DebugPrintInstructions, STDERR_ALL) ||
      optionIsSet(DebugPrintInstructions, FILE_ALL))
    (*stream) << ":" << *(state.pc->inst);
  (*stream) << "\n";

  if (optionIsSet(DebugPrintInstructions, FILE_ALL) ||
      optionIsSet(DebugPrintInstructions, FILE_COMPACT) ||
      optionIsSet(DebugPrintInstructions, FILE_SRC)) {
    debugLogBuffer.flush();
    (*debugInstFile) << debugLogBuffer.str();
    debugBufferString = "";
  }
}

bool Executor::stepInstruction(ExecutionState &state) {
  if (MaxTestInstructions && testInstructions >= MaxTestInstructions) {
    terminateStateEarly(state, "Maximum number of instructions reached.", true);
    return true;
  }

  printDebugInstructions(state);
  if (statsTracker)
    statsTracker->stepInstruction(state);

  ++stats::instructions;
  ++testInstructions;
  state.prevPC = state.pc;
  ++state.pc;

  if (stats::instructions==StopAfterNInstructions) {
    haltExecution = true;
    return true;
  }

  return false;
}

void Executor::executeCall(ExecutionState &state,
                           KInstruction *ki,
                           Function *f,
                           std::vector< ref<Expr> > &arguments,
                           const llvm::APInt *concreteArguments) {
  Instruction *i = ki->inst;
  if (f && f->isDeclaration()) {
    switch(f->getIntrinsicID()) {
    case Intrinsic::not_intrinsic:
      // state may be destroyed by this call, cannot touch
      callExternalFunction(state, ki, f, arguments, concreteArguments);
      break;
        
      // va_arg is handled by caller and intrinsic lowering, see comment for
      // ExecutionState::varargs
    case Intrinsic::vastart:  {
      StackFrame &sf = state.stack.back();

      // varargs can be zero if no varargs were provided
      if (!sf.varargs)
        return;

      // FIXME: This is really specific to the architecture, not the pointer
      // size. This happens to work fir x86-32 and x86-64, however.
      Expr::Width WordSize = Context::get().getPointerWidth();
      uint64_t base = concreteArguments[0].getZExtValue();
      if (WordSize == Expr::Int32) {
        executeMemoryOperation(state, true, arguments[0], base,
                               sf.varargs->getBaseExpr(), 0);
      } else {
        assert(WordSize == Expr::Int64 && "Unknown word size!");

        // X86-64 has quite complicated calling convention. However,
        // instead of implementing it, we can do a simple hack: just
        // make a function believe that all varargs are on stack.
        executeMemoryOperation(state, true, arguments[0], base,
                               ConstantExpr::create(48, 32), 0); // gp_offset
        executeMemoryOperation(state, true,
                               AddExpr::create(arguments[0],
                                               ConstantExpr::create(4, 64)),
                               base + 4,
                               ConstantExpr::create(304, 32), 0); // fp_offset
        executeMemoryOperation(state, true,
                               AddExpr::create(arguments[0], 
                                               ConstantExpr::create(8, 64)),
                               base + 8,
                               sf.varargs->getBaseExpr(), 0); // overflow_arg_area
        executeMemoryOperation(state, true,
                               AddExpr::create(arguments[0], 
                                               ConstantExpr::create(16, 64)),
                               base + 16,
                               ConstantExpr::create(0, 64), 0); // reg_save_area
      }
      break;
    }
    case Intrinsic::vaend:
      // va_end is a noop for the interpreter.
      //
      // FIXME: We should validate that the target didn't do something bad
      // with vaeend, however (like call it twice).
      break;
        
    case Intrinsic::vacopy:
      // va_copy should have been lowered.
      //
      // FIXME: It would be nice to check for errors in the usage of this as
      // well.
    default:
      error("unknown intrinsic: %s", f->getName().data());
    }

    if (InvokeInst *ii = dyn_cast<InvokeInst>(i))
      transferToBasicBlock(ii->getNormalDest(), i->getParent(), state);
  } else {
    // FIXME: I'm not really happy about this reliance on prevPC but it is ok, I
    // guess. This just done to avoid having to pass KInstIterator everywhere
    // instead of the actual instruction, since we can't make a KInstIterator
    // from just an instruction (unlike LLVM).
    KFunction *kf = kmodule->functionMap[f];
    state.pushFrame(state.prevPC, kf);
    state.pc = kf->instructions;

    if (statsTracker)
      statsTracker->framePushed(state, &state.stack[state.stack.size()-2]);

    if (kf->trackCoverage)
      recordEnteringBlock(&f->getEntryBlock());

     // TODO: support "byval" parameter attribute
     // TODO: support zeroext, signext, sret attributes

    unsigned callingArgs = arguments.size();
    unsigned funcArgs = f->arg_size();
    if (!f->isVarArg()) {
      if (callingArgs > funcArgs) {
        warning_once(f, "calling %s with extra arguments.",
                     f->getName().data());
      } else if (callingArgs < funcArgs) {
        terminateStateOnError(state, "calling function with too few arguments",
                              User);
        return;
      }
    } else {
      Expr::Width WordSize = Context::get().getPointerWidth();

      if (callingArgs < funcArgs) {
        terminateStateOnError(state, "calling function with too few arguments",
                              User);
        return;
      }

      StackFrame &sf = state.stack.back();
      unsigned size = 0;
      bool requires16ByteAlignment = false;
      for (unsigned i = funcArgs; i < callingArgs; i++) {
        // FIXME: This is really specific to the architecture, not the pointer
        // size. This happens to work for x86-32 and x86-64, however.
        if (WordSize == Expr::Int32) {
          size += Expr::getMinBytesForWidth(arguments[i]->getWidth());
        } else {
          Expr::Width argWidth = arguments[i]->getWidth();
          // AMD64-ABI 3.5.7p5: Step 7. Align l->overflow_arg_area upwards to a
          // 16 byte boundary if alignment needed by type exceeds 8 byte
          // boundary.
          //
          // Alignment requirements for scalar types is the same as their size
          if (argWidth > Expr::Int64) {
             size = llvm::RoundUpToAlignment(size, 16);
             requires16ByteAlignment = true;
          }
          size += llvm::RoundUpToAlignment(argWidth, WordSize) / 8;
        }
      }

      MemoryObject *mo = sf.varargs =
          memory->allocate(size, true, false, state.prevPC->inst,
                           (requires16ByteAlignment ? 16 : 8));
      if (!mo && size) {
        terminateStateOnExecError(state, "out of memory (varargs)");
        return;
      }

      if (mo) {
        if ((WordSize == Expr::Int64) && (mo->address & 15) &&
            requires16ByteAlignment) {
          // Both 64bit Linux/Glibc and 64bit MacOSX should align to 16 bytes.
          klee_warning_once(
              0, "While allocating varargs: malloc did not align to 16 bytes.");
        }

        ObjectState *os = bindObjectInState(state, mo, true);
        unsigned offset = 0;
        for (unsigned i = funcArgs; i < callingArgs; i++) {
          // FIXME: This is really specific to the architecture, not the pointer
          // size. This happens to work for x86-32 and x86-64, however.
          if (WordSize == Expr::Int32) {
            os->write(offset, arguments[i], concreteArguments[i]);
            offset += Expr::getMinBytesForWidth(arguments[i]->getWidth());
          } else {
            assert(WordSize == Expr::Int64 && "Unknown word size!");

            Expr::Width argWidth = arguments[i]->getWidth();
            if (argWidth > Expr::Int64) {
              offset = llvm::RoundUpToAlignment(offset, 16);
            }
            os->write(offset, arguments[i], concreteArguments[i]);
            offset += llvm::RoundUpToAlignment(argWidth, WordSize) / 8;
          }
        }
      }
    }

    unsigned numFormals = f->arg_size();
    for (unsigned i=0; i<numFormals; ++i) 
      bindArgument(kf, i, state, arguments[i], concreteArguments[i]);
  }
}

void Executor::transferToBasicBlock(BasicBlock *dst, BasicBlock *src,
                                    ExecutionState &state) {
  // Note that in general phi nodes can reuse phi values from the same
  // block but the incoming value is the eval() result *before* the
  // execution of any phi nodes. this is pathological and doesn't
  // really seem to occur, but just in case we run the PhiCleanerPass
  // which makes sure this cannot happen and so it is safe to just
  // eval things in order. The PhiCleanerPass also makes sure that all
  // incoming blocks have the same order for each PHINode so we only
  // have to compute the index once.
  //
  // With that done we simply set an index in the state so that PHI
  // instructions know which argument to eval, set the pc, and continue.

  // XXX this lookup has to go ?
  KFunction *kf = state.stack.back().kf;
  unsigned entry = kf->basicBlockEntry[dst];
  state.pc = &kf->instructions[entry];
  if (state.pc->inst->getOpcode() == Instruction::PHI) {
    PHINode *first = static_cast<PHINode*>(state.pc->inst);
    state.incomingBBIndex = first->getBasicBlockIndex(src);
  }

  if (kf->trackCoverage)
    recordEnteringBlock(dst);
}

void Executor::recordEnteringBlock(const BasicBlock *block) {
  std::pair<std::unordered_set<const BasicBlock *>::iterator, bool> res = blocksCovered.insert(block);
  if (res.second) {   // dst was not previously covered
    ++newBlocksCovered;
    currentTestState->coveredNew = true;    // set this in case IStats monitoring is disabled
  }
}

/// Compute the true target of a function call, resolving LLVM and KLEE aliases
/// and bitcasts.
Function* Executor::getTargetFunction(Value *calledVal, ExecutionState &state) {
  SmallPtrSet<const GlobalValue*, 3> Visited;

  Constant *c = dyn_cast<Constant>(calledVal);
  if (!c)
    return 0;

  while (true) {
    if (GlobalValue *gv = dyn_cast<GlobalValue>(c)) {
      if (!Visited.insert(gv).second)
        return 0;
      std::string alias = state.getFnAlias(gv->getName());
      if (alias != "") {
        llvm::Module* currModule = kmodule->module;
        GlobalValue *old_gv = gv;
        gv = currModule->getNamedValue(alias);
        if (!gv) {
          klee_error("Function %s(), alias for %s not found!\n", alias.c_str(),
                     old_gv->getName().str().c_str());
        }
      }

      if (Function *f = dyn_cast<Function>(gv))
        return f;
      else if (GlobalAlias *ga = dyn_cast<GlobalAlias>(gv))
        c = ga->getAliasee();
      else
        return 0;
    } else if (llvm::ConstantExpr *ce = dyn_cast<llvm::ConstantExpr>(c)) {
      if (ce->getOpcode()==Instruction::BitCast)
        c = ce->getOperand(0);
      else
        return 0;
    } else
      return 0;
  }
}

/// TODO remove?
static bool isDebugIntrinsic(const Function *f, KModule *KM) {
  return false;
}

static inline const llvm::fltSemantics * fpWidthToSemantics(unsigned width) {
  switch(width) {
  case Expr::Int32:
    return &llvm::APFloat::IEEEsingle;
  case Expr::Int64:
    return &llvm::APFloat::IEEEdouble;
  case Expr::Fl80:
    return &llvm::APFloat::x87DoubleExtended;
  default:
    return 0;
  }
}

void Executor::executeInstruction(ExecutionState &state, KInstruction *ki) {
  Instruction *i = ki->inst;
  switch (i->getOpcode()) {
    // Control flow
  case Instruction::Ret: {
    ReturnInst *ri = cast<ReturnInst>(i);
    KInstIterator kcaller = state.stack.back().caller;
    Instruction *caller = kcaller ? kcaller->inst : 0;
    bool isVoidReturn = (ri->getNumOperands() == 0);
    ref<Expr> result = ConstantExpr::False;
    llvm::APInt concreteResult = APInt(Expr::Bool, 0);
    
    if (!isVoidReturn) {
      const Cell &cell = eval(ki, 0, state);
      result = cell.value;
      concreteResult = cell.concreteValue;
    }
    
    if (state.stack.size() <= 1) {
      assert(!caller && "caller set on initial stack frame");
      terminateStateOnExit(state);
    } else {
      state.popFrame();

      if (statsTracker)
        statsTracker->framePopped(state);

      if (InvokeInst *ii = dyn_cast<InvokeInst>(caller)) {
        transferToBasicBlock(ii->getNormalDest(), caller->getParent(), state);
      } else {
        state.pc = kcaller;
        ++state.pc;
      }

      if (!isVoidReturn) {
        Type *t = caller->getType();
        if (t != Type::getVoidTy(i->getContext())) {
          // may need to do coercion due to bitcasts
          Expr::Width from = result->getWidth();
          Expr::Width to = getWidthForLLVMType(t);
            
          if (from != to) {
            CallSite cs = (isa<InvokeInst>(caller) ? CallSite(cast<InvokeInst>(caller)) : 
                           CallSite(cast<CallInst>(caller)));

            // XXX need to check other param attrs ?
            bool isSExt = cs.paramHasAttr(0, llvm::Attribute::SExt);
            if (isSExt) {
              result = SExtExpr::create(result, to);
              concreteResult = concreteResult.sextOrTrunc(to);
            } else {
              result = ZExtExpr::create(result, to);
              concreteResult = concreteResult.zextOrTrunc(to);
            }
          }

          bindLocal(kcaller, state, result, concreteResult);
        }
      } else {
        // We check that the return value has no users instead of
        // checking the type, since C defaults to returning int for
        // undeclared functions.
        if (!caller->use_empty()) {
          terminateStateOnExecError(state, "return void when caller expected a result");
        }
      }
    }      
    break;
  }

  case Instruction::Br: {
    BranchInst *bi = cast<BranchInst>(i);
    if (bi->isUnconditional()) {
      transferToBasicBlock(bi->getSuccessor(0), bi->getParent(), state);
    } else {
      // FIXME: Find a way that we don't have this hidden dependency.
      assert(bi->getCondition() == bi->getOperand(0) &&
             "Wrong operand index!");
      const Cell &cell = eval(ki, 0, state);
      Executor::StatePair branches = fork(state, cell.value, cell.concreteValue.getBoolValue(), false);

      // NOTE: There is a hidden dependency here, markBranchVisited
      // requires that we still be in the context of the branch
      // instruction (it reuses its statistic id). Should be cleaned
      // up with convenient instruction specific data.
      if (statsTracker && state.stack.back().kf->trackCoverage)
        statsTracker->markBranchVisited(branches.first, branches.second);

      if (branches.first)
        transferToBasicBlock(bi->getSuccessor(0), bi->getParent(), *branches.first);
      if (branches.second)
        transferToBasicBlock(bi->getSuccessor(1), bi->getParent(), *branches.second);
    }
    break;
  }
  case Instruction::Switch: {   // TODO put back native switch handling?
    terminateStateOnExecError(state, "unhandled switch operation");
    break;
 }
  case Instruction::Unreachable:
    // Note that this is not necessarily an internal bug, llvm will
    // generate unreachable instructions in cases where it knows the
    // program will crash. So it is effectively a SEGV or internal
    // error.
    terminateStateOnExecError(state, "reached \"unreachable\" instruction");
    break;

  case Instruction::Invoke:
  case Instruction::Call: {
    CallSite cs(i);

    unsigned numArgs = cs.arg_size();
    Value *fp = cs.getCalledValue();
    Function *f = getTargetFunction(fp, state);

    // Skip debug intrinsics, we can't evaluate their metadata arguments.
    if (f && isDebugIntrinsic(f, kmodule))
      break;

    if (isa<InlineAsm>(fp)) {
      terminateStateOnExecError(state, "inline assembly is unsupported");
      break;
    }
    // evaluate arguments
    std::vector< ref<Expr> > arguments;
    llvm::APInt *concreteArguments = new llvm::APInt[numArgs];
    arguments.reserve(numArgs);

    for (unsigned j=0; j<numArgs; ++j) {
      const Cell &cell = eval(ki, j + 1, state);
      arguments.push_back(cell.value);
      concreteArguments[j] = cell.concreteValue;
    }

    if (f) {
      const FunctionType *fType = 
        dyn_cast<FunctionType>(cast<PointerType>(f->getType())->getElementType());
      const FunctionType *fpType =
        dyn_cast<FunctionType>(cast<PointerType>(fp->getType())->getElementType());

      // special case the call with a bitcast case
      if (fType != fpType) {
        assert(fType && fpType && "unable to get function type");

        // XXX check result coercion

        // XXX this really needs thought and validation
        for (unsigned i = 0; i < numArgs; i++) {
          ref<Expr> arg = arguments[i];
          const llvm::APInt &carg = concreteArguments[i];
          Expr::Width to, from = arg->getWidth();
            
          if (i<fType->getNumParams()) {
            to = getWidthForLLVMType(fType->getParamType(i));

            if (from != to) {
              // XXX need to check other param attrs ?
              bool isSExt = cs.paramHasAttr(i+1, llvm::Attribute::SExt);
              if (isSExt) {
                arguments[i] = SExtExpr::create(arg, to);
                concreteArguments[i] = carg.sextOrTrunc(to);
              } else {
                arguments[i] = ZExtExpr::create(arg, to);
                concreteArguments[i] = carg.zextOrTrunc(to);
              }
            }
          }
            
          i++;
        }
      }

      executeCall(state, ki, f, arguments, concreteArguments);
    } else {
      const Cell &cell = eval(ki, 0, state);
      ref<Expr> v = cell.value;
      const llvm::APInt &cv = cell.concreteValue;
      uint64_t addr = cv.getZExtValue();

      // Try to resolve the concrete address
      bool found = legalFunctions.count(addr);
      uint64_t ID = found ? addr : 0;

      if (NoSymbolicFunctionPointers || inhibitForking || state.forkDisabled) {
        ;   // do nothing
      } else {
        // Check if there is an expected resolution
        bool doResolution = !runningConcretely;
        bool extendHistory = true;
        unsigned numResolutions = state.functionResolutionHistory.size();
        if (numResolutions < currentTest->functionResolutionDepth) {   // resolution should occur according to expectedFunctionResolutions
          bool checkForDivergence = runningConcretely || CheckSymbolicDivergences;
          bool divergence = false;
          if (checkForDivergence) {
            assert(expectedFunctionResolutions.size() > numResolutions && "missing expected function resolution!");
            if (ID != expectedFunctionResolutions[numResolutions]) {
              divergence = true;
              handleDivergence(state, "unexpected function pointer resolution");
              if (testTerminated) {
                delete[] concreteArguments;
                break;
              }
            } else {
              state.functionResolutionHistory.extendAlong(expectedFunctionResolutions);
            }
          } else {
            state.functionResolutionHistory.push_back(ID);
          }
          if (!divergence) {    // Continue according to the expected resolution
            extendHistory = false;
            doResolution = false;
          }
        } else {
          testIsFertile = true;
        }

        if (doResolution) {
          // Find all possible addresses the function pointer can point to;
          // to make the forks deterministic we find the addresses and sort
          // them before actually forking
          std::vector<ref<ConstantExpr> > addresses;
          bool success __attribute__ ((unused)) = solver->getAllValues(state, v, addresses);
          assert(success && "FIXME: Unhandled solver failure");

          // Sort the addresses
          std::sort(addresses.begin(), addresses.end(), constantExprComparator);

          // Fork for all addresses other than the concrete one
          for (const ref<ConstantExpr> &address : addresses) {
            uint64_t newAddr = address->getZExtValue(64);
            if (newAddr == addr)
              continue;

            TimerStatIncrementer timer(stats::forkTime);
            ++stats::forks;

            ConstraintManager newConditions = state.constraints;
            newConditions.addConstraint(EqExpr::create(v, address));

            std::unique_ptr<ConcolicTest> test = generateConcolicTest(state, newConditions);
            uint64_t newID = legalFunctions.count(newAddr) ? newAddr : 0;
            test->pushFunctionResolution(newID);
            childTests.push_back(std::move(test));
          }
        }

        if (extendHistory)
          state.functionResolutionHistory.push_back(ID);
      }

      // Handle concrete resolution
      if (found) {
        if (!runningConcretely)
          state.addConstraint(EqExpr::create(v, ConstantExpr::alloc(cv)));
        executeCall(state, ki, reinterpret_cast<Function *>(addr), arguments, concreteArguments);
      } else {
        terminateStateOnExecError(state, "invalid function pointer");
      }
    }
    delete[] concreteArguments;
    break;
  }
  case Instruction::PHI: {
    const Cell &cell = eval(ki, state.incomingBBIndex, state);
    bindLocal(ki, state, cell.value, cell.concreteValue);
    break;
  }

    // Special instructions
  case Instruction::Select: {
    // NOTE: It is not required that operands 1 and 2 be of scalar type.
    const Cell &cond = eval(ki, 0, state);
    const Cell &tExpr = eval(ki, 1, state);
    const Cell &fExpr = eval(ki, 2, state);
    ref<Expr> result = SelectExpr::create(cond.value, tExpr.value, fExpr.value);
    bindLocal(ki, state, result, cond.concreteValue == 1 ? tExpr.concreteValue : fExpr.concreteValue);
    break;
  }

  case Instruction::VAArg:
    terminateStateOnExecError(state, "unexpected VAArg instruction");
    break;

    // Arithmetic / logical

  case Instruction::Add: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, AddExpr::create(left.value, right.value), left.concreteValue + right.concreteValue);
    break;
  }

  case Instruction::Sub: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, SubExpr::create(left.value, right.value), left.concreteValue - right.concreteValue);
    break;
  }
 
  case Instruction::Mul: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    const llvm::APInt cval = left.concreteValue * right.concreteValue;
    ref<Expr> val;
    if (NoNonlinearArithmetic)
      val = ConstantExpr::alloc(cval);
    else
      val = MulExpr::create(left.value, right.value);
    bindLocal(ki, state, val, cval);
    break;
  }

  case Instruction::UDiv: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    // NOTE: asserts on divide-by-zero! will need to handle if check-div-zero ever disabled;
    // see test/Feature/DivideByZeroConcrete.c
    const llvm::APInt cval = left.concreteValue.udiv(right.concreteValue);
    ref<Expr> val;
    if (NoNonlinearArithmetic)
      val = ConstantExpr::alloc(cval);
    else
      val = UDivExpr::create(left.value, right.value);
    bindLocal(ki, state, val, cval);
    break;
  }

  case Instruction::SDiv: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    const llvm::APInt cval = left.concreteValue.sdiv(right.concreteValue);
    ref<Expr> val;
    if (NoNonlinearArithmetic)
      val = ConstantExpr::alloc(cval);
    else
      val = SDivExpr::create(left.value, right.value);
    bindLocal(ki, state, val, cval);
    break;
  }

  case Instruction::URem: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    const llvm::APInt cval = left.concreteValue.urem(right.concreteValue);
    ref<Expr> val;
    if (NoNonlinearArithmetic)
      val = ConstantExpr::alloc(cval);
    else
      val = URemExpr::create(left.value, right.value);
    bindLocal(ki, state, val, cval);
    break;
  }
 
  case Instruction::SRem: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    const llvm::APInt cval = left.concreteValue.srem(right.concreteValue);
    ref<Expr> val;
    if (NoNonlinearArithmetic)
      val = ConstantExpr::alloc(cval);
    else
      val = SRemExpr::create(left.value, right.value);
    bindLocal(ki, state, val, cval);
    break;
  }

  case Instruction::And: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, AndExpr::create(left.value, right.value), left.concreteValue & right.concreteValue);
    break;
  }

  case Instruction::Or: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, OrExpr::create(left.value, right.value), left.concreteValue | right.concreteValue);
    break;
  }

  case Instruction::Xor: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, XorExpr::create(left.value, right.value), left.concreteValue ^ right.concreteValue);
    break;
  }

  case Instruction::Shl: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    // TODO ignores overshift; will need to handle if check-overshift ever disabled
    bindLocal(ki, state, ShlExpr::create(left.value, right.value), left.concreteValue.shl(right.concreteValue));
    break;
  }

  case Instruction::LShr: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, LShrExpr::create(left.value, right.value), left.concreteValue.lshr(right.concreteValue));
    break;
  }

  case Instruction::AShr: {
    const Cell &left = eval(ki, 0, state);
    const Cell &right = eval(ki, 1, state);
    bindLocal(ki, state, AShrExpr::create(left.value, right.value), left.concreteValue.ashr(right.concreteValue));
    break;
  }

    // Compare

  case Instruction::ICmp: {
    CmpInst *ci = cast<CmpInst>(i);
    ICmpInst *ii = cast<ICmpInst>(ci);
 
    switch(ii->getPredicate()) {
    case ICmpInst::ICMP_EQ: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, EqExpr::create(left.value, right.value), left.concreteValue == right.concreteValue ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_NE: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, NeExpr::create(left.value, right.value), left.concreteValue != right.concreteValue ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_UGT: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, UgtExpr::create(left.value, right.value), left.concreteValue.ugt(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_UGE: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, UgeExpr::create(left.value, right.value), left.concreteValue.uge(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_ULT: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, UltExpr::create(left.value, right.value), left.concreteValue.ult(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_ULE: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, UleExpr::create(left.value, right.value), left.concreteValue.ule(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_SGT: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, SgtExpr::create(left.value, right.value), left.concreteValue.sgt(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_SGE: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, SgeExpr::create(left.value, right.value), left.concreteValue.sge(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_SLT: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, SltExpr::create(left.value, right.value), left.concreteValue.slt(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    case ICmpInst::ICMP_SLE: {
      const Cell &left = eval(ki, 0, state);
      const Cell &right = eval(ki, 1, state);
      bindLocal(ki, state, SleExpr::create(left.value, right.value), left.concreteValue.sle(right.concreteValue) ? APInt(1, 1) : APInt(1, 0));
      break;
    }

    default:
      terminateStateOnExecError(state, "invalid ICmp predicate");
    }
    break;
  }

    // Memory instructions...
  case Instruction::Alloca: {
    AllocaInst *ai = cast<AllocaInst>(i);
    unsigned elementSize = 
      kmodule->targetData->getTypeStoreSize(ai->getAllocatedType());
    ref<Expr> size = memory->createPointer(elementSize);
    uint64_t csize = elementSize;
    if (ai->isArrayAllocation()) {
      const Cell &cell = eval(ki, 0, state);
      ref<Expr> count = memory->createZExtToPointerWidth(cell.value);
      size = MulExpr::create(size, count);
      csize *= cell.concreteValue.getZExtValue();
    }
    bool staticSize = false;
    const Value *arraySize = ai->getArraySize();
    if (const ConstantInt *cas __attribute__ ((unused)) = dyn_cast<ConstantInt>(arraySize)) {
      assert(cas->getZExtValue() * elementSize == csize && "inconsistent constant alloca size!");
      staticSize = true;
    }
    executeAlloc(state, size, csize, true, ki, false, 0, staticSize);
    break;
  }

  case Instruction::Load: {
    const Cell &base = eval(ki, 0, state);
    executeMemoryOperation(state, false, base.value, base.concreteValue.getZExtValue(), 0, APInt(), ki);
    break;
  }
  case Instruction::Store: {
    const Cell &base = eval(ki, 1, state);
    const Cell &value = eval(ki, 0, state);
    executeMemoryOperation(state, true, base.value, base.concreteValue.getZExtValue(), value.value, value.concreteValue, 0);
    break;
  }

  case Instruction::GetElementPtr: {
    KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(ki);
    const Cell &cell = eval(ki, 0, state);
    ref<Expr> base = cell.value;
    llvm::APInt concreteBase = cell.concreteValue;
    Expr::Width pointerWidth = Context::get().getPointerWidth();

    for (std::vector< std::pair<unsigned, uint64_t> >::iterator 
           it = kgepi->indices.begin(), ie = kgepi->indices.end(); 
         it != ie; ++it) {
      uint64_t elementSize = it->second;
      const Cell &icell = eval(ki, it->first, state);
      ref<Expr> index = icell.value;
      llvm::APInt concreteIndex = icell.concreteValue;
      base = AddExpr::create(base,
                             MulExpr::create(memory->createSExtToPointerWidth(index),
                                             memory->createPointer(elementSize)));
      concreteBase += concreteIndex.sextOrTrunc(pointerWidth) * APInt(pointerWidth, elementSize);
    }
    if (kgepi->offset) {
      base = AddExpr::create(base,
                             memory->createPointer(kgepi->offset));
      concreteBase += APInt(pointerWidth, kgepi->offset);
    }
    bindLocal(ki, state, base, concreteBase);
    break;
  }

    // Conversion
  case Instruction::Trunc: {
    CastInst *ci = cast<CastInst>(i);
    const Cell &cell = eval(ki, 0, state);
    Expr::Width width = getWidthForLLVMType(ci->getType());
    bindLocal(ki, state, ExtractExpr::create(cell.value, 0, width), cell.concreteValue.trunc(width));
    break;
  }
  case Instruction::ZExt: {
    CastInst *ci = cast<CastInst>(i);
    const Cell &cell = eval(ki, 0, state);
    Expr::Width width = getWidthForLLVMType(ci->getType());
    bindLocal(ki, state, ZExtExpr::create(cell.value, width), cell.concreteValue.zext(width));
    break;
  }
  case Instruction::SExt: {
    CastInst *ci = cast<CastInst>(i);
    const Cell &cell = eval(ki, 0, state);
    Expr::Width width = getWidthForLLVMType(ci->getType());
    bindLocal(ki, state, SExtExpr::create(cell.value, width), cell.concreteValue.sext(width));
    break;
  }

  case Instruction::IntToPtr: {
    CastInst *ci = cast<CastInst>(i);
    Expr::Width pType = getWidthForLLVMType(ci->getType());
    const Cell &cell = eval(ki, 0, state);
    bindLocal(ki, state, ZExtExpr::create(cell.value, pType), cell.concreteValue.zextOrTrunc(pType));
    break;
  } 
  case Instruction::PtrToInt: {
    CastInst *ci = cast<CastInst>(i);
    Expr::Width iType = getWidthForLLVMType(ci->getType());
    const Cell &cell = eval(ki, 0, state);
    bindLocal(ki, state, ZExtExpr::create(cell.value, iType), cell.concreteValue.zextOrTrunc(iType));
    break;
  }

  case Instruction::BitCast: {
    const Cell &cell = eval(ki, 0, state);
    bindLocal(ki, state, cell.value, cell.concreteValue);
    break;
  }

    // Floating point instructions

  case Instruction::FAdd: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state),
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FAdd operation");

    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.add(APFloat(*fpWidthToSemantics(right->getWidth()),right->getAPValue()), APFloat::rmNearestTiesToEven);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }

  case Instruction::FSub: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state),
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FSub operation");
    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.subtract(APFloat(*fpWidthToSemantics(right->getWidth()), right->getAPValue()), APFloat::rmNearestTiesToEven);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }
 
  case Instruction::FMul: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state),
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FMul operation");

    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.multiply(APFloat(*fpWidthToSemantics(right->getWidth()), right->getAPValue()), APFloat::rmNearestTiesToEven);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }

  case Instruction::FDiv: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state),
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FDiv operation");

    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.divide(APFloat(*fpWidthToSemantics(right->getWidth()), right->getAPValue()), APFloat::rmNearestTiesToEven);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }

  case Instruction::FRem: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state),
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FRem operation");
    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.mod(APFloat(*fpWidthToSemantics(right->getWidth()),right->getAPValue()),
            APFloat::rmNearestTiesToEven);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }

  case Instruction::FPTrunc: {
    FPTruncInst *fi = cast<FPTruncInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state),
                                       "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || resultType > arg->getWidth())
      return terminateStateOnExecError(state, "Unsupported FPTrunc operation");
    llvm::APFloat Res(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    bool losesInfo = false;
    Res.convert(*fpWidthToSemantics(resultType),
                llvm::APFloat::rmNearestTiesToEven,
                &losesInfo);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }

  case Instruction::FPExt: {
    FPExtInst *fi = cast<FPExtInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || arg->getWidth() > resultType)
      return terminateStateOnExecError(state, "Unsupported FPExt operation");
    llvm::APFloat Res(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    bool losesInfo = false;
    Res.convert(*fpWidthToSemantics(resultType),
                llvm::APFloat::rmNearestTiesToEven,
                &losesInfo);
    llvm::APInt iRes = Res.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(iRes), iRes);
    break;
  }

  case Instruction::FPToUI: {
    FPToUIInst *fi = cast<FPToUIInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state),
                                       "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || resultType > 64)
      return terminateStateOnExecError(state, "Unsupported FPToUI operation");

    llvm::APFloat Arg(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    uint64_t value = 0;
    bool isExact = true;
    Arg.convertToInteger(&value, resultType, false,
                         llvm::APFloat::rmTowardZero, &isExact);
    bindLocal(ki, state, ConstantExpr::alloc(value, resultType), APInt(resultType, value));
    break;
  }

  case Instruction::FPToSI: {
    FPToSIInst *fi = cast<FPToSIInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state),
                                       "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || resultType > 64)
      return terminateStateOnExecError(state, "Unsupported FPToSI operation");
    llvm::APFloat Arg(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    uint64_t value = 0;
    bool isExact = true;
    Arg.convertToInteger(&value, resultType, true,
                         llvm::APFloat::rmTowardZero, &isExact);
    bindLocal(ki, state, ConstantExpr::alloc(value, resultType), APInt(resultType, value));
    break;
  }

  case Instruction::UIToFP: {
    UIToFPInst *fi = cast<UIToFPInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state),
                                       "floating point");
    const llvm::fltSemantics *semantics = fpWidthToSemantics(resultType);
    if (!semantics)
      return terminateStateOnExecError(state, "Unsupported UIToFP operation");
    llvm::APFloat f(*semantics, 0);
    f.convertFromAPInt(arg->getAPValue(), false,
                       llvm::APFloat::rmNearestTiesToEven);

    llvm::APInt fInt = f.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(fInt), fInt);
    break;
  }

  case Instruction::SIToFP: {
    SIToFPInst *fi = cast<SIToFPInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state),
                                       "floating point");
    const llvm::fltSemantics *semantics = fpWidthToSemantics(resultType);
    if (!semantics)
      return terminateStateOnExecError(state, "Unsupported SIToFP operation");
    llvm::APFloat f(*semantics, 0);
    f.convertFromAPInt(arg->getAPValue(), true,
                       llvm::APFloat::rmNearestTiesToEven);

    llvm::APInt fInt = f.bitcastToAPInt();
    bindLocal(ki, state, ConstantExpr::alloc(fInt), fInt);
    break;
  }

  case Instruction::FCmp: {
    FCmpInst *fi = cast<FCmpInst>(i);
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state),
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state),
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FCmp operation");

    APFloat LHS(*fpWidthToSemantics(left->getWidth()),left->getAPValue());
    APFloat RHS(*fpWidthToSemantics(right->getWidth()),right->getAPValue());
    APFloat::cmpResult CmpRes = LHS.compare(RHS);

    bool Result = false;
    switch( fi->getPredicate() ) {
      // Predicates which only care about whether or not the operands are NaNs.
    case FCmpInst::FCMP_ORD:
      Result = CmpRes != APFloat::cmpUnordered;
      break;

    case FCmpInst::FCMP_UNO:
      Result = CmpRes == APFloat::cmpUnordered;
      break;

      // Ordered comparisons return false if either operand is NaN.  Unordered
      // comparisons return true if either operand is NaN.
    case FCmpInst::FCMP_UEQ:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
      [[fallthrough]];
    case FCmpInst::FCMP_OEQ:
      Result = CmpRes == APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_UGT:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
      [[fallthrough]];
    case FCmpInst::FCMP_OGT:
      Result = CmpRes == APFloat::cmpGreaterThan;
      break;

    case FCmpInst::FCMP_UGE:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
      [[fallthrough]];
    case FCmpInst::FCMP_OGE:
      Result = CmpRes == APFloat::cmpGreaterThan || CmpRes == APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_ULT:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
      [[fallthrough]];
    case FCmpInst::FCMP_OLT:
      Result = CmpRes == APFloat::cmpLessThan;
      break;

    case FCmpInst::FCMP_ULE:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
      [[fallthrough]];
    case FCmpInst::FCMP_OLE:
      Result = CmpRes == APFloat::cmpLessThan || CmpRes == APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_UNE:
      Result = CmpRes == APFloat::cmpUnordered || CmpRes != APFloat::cmpEqual;
      break;
    case FCmpInst::FCMP_ONE:
      Result = CmpRes != APFloat::cmpUnordered && CmpRes != APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_FALSE:
      Result = false;
      break;
    case FCmpInst::FCMP_TRUE:
      Result = true;
      break;

    default:
      llvm_unreachable("Invalid FCMP predicate!");
    }

    bindLocal(ki, state, ConstantExpr::alloc(Result, Expr::Bool), Result ? APInt(1, 1) : APInt(1, 0));
    break;
  }
  case Instruction::InsertValue: {
    KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(ki);

    const Cell &agg = eval(ki, 0, state);
    const Cell &val = eval(ki, 1, state);

    Expr::Width aggWidth = agg.value->getWidth();
    Expr::Width valWidth = val.value->getWidth();

    ref<Expr> l = NULL, r = NULL;
    llvm::APInt cl, cr;
    unsigned lOffset = kgepi->offset*8, rOffset = kgepi->offset*8 + valWidth;

    if (lOffset > 0) {
      l = ExtractExpr::create(agg.value, 0, lOffset);
      cl = agg.concreteValue.trunc(lOffset);
    }
    if (rOffset < aggWidth) {
      r = ExtractExpr::create(agg.value, rOffset, aggWidth - rOffset);
      cr = agg.concreteValue.lshr(rOffset).trunc(aggWidth - rOffset);
    }

    ref<Expr> result;
    llvm::APInt cresult;
    if (!l.isNull() && !r.isNull()) {
      result = ConcatExpr::create(r, ConcatExpr::create(val.value, l));
      Expr::Width W = valWidth + lOffset;
      cresult = val.concreteValue.zext(aggWidth);
      cresult <<= lOffset;
      cresult |= cl.zext(aggWidth);
      cresult |= cr.zext(aggWidth) << W;
    } else if (!l.isNull()) {
      result = ConcatExpr::create(val.value, l);
      cresult = val.concreteValue.zext(aggWidth);
      cresult <<= lOffset;
      cresult |= cl.zext(aggWidth);
    } else if (!r.isNull()) {
      result = ConcatExpr::create(r, val.value);
      cresult = cr.zext(aggWidth);
      cresult <<= valWidth;
      cresult |= val.concreteValue.zext(aggWidth);
    } else {
      result = val.value;
      cresult = val.concreteValue;
    }

    bindLocal(ki, state, result, cresult);
    break;
  }
  case Instruction::ExtractValue: {
    KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(ki);

    const Cell &agg = eval(ki, 0, state);

    Expr::Width W = getWidthForLLVMType(i->getType());
    unsigned int offset = kgepi->offset*8;
    ref<Expr> result = ExtractExpr::create(agg.value, offset, W);
    llvm::APInt cresult = agg.concreteValue.lshr(offset).trunc(W);

    bindLocal(ki, state, result, cresult);
    break;
  }

  case Instruction::Fence: {
    // Ignore for now
    break;
  }

  case Instruction::InsertElement: {
    InsertElementInst *iei = cast<InsertElementInst>(i);
    const Cell &vec = eval(ki, 0, state);
    const Cell &newElt = eval(ki, 1, state);
    const Cell &idx = eval(ki, 2, state);

    ConstantExpr *cIdx = dyn_cast<ConstantExpr>(idx.value);
    if (cIdx == NULL) {
      terminateStateOnError(
          state, "InsertElement, support for symbolic index not implemented",
          Unhandled);
      return;
    }
    uint64_t iIdx = cIdx->getZExtValue();
    const llvm::VectorType *vt = iei->getType();
    unsigned EltBits = getWidthForLLVMType(vt->getElementType());
    const unsigned elementCount = vt->getNumElements();
    if (iIdx >= elementCount) {
      // Out of bounds write
      terminateStateOnError(state, "Out of bounds write when inserting element",
                            BadVectorAccess);
      return;
    }

    llvm::SmallVector<ref<Expr>, 8> elems;
    elems.reserve(elementCount);
    for (unsigned i = elementCount; i != 0; --i) {
      auto of = i - 1;
      unsigned bitOffset = EltBits * of;
      elems.push_back(
          of == iIdx ? newElt.value : ExtractExpr::create(vec.value, bitOffset, EltBits));
    }
    llvm::APInt cresult(vec.concreteValue);
    unsigned width = cresult.getBitWidth();
    unsigned low = EltBits * iIdx;
    llvm::APInt mask = llvm::APInt::getBitsSet(width, low, low + EltBits);
    cresult &= ~mask;
    assert(newElt.concreteValue.getBitWidth() <= width);
    cresult |= newElt.concreteValue.zextOrSelf(width).shl(low);

    assert(Context::get().isLittleEndian() && "FIXME:Broken for big endian");
    ref<Expr> Result = ConcatExpr::createN(elementCount, elems.data());
    bindLocal(ki, state, Result, cresult);
    break;
  }
  case Instruction::ExtractElement: {
    ExtractElementInst *eei = cast<ExtractElementInst>(i);
    const Cell &vec = eval(ki, 0, state);
    const Cell &idx = eval(ki, 1, state);

    ConstantExpr *cIdx = dyn_cast<ConstantExpr>(idx.value);
    if (cIdx == NULL) {
      terminateStateOnError(
          state, "ExtractElement, support for symbolic index not implemented",
          Unhandled);
      return;
    }
    uint64_t iIdx = cIdx->getZExtValue();
    const llvm::VectorType *vt = eei->getVectorOperandType();
    unsigned EltBits = getWidthForLLVMType(vt->getElementType());

    if (iIdx >= vt->getNumElements()) {
      // Out of bounds read
      terminateStateOnError(state, "Out of bounds read when extracting element",
                            BadVectorAccess);
      return;
    }

    unsigned bitOffset = EltBits * iIdx;
    ref<Expr> Result = ExtractExpr::create(vec.value, bitOffset, EltBits);
    llvm::APInt cresult = vec.concreteValue.lshr(bitOffset).zextOrTrunc(EltBits);
    bindLocal(ki, state, Result, cresult);
    break;
  }
  case Instruction::ShuffleVector:
    // Should never happen due to Scalarizer pass removing ShuffleVector
    // instructions.
    terminateStateOnExecError(state, "Unexpected ShuffleVector instruction");
    break;

  // Other instructions...
  // Unhandled
  default:
    terminateStateOnExecError(state, "illegal instruction");
    break;
  }
}

template <typename TypeIt>
void Executor::computeOffsets(KGEPInstruction *kgepi, TypeIt ib, TypeIt ie) {
  ref<ConstantExpr> constantOffset =
    ConstantExpr::alloc(0, Context::get().getPointerWidth());
  uint64_t index = 1;
  for (TypeIt ii = ib; ii != ie; ++ii) {
    if (StructType *st = dyn_cast<StructType>(*ii)) {
      const StructLayout *sl = kmodule->targetData->getStructLayout(st);
      const ConstantInt *ci = cast<ConstantInt>(ii.getOperand());
      uint64_t addend = sl->getElementOffset((unsigned) ci->getZExtValue());
      constantOffset = constantOffset->Add(ConstantExpr::alloc(addend,
                                                               Context::get().getPointerWidth()));
    } else {
      const SequentialType *set = cast<SequentialType>(*ii);
      uint64_t elementSize =
        kmodule->targetData->getTypeStoreSize(set->getElementType());
      Value *operand = ii.getOperand();
      if (Constant *c = dyn_cast<Constant>(operand)) {
        ref<ConstantExpr> index =
          evalConstant(c)->SExt(Context::get().getPointerWidth());
        ref<ConstantExpr> addend =
          index->Mul(ConstantExpr::alloc(elementSize,
                                         Context::get().getPointerWidth()));
        constantOffset = constantOffset->Add(addend);
      } else {
        kgepi->indices.push_back(std::make_pair(index, elementSize));
      }
    }
    index++;
  }
  kgepi->offset = constantOffset->getZExtValue();
}

void Executor::bindInstructionConstants(KInstruction *KI) {
  KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(KI);

  if (GetElementPtrInst *gepi = dyn_cast<GetElementPtrInst>(KI->inst)) {
    computeOffsets(kgepi, gep_type_begin(gepi), gep_type_end(gepi));
  } else if (InsertValueInst *ivi = dyn_cast<InsertValueInst>(KI->inst)) {
    computeOffsets(kgepi, iv_type_begin(ivi), iv_type_end(ivi));
    assert(kgepi->indices.empty() && "InsertValue constant offset expected");
  } else if (ExtractValueInst *evi = dyn_cast<ExtractValueInst>(KI->inst)) {
    computeOffsets(kgepi, ev_type_begin(evi), ev_type_end(evi));
    assert(kgepi->indices.empty() && "ExtractValue constant offset expected");
  }
}

void Executor::bindModuleConstants() {
  for (std::vector<KFunction*>::iterator it = kmodule->functions.begin(),
         ie = kmodule->functions.end(); it != ie; ++it) {
    KFunction *kf = *it;
    for (unsigned i=0; i<kf->numInstructions; ++i)
      bindInstructionConstants(kf->instructions[i]);
  }

  kmodule->constantTable = new Cell[kmodule->constants.size()];
  for (unsigned i=0; i<kmodule->constants.size(); ++i) {
    Cell &c = kmodule->constantTable[i];
    ref<ConstantExpr> val = evalConstant(kmodule->constants[i]);
    c.value = val;
    c.concreteValue = val->getAPValue();
  }
}

void Executor::checkMemoryUsage() {
  if (!MaxMemory)
    return;
  if ((stats::instructions & 0xFFFF) == 0) {
    // We need to avoid calling GetTotalMallocUsage() often because it
    // is O(elts on freelist). This is really bad since we start
    // to pummel the freelist once we hit the memory cap.
    unsigned mbs = (util::GetTotalMallocUsage() >> 20) +
                   (memory->getUsedDeterministicSize() >> 20);

    if (mbs > MaxMemory) {
      atMemoryLimit = true;
    } else {
      atMemoryLimit = false;
    }
  }
}

void Executor::doDumpStates() {
  if (!DumpStatesOnHalt || states.empty())
    return;
  klee_message("halting execution, dumping remaining states");
  for (std::set<ExecutionState *>::iterator it = states.begin(),
                                            ie = states.end();
       it != ie; ++it) {
    ExecutionState &state = **it;
    stepInstruction(state); // keep stats rolling
    terminateStateEarly(state, "Execution halting.");
  }
}

void Executor::run(ExecutionState &initialState) {
  bindModuleConstants();

  // Delay init till now so that ticks don't accrue during
  // optimization and such.
  initTimers();

  symbolicCount = 0;   // total number of tests executed symbolically

  // Scenario testing loop: scenario will be parsed and instantiated as a
  // skeleton (sequence of actions) during the first iteration
  do {
    // If we parsed the scenario during testing of a previous skeleton, don't
    // parse again, but start new tests from just after parsing; after finding
    // the first call to klee_make_symbolic we will start future tests from
    // that point instead
    startingState = ScenarioHandler::handler.getStartingState();
    startingStateFinalized = false;   // update startingState on the first call to klee_make_symbolic

    // Set up seed test for concolic testing
    std::unique_ptr<ConcolicTest> seed(new ConcolicTest(initialState.resolutionHistory,
                                                        initialState.offsetHistory,
                                                        initialState.functionResolutionHistory,
                                                        initialState.allocationHistory));
    unsigned lastInterestingTest = seed->testID;
    if (GenerationalSearch) {
      // Run first test concretely; this is redundant, but ensures that the seed
      // test completes even if we get bogged down in symbolic execution
      runTest(initialState, seed.get(), true);
      seed->newBlocks = newBlocksCovered;
      if (seed->testID == 0)    // very first test gets a point for covering the entry block
        seed->newBlocks++;

      if (startingStateFinalized) {
        tests.push(std::move(seed));
      } else {
        // no symbolics ever created, so no point in symbolic execution;
        // warn in case this was not intentional
        klee_warning_once(0, "no symbolic inputs were ever created in this test");
      }
    } else {
      tests.push(std::move(seed));
    }

    // Concolic testing loop
    while (!tests.empty() && !haltExecution) {
      // Pick highest-scored test
      std::unique_ptr<ConcolicTest> test = tests.pop();

      // Run test symbolically, generating child tests
      runTest(initialState, test.get(), false);

      // Process child tests
      for (std::unique_ptr<ConcolicTest> &childTest : childTests) {
        if (haltExecution)
          break;

        if (GenerationalSearch) {
          // Run child concretely and score it
          bool fertile = runTest(initialState, childTest.get(), true);
          childTest->newBlocks += newBlocksCovered;   // accumulate, since replacements may have value already
          if (childTest->endedInError || childTest->newBlocks > 0)
            lastInterestingTest = childTest->testID;

          // Only add the child to the queue if it could yield further tests, or if
          // we need to execute it symbolically so we can dump the path conditions
          if (fertile || childTest->endedInError || interpreterHandler->needsSymbolicExecution())
            tests.push(std::move(childTest));
        } else {
          tests.push(std::move(childTest));
        }
      }
      childTests.clear();

      if (!GenerationalSearch && (test->endedInError || newBlocksCovered > 0))
        lastInterestingTest = test->testID;

      if (SkeletonDelay && (ConcolicTest::nextTestID - lastInterestingTest) >= SkeletonDelay) {
        klee_warning_once(0, "hit skeleton-delay; generating new skeleton");
        break;
      }
    }

    // Delete any tests left over from stopping concolic testing early
    tests.clear();

    // If possible, generate a new skeleton from the scenario and restart (unless we're halting)
    if (ScenarioHandler::handler.usedScenarioDSL())
      ++stats::skeletons;
  } while(!haltExecution && ScenarioHandler::handler.generateNewSkeleton());

  // Clean up
  startingState = SavedState();
}

bool Executor::runTest(const ExecutionState &initialState, ConcolicTest *test, bool concretely) {
  // Start timer
  Statistic &stat = concretely ? stats::concreteTime : stats::symbolicTime;
  TimerStatIncrementer timer(stat);

  // Set up Executor state
  runningConcretely = concretely;
  newBlocksCovered = 0;
  testInstructions = 0;
  currentTest = test;
  testTerminated = false;
  testIsFertile = false;

  // Extract test expectations from tries
  expectedResolutions = ReverseTrie<unsigned>::Path(currentTest->expectedResolutions);
  expectedOffsets = ReverseTrie<unsigned>::Path(currentTest->expectedOffsets);
  expectedFunctionResolutions = ReverseTrie<uint64_t>::Path(currentTest->expectedFunctionResolutions);
  expectedAllocations = ReverseTrie<unsigned>::Path(currentTest->expectedAllocations);

  // Create a copy of the initial state to run the test from
  int oldMemoryCounter = MemoryObject::counter;
  MemoryManager::ResetToken token = memory->getResetToken();
  if (startingState) {    // starting from some time after initialization
    restoreState(startingState);
    ScenarioHandler::handler.restoreState();
  } else {
    currentTestState = new ExecutionState(initialState);
  }
  ExecutionState &state = *currentTestState;
  if (pathWriter)
    state.pathOS = pathWriter->open();
  if (symPathWriter)
    state.symPathOS = symPathWriter->open();

  // Run test to completion (can be aborted early by functions setting haltExecution;
  // these also set testTerminated)
  while (!testTerminated) {
    KInstruction *ki = state.pc;
    bool terminateEarly = stepInstruction(state);
    if (terminateEarly)
      break;

    executeInstruction(state, ki);
    processTimers(&state, MaxInstructionTime);

    //checkMemoryUsage();   // TODO make this kill tests instead?
  }

  currentTest->alreadyRun = true;
  if (!CheckSymbolicDivergences) {
    // From now on we don't need the expectations, only the corresponding depths;
    // so we clear them to save memory
    currentTest->expectedPath = std::vector<bool>();
    currentTest->expectedResolutions = resolutionTrie.getRoot();
    currentTest->expectedOffsets = offsetTrie.getRoot();
    currentTest->expectedFunctionResolutions = functionResolutionTrie.getRoot();
    currentTest->expectedAllocations = allocationTrie.getRoot();
  }

  assert(!startingStateFinalized == state.symbolics.empty() && "starting state not updated correctly!");
  assert((startingState || !startingStateFinalized)  && "starting state not set correctly!");

  delete currentTestState;

  // Reset some global state
  if (startingState) {   // this may have been set during test execution
    // Memory will be reset above when launching a new test
  } else {
    // Reset memory to initial state
    MemoryObject::counter = oldMemoryCounter;
    memory->reset(token);
  }

  // Check if we've hit any test limits
  if (MaxTests && ConcolicTest::nextTestID >= MaxTests) {
    klee_warning_once(0, "hit max-tests; stopping early");
    haltExecution = true;
  } else if (MaxSymbolicTests && !concretely && ++symbolicCount >= MaxSymbolicTests) {
    klee_warning_once(0, "hit max-symbolic-tests; stopping early");
    haltExecution = true;
  }

  return testIsFertile;
}

std::string Executor::getAddressInfo(ExecutionState &state,
                                     ref<Expr> address) const{
  std::string Str;
  llvm::raw_string_ostream info(Str);
  info << "\taddress: " << address << "\n";
  uint64_t example;
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(address)) {
    example = CE->getZExtValue();
  } else {
    ref<ConstantExpr> value;
    bool success = solver->getValue(state, address, value);
    assert(success && "FIXME: Unhandled solver failure");
    (void) success;
    example = value->getZExtValue();
    info << "\texample: " << example << "\n";
    std::pair< ref<Expr>, ref<Expr> > res = solver->getRange(state, address);
    info << "\trange: [" << res.first << ", " << res.second <<"]\n";
  }

  return info.str();
}

std::unique_ptr<ConcolicTest> Executor::generateConcolicTest(const ExecutionState &state, const ConstraintManager &constraints) {
  assert(!runningConcretely && "cannot generate new test from concrete execution!");

  // Get concrete values that should (modulo divergences) lead to this state.
  // Note that we do not print a warning if the computation fails, since one
  // has already been printed in computeInitialValues.
  std::vector< std::vector<unsigned char> > vals;
  if (!computeInitialValues(state.symbolics, constraints, vals))
    return 0;

  std::unique_ptr<ConcolicTest> test(new ConcolicTest(state.resolutionHistory,
                                                      state.offsetHistory,
                                                      state.functionResolutionHistory,
                                                      state.allocationHistory));
  unsigned numObjects = vals.size();
  test->objects.reserve(numObjects);
  for (unsigned i = 0; i < numObjects; i++) {
    ConcolicTestObject o;
    const std::string *name = objectNames.find(state.symbolics[i].first->name);
    assert(name && "object name not in cache!");
    o.name = name;
    o.bytes = objectBytes.emplace(vals[i]);
    test->objects.push_back(o);
  }

  test->expectedPath = state.forkHistory;
  test->branchDepth = state.forkHistory.size();

  test->generation = currentTest->generation + 1;

//  uint64_t md2u = computeMinDistToUncovered(state.pc, state.stack.back().minDistToUncoveredOnReturn);
//  double invMD2U = 1. / (md2u ? md2u : 10000);
//  double invCovNew = 0.;
//  if (state.instsSinceCovNew)
//    invCovNew = 1. / std::max(1, (int) state.instsSinceCovNew - 1000);
//  test->predictionScore = (invCovNew * invCovNew + invMD2U * invMD2U);

  return test;
}

void Executor::handleDivergence(ExecutionState &state, const char *reason) {
  warning_once(0, "divergence detected (%s)", reason);
  if (runningConcretely)
    ++stats::divergences;
  else
    warning_once(0, "divergence replaying concrete execution: environment may not be static");

  // no expectations from now on
  currentTest->truncateExpectationsTo(state, expectedResolutions, expectedOffsets,
                                      expectedFunctionResolutions, expectedAllocations);

  // might as well clear these to save a bit of memory
  expectedResolutions = ReverseTrie<unsigned>::Path();
  expectedOffsets = ReverseTrie<unsigned>::Path();
  expectedFunctionResolutions = ReverseTrie<uint64_t>::Path();
  expectedAllocations = ReverseTrie<unsigned>::Path();

  if (ExitOnDivergence) {
    if (!testTerminated)
      terminateStateOnError(state, "divergence detected", Divergence);
    haltExecution = true;
  }
}

void Executor::terminateState(ExecutionState &state) {
  if (replayKTest && replayPosition != replayKTest->numObjects) {
    klee_warning_once(replayKTest,
                      "replay did not consume all objects in test input.");
  }
  if (currentTest->objects.size() > state.symbolics.size())
    handleDivergence(state, "unused klee_make_symbolic");
  else if (currentTest->expectedPath.size() > state.forkHistory.size())
    handleDivergence(state, "unused fork");
  else if (currentTest->expectedResolutions.size() > state.resolutionHistory.size())
    handleDivergence(state, "unused pointer resolution");
  else if (currentTest->expectedFunctionResolutions.size() > state.functionResolutionHistory.size())
    handleDivergence(state, "unused function pointer resolution");
  else if (currentTest->expectedAllocations.size() > state.allocationHistory.size())
    handleDivergence(state, "unused allocation");

  if (testTerminated)   // handleDivergence may have terminated the test
    return;

  terminateStateUnchecked(state);
}

void Executor::terminateStateUnchecked(ExecutionState &state, bool abandonPath) {
  assert(&state == currentTestState && "terminated a state other than the current test!");
  assert(!testTerminated && "terminated the same test twice!");

  if (!currentTest->alreadyRun) {
    ++stats::concreteTests;
    if (abandonPath)
      ++stats::abandonedTests;
  }

  if (!runningConcretely)
    ++stats::symbolicTests;

  testTerminated = true;

  ScenarioHandler::handler.testTerminated();
}

void Executor::terminateStateEarly(ExecutionState &state,
                                   const Twine &message, bool suppressExitOnError) {
  currentTest->endedInError = true;

  if (OutputAllTests || state.coveredNew || currentTest->newBlocks > 0 || currentTest->needsProcessing) {
    currentTest->needsProcessing = true;
    interpreterHandler->processTestCase(state, (message + "\n").str().c_str(),
                                        "early", currentTest->kTestID, suppressExitOnError);
  }
  terminateStateUnchecked(state);
}

void Executor::terminateStateOnExit(ExecutionState &state) {
  if (OutputAllTests || state.coveredNew || currentTest->newBlocks > 0 || currentTest->needsProcessing) {
    currentTest->needsProcessing = true;
    interpreterHandler->processTestCase(state, 0, 0, currentTest->kTestID);
  }
  terminateState(state);
}

const InstructionInfo & Executor::getLastNonKleeInternalInstruction(const ExecutionState &state,
    Instruction ** lastInstruction) {
  // unroll the stack of the applications state and find
  // the last instruction which is not inside a KLEE internal function
  ExecutionState::stack_ty::const_reverse_iterator it = state.stack.rbegin(),
      itE = state.stack.rend();

  // don't check beyond the outermost function (i.e. main())
  itE--;

  const InstructionInfo * ii = 0;
  if (kmodule->internalFunctions.count(it->kf->function) == 0){
    ii =  state.prevPC->info;
    *lastInstruction = state.prevPC->inst;
    //  Cannot return yet because even though
    //  it->function is not an internal function it might of
    //  been called from an internal function.
  }

  // Wind up the stack and check if we are in a KLEE internal function.
  // We visit the entire stack because we want to return a CallInstruction
  // that was not reached via any KLEE internal functions.
  for (;it != itE; ++it) {
    // check calling instruction and if it is contained in a KLEE internal function
    const Function * f = (*it->caller).inst->getParent()->getParent();
    if (kmodule->internalFunctions.count(f)){
      ii = 0;
      continue;
    }
    if (!ii){
      ii = (*it->caller).info;
      *lastInstruction = (*it->caller).inst;
    }
  }

  if (!ii) {
    // something went wrong, play safe and return the current instruction info
    *lastInstruction = state.prevPC->inst;
    return *state.prevPC->info;
  }
  return *ii;
}

bool Executor::shouldExitOn(enum TerminateReason termReason) {
  std::vector<TerminateReason>::iterator s = ExitOnErrorType.begin();
  std::vector<TerminateReason>::iterator e = ExitOnErrorType.end();

  for (; s != e; ++s)
    if (termReason == *s)
      return true;

  return false;
}

void Executor::terminateStateOnError(ExecutionState &state,
                                     const llvm::Twine &messaget,
                                     enum TerminateReason termReason,
                                     const char *suffix,
                                     const llvm::Twine &info) {
  currentTest->endedInError = true;

  std::string message = messaget.str();
  static std::set<std::pair<Instruction *, std::string> > emittedErrors;
  Instruction *lastInst;
  const InstructionInfo &ii = getLastNonKleeInternalInstruction(state, &lastInst);

  if (EmitAllErrors ||
      emittedErrors.insert(std::make_pair(lastInst, message)).second) {
    nonfatal_error_at(ii, "%s", message.c_str());
    if (!EmitAllErrors)
      note_at(ii, "now ignoring this error at this location");

    std::string MsgString;
    llvm::raw_string_ostream msg(MsgString);
    msg << "Error: " << message << "\n";
    if (ii.file != "") {
      msg << "File: " << ii.file << "\n";
      msg << "Line: " << ii.line << "\n";
      msg << "assembly.ll line: " << ii.assemblyLine << "\n";
    }
    msg << "Stack: \n";
    state.dumpStack(msg);

    std::string info_str = info.str();
    if (info_str != "")
      msg << "Info: \n" << info_str;

    std::string suffix_buf;
    if (!suffix) {
      suffix_buf = TerminateReasonNames[termReason];
      suffix_buf += ".err";
      suffix = suffix_buf.c_str();
    }

    interpreterHandler->processTestCase(state, msg.str().c_str(), suffix, currentTest->kTestID);
  }

  terminateStateUnchecked(state);

  if (shouldExitOn(termReason))
    haltExecution = true;
}

// XXX shoot me
static const char *okExternalsList[] = { "printf",
                                         "fprintf",
                                         "puts",
                                         "getpid" };
static std::set<std::string> okExternals(okExternalsList,
                                         okExternalsList +
                                         (sizeof(okExternalsList)/sizeof(okExternalsList[0])));

void Executor::callExternalFunction(ExecutionState &state,
                                    KInstruction *target,
                                    Function *function,
                                    std::vector< ref<Expr> > &arguments,
                                    const llvm::APInt *concreteArguments) {
  // check if specialFunctionHandler wants it
  if (specialFunctionHandler->handle(state, function, target, arguments, concreteArguments))
    return;
  
  if (NoExternals && !okExternals.count(function->getName())) {
    warning("Calling not-OK external function : %s\n",
               function->getName().str().c_str());
    terminateStateOnError(state, "externals disallowed", User);
    return;
  }

  // normal external function handling path
  // allocate 128 bits for each argument (+return value) to support fp80's;
  // we could iterate through all the arguments first and determine the exact
  // size we need, but this is faster, and the memory usage isn't significant.
  uint64_t *args = (uint64_t*) alloca(2*sizeof(*args) * (arguments.size() + 1));
  memset(args, 0, 2 * sizeof(*args) * (arguments.size() + 1));
  unsigned wordIndex = 2;
  for (unsigned i = 0; i < arguments.size(); i++) {
    const llvm::APInt &carg = concreteArguments[i];

    if (!AllowExternalSymCalls) {
      ref<Expr> arg = toUnique(state, arguments[i]);
      if (ConstantExpr *ce __attribute__ ((unused)) = dyn_cast<ConstantExpr>(arg)) {
        assert(ce->getAPValue() == carg && "concreteArgument does not match argument!");
      } else {
        terminateStateOnExecError(state, 
                                  "external call with symbolic argument: " + 
                                  function->getName());
        return;
      }
    }

    uint64_t av = carg.getZExtValue();
    Expr::Width width = carg.getBitWidth();
    unsigned int bytes = (width == Expr::Bool ? 1 : width / 8);
    memcpy(&args[wordIndex], &av, bytes);
    wordIndex += (width + 63) / 64;
  }

  state.addressSpace.copyOutConcretes();

  if (!SuppressExternalWarnings) {
    std::string TmpStr;
    llvm::raw_string_ostream os(TmpStr);
    os << "calling external: " << function->getName().str() << "(";
    for (unsigned i=0; i<arguments.size(); i++) {
      os << arguments[i];
      if (i != arguments.size()-1)
        os << ", ";
    }
    os << ")";
    
    if (AllExternalWarnings)
      warning("%s", os.str().c_str());
    else
      warning_once(function, "%s", os.str().c_str());
  }
  bool success = externalDispatcher->executeCall(function, target->inst, args);
  if (!success) {
    terminateStateOnError(state, "failed external call: " + function->getName(),
                          External);
    return;
  }

  if (!state.addressSpace.copyInConcretes()) {
    terminateStateOnError(state, "external modified read-only object",
                          External);
    return;
  }

  Type *resultType = target->inst->getType();
  if (resultType != Type::getVoidTy(function->getContext())) {
    ref<ConstantExpr> e = ConstantExpr::fromMemory((void*) args,
                                           getWidthForLLVMType(resultType));
    bindLocal(target, state, e);
  }
}

/***/

ObjectState *Executor::bindObjectInState(ExecutionState &state,
                                         const MemoryObject *mo,
                                         bool isLocal,
                                         const Array *array) {
  ObjectState *os = array ? new ObjectState(mo, state.addressSpace, array) : new ObjectState(mo, state.addressSpace);
  state.addressSpace.bindObject(mo, os);

  // Its possible that multiple bindings of the same mo in the state
  // will put multiple copies on this list, but it doesn't really
  // matter because all we use this list for is to unbind the object
  // on function return.
  if (isLocal)
    state.stack.back().allocas.push_back(mo);

  return os;
}

void Executor::executeAlloc(ExecutionState &state,
                            ref<Expr> size,
                            uint64_t concreteSize,
                            bool isLocal,
                            KInstruction *target,
                            bool zeroMemory,
                            const ObjectState *reallocFrom,
                            bool staticSize) {
  // We simulate allocation failure for sufficiently large sizes;
  // To save a little memory, we clamp concrete sizes at this threshold,
  // which could make us fail to detect some divergences.
  unsigned failureThreshold = 1 << 31;
  assert(failureThreshold <= UINT_MAX && "allocation failure threshold set too low!");
  unsigned sizeID = (concreteSize > failureThreshold ? failureThreshold : (unsigned) concreteSize);

  if (staticSize) {   // Don't bother saving/checking the allocation size if it's known statically
    assert(isa<ConstantExpr>(size) && "supposedly-static size is not constant!");
    assert(cast<ConstantExpr>(size)->getZExtValue() == concreteSize && "size does not match concreteSize!");
  } else if (!(NoSymbolicAllocations || inhibitForking || state.forkDisabled)) {
    // Check if there is an expected size
    bool findSizes = !runningConcretely;
    bool extendHistory = true;
    unsigned numAllocs = state.allocationHistory.size();
    if (numAllocs < currentTest->allocationDepth) {   // allocation should occur according to expectedAllocations
      bool checkForDivergence = runningConcretely || CheckSymbolicDivergences;
      bool divergence = false;
      if (checkForDivergence) {
        assert(expectedAllocations.size() > numAllocs && "missing expected allocation!");
        if (sizeID != expectedAllocations[numAllocs]) {
          divergence = true;
          handleDivergence(state, "unexpected allocation size");
          if (testTerminated)
            return;
        } else {
          state.allocationHistory.extendAlong(expectedAllocations);
        }
      } else {
        state.allocationHistory.push_back(sizeID);
      }
      if (!divergence) {    // Continue according to the expected allocation
        extendHistory = false;
        findSizes = false;
      }
    } else {
      testIsFertile = true;
    }

    if (findSizes) {
      // Find all possible sizes for the allocation; actually we only allow
      // a limited set of possibilities

      unsigned maxSizes = MaxSizesPerAllocation > 0 ? MaxSizesPerAllocation : UINT_MAX;
      assert(maxSizes > 0 && "maxSizes cannot be zero!");
      std::vector<ref<ConstantExpr> > sizes;
      bool success = solver->getAllValues(state, size, sizes, maxSizes);
      unsigned numSizes = sizes.size();

      if (numSizes >= maxSizes) {
        if (WarnOnConcretization) {
          const InstructionInfo *ii = state.prevPC->info;
          warning_once(ii, "potentially incomplete concretization of symbolic size");
        }

        // We haven't necessarily found all possible sizes, so we do a special
        // check to see if huge sizes are possible
        if ((*std::max_element(sizes.begin(), sizes.end(), constantExprComparator))->getZExtValue() <
            failureThreshold) {
          ref<ConstantExpr> hugeSize;
          success = solver->getValueAssuming(state, size,
                                             UleExpr::create(ConstantExpr::alloc(failureThreshold, size->getWidth()),
                                                             size),
                                             hugeSize);
          if (success && !hugeSize.isNull())
            sizes.push_back(hugeSize);
        }
      }

      // Sort the sizes
      std::sort(sizes.begin(), sizes.end(), constantExprComparator);

      // Fork non-concrete sizes
      for (const ref<ConstantExpr> &newSize : sizes) {
        uint64_t cnewSize = newSize->getZExtValue();
        if (cnewSize == concreteSize)
          continue;

        TimerStatIncrementer timer(stats::forkTime);
        ++stats::forks;

        ConstraintManager newConditions = state.constraints;
        newConditions.addConstraint(EqExpr::create(size, newSize));

        std::unique_ptr<ConcolicTest> test = generateConcolicTest(state, newConditions);
        unsigned newID = (cnewSize > failureThreshold ? failureThreshold : (unsigned) cnewSize);
        test->pushAllocation(newID);
        childTests.push_back(std::move(test));
      }
    }

    if (extendHistory)
      state.allocationHistory.push_back(sizeID);
  }

  // Handle concrete size
  if (!runningConcretely)
    state.addConstraint(EqExpr::create(size, ConstantExpr::create(concreteSize, size->getWidth())));

  if (concreteSize >= failureThreshold) {
    note("found huge malloc, returning 0");
    bindLocal(target, state, ConstantExpr::alloc(0, Context::get().getPointerWidth()));
  } else {
    const llvm::Value *allocSite = state.prevPC->inst;
    size_t allocationAlignment = getAllocationAlignment(allocSite);
    MemoryObject *mo =
        memory->allocate(concreteSize, isLocal, /*isGlobal=*/false,
                         allocSite, allocationAlignment);
    if (!mo) {
      if (ExitOnAllocFailure) {
        terminateStateEarly(state, "Memory allocation failed.", true);
        haltExecution = true;
      } else {
        bindLocal(target, state,
                  ConstantExpr::alloc(0, Context::get().getPointerWidth()));
      }
    } else {
      ObjectState *os = bindObjectInState(state, mo, isLocal);
      if (zeroMemory) {
        os->initializeToZero();
      } else {
        os->initializeToRandom();
      }
      bindLocal(target, state, mo->getBaseExpr());
      
      if (reallocFrom) {
        unsigned count = std::min(reallocFrom->size, os->size);
        reallocFrom->copyTo(os, count);
        state.addressSpace.unbindObject(reallocFrom->getObject());
      }
    }
  }
}

void Executor::executeFree(ExecutionState &state,
                           ref<Expr> address,
                           uint64_t concreteAddress,
                           KInstruction *target) {
  StatePair zeroPointer = fork(state, Expr::createIsZero(address), concreteAddress == 0, true);
  if (zeroPointer.first) {
    if (target)
      bindLocal(target, *zeroPointer.first, memory->createNullPointer());
  }
  if (zeroPointer.second) { // address != 0
    assert (zeroPointer.second == &state && "fork did not return current state!");
    ObjectPair op;
    bool found = resolveExact(state, address, concreteAddress, "free", op);

    if (found) {
      const MemoryObject *mo = op.first;
      if (mo->isLocal) {
        terminateStateOnError(state, "free of alloca", Free, NULL,
                              getAddressInfo(state, address));
      } else if (mo->isGlobal) {
        terminateStateOnError(state, "free of global", Free, NULL,
                              getAddressInfo(state, address));
      } else {
        state.addressSpace.unbindObject(mo);
        if (target)
          bindLocal(target, state, memory->createNullPointer());
      }
    }
  }
}

bool Executor::resolveExact(ExecutionState &state,
                            ref<Expr> p,
                            uint64_t concreteP,
                            const std::string &name,
                            ObjectPair &op) {
  // XXX we may want to be capping this?
  bool found = resolve(state, p, concreteP, -1, op);
  if (testTerminated)
    return false;
  if (!found) {
    terminateStateOnError(state, "memory error: invalid pointer: " + name,
                          Ptr, NULL, getAddressInfo(state, p));
  }
  return found;
}

bool Executor::resolve(ExecutionState &state,
                       ref<Expr> &address, uint64_t caddr, int bytes,
                       ObjectPair &op) {
  // DJF: Do not track stats::resolveTime every time this method is called!
  // While it's useful to see how much time is spent in resolution (a lot, even
  // with concrete addresses), this function is called extremely frequently and
  // the overhead of calling time() is incredible. In my tests, putting a timer
  // here caused KLEE to spend almost 10% of its time checking the time! (with
  // most of that in __vdso_time). So use a sampling profiler instead.

  // Resolve address to a MemoryObject
  const ref<ConstantExpr> concreteAddress = runningConcretely ? cast<ConstantExpr>(address)
                                                              : ConstantExpr::create(caddr, address->getWidth());
  bool found = resolveToObject(state, address, concreteAddress, bytes, op);

  // If desired, resolve further to a concrete offset into the object
  if (found) {
    const char *concretizationReason = 0;
    if (SymbolicPointerMode == eSymbolicPointerFork) {
      if (state.forkDisabled || inhibitForking)
        concretizationReason = "disabled fork on symbolic offset";
      else
        resolveToConcrete(state, address, concreteAddress, op.first);
    } else if (SymbolicPointerMode == eSymbolicPointerPartial) {
      concretizationReason = "symbolic offsets disabled";
    }
    if (concretizationReason != 0)
      address = toConstant(state, address, concreteAddress, concretizationReason);
  }

  return found;
}

bool Executor::resolveToObject(ExecutionState &state,
                               ref<Expr> &address, const ref<ConstantExpr> &concreteAddress, int bytes,
                               ObjectPair &op) {
  uint64_t caddr = concreteAddress->getZExtValue();
  if (SimplifySymIndices && !runningConcretely) {
    if (!isa<ConstantExpr>(address))
      address = state.constraints.simplifyExpr(address);
  }

  // Decide whether to actually do full resolution
  bool doResolution = true;
  if (SymbolicPointerMode == eSymbolicPointerNone) {
    // concretize address
    address = toConstant(state, address, concreteAddress, "symbolic pointers disabled");
    doResolution = false;   // no other resolutions are possible for a concrete address
  } else if (SymbolicPointerMode == eSymbolicPointerUnchecked) {
    doResolution = false;
  }

  // Try to resolve the concrete address
  bool found = state.addressSpace.resolveOne(caddr, op);
  const MemoryObject *mo = found ? op.first : 0;
  unsigned cID;
  if (found) {
    bool inBounds;
    if (bytes == -1) {
      inBounds = (caddr == mo->address);
    } else {
      inBounds = mo->boundsCheckPointer(caddr, bytes);
    }
    if (inBounds) {
      cID = mo->id;
      assert(cID < UINT_MAX && "incredibly large number of MemoryObjects allocated!");
    } else {
      cID = UINT_MAX;
      found = false;
      mo = 0;
    }
  } else {
    cID = UINT_MAX;
  }

  // If we are not solving for other resolutions, we are done
  if (!doResolution)
    return found;

  // If forking is disabled, we restrict to the concrete resolution
  if (state.forkDisabled || inhibitForking) {
    if (!runningConcretely)
      state.addConstraint(createResolutionConstraint(mo, bytes, address, concreteAddress));
    return found;
  }

  // Check if there is an expected resolution
  unsigned numResolutions = state.resolutionHistory.size();
  if (numResolutions < currentTest->resolutionDepth) {   // resolution should occur according to expectedResolutions
    bool checkForDivergence = runningConcretely || CheckSymbolicDivergences;
    bool divergence = false;
    if (checkForDivergence) {
      assert(expectedResolutions.size() > numResolutions && "missing expected resolution!");
      if (cID != expectedResolutions[numResolutions]) {
        divergence = true;
        handleDivergence(state, "unexpected pointer resolution");
        if (testTerminated)
          return false;
      } else {
        state.resolutionHistory.extendAlong(expectedResolutions);
      }
    } else {
      state.resolutionHistory.push_back(cID);
    }
    if (!divergence) {    // Continue according to the concrete resolution
      if (!runningConcretely)
        state.addConstraint(createResolutionConstraint(mo, bytes, address, concreteAddress));
      return found;
    }
  } else {
    testIsFertile = true;
  }

  // Either there was no expected resolution or we diverged from it;
  // either way, we now need to compute all possible resolutions
  // (unless we're running concretely).

  if (runningConcretely) {
    state.resolutionHistory.push_back(cID);
    return found;
  }

  // Generate constraint representing correctness of the concrete resolution
  ref<Expr> resolutionConstraint = createResolutionConstraint(mo, bytes, address, concreteAddress);

  // fast path: single in-bounds resolution
  if (found) {
    if (MaxSymArraySize && mo->size >= MaxSymArraySize) {
      address = toConstant(state, address, concreteAddress, "max-sym-array-size");
    }

    bool inBounds;
    solver->setTimeout(coreSolverTimeout);
    bool success = solver->mustBeTrue(state, resolutionConstraint, inBounds);
    solver->setTimeout(0);
    if (!success) {   // to recover from timeout, concretize address
      address = toConstant(state, address, concreteAddress, "bounds check timeout");
      state.resolutionHistory.push_back(cID);
      return found;
    }

    if (inBounds) {
      if (!runningConcretely)
        state.addConstraint(resolutionConstraint);    // implied, but potentially helpful
      state.resolutionHistory.push_back(cID);
      return true;
    }
  }

  // slow path: no resolution, multiple resolution, or single resolution with out of bounds

  ResolutionList rl;
  ref<ConstantExpr> outOfBoundsAddr;
  solver->setTimeout(coreSolverTimeout);
  bool incomplete = state.addressSpace.resolve(state, solver, address, caddr, bytes,
                                               rl, outOfBoundsAddr,
                                               0, coreSolverTimeout);
  solver->setTimeout(0);
  if (incomplete) {
    if (WarnOnConcretization) {
      warning_once(0, outOfBoundsAddr.isNull()
                      ? "incomplete resolution from timeout"
                      : "incomplete resolution from out-of-bounds");
    }
  }

  // Fork new states for all resolutions other than the concrete one
  if (found) {   // remove the concrete resolution from rl
    ResolutionList::iterator end = rl.end();
    ResolutionList::iterator it = std::find(rl.begin(), end, op);
    if (it != end)    // if the resolution was incomplete it is possible op won't be found
      rl.erase(it);
  } else {
    // concrete address is out of bounds, so don't fork a new out-of-bounds state
    outOfBoundsAddr = ref<ConstantExpr>();
  }
  forkResolutions(state, rl, address, bytes, outOfBoundsAddr);

  // Enforce the concrete resolution
  state.addConstraint(resolutionConstraint);
  state.resolutionHistory.push_back(cID);
  return found;
}

ref<Expr> Executor::createResolutionConstraint(const MemoryObject *mo, int bytes,
                                               const ref<Expr> &address, const ref<ConstantExpr> &concreteAddress) {
  if (mo) {
    if (bytes == -1) {
      return EqExpr::create(address, mo->getBaseExpr());
    } else {
      return mo->getBoundsCheckPointer(address, bytes);
    }
  } else {
    // for out-of-bounds pointers the resolution was necessarily incomplete;
    // so we might as well constrain the address to the concrete value
    return EqExpr::create(address, concreteAddress);
  }
}

void Executor::resolveToConcrete(ExecutionState &state,
                                 ref<Expr> &address,
                                 const ref<ConstantExpr> &concreteAddress,
                                 const MemoryObject *mo) {
  assert(mo && "tried to resolve offset of out-of-bounds pointer!");

  const unsigned caddr = concreteAddress->getZExtValue();
  assert(mo->getOffset(caddr) <= UINT_MAX && "offset does not fit in 32 bits!");
  const unsigned concreteOffset = mo->getOffset(caddr);

  // Check if there is an expected offset
  unsigned numOffsets = state.offsetHistory.size();
  if (numOffsets < currentTest->offsetDepth) {
    bool checkForDivergence = runningConcretely || CheckSymbolicDivergences;
    bool divergence = false;
    if (checkForDivergence) {
      assert(expectedOffsets.size() > numOffsets && "missing expected offset!");
      if (concreteOffset != expectedOffsets[numOffsets]) {
        divergence = true;
        handleDivergence(state, "unexpected pointer offset resolution");
        if (testTerminated)
          return;
      } else {
        state.offsetHistory.extendAlong(expectedOffsets);
      }
    } else {
      state.offsetHistory.push_back(concreteOffset);
    }
    if (!divergence) {    // Continue according to the expected offset
      if (!runningConcretely)
        state.addConstraint(EqExpr::create(address, concreteAddress));
      address = concreteAddress;
      return;
    }
  } else {
    testIsFertile = true;
  }

  // Either there was no expected offset or we diverged from it;
  // either way, we now need to compute all possible offsets
  // (unless we're running concretely).

  if (runningConcretely) {
    state.offsetHistory.push_back(concreteOffset);
    address = concreteAddress;
    return;
  }

  // Find all possible offsets, up to a bound
  const ref<Expr> offset = mo->getOffsetExpr(address);
  unsigned maxOffsets = MaxConcreteOffsets > 0 ? MaxConcreteOffsets : UINT_MAX;
  assert(maxOffsets > 0 && "maxOffsets must be positive!");
  std::vector<ref<ConstantExpr> > offsets;
  bool success;
  solver->setTimeout(coreSolverTimeout);
  success = solver->getAllValues(state, offset, offsets, maxOffsets);
  solver->setTimeout(0);
  if (!success) {
    if (WarnOnConcretization) {
      const InstructionInfo *ii = state.prevPC->info;
      warning_once(ii, "potentially incomplete concretization of symbolic address");
    }
  }

  // Sort the offsets
  std::sort(offsets.begin(), offsets.end(), constantExprComparator);

  // Fork for all offsets other than the concrete one
  for (const ref<ConstantExpr> &possible : offsets) {
    unsigned newOffset = possible->getZExtValue();
    if (newOffset == concreteOffset)
      continue;
    const ref<ConstantExpr> newAddress = ConstantExpr::alloc(mo->getAddress(newOffset), address->getWidth());

    TimerStatIncrementer timer(stats::forkTime);
    ++stats::forks;

    ConstraintManager newConditions = state.constraints;
    newConditions.addConstraint(EqExpr::create(address, newAddress));

    std::unique_ptr<ConcolicTest> test = generateConcolicTest(state, newConditions);
    test->pushOffset(newOffset);
    childTests.push_back(std::move(test));
  }

  // Enforce the concrete offset
  state.addConstraint(EqExpr::create(address, concreteAddress));
  address = concreteAddress;
  state.offsetHistory.push_back(concreteOffset);
}

void Executor::executeMemoryOperation(ExecutionState &state,
                            bool isWrite,
                            ref<Expr> address,
                            uint64_t concreteAddress,
                            ref<ConstantExpr> value /* undef if read */,
                            KInstruction *target /* undef if write */) {
  executeMemoryOperation(state, isWrite, address, concreteAddress, value, value->getAPValue(), target);
}

void Executor::executeMemoryOperation(ExecutionState &state,
                                      bool isWrite,
                                      ref<Expr> address,
                                      uint64_t concreteAddress,
                                      ref<Expr> value /* undef if read */,
                                      const llvm::APInt &concreteValue,
                                      KInstruction *target /* undef if write */) {
  Expr::Width type = (isWrite ? value->getWidth() :
                     getWidthForLLVMType(target->inst->getType()));
  unsigned bytes = Expr::getMinBytesForWidth(type);

  ObjectPair op;
  bool found = resolve(state, address, concreteAddress, bytes, op);
  if (testTerminated)
    return;

  if (found) {
    const MemoryObject *mo = op.first;
    ObjectState *os = op.second;

    if (SimplifySymIndices) {   // address will have already been simplified by resolve
      if (isWrite && !isa<ConstantExpr>(value))
        value = state.constraints.simplifyExpr(value);
    }

    uint64_t concreteOffset = concreteAddress - mo->address;

    if (isWrite) {
      if (os->readOnly) {
        terminateStateOnError(state, "memory error: object read only",
                              ReadOnly);
      } else {
        if (runningConcretely) {
          os->writeConcrete(concreteOffset, concreteValue);
        } else {
          ref<Expr> offset = mo->getOffsetExpr(address);
          os->write(offset, concreteOffset, value, concreteValue);
        }
      }
    } else {
      llvm::APInt concreteResult(type, 0);
      ref<Expr> result;
      if (runningConcretely) {
        result = os->readConcrete(concreteOffset, type, concreteResult);    // faster concrete-only version
      } else {
        ref<Expr> offset = mo->getOffsetExpr(address);
        result = os->read(offset, concreteOffset, type, concreteResult, &state.constraints);
      }

      bindLocal(target, state, result, concreteResult);
    }
  } else {
    terminateStateOnError(state, "memory error: out of bound pointer", Ptr,
                          NULL, getAddressInfo(state, address));
  }
}

void Executor::executeAssume(ExecutionState &state, const ref<Expr> &condition, bool concreteCondition, bool silent) {
  if (!runningConcretely) {
    bool res;
    bool success __attribute__ ((unused)) = solver->mustBeFalse(state, condition, res);
    assert(success && "FIXME: Unhandled solver failure");
    if (res) {
      if (silent) {
        terminateState(state);
      } else {
        terminateStateOnError(state,
                              "invalid klee_assume call (provably false)",
                              Executor::User,
                              NULL,
                              "");
      }
      return;
    } else {
      state.addConstraint(condition);   // we may break the symbolic-concrete invariant for a moment...

      if (!concreteCondition) {   // killing the state here if so
        warning_once(0, "failed klee_assume; generating new test");
        std::unique_ptr<ConcolicTest> test = generateConcolicTest(state, state.constraints);
        test->isReplacement = true;    // force the replacement to be the next test chosen
        // Even if the current test covered new code, the replacement may not since
        // now that code has been registered as covered. So we need to force the
        // replacement to be dumped.
        test->newBlocks = currentTest->newBlocks + newBlocksCovered;
        test->needsProcessing = state.coveredNew || (test->newBlocks > 0);
        childTests.push_back(std::move(test));

        terminateStateUnchecked(state, true);
      }
    }
  } else if (!concreteCondition) {
    terminateStateUnchecked(state, true);    // abort test without counting this as a completed path

    // We need to rerun this test symbolically in order to generate a test
    // satisfying the assumption (if possible). Schedule that to happen
    // immediately.
    currentTest->isReplacement = true;
  }
}

void Executor::executeMakeSymbolic(ExecutionState &state,
                                   const MemoryObject *mo,
                                   const std::string &name,
                                   ConstraintEnforcer enforcer) {
  // Do not create symbolic if we are in the middle of scenario parsing,
  // and reuse cached value if we are simulating an earlier call
  ObjectState *cached;
  if (!ScenarioHandler::handler.allowSymbolic(mo, cached))
    return;
  if (cached) {
    ObjectState *os = new ObjectState(*cached, state.addressSpace);
    state.addressSpace.rebindObject(mo, os);
    return;
  }

  mo->setName(name);

  if (!replayKTest) {
    // If this is the first symbolic ever created, set the starting point
    // for future tests to be here (in general, it cannot be later)
    if (!startingStateFinalized) {
      assert(currentTestState->symbolics.empty() && "did not set starting state correctly!");
      assert(currentTest->generation == 0 && "running a child test without symbolics!");
      assert(tests.size() == 0 && childTests.empty() && "generated tests before setting starting state!");

      startingStateFinalized = true;
      startingState = saveCurrentState();
      ScenarioHandler::handler.saveState();

      // startingState needs to be immediately before the make_symbolic, but we've
      // already advanced the pc; so back it up (prevPC will be corrected when a
      // new test is started)
      startingState.state->pc = currentTestState->prevPC;
    }

    // Get current concrete value
    ObjectState *old = state.addressSpace.findObject(mo);
    assert(old != 0 && "cannot find old state of new symbolic object!");
    old->claim();   // prevent this from being deallocated after rebinding below

    // Find a unique name for this array.  First try the original name,
    // or if that fails try adding a unique identifier.
    unsigned id = 0;
    std::string uniqueName = name;
    while (!state.arrayNames.insert(uniqueName).second) {
      uniqueName = name + "_" + llvm::utostr(++id);
    }

    // If necessary, create a symbolic object state for the memory object
    ObjectState *os;
    const Array *array = 0;
    if (runningConcretely) {
      os = old;   // don't need a symbolic state; can reuse the concrete one
    } else {
      array = arrayCache.CreateArray(uniqueName, mo->size);
      assert(state.addressSpace.findObject(mo) && "klee_make_symbolic argument has unbound memory object!");
      os = new ObjectState(mo, state.addressSpace, array);
      state.addressSpace.rebindObject(mo, os);
    }

    // If following a ConcolicTest, write the concrete value it provides
    bool replaying = false;
    unsigned numObjects = state.symbolics.size();
    if (numObjects < currentTest->objects.size()) {
      const ConcolicTestObject &obj = currentTest->objects[numObjects];
      if (obj.bytes->size != mo->size) {
        // this will clear the mismatched and any other pending objects
        handleDivergence(state, "unmatched klee_make_symbolic");
        if (testTerminated)
          return;
      } else {
        // ConcolicTestObject matches; replay from it
        replaying = true;
        os->overwriteConcretesFrom(*obj.bytes);
      }
    }

    // Otherwise use the old concrete value
    if (!replaying && os != old) {
      os->overwriteConcretesFrom(old->allConcretes(), mo->size);
    }

    // If necessary, modify concrete value to satisfy constraints
    if (enforcer) {
      bool overwrote;
      bool success = enforcer(this, state, os, overwrote);
      if (!success)
        return;   // enforcer has already issued an appropriate error
      if (overwrote && replaying) {
        // Ordinarily this should not be possible, since the replayed value was
        // passed through the enforcer when first generated and so should satisfy
        // the constraints. However, the constraints themselves could have changed
        // (e.g. due to unmodeled concretization).
        handleDivergence(state, "symbolic initialization constraints changed");
        if (testTerminated)
          return;
        replaying = false;
      }
    }

    // If we aren't following a ConcolicTest, save the new value
    if (!replaying) {
      ConcolicTestObject obj;
      obj.name = objectNames.insert(uniqueName);
      obj.bytes = objectBytes.emplace(os->allConcretes(), mo->size);
      currentTest->objects.push_back(obj);
    }

    // Register the new symbolic and clean up
    state.addSymbolic(mo, array);
    ScenarioHandler::handler.addSymbolic(os, state.addressSpace);
    old->release();
  } else {
    ObjectState *os = state.addressSpace.findObject(mo);
    if (replayPosition >= replayKTest->numObjects) {
      terminateStateOnError(state, "replay count mismatch", User);
    } else {
      KTestObject *obj = &replayKTest->objects[replayPosition++];
      if (obj->numBytes != mo->size) {
        terminateStateOnError(state, "replay size mismatch", User);
      } else {
        for (unsigned i=0; i<mo->size; i++)
          os->write8(i, obj->bytes[i]);
      }
    }
  }
}

SavedState Executor::saveCurrentState() {
  return SavedState(*currentTestState, MemoryObject::counter, memory->getResetToken());
}

void Executor::restoreState(const SavedState &ss) {
  assert(ss.state && "tried to restore from empty SavedState!");
  MemoryObject::counter = ss.memoryCounter;
  memory->reset(ss.resetToken);
  currentTestState = new ExecutionState(*ss.state);
}

/***/

void Executor::runFunctionAsMain(Function *f,
				 int argc,
				 char **argv,
				 char **envp) {
  std::vector<ref<ConstantExpr> > arguments;

  // Seed random number generator
  unsigned seed = RandomSeed;
  if (seed == 0) {
    std::random_device rd;
    seed = rd();
  }
  theRNG.seed(seed);

  MemoryObject *argvMO = 0;

  // In order to make uclibc happy and be closer to what the system is
  // doing we lay out the environments at the end of the argv array
  // (both are terminated by a null). There is also a final terminating
  // null that uclibc seems to expect, possibly the ELF header?

  int envc;
  for (envc=0; envp[envc]; ++envc) ;

  unsigned NumPtrBytes = Context::get().getPointerWidth() / 8;
  KFunction *kf = kmodule->functionMap[f];
  assert(kf);
  Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
  if (ai!=ae) {
    arguments.push_back(ConstantExpr::alloc(argc, Expr::Int32));
    if (++ai!=ae) {
      Instruction *first = static_cast<Instruction *>(f->begin()->begin());
      argvMO =
          memory->allocate((argc + 1 + envc + 1 + 1) * NumPtrBytes,
                           /*isLocal=*/false, /*isGlobal=*/true,
                           /*allocSite=*/first, /*alignment=*/8);

      if (!argvMO)
        klee_error("Could not allocate memory for function arguments");

      arguments.push_back(argvMO->getBaseExpr());

      if (++ai!=ae) {
        uint64_t envp_start = argvMO->address + (argc+1)*NumPtrBytes;
        arguments.push_back(memory->createPointer(envp_start));

        if (++ai!=ae)
          klee_error("invalid main function (expect 0-3 arguments)");
      }
    }
  }

  ExecutionState *state = new ExecutionState(kmodule->functionMap[f],
                                             resolutionTrie,
                                             offsetTrie,
                                             functionResolutionTrie,
                                             allocationTrie);
  blocksCovered.insert(&f->getEntryBlock());

  if (statsTracker)
    statsTracker->framePushed(*state, 0);

  assert(arguments.size() == f->arg_size() && "wrong number of arguments");
  for (unsigned i = 0, e = f->arg_size(); i != e; ++i)
    bindArgument(kf, i, *state, arguments[i]);

  if (argvMO) {
    ObjectState *argvOS = bindObjectInState(*state, argvMO, false);

    for (int i=0; i<argc+1+envc+1+1; i++) {
      if (i==argc || i>=argc+1+envc) {
        // Write NULL pointer
        argvOS->write(i * NumPtrBytes, memory->createNullPointer());
      } else {
        char *s = i<argc ? argv[i] : envp[i-(argc+1)];
        int j, len = strlen(s);

        MemoryObject *arg =
            memory->allocate(len + 1, /*isLocal=*/false, /*isGlobal=*/true,
                             /*allocSite=*/state->pc->inst, /*alignment=*/8);
        if (!arg)
          klee_error("Could not allocate memory for function arguments");
        ObjectState *os = bindObjectInState(*state, arg, false);
        for (j=0; j<len+1; j++)
          os->write8(j, s[j]);

        // Write pointer to newly allocated and initialised argv/envp c-string
        argvOS->write(i * NumPtrBytes, arg->getBaseExpr());
      }
    }
  }

  initializeGlobals(*state);

  run(*state);

  delete state;

  if (statsTracker)
    statsTracker->done();

  ScenarioHandler::handler.cleanup();   // needed because of hack below inherited from KLEE

  // hack to clear memory objects
  // should not be cleared before now as statsTracker may query MemoryManager
  delete memory;
  memory = nullptr;

  globalObjects.clear();
  globalAddresses.clear();
}

unsigned Executor::getPathStreamID(const ExecutionState &state) {
  assert(pathWriter);
  return state.pathOS.getID();
}

unsigned Executor::getSymbolicPathStreamID(const ExecutionState &state) {
  assert(symPathWriter);
  return state.symPathOS.getID();
}

void Executor::getConstraintLog(const ExecutionState &state, std::string &res,
                                Interpreter::LogType logFormat) {

  std::ostringstream info;

  switch (logFormat) {
  case STP: {
    Query query(state.constraints, ConstantExpr::False);
    char *log = solver->getConstraintLog(query);
    res = std::string(log);
    free(log);
  } break;

  case KQUERY: {
    std::string Str;
    llvm::raw_string_ostream info(Str);
    ExprPPrinter::printConstraints(info, state.constraints);
    res = info.str();
  } break;

  case SMTLIB2: {
    std::string Str;
    llvm::raw_string_ostream info(Str);
    ExprSMTLIBPrinter printer;
    printer.setOutput(info);
    Query query(state.constraints, ConstantExpr::False);
    printer.setQuery(query);
    printer.generateOutput();
    res = info.str();
  } break;

  default:
    klee_warning("Executor::getConstraintLog() : Log format not supported!");
  }
}

KTest *Executor::getSymbolicSolution(const ExecutionState &state, unsigned testID, bool &ranSymbolically) {
  assert(&state == currentTestState && "tried getSymbolicSolution for state other than current test!");
  currentTest->kTestID = testID;
  ranSymbolically = !runningConcretely;

  KTest *b = new KTest;
  b->numObjects = currentTest->objects.size();
  b->objects = new KTestObject[b->numObjects];
  for (unsigned i = 0; i < b->numObjects; i++) {
    const ConcolicTestObject &obj = currentTest->objects[i];
    KTestObject &kobj = b->objects[i];
    kobj.name = const_cast<char *>(obj.name->c_str());    // work around C type of KTestObject
    kobj.numBytes = obj.bytes->size;
    kobj.bytes = obj.bytes->bytes;
  }

  ScenarioHandler::handler.saveSkeleton(b);

  return b;
}

bool Executor::computeInitialValues(const std::vector<std::pair<const MemoryObject *, const Array *> > &symbolics,
                                    const ConstraintManager &constraints, std::vector< std::vector<unsigned char> > &vals) {
  solver->setTimeout(coreSolverTimeout);
  std::vector<const Array *> objects;
  for (unsigned i = 0; i != symbolics.size(); ++i)
    objects.push_back(symbolics[i].second);
  bool success = solver->getInitialValues(constraints, objects, vals);
  solver->setTimeout(0);
  if (!success) {
    klee_warning("unable to compute initial values (invalid constraints?)!");
    dumpConstraints(constraints);
    return false;
  }

  return true;
}

void Executor::dumpConstraints(const ConstraintManager &constraints) {
  ExprPPrinter::printConstraints(llvm::errs(), constraints);
}
void Executor::dumpState(const ExecutionState &state) {
  llvm::errs() << "path: ";
  for (bool b : state.forkHistory)
    llvm::errs() << b;
  llvm::errs() << "\n";
}

void Executor::getCoveredLines(const ExecutionState &state,
                               std::map<const std::string*, std::set<unsigned> > &res) {
  res = state.coveredLines;
}

void Executor::doImpliedValueConcretization(ExecutionState &state,
                                            ref<Expr> e,
                                            ref<ConstantExpr> value) {
  if (DebugCheckForImpliedValues)
    ImpliedValue::checkForImpliedValues(solver->solver, e, value);

  ImpliedValueList results;
  ImpliedValue::getImpliedValues(e, value, results);
  for (ImpliedValueList::iterator it = results.begin(), ie = results.end();
       it != ie; ++it) {
    ReadExpr *re = it->first.get();

    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(re->index)) {
      ObjectState *os = state.addressSpace.findBackedObject(re->updates.root);

      if (!os) {
        // object has been free'd, no need to concretize (although as
        // in other cases we would like to concretize the outstanding
        // reads, but we have no facility for that yet)
      } else {
        assert(!os->readOnly &&
               "not possible? read only object with static read?");
        os->write(CE->getZExtValue(), it->second);
      }
    }
  }
}

Expr::Width Executor::getWidthForLLVMType(llvm::Type *type) const {
  return kmodule->targetData->getTypeSizeInBits(type);
}

size_t Executor::getAllocationAlignment(const llvm::Value *allocSite) const {
  // FIXME: 8 was the previous default. We shouldn't hard code this
  // and should fetch the default from elsewhere.
  const size_t forcedAlignment = 8;
  size_t alignment = 0;
  llvm::Type *type = NULL;
  std::string allocationSiteName(allocSite->getName().str());
  if (const GlobalValue *GV = dyn_cast<GlobalValue>(allocSite)) {
    alignment = GV->getAlignment();
    if (const GlobalVariable *globalVar = dyn_cast<GlobalVariable>(GV)) {
      // All GlobalVariables's have pointer type
      llvm::PointerType *ptrType =
          dyn_cast<llvm::PointerType>(globalVar->getType());
      assert(ptrType && "globalVar's type is not a pointer");
      type = ptrType->getElementType();
    } else {
      type = GV->getType();
    }
  } else if (const AllocaInst *AI = dyn_cast<AllocaInst>(allocSite)) {
    alignment = AI->getAlignment();
    type = AI->getAllocatedType();
  } else if (isa<InvokeInst>(allocSite) || isa<CallInst>(allocSite)) {
    // FIXME: Model the semantics of the call to use the right alignment
    llvm::Value *allocSiteNonConst = const_cast<llvm::Value *>(allocSite);
    const CallSite cs = (isa<InvokeInst>(allocSiteNonConst)
                             ? CallSite(cast<InvokeInst>(allocSiteNonConst))
                             : CallSite(cast<CallInst>(allocSiteNonConst)));
    llvm::Function *fn =
        klee::getDirectCallTarget(cs, /*moduleIsFullyLinked=*/true);
    if (fn)
      allocationSiteName = fn->getName().str();

    klee_warning_once(fn != NULL ? fn : allocSite,
                      "Alignment of memory from call \"%s\" is not "
                      "modelled. Using alignment of %zu.",
                      allocationSiteName.c_str(), forcedAlignment);
    alignment = forcedAlignment;
  } else {
    llvm_unreachable("Unhandled allocation site");
  }

  if (alignment == 0) {
    assert(type != NULL);
    // No specified alignment. Get the alignment for the type.
    if (type->isSized()) {
      alignment = kmodule->targetData->getPrefTypeAlignment(type);
    } else {
      klee_warning_once(allocSite, "Cannot determine memory alignment for "
                                   "\"%s\". Using alignment of %zu.",
                        allocationSiteName.c_str(), forcedAlignment);
      alignment = forcedAlignment;
    }
  }

  // Currently we require alignment be a power of 2
  if (!bits64::isPowerOfTwo(alignment)) {
    klee_warning_once(allocSite, "Alignment of %zu requested for %s but this "
                                 "not supported. Using alignment of %zu",
                      alignment, allocSite->getName().str().c_str(),
                      forcedAlignment);
    alignment = forcedAlignment;
  }
  assert(bits64::isPowerOfTwo(alignment) &&
         "Returned alignment must be a power of two");
  return alignment;
}

void Executor::prepareForEarlyExit() {
  if (statsTracker) {
    // Make sure stats get flushed out
    statsTracker->done();
  }
}

void Executor::message(const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_message_at(currentTestState->sourceLocation().c_str(), msg, ap);
  va_end(ap);
}

void Executor::note(const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_note_at(currentTestState->sourceLocation().c_str(), msg, ap);
  va_end(ap);
}

void Executor::note_at(const InstructionInfo &ii, const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_note_at(ii.sourceLocation().c_str(), msg, ap);
  va_end(ap);
}

void Executor::error(const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_error_at(currentTestState->sourceLocation().c_str(), msg, ap);
  va_end(ap);
}

void Executor::nonfatal_error_at(const InstructionInfo &ii, const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_nonfatal_error_at(ii.sourceLocation().c_str(), msg, ap);
  va_end(ap);
}

void Executor::warning(const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_warning_at(currentTestState->sourceLocation().c_str(), msg, ap);
  va_end(ap);
}

void Executor::warning_once(const void *id, const char *msg, ...) const {
  va_list ap;
  va_start(ap, msg);
  klee_warning_once_at(currentTestState->sourceLocation().c_str(), id, msg, ap);
  va_end(ap);
}

///

Interpreter *Interpreter::create(LLVMContext &ctx, const InterpreterOptions &opts,
                                 InterpreterHandler *ih) {
  return new Executor(ctx, opts, ih);
}
