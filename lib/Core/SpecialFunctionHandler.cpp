//===-- SpecialFunctionHandler.cpp ----------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Memory.h"
#include "SpecialFunctionHandler.h"
#include "TimingSolver.h"

#include "klee/ExecutionState.h"

#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "klee/Internal/Support/Debug.h"
#include "klee/Internal/Support/ErrorHandling.h"

#include "Executor.h"
#include "MemoryManager.h"

#include "klee/CommandLine.h"

#include "llvm/IR/Module.h"
#include "llvm/ADT/Twine.h"
#include "llvm/IR/DataLayout.h"

#include <errno.h>

using namespace llvm;
using namespace klee;

namespace {
  cl::opt<bool>
  SilentKleeAssume("silent-klee-assume",
                   cl::init(false),
                   cl::desc("Silently terminate paths with an infeasible "
                            "condition given to klee_assume() rather than "
                            "emitting an error (default=false)"));
}


/// \todo Almost all of the demands in this file should be replaced
/// with terminateState calls.

///



// FIXME: We are more or less committed to requiring an intrinsic
// library these days. We can move some of this stuff there,
// especially things like realloc which have complicated semantics
// w.r.t. forking. Among other things this makes delayed query
// dispatch easier to implement.
static SpecialFunctionHandler::HandlerInfo handlerInfo[] = {
#define add(name, handler, ret) { name, \
                                  &SpecialFunctionHandler::handler, \
                                  false, ret, false }
#define addDNR(name, handler) { name, \
                                &SpecialFunctionHandler::handler, \
                                true, false, false }
  addDNR("__assert_rtn", handleAssertFail),
  addDNR("__assert_fail", handleAssertFail),
  addDNR("klee_assert_fail", handleAssertFail),   // platform-independent version of the above
  addDNR("_assert", handleAssert),
  addDNR("abort", handleAbort),
  addDNR("_exit", handleExit),
  { "exit", &SpecialFunctionHandler::handleExit, true, false, true },
  addDNR("klee_abort", handleAbort),
  addDNR("klee_silent_exit", handleSilentExit),  
  addDNR("klee_report_error", handleReportError),

  add("klee_assume", handleAssume, false),
  add("klee_check_memory_access", handleCheckMemoryAccess, false),
  add("klee_get_valuef", handleGetValue, true),
  add("klee_get_valued", handleGetValue, true),
  add("klee_get_valuel", handleGetValue, true),
  add("klee_get_valuell", handleGetValue, true),
  add("klee_get_value_i32", handleGetValue, true),
  add("klee_get_value_i64", handleGetValue, true),
  add("klee_define_fixed_object", handleDefineFixedObject, false),
  add("klee_define_external_object", handleDefineExternalObject, false),
  add("klee_get_obj_size", handleGetObjSize, true),
  add("klee_get_errno", handleGetErrno, true),
  add("klee_is_symbolic", handleIsSymbolic, true),
  add("klee_is_symbolic_in_symbolic_mode", handleIsSymbolicInSymbolicMode, true),
  add("klee_make_concrete", handleMakeConcrete, true),
  add("klee_make_symbolic", handleMakeSymbolic, false),
  add("klee_make_symbolic_range", handleMakeSymbolic, false),
  add("klee_mark_global", handleMarkGlobal, false),
  add("klee_merge", handleMerge, false),
  add("klee_prefer_cex", handlePreferCex, false),
  add("klee_posix_prefer_cex", handlePosixPreferCex, false),
  add("klee_print_expr", handlePrintExpr, false),
  add("klee_print_range", handlePrintRange, false),
  add("klee_require_in_list", handleRequireInList, true),
  add("klee_obtain_true_index", handleObtainTrueIndex, false),
  add("klee_set_forking", handleSetForking, false),
  add("klee_stack_trace", handleStackTrace, false),
  add("klee_warning", handleWarning, false),
  add("klee_warning_once", handleWarningOnce, false),
  add("klee_alias_function", handleAliasFunction, false),

  add("kontest_internal_mode", handleScenarioMode, true),
  add("kontest_internal_begin_block", handleBeginBlock, true),
  add("kontest_internal_end_block", handleEndBlock, true),
  add("kontest_internal_loop", handleLoopBlock, true),

  add("calloc", handleCalloc, true),
  add("free", handleFree, false),
  add("malloc", handleMalloc, true),
  add("realloc", handleRealloc, true),

  // operator delete[](void*)
  add("_ZdaPv", handleDeleteArray, false),
  // operator delete(void*)
  add("_ZdlPv", handleDelete, false),

  // operator new[](unsigned int)
  add("_Znaj", handleNewArray, true),
  // operator new(unsigned int)
  add("_Znwj", handleNew, true),

  // FIXME-64: This is wrong for 64-bit long...

  // operator new[](unsigned long)
  add("_Znam", handleNewArray, true),
  // operator new(unsigned long)
  add("_Znwm", handleNew, true),

  // clang -fsanitize=unsigned-integer-overflow
  add("__ubsan_handle_add_overflow", handleAddOverflow, false),
  add("__ubsan_handle_sub_overflow", handleSubOverflow, false),
  add("__ubsan_handle_mul_overflow", handleMulOverflow, false),
  add("__ubsan_handle_divrem_overflow", handleDivRemOverflow, false),

#undef addDNR
#undef add  
};

SpecialFunctionHandler::const_iterator SpecialFunctionHandler::begin() {
  return SpecialFunctionHandler::const_iterator(handlerInfo);
}

SpecialFunctionHandler::const_iterator SpecialFunctionHandler::end() {
  // NULL pointer is sentinel
  return SpecialFunctionHandler::const_iterator(0);
}

SpecialFunctionHandler::const_iterator& SpecialFunctionHandler::const_iterator::operator++() {
  ++index;
  if ( index >= SpecialFunctionHandler::size())
  {
    // Out of range, return .end()
    base=0; // Sentinel
    index=0;
  }

  return *this;
}

int SpecialFunctionHandler::size() {
	return sizeof(handlerInfo)/sizeof(handlerInfo[0]);
}

SpecialFunctionHandler::SpecialFunctionHandler(Executor &_executor) 
  : executor(_executor) {}


void SpecialFunctionHandler::prepare() {
  unsigned N = size();

  for (unsigned i=0; i<N; ++i) {
    HandlerInfo &hi = handlerInfo[i];
    Function *f = executor.kmodule->module->getFunction(hi.name);
    
    // No need to create if the function doesn't exist, since it cannot
    // be called in that case.
  
    if (f && (!hi.doNotOverride || f->isDeclaration())) {
      // Make sure NoReturn attribute is set, for optimization and
      // coverage counting.
      if (hi.doesNotReturn)
        f->addFnAttr(Attribute::NoReturn);

      // Change to a declaration since we handle internally (simplifies
      // module and allows deleting dead code).
      if (!f->isDeclaration())
        f->deleteBody();
    }
  }
}

void SpecialFunctionHandler::bind() {
  unsigned N = sizeof(handlerInfo)/sizeof(handlerInfo[0]);

  for (unsigned i=0; i<N; ++i) {
    HandlerInfo &hi = handlerInfo[i];
    Function *f = executor.kmodule->module->getFunction(hi.name);
    
    if (f && (!hi.doNotOverride || f->isDeclaration()))
      handlers[f] = std::make_pair(hi.handler, hi.hasReturnValue);
  }
}


bool SpecialFunctionHandler::handle(ExecutionState &state, 
                                    Function *f,
                                    KInstruction *target,
                                    std::vector< ref<Expr> > &arguments,
                                    const llvm::APInt *concreteArguments) {
  handlers_ty::iterator it = handlers.find(f);
  if (it != handlers.end()) {    
    Handler h = it->second.first;
    bool hasReturnValue = it->second.second;
     // FIXME: Check this... add test?
    if (!hasReturnValue && !target->inst->use_empty()) {
      executor.terminateStateOnExecError(state, 
                                         "expected return value from void special function");
    } else {
      (this->*h)(state, target, arguments, concreteArguments);
    }
    return true;
  } else {
    return false;
  }
}

/****/

// reads a concrete string from memory
std::string 
SpecialFunctionHandler::readStringAtAddress(ExecutionState &state, 
                                            uint64_t address) {
  ObjectPair op;
  if (!state.addressSpace.resolveOne(address, op))
    executor.error("KLEE intrinsic given out-of-bounds or symbolic string pointer");
  assert(op.first->address == address && "XXX interior pointer unhandled");

  const MemoryObject *mo = op.first;
  const ObjectState *os = op.second;

  char *buf = new char[mo->size];
  memcpy(buf, os->allConcretes(), mo->size - 1);
  buf[mo->size - 1] = 0;
  
  std::string result(buf);
  delete[] buf;
  return result;
}

/****/

void SpecialFunctionHandler::handleAbort(ExecutionState &state,
                           KInstruction *target,
                           std::vector<ref<Expr> > &arguments,
                           const llvm::APInt *concreteArguments) {
  assert(arguments.size()==0 && "invalid number of arguments to abort");
  executor.terminateStateOnError(state, "abort failure", Executor::Abort);
}

void SpecialFunctionHandler::handleExit(ExecutionState &state,
                           KInstruction *target,
                           std::vector<ref<Expr> > &arguments,
                           const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 && "invalid number of arguments to exit");
  executor.terminateStateOnExit(state);
}

void SpecialFunctionHandler::handleSilentExit(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments,
                                              const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 && "invalid number of arguments to exit");
  executor.terminateState(state);
}

void SpecialFunctionHandler::handleAliasFunction(ExecutionState &state,
						 KInstruction *target,
						 std::vector<ref<Expr> > &arguments,
             const llvm::APInt *concreteArguments) {
  assert(arguments.size()==2 && 
         "invalid number of arguments to klee_alias_function");
  std::string old_fn = readStringAtAddress(state, concreteArguments[0].getZExtValue());
  std::string new_fn = readStringAtAddress(state, concreteArguments[1].getZExtValue());
  KLEE_DEBUG_WITH_TYPE("alias_handling", llvm::errs() << "Replacing " << old_fn
                                           << "() with " << new_fn << "()\n");
  if (old_fn == new_fn)
    state.removeFnAlias(old_fn);
  else state.addFnAlias(old_fn, new_fn);
}

void SpecialFunctionHandler::handleAssert(ExecutionState &state,
                                          KInstruction *target,
                                          std::vector<ref<Expr> > &arguments,
                                          const llvm::APInt *concreteArguments) {
  assert(arguments.size()==3 && "invalid number of arguments to _assert");  
  executor.terminateStateOnError(state,
				 "ASSERTION FAIL: " + readStringAtAddress(state, concreteArguments[0].getZExtValue()),
				 Executor::Assert);
}

void SpecialFunctionHandler::handleAssertFail(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments,
                                              const llvm::APInt *concreteArguments) {
  assert(arguments.size()==4 && "invalid number of arguments to __assert_fail");
  executor.terminateStateOnError(state,
				 "ASSERTION FAIL: " + readStringAtAddress(state, concreteArguments[0].getZExtValue()),
				 Executor::Assert);
}

void SpecialFunctionHandler::handleReportError(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments,
                                               const llvm::APInt *concreteArguments) {
  assert(arguments.size()==4 && "invalid number of arguments to klee_report_error");
  
  // arguments[0], arguments[1] are file, line
  executor.terminateStateOnError(state,
				 readStringAtAddress(state, concreteArguments[2].getZExtValue()),
				 Executor::ReportError,
				 readStringAtAddress(state, concreteArguments[3].getZExtValue()).c_str());
}

void SpecialFunctionHandler::handleMerge(ExecutionState &state,
                           KInstruction *target,
                           std::vector<ref<Expr> > &arguments,
                           const llvm::APInt *concreteArguments) {
  // nop
}

void SpecialFunctionHandler::handleNew(ExecutionState &state,
                         KInstruction *target,
                         std::vector<ref<Expr> > &arguments,
                         const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to new");

  bool staticSize = (target->operands[1] < 0);
  executor.executeAlloc(state, arguments[0], concreteArguments[0].getZExtValue(), false, target, false, 0, staticSize);
}

void SpecialFunctionHandler::handleDelete(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments,
                            const llvm::APInt *concreteArguments) {
  // FIXME: Should check proper pairing with allocation type (malloc/free,
  // new/delete, new[]/delete[]).

  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to delete");
  executor.executeFree(state, arguments[0], concreteArguments[0].getZExtValue());
}

void SpecialFunctionHandler::handleNewArray(ExecutionState &state,
                              KInstruction *target,
                              std::vector<ref<Expr> > &arguments,
                              const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to new[]");
  bool staticSize = (target->operands[1] < 0);
  executor.executeAlloc(state, arguments[0], concreteArguments[0].getZExtValue(), false, target, false, 0, staticSize);
}

void SpecialFunctionHandler::handleDeleteArray(ExecutionState &state,
                                 KInstruction *target,
                                 std::vector<ref<Expr> > &arguments,
                                 const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to delete[]");
  executor.executeFree(state, arguments[0], concreteArguments[0].getZExtValue());
}

void SpecialFunctionHandler::handleMalloc(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments,
                                  const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to malloc");
  bool staticSize = (target->operands[1] < 0);
  executor.executeAlloc(state, arguments[0], concreteArguments[0].getZExtValue(), false, target, false, 0, staticSize);
}

void SpecialFunctionHandler::handleAssume(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments,
                            const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_assume");
  
  ref<Expr> e = arguments[0];
  
  if (e->getWidth() != Expr::Bool)
    e = NeExpr::create(e, ConstantExpr::create(0, e->getWidth()));

  executor.executeAssume(state, e, concreteArguments[0].getBoolValue(), SilentKleeAssume);
}

void SpecialFunctionHandler::handleIsSymbolic(ExecutionState &state,
                                KInstruction *target,
                                std::vector<ref<Expr> > &arguments,
                                const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_is_symbolic");

  bool symb = !isa<ConstantExpr>(arguments[0]);
  executor.bindLocal(target, state, ConstantExpr::create(symb, Expr::Int32), APInt(Expr::Int32, symb));
}

void SpecialFunctionHandler::handleIsSymbolicInSymbolicMode(ExecutionState &state,
                                                            KInstruction *target,
                                                            std::vector<ref<Expr> > &arguments,
                                                            const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_is_symbolic_in_symbolic_mode");

  bool symb = executor.runningConcretely || !isa<ConstantExpr>(arguments[0]);
  executor.bindLocal(target, state, ConstantExpr::create(symb, Expr::Int32), APInt(Expr::Int32, symb));
}

void SpecialFunctionHandler::handlePreferCex(ExecutionState &state,
                                             KInstruction *target,
                                             std::vector<ref<Expr> > &arguments,
                                             const llvm::APInt *concreteArguments) {
  klee_warning_once(0, "klee_prefer_cex not supported - ignoring");
}

void SpecialFunctionHandler::handlePosixPreferCex(ExecutionState &state,
                                             KInstruction *target,
                                             std::vector<ref<Expr> > &arguments,
                                             const llvm::APInt *concreteArguments) {
  klee_warning_once(0, "klee_posix_prefer_cex not supported - ignoring");
}

void SpecialFunctionHandler::handlePrintExpr(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments,
                                  const llvm::APInt *concreteArguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_print_expr");

  std::string msg_str = readStringAtAddress(state, concreteArguments[0].getZExtValue());
  llvm::errs() << msg_str << ":" << arguments[1] << "\n";
}

void SpecialFunctionHandler::handleSetForking(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments,
                                              const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_set_forking");

  state.forkDisabled = !concreteArguments[0].getBoolValue();
}

void SpecialFunctionHandler::handleStackTrace(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments,
                                              const llvm::APInt *concreteArguments) {
  state.dumpStack(outs());
}

void SpecialFunctionHandler::handleWarning(ExecutionState &state,
                                           KInstruction *target,
                                           std::vector<ref<Expr> > &arguments,
                                           const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_warning");

  std::string msg_str = readStringAtAddress(state, concreteArguments[0].getZExtValue());
  klee_warning("%s: %s", state.stack.back().kf->function->getName().data(), 
               msg_str.c_str());
}

void SpecialFunctionHandler::handleWarningOnce(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments,
                                               const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_warning_once");

  std::string msg_str = readStringAtAddress(state, concreteArguments[0].getZExtValue());
  klee_warning_once(0, "%s: %s", state.stack.back().kf->function->getName().data(),
                    msg_str.c_str());
}

void SpecialFunctionHandler::handlePrintRange(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments,
                                  const llvm::APInt *concreteArguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_print_range");

  std::string msg_str = readStringAtAddress(state, concreteArguments[0].getZExtValue());
  llvm::errs() << msg_str << ":" << arguments[1];
  if (!isa<ConstantExpr>(arguments[1])) {
    // FIXME: Pull into a unique value method?
    ref<ConstantExpr> value;
    bool success __attribute__ ((unused)) = executor.solver->getValue(state, arguments[1], value);
    assert(success && "FIXME: Unhandled solver failure");
    bool res;
    success = executor.solver->mustBeTrue(state, 
                                          EqExpr::create(arguments[1], value), 
                                          res);
    assert(success && "FIXME: Unhandled solver failure");
    if (res) {
      llvm::errs() << " == " << value;
    } else { 
      llvm::errs() << " ~= " << value;
      std::pair< ref<Expr>, ref<Expr> > res =
        executor.solver->getRange(state, arguments[1]);
      llvm::errs() << " (in [" << res.first << ", " << res.second <<"])";
    }
  }
  llvm::errs() << "\n";
}

void SpecialFunctionHandler::handleGetObjSize(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments,
                                  const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_get_obj_size");
  ObjectPair op;
  bool found = executor.resolveExact(state, arguments[0], concreteArguments[0].getZExtValue(), "klee_get_obj_size", op);
  if (found) {
    executor.bindLocal(
        target, state,
        ConstantExpr::create(op.first->size,
                             executor.kmodule->targetData->getTypeSizeInBits(
                                 target->inst->getType())));
  }
}

void SpecialFunctionHandler::handleGetErrno(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments,
                                            const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==0 &&
         "invalid number of arguments to klee_get_errno");
  executor.bindLocal(target, state,
                     ConstantExpr::create(errno, Expr::Int32));
}

void SpecialFunctionHandler::handleCalloc(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments,
                            const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==2 &&
         "invalid number of arguments to calloc");

  ref<Expr> size = MulExpr::create(arguments[0],
                                   arguments[1]);
  uint64_t csize = concreteArguments[0].getZExtValue() * concreteArguments[1].getZExtValue();
  bool staticSize = (target->operands[1] < 0) && (target->operands[2] < 0);
  executor.executeAlloc(state, size, csize, false, target, true, 0, staticSize);
}

void SpecialFunctionHandler::handleRealloc(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments,
                            const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==2 &&
         "invalid number of arguments to realloc");
  ref<Expr> address = arguments[0];
  ref<Expr> size = arguments[1];
  uint64_t caddress = concreteArguments[0].getZExtValue();
  uint64_t csize = concreteArguments[1].getZExtValue();
  bool staticSize = (target->operands[2] < 0);

  bool zeroSize = (csize == 0);
  if (staticSize) {   // don't bother forking if the size is a fixed constant
    assert(isa<ConstantExpr>(size) && "supposedly-static size is not constant!");
    assert(cast<ConstantExpr>(size)->getZExtValue() == csize && "inconsistent constant sizes!");
  } else {
    executor.fork(state, Expr::createIsZero(size), zeroSize, true);
  }
  
  if (zeroSize) {
    executor.executeFree(state, address, caddress, target);
  } else {
    Executor::StatePair zeroPointer = executor.fork(state,
                                                    Expr::createIsZero(address),
                                                    caddress == 0,
                                                    true);
    
    if (zeroPointer.first) { // address == 0
      executor.executeAlloc(*zeroPointer.first, size, csize, false, target, false, 0, staticSize);
    } 
    if (zeroPointer.second) { // address != 0
      assert(zeroPointer.second == &state && "fork did not return current state!");
      ObjectPair op;
      bool found = executor.resolveExact(state, address, caddress, "realloc", op);
      
      if (found) {
        executor.executeAlloc(state, size, csize, false, target, false, op.second, staticSize);
      }
    }
  }
}

void SpecialFunctionHandler::handleFree(ExecutionState &state,
                          KInstruction *target,
                          std::vector<ref<Expr> > &arguments,
                          const llvm::APInt *concreteArguments) {
  // XXX should type check args
  assert(arguments.size()==1 &&
         "invalid number of arguments to free");
  executor.executeFree(state, arguments[0], concreteArguments[0].getZExtValue());
}

void SpecialFunctionHandler::handleCheckMemoryAccess(ExecutionState &state,
                                                     KInstruction *target,
                                                     std::vector<ref<Expr> > 
                                                       &arguments,
                                                     const llvm::APInt *concreteArguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_check_memory_access");

  ref<Expr> address= arguments[0];
  uint64_t caddress = concreteArguments[0].getZExtValue();
  uint64_t csize = concreteArguments[1].getZExtValue();

  ObjectPair op;

  if (!state.addressSpace.resolveOne(caddress, op)) {
    executor.terminateStateOnError(state,
                                   "check_memory_access: memory error",
                                   Executor::Ptr, NULL,
                                   executor.getAddressInfo(state, address));
  } else {
    ref<Expr> chk =
        op.first->getBoundsCheckPointer(address, csize);
    if (!chk->isTrue()) {
      executor.terminateStateOnError(state,
                                     "check_memory_access: memory error",
                                     Executor::Ptr, NULL,
                                     executor.getAddressInfo(state, address));
    }
  }
}

void SpecialFunctionHandler::handleGetValue(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments,
                                            const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_get_value");

  executor.executeGetValue(state, arguments[0], concreteArguments[0], target);
}

void SpecialFunctionHandler::handleDefineFixedObject(ExecutionState &state,
                                                     KInstruction *target,
                                                     std::vector<ref<Expr> > &arguments,
                                                     const llvm::APInt *concreteArguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_define_fixed_object");
  assert(isa<ConstantExpr>(arguments[0]) &&
         "expect constant address argument to klee_define_fixed_object");
  assert(isa<ConstantExpr>(arguments[1]) &&
         "expect constant size argument to klee_define_fixed_object");

  assert(cast<ConstantExpr>(arguments[0])->getAPValue() == concreteArguments[0] && "concreteArgument does not match argument!");
  assert(cast<ConstantExpr>(arguments[1])->getAPValue() == concreteArguments[1] && "concreteArgument does not match argument!");

  uint64_t address = concreteArguments[0].getZExtValue();
  uint64_t size = concreteArguments[1].getZExtValue();
  MemoryObject *mo = executor.memory->allocateFixed(address, size, state.prevPC->inst);
  executor.bindObjectInState(state, mo, false);
  mo->isUserSpecified = true; // XXX hack;
}

void SpecialFunctionHandler::handleDefineExternalObject(ExecutionState &state,
                                                     KInstruction *target,
                                                     std::vector<ref<Expr> > &arguments,
                                                     const llvm::APInt *concreteArguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_define_external_object");
  assert(isa<ConstantExpr>(arguments[0]) &&
         "expect constant address argument to klee_define_external_object");
  assert(isa<ConstantExpr>(arguments[1]) &&
         "expect constant size argument to klee_define_external_object");

  assert(cast<ConstantExpr>(arguments[0])->getAPValue() == concreteArguments[0] && "concreteArgument does not match argument!");
  assert(cast<ConstantExpr>(arguments[1])->getAPValue() == concreteArguments[1] && "concreteArgument does not match argument!");

  uint64_t address = concreteArguments[0].getZExtValue();
  uint64_t size = concreteArguments[1].getZExtValue();
  MemoryObject *mo = executor.memory->allocateFixed(address, size, state.prevPC->inst);
  ObjectState *os = executor.bindObjectInState(state, mo, false);
  uint8_t *loc = reinterpret_cast<uint8_t*>(address);
  os->overwriteConcretesFrom(loc, size);
}

void SpecialFunctionHandler::handleMakeConcrete(ExecutionState &state,
                                                KInstruction *target,
                                                std::vector<ref<Expr> > &arguments,
                                                const llvm::APInt *concreteArguments) {
  // FIXME: Should be a user.err, not an assert.
  assert(arguments.size() == 1 && "invalid number of arguments to klee_make_concrete");

  executor.executeMakeConcrete(state, arguments[0], concreteArguments[0], target);
}

void SpecialFunctionHandler::handleMakeSymbolic(ExecutionState &state,
                                                KInstruction *target,
                                                std::vector<ref<Expr> > &arguments,
                                                const llvm::APInt *concreteArguments) {
  std::string name;

  // FIXME: For backwards compatibility, we should eventually enforce the
  // correct arguments.
  bool hasBounds = false;
  if (arguments.size() == 2) {
    name = "unnamed";
  } else if (arguments.size() == 3) {
    name = readStringAtAddress(state, concreteArguments[2].getZExtValue());
  } else {
    // FIXME: Should be a user.err, not an assert.
    assert(arguments.size() == 5 &&
           "invalid number of arguments to klee_make_symbolic");
    name = readStringAtAddress(state, concreteArguments[2].getZExtValue());
    hasBounds = true;
  }

  ObjectPair op;
  bool found = executor.resolveExact(state, arguments[0], concreteArguments[0].getZExtValue(), "make_symbolic", op);
  if (!found)
    return;   // resolveExact has already issued an error

  const MemoryObject *mo = op.first;
  const ObjectState *old = op.second;

  if (old->readOnly) {
    executor.terminateStateOnError(state, "cannot make readonly object symbolic",
                                   Executor::User);
    return;
  }

  if (concreteArguments[1] != mo->size) {
    executor.terminateStateOnError(state,
                                   "wrong size given to klee_make_symbolic[_name]",
                                   Executor::User);
    return;
  }

  Executor::ConstraintEnforcer enforcer;
  if (hasBounds) {
    ConstantExpr *lower = dyn_cast<ConstantExpr>(arguments[3]);
    ConstantExpr *upper = dyn_cast<ConstantExpr>(arguments[4]);

    if (!lower || !upper) {
      executor.terminateStateOnError(state, "symbolic bounds given to klee_make_symbolic", Executor::User);
      return;
    }
    if (lower->getWidth() != Expr::Int32 || upper->getWidth() != Expr::Int32 || mo->size != 4) {
      executor.terminateStateOnError(state,
                                     "bounds or bounded object in klee_make_symbolic not 4-byte",
                                     Executor::User);
      return;
    }
    int32_t clower = lower->getZExtValue();
    int32_t cupper = upper->getZExtValue();
    if (clower > cupper) {
      executor.terminateStateOnError(state, "reversed bounds given to klee_make_symbolic", Executor::User);
      return;
    }

    if (clower < cupper) {    // lower == upper indicates no bounds
      const llvm::APInt &apLower = concreteArguments[3];
      enforcer = [&lower, &upper, &clower, &cupper, apLower](Executor *executor, ExecutionState &state,
                                                    ObjectState *os, bool &overwrote) {
        // Read current value
        llvm::APInt cres;
        const ref<Expr> val = os->read(0, Expr::Int32, cres, &state.constraints);
        int32_t cval = cres.getZExtValue();

        // If concrete value violates bounds, overwrite it
        overwrote = false;
        if (cval < clower || cval >= cupper) {
          os->writeConcrete(0, apLower);
          overwrote = true;
        }

        // Constrain symbolic value
        if (!executor->runningConcretely) {
          if (lower->getZExtValue() == 0) {
            state.addConstraint(UltExpr::create(val, upper));
          } else {
            state.addConstraint(SleExpr::create(lower, val));
            state.addConstraint(SltExpr::create(val, upper));
          }
        }

        return true;
      };
    }
  }

  executor.executeMakeSymbolic(state, mo, name, enforcer);
}

void SpecialFunctionHandler::handleMarkGlobal(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments,
                                              const llvm::APInt *concreteArguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_mark_global");  

  ObjectPair op;
  bool found = executor.resolveExact(state, arguments[0], concreteArguments[0].getZExtValue(), "mark_global", op);
  
  if (found) {
    const MemoryObject *mo = op.first;
    assert(!mo->isLocal);
    mo->isGlobal = true;
  }
}

void SpecialFunctionHandler::handleAddOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments,
                                               const llvm::APInt *concreteArguments) {
  executor.terminateStateOnError(state, "overflow on unsigned addition",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleSubOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments,
                                               const llvm::APInt *concreteArguments) {
  executor.terminateStateOnError(state, "overflow on unsigned subtraction",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleMulOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments,
                                               const llvm::APInt *concreteArguments) {
  executor.terminateStateOnError(state, "overflow on unsigned multiplication",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleDivRemOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments,
                                                  const llvm::APInt *concreteArguments) {
  executor.terminateStateOnError(state, "overflow on division or remainder",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleRequireInList(ExecutionState &state,
                                                 KInstruction *target,
                                                 std::vector<ref<Expr> > &arguments,
                                                 const llvm::APInt *concreteArguments) {
  const ref<Expr> &value = arguments[0];
  const llvm::APInt &cvalue = concreteArguments[0];
  Expr::Width width = value->getWidth();
  unsigned bytes = Expr::getMinBytesForWidth(width);

  const ConstantExpr *listSize = dyn_cast<ConstantExpr>(arguments[1]);
  assert(listSize && "symbolic list size given to klee_require_in_list!");
  assert(listSize->getZExtValue() == concreteArguments[1].getZExtValue() && "symbolic-concrete mismatch!");

  const ConstantExpr *list = dyn_cast<ConstantExpr>(arguments[2]);
  assert(list && "symbolic list given to klee_require_in_list!");
  assert(list->getZExtValue() == concreteArguments[2].getZExtValue() && "symbolic-concrete mismatch!");

  ObjectPair op;
  bool found = executor.resolveExact(state, arguments[2], list->getZExtValue(), "require_in_list", op);
  if (!found)
    return;   // error message already output in resolveExact

  unsigned ls = listSize->getZExtValue();
  assert((op.first->size >= ls * bytes) && "size longer than list in klee_require_in_list!");

  ref<Expr> disjunction = ConstantExpr::create(0, Expr::Bool);
  llvm::APInt concreteResult;
  bool foundInList __attribute__ ((unused)) = false;
  for (unsigned i = 0; i < ls; i++) {
    const ref<Expr> entry = op.second->read(i * bytes, width, concreteResult, &state.constraints);
    if (cvalue == concreteResult)
      foundInList = true;
    disjunction = OrExpr::create(disjunction, EqExpr::create(value, entry));
  }

  assert(foundInList && "klee_require_in_list given concrete value not in list!");

  state.addConstraint(disjunction);
  executor.bindLocal(target, state, value, cvalue);
}

void SpecialFunctionHandler::handleObtainTrueIndex(ExecutionState &state,
                                                   KInstruction *target,
                                                   std::vector<ref<Expr> > &arguments,
                                                   const llvm::APInt *concreteArguments) {
  // Find index
  ObjectPair op;
  bool found = executor.resolveExact(state, arguments[0], concreteArguments[0].getZExtValue(), "obtain_true_index", op);
  if (!found)
    return;   // error message already output in resolveExact

  Expr::Width indexWidth;
  if (op.first->size == 4)
    indexWidth = Expr::Int32;
  else if (op.first->size == 8)
    indexWidth = Expr::Int64;
  else
    assert(0 && "index of invalid width given to obtain_true_index!");

  const MemoryObject *indexMO = op.first;
  std::string name = readStringAtAddress(state, concreteArguments[3].getZExtValue());

  ObjectState *indexOS = op.second;
  if (indexOS->readOnly) {
    executor.terminateStateOnError(state, "cannot make readonly index symbolic",
                                   Executor::User);
    return;
  }

  // Find list
  const ConstantExpr *listSize = dyn_cast<ConstantExpr>(arguments[1]);
  assert(listSize && "symbolic list size given to obtain_true_index");
  assert(listSize->getZExtValue() == concreteArguments[1].getZExtValue() && "symbolic-concrete mismatch!");

  const ConstantExpr *list = dyn_cast<ConstantExpr>(arguments[2]);
  assert(list && "symbolic list given to obtain_true_index!");
  assert (list->getZExtValue() == concreteArguments[2].getZExtValue() && "symbolic-concrete mismatch!");

  found = executor.resolveExact(state, arguments[2], list->getZExtValue(), "obtain_true_index", op);
  if (!found)
    return;   // error message already output in resolveExact

  unsigned ls = listSize->getZExtValue();
  assert((op.first->size >= ls) && "size longer than list in obtain_true_index!");

  // Set up constraints on the index
  auto enforcer = [&indexWidth, &ls, &op](Executor *executor, ExecutionState &state,
                                             ObjectState *os, bool &overwrote) {
    // Read current index
    llvm::APInt cres;
    const ref<Expr> index = os->read(0, indexWidth, cres, &state.constraints);
    uint64_t cindex = cres.getZExtValue();

    // If concrete value is not a true index, overwrite it
    overwrote = false;
    const uint8_t *clist = op.second->allConcretes();
    if (cindex >= ls || !clist[cindex]) {
      // Search for true index
      for (cindex = 0; cindex < ls; cindex++) {
        if (clist[cindex])
          break;
      }

      if (cindex >= ls) {
        executor->terminateStateOnError(state, "klee_obtain_true_index given list with no true indices",
                                        Executor::User);
        return false;
      }

      os->writeConcrete(0, APInt(indexWidth, cindex));
      overwrote = true;
    }

    // Constrain symbolic value
    if (!executor->runningConcretely) {
      // Get symbolic expression for value
      const ref<Expr> value = op.second->read(index, cindex, Expr::Int8, cres, &state.constraints);

      // Constrain the index to be valid and the value to be true
      state.addConstraint(op.first->getBoundsCheckOffset(executor->memory->createZExtToPointerWidth(index)));
      state.addConstraint(NeExpr::create(value, ConstantExpr::create(0, Expr::Int8)));
    }

    return true;
  };

  // Make index symbolic
  executor.executeMakeSymbolic(state, indexMO, name, enforcer);
}

void SpecialFunctionHandler::handleScenarioMode(ExecutionState &state,
                                                KInstruction *target,
                                                std::vector<ref<Expr> > &arguments,
                                                const llvm::APInt *concreteArguments) {
  assert(arguments.size() == 1 && "invalid number of arguments given to scenario_mode!");
  assert(isa<ConstantExpr>(arguments[0]) && "symbolic argument given to scenario_mode!");

  ScenarioHandler::handler.setMode(concreteArguments[0].getZExtValue(), state);

  executor.bindLocal(target, state, ConstantExpr::create(1, Expr::Int32));
}

void SpecialFunctionHandler::handleBeginBlock(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments,
                                              const llvm::APInt *concreteArguments) {
  assert(arguments.size() == 4 && "invalid number of arguments given to begin_block!");
  assert(isa<ConstantExpr>(arguments[0]) && "symbolic block type given to begin_block!");
  assert(isa<ConstantExpr>(arguments[1]) && "symbolic identifier given to begin_block!");

  unsigned type = concreteArguments[0].getZExtValue();
  uint64_t id = concreteArguments[1].getZExtValue();
  const ref<Expr> &data1 = arguments[2];
  const ref<Expr> &data2 = arguments[3];

  std::string name = id ? readStringAtAddress(state, id) : "";

  bool res = ScenarioHandler::handler.beginBlock(type, id, data1, data2, name, state);

  executor.bindLocal(target, state, ConstantExpr::create(res, Expr::Int32));
}

void SpecialFunctionHandler::handleEndBlock(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments,
                                            const llvm::APInt *concreteArguments) {
  assert(arguments.size() == 0 && "invalid number of arguments given to end_block!");

  bool res = ScenarioHandler::handler.endBlock(state);

  executor.bindLocal(target, state, ConstantExpr::create(res, Expr::Int32));
}

void SpecialFunctionHandler::handleLoopBlock(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments,
                                            const llvm::APInt *concreteArguments) {
  assert(arguments.size() == 0 && "invalid number of arguments given to loop_block!");

  bool res = ScenarioHandler::handler.loopBlock(state);

  executor.bindLocal(target, state, ConstantExpr::create(res, Expr::Int32));
}