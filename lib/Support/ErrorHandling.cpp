//===-- ErrorHandling.cpp -------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Internal/Support/ErrorHandling.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>

#include <set>

using namespace klee;
using namespace llvm;

FILE *klee::klee_warning_file = NULL;
FILE *klee::klee_message_file = NULL;

struct MessageType {
  static const MessageType Warning;
  static const MessageType WarningOnce;
  static const MessageType Error;
  static const MessageType Note;
  static const MessageType Message;

  const char *prefix;
  bool useWarningFile;
  bool bold;
  llvm::raw_ostream::Colors color;
};

const MessageType MessageType::Warning = { "WARNING", true, false, llvm::raw_ostream::MAGENTA };
const MessageType MessageType::WarningOnce = { "WARNING ONCE", true, true, llvm::raw_ostream::MAGENTA };
const MessageType MessageType::Error = { "ERROR", true, true, llvm::raw_ostream::RED };
const MessageType MessageType::Note = { "NOTE", true, true, llvm::raw_ostream::WHITE };
const MessageType MessageType::Message = { nullptr, false, false, llvm::raw_ostream::SAVEDCOLOR };

namespace {
cl::opt<bool> WarningsOnlyToFile(
    "warnings-only-to-file", cl::init(false),
    cl::desc("All warnings will be written to warnings.txt only.  If disabled, "
             "they are also written on screen."));
}

static void klee_vfmessage(FILE *fp, const MessageType &type, const char *loc, const char *msg,
                           va_list ap) {
  if (!fp)
    return;

  llvm::raw_fd_ostream fdos(fileno(fp), /*shouldClose=*/false,
                            /*unbuffered=*/true);
  bool modifyConsoleColor = fdos.is_displayed() && (fp == stderr);

  if (modifyConsoleColor && type.prefix)
    fdos.changeColor(type.color, type.bold, false);

  fdos << "KLEE: ";
  if (type.prefix)
    fdos << type.prefix << ": ";
  if (loc)
    fdos << loc << ": ";

  // FIXME: Can't use fdos here because we need to print
  // a variable number of arguments and do substitution
  vfprintf(fp, msg, ap);
  fflush(fp);

  fdos << "\n";

  if (modifyConsoleColor)
    fdos.resetColor();

  fdos.flush();
}

/* Prints a message/warning.

   If pfx is NULL, this is a regular message, and it's sent to
   klee_message_file (messages.txt).  Otherwise, it is sent to
   klee_warning_file (warnings.txt).

   Iff onlyToFile is false, the message is also printed on stderr.
*/
static void klee_vmessage(const MessageType &type, const char *loc, bool onlyToFile, const char *msg,
                          va_list ap) {
  if (!onlyToFile) {
    va_list ap2;
    va_copy(ap2, ap);
    klee_vfmessage(stderr, type, loc, msg, ap2);
    va_end(ap2);
  }

  klee_vfmessage(type.useWarningFile ? klee_warning_file : klee_message_file, type, loc, msg, ap);
}

void klee::klee_message(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_vmessage(MessageType::Message, NULL, false, msg, ap);
  va_end(ap);
}

void klee::klee_message_at(const char *loc, const char *msg, va_list ap) {
  klee_vmessage(MessageType::Message, loc, false, msg, ap);
}

/* Message to be written only to file */
void klee::klee_message_to_file(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_vmessage(MessageType::Message, NULL, true, msg, ap);
  va_end(ap);
}

void klee::klee_note(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_vmessage(MessageType::Note, NULL, false, msg, ap);
  va_end(ap);
}

void klee::klee_note_at(const char *loc, const char *msg, va_list ap) {
  klee_vmessage(MessageType::Note, loc, false, msg, ap);
}

void klee::klee_error(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_vmessage(MessageType::Error, NULL, false, msg, ap);
  va_end(ap);
  exit(1);
}

void klee::klee_error_at(const char *loc, const char *msg, va_list ap) {
  klee_vmessage(MessageType::Error, loc, false, msg, ap);
  exit(1);
}

void klee::klee_nonfatal_error(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_vmessage(MessageType::Error, NULL, false, msg, ap);
  va_end(ap);
}

void klee::klee_nonfatal_error_at(const char *loc, const char *msg, va_list ap) {
  klee_vmessage(MessageType::Error, loc, false, msg, ap);
}

void klee::klee_warning(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_warning_at(NULL, msg, ap);
  va_end(ap);
}

void klee::klee_warning_at(const char *loc, const char *msg, va_list ap) {
  klee_vmessage(MessageType::Warning, loc, WarningsOnlyToFile, msg, ap);
}

/* Prints a warning once per message. */
void klee::klee_warning_once(const void *id, const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  klee_warning_once_at(nullptr, id, msg, ap);
  va_end(ap);
}

void klee::klee_warning_once_at(const char *loc, const void *id,
                                const char *msg, va_list ap) {
  static std::set<std::pair<const void *, const char *> > keys;
  std::pair<const void *, const char *> key;

  /* "calling external" messages contain the actual arguments with
     which we called the external function, so we need to ignore them
     when computing the key. */
  if (strncmp(msg, "calling external", strlen("calling external")) != 0)
    key = std::make_pair(id, msg);
  else
    key = std::make_pair(id, "calling external");

  if (!keys.count(key)) {
    keys.insert(key);
    klee_vmessage(MessageType::WarningOnce, loc, WarningsOnlyToFile, msg, ap);
  }
}
