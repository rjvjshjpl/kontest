//===-- intrinsics.c ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

/* Straight C for linking simplicity */

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>

#include "klee/klee.h"
#include "klee/Internal/intrinsics.h"   // private intrinsics we need to implement
#include "kontest/scenarios.h"

#include "klee/Internal/ADT/KTest.h"

static KTest *testData = 0;
static unsigned testPosition = 0;

static unsigned beginBlockPosition = 0, endBlockPosition = 0, loopBlockPosition = 0;
static unsigned char *beginBlockActive = 0, *endBlockActive = 0, *loopBlockActive = 0;
static unsigned beginBlockActiveSize, endBlockActiveSize, loopBlockActiveSize;

static enum {
  UNKNOWN = -1,
  REPLAY = 0,   // nonzero values indicate non-replay mode
  RANDOM = 1,
  CONCRETE = 2
} rand_mode = UNKNOWN;

static unsigned char rand_byte(void) {
  unsigned x = rand();
  x ^= x>>16;
  x ^= x>>8;
  return x & 0xFF;
}

static void load_test_data() {
  if (!testData) {
    char tmp[256];
    char *name = getenv("KTEST_FILE");

    if (!name) {
      fprintf(stdout, "KLEE-RUNTIME: KTEST_FILE not set, please enter .ktest path: ");
      fflush(stdout);
      name = tmp;
      if (!fgets(tmp, sizeof tmp, stdin) || !strlen(tmp)) {
        fprintf(stderr, "KLEE-RUNTIME: cannot replay, no KTEST_FILE or user input\n");
        exit(1);
      }
      tmp[strlen(tmp)-1] = '\0'; /* kill newline */
    }
    testData = kTest_fromFile(name);
    if (!testData) {
      fprintf(stderr, "KLEE-RUNTIME: unable to open .ktest file\n");
      exit(1);
    }
  }
}

void klee_assert_fail(const char *expr, const char *file, int line, const char *func) {
  printf("%s:%u: failed klee_assert `%s'\n", file, line, expr);
  abort();
}

static void report_internal_error(const char *msg, ...)
    __attribute__((format(printf, 1, 2)));
static void report_internal_error(const char *msg, ...) {
  fprintf(stderr, "KLEE_RUN_TEST_ERROR: ");
  va_list ap;
  va_start(ap, msg);
  vfprintf(stderr, msg, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  char *testErrorsNonFatal = getenv("KLEE_RUN_TEST_ERRORS_NON_FATAL");
  if (testErrorsNonFatal) {
    fprintf(stderr, "KLEE_RUN_TEST_ERROR: Forcing execution to continue\n");
  } else {
    exit(1);
  }
}

__attribute__((noreturn))
void klee_report_error(const char *file,
                       int line,
                       const char *message,
                       const char *suffix) {
  report_internal_error("klee_report_error: %s:%d: %s", file, line, message);
  exit(1);
}

void klee_make_symbolic_range(void *array, size_t nbytes, const char *name, int begin, int end) {
  if (rand_mode == UNKNOWN) {
    if (getenv("KLEE_CONCRETE")) {
      rand_mode = CONCRETE;
    } else if (getenv("KLEE_RANDOM")) {
      struct timeval tv;
      gettimeofday(&tv, 0);
      rand_mode = RANDOM;
      srand(tv.tv_sec ^ tv.tv_usec);
    } else {
      rand_mode = REPLAY;
    }
  }

  if (rand_mode) {
    if (rand_mode == CONCRETE)
      return;
    if (!strcmp(name,"syscall_a0")) {
      unsigned long long *v = array;
      assert(nbytes == 8);
      *v = rand() % 69;
    } else if (begin != end && nbytes == sizeof(int)) {
      int *i = array;
      *i = (rand() % (end - begin)) + begin;
    } else {
      char *c = array;
      size_t i;
      for (i=0; i<nbytes; i++)
        c[i] = rand_byte();
    }
    return;
  }

  load_test_data();

  for (;; ++testPosition) {
    if (testPosition >= testData->numObjects) {
      report_internal_error("out of inputs. Will use zero if continuing.");
      memset(array, 0, nbytes);
      break;
    } else {
      KTestObject *o = &testData->objects[testPosition];
      int nameLen = strlen(name);
      int oNameLen = strlen(o->name);
      if (strcmp("model_version", o->name) == 0 &&
          strcmp("model_version", name) != 0) {
        // Skip over this KTestObject because we've hit
        // `model_version` which is from the POSIX runtime
        // and the caller didn't ask for it.
        continue;
      }
      // We tolerate mismatches from KLEE appending "_ID" to make object names
      // unique. Rather than keep track of the number of requests per name we
      // just check that the name either matches exactly or has an extra suffix
      // starting with an underscore.
      if (strncmp(name, o->name, nameLen) != 0
          || (oNameLen > nameLen && o->name[nameLen] != '_')) {
        report_internal_error(
            "object name mismatch. Requesting \"%s\" but returning \"%s\"",
            name, o->name);
      }
      memcpy(array, o->bytes, nbytes < o->numBytes ? nbytes : o->numBytes);
      if (nbytes != o->numBytes) {
        report_internal_error("object sizes differ. Expected %zu but got %u",
                              nbytes, o->numBytes);
        if (o->numBytes < nbytes)
          memset((char *)array + o->numBytes, 0, nbytes - o->numBytes);
      }
      if (begin != end && nbytes == sizeof(int)) {
        int *iaddr = array;
        int val = *iaddr;
        if (val < begin || val >= end)
          report_internal_error("invalid klee_make_symbolic_range(%u,%u,%s) value, got: %u\n", begin, end, name, val);
      }
      ++testPosition;
      break;
    }
  }
}

void klee_make_symbolic(void *array, size_t nbytes, const char *name) {
  klee_make_symbolic_range(array, nbytes, name, 0, 0);
}

void klee_silent_exit(int x) {
  exit(x);
}

void klee_assume(uintptr_t x) {
  if (!x) {
    report_internal_error("invalid klee_assume");
  }
}

#define KLEE_GET_VALUE_STUB(suffix, type)	\
	type klee_get_value##suffix(type x) { \
		return x; \
	}

KLEE_GET_VALUE_STUB(f, float)
KLEE_GET_VALUE_STUB(d, double)
KLEE_GET_VALUE_STUB(l, long)
KLEE_GET_VALUE_STUB(ll, long long)
KLEE_GET_VALUE_STUB(_i32, int32_t)
KLEE_GET_VALUE_STUB(_i64, int64_t)

#undef KLEE_GET_VALUE_STUB

/* not sure we should even define.  is for debugging. */
void klee_print_expr(const char *msg, ...) { }

void klee_set_forking(unsigned enable) { }

void klee_alias_function(const char* fn_name, const char* new_fn_name) { }

unsigned klee_is_symbolic(uintptr_t n) {
  return 0;
}

unsigned klee_is_symbolic_in_symbolic_mode(uintptr_t n) {
  return 1;
}

int klee_require_in_list(int x, int n, int *choices) {
  if (n < 1) {
    report_internal_error("invalid call to klee_require_in_list");
    return 0;   // what else can we do?
  }
  if (rand_mode == RANDOM) {
    // concrete value was chosen randomly; need to overwrite
    return choices[rand() % n];
  } else {
    // just verify that the concrete value is one of the choices
    int i;
    for (i = 0; i < n; i++) {
      if (x == choices[i])
        return x;
    }
    report_internal_error("klee_require_in_list failed");
    return choices[0];
  }
}

void klee_obtain_true_index(int *x, int n, char *list, const char *name) {
  if (n < 1)
    report_internal_error("invalid call to klee_obtain_true_index");

  klee_make_symbolic(x, sizeof(int), name);

  if (rand_mode != REPLAY) {
    // need to overwrite concrete value
    int i, count = 0;
    for (i = 0; i < n; i++) {
      if (list[i])
        count++;
    }

    count = rand() % count;
    for (i = 0; i < n; i++) {
      if (list[i] && count-- == 0) {
        *x = i;
        return;
      }
    }
    report_internal_error("call to klee_obtain_true_index with no true indices");
  } else {
    // just verify that the concrete value is a true index
    if (!list[*x])
      report_internal_error("klee_obtain_true_index failed");
  }
}

static void read_active_vector(char *name, unsigned char **vector, unsigned *size) {
  KTestObject *obj = &testData->objects[testPosition++];
  if (strcmp(obj->name, name))
    report_internal_error("scenario replay data corrupted");
  *vector = obj->bytes;
  *size = obj->numBytes;
}

int kontest_internal_mode(unsigned mode) {
  if (mode != KT_RUNNING)
    return 0;
  if (testPosition != 0)
    report_internal_error("scenario sequence corrupted (parse began after creating symbolic)");

  // Load active block vectors from KTest file
  load_test_data();
  read_active_vector("kontest_beginBlockActive", &beginBlockActive, &beginBlockActiveSize);
  read_active_vector("kontest_endBlockActive", &endBlockActive, &endBlockActiveSize);
  read_active_vector("kontest_loopBlockActive", &loopBlockActive, &loopBlockActiveSize);

  return 1;
}

int kontest_internal_begin_block(unsigned type, KontestIdentifier id, int data1, int data2) {
  if (!beginBlockActive)
    report_internal_error("scenario sequence corrupted (block begun before parsing)");
  if (beginBlockPosition >= beginBlockActiveSize)
    report_internal_error("scenario sequence corrupted (block begun too many times)");

  return beginBlockActive[beginBlockPosition++];
}

int kontest_internal_end_block() {
  if (!endBlockActive)
    report_internal_error("scenario sequence corrupted (block ended before parsing)");
  if (endBlockPosition >= endBlockActiveSize)
    report_internal_error("scenario sequence corrupted (block ended too many times)");

  return endBlockActive[endBlockPosition++];
}

int kontest_internal_loop() {
  if (!loopBlockActive)
    report_internal_error("scenario sequence corrupted (block looped before parsing)");
  if (loopBlockPosition >= loopBlockActiveSize)
    report_internal_error("scenario sequence corrupted (block looped too many times)");

  return loopBlockActive[loopBlockPosition++];
}
