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
// Created by dfremont on 7/18/17.
//

#include "Utils.h"

#include <klee/klee.h>

#include <stdarg.h>
#include <stdio.h>

// State used by the scenario DSL
int kontest_internal_active;
int kontest_internal_running;

void kontest_error_inner(const char *file, int line,
                         const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  char message[256];
  vsnprintf(message, sizeof message, fmt, ap);
  klee_report_error(file, line, message, "user");
  va_end(ap);
}
