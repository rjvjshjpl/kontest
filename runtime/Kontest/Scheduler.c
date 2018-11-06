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

#include <klee/kontest.h>
#include <klee/klee.h>
#include "Scheduler.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

unsigned numSchedulers = 0;
unsigned schedulerListSize = 0;
Scheduler **schedulers = 0;

void kontest_registerScheduler(Scheduler *s) {
  s->id = numSchedulers++;
  if (!schedulers) {
    schedulerListSize = 1;
    schedulers = malloc(schedulerListSize * sizeof(Scheduler*));
  }
  if (numSchedulers > schedulerListSize) {
    schedulerListSize *= 2;
    schedulers = realloc(schedulers, schedulerListSize * sizeof(Scheduler*));
  }
  schedulers[s->id] = s;
}

Scheduler *kontest_schedulerFromID(KontestSchedulerID id) {
  if (id > numSchedulers)
    kontest_error("invalid KontestSchedulerID %d", id);
  return schedulers[id];
};

void kontest_runScheduler(Scheduler *s, unsigned steps, unsigned *hint) {
  unsigned i;

  // Call initializer
  void *userData = 0;
  if (s->initializer) {
    userData = s->initializer();
    if (!userData)
      kontest_error("initializer for scheduler %d failed", s->id);
  }

  // Set up initial state
  KontestSchedulerState state;
  state.step = 0;
  state.userData = userData;
  bool *ruleActive = malloc(s->numRules * sizeof(bool));

#ifdef KONTEST_FOR_RUNTEST
  // Set up diagnostics
  bool verbose = getenv("KONTEST_VERBOSE");
#endif    // KONTEST_FOR_RUNTEST

  // Run steps
  for (; state.step < steps; state.step++) {
    // Run monitors
    for (i = s->numMonitors; i-- > 0;)
      s->monitors[i](state);

    // Find applicable rules
    unsigned numEnabled = 0;
    unsigned hint_skips = 0;
    for (i = s->numRules; i-- > 0;) {
      KontestRule rule = s->rules[i];
      if (!rule.precondition || rule.precondition(state)) {
        ruleActive[i] = true;
        if (hint && hint[state.step] == i)
          hint_skips = numEnabled;
        numEnabled++;
      } else {
        ruleActive[i] = false;
      }
    }
    if (numEnabled == 0)
      kontest_error("deadlock in scheduler %d", s->id);

    // Apply a rule nondeterministically
    unsigned skips = klee_range_hinted(0, hint_skips, numEnabled, "kontest_rule_to_apply");
    for (i = s->numRules; i-- > 0;) {
      if (ruleActive[i] && skips-- == 0) {
#ifdef KONTEST_FOR_RUNTEST
        if (verbose)
          printf("KONTEST: scheduler %d, step %d: rule '%s'\n", s->id, state.step, s->rules[i].name);
#endif  // KONTEST_FOR_RUNTEST
        s->rules[i].action(state);
        break;
      }
    }
  }

  // Run monitors one last time
  for (i = s->numMonitors; i-- > 0;)
    s->monitors[i](state);

  // Clean up
  free(ruleActive);
}

/*
 * External API
 */

KontestSchedulerID kontest_create_scheduler(KontestSchedulerInit initializer,
                                            unsigned numRules, KontestRule *rules,
                                            unsigned numMonitors, KontestMonitor *monitors) {
  Scheduler *s = malloc(sizeof(Scheduler));
  s->initializer = initializer;
  s->numRules = numRules;
  s->rules = rules;
  s->numMonitors = numMonitors;
  s->monitors = monitors;

  kontest_registerScheduler(s);   // sets s->id

  return s->id;
}

void kontest_run_scheduler(KontestSchedulerID s, unsigned steps, unsigned *hint) {
  Scheduler *scheduler = kontest_schedulerFromID(s);
  kontest_runScheduler(scheduler, steps, hint);
}
