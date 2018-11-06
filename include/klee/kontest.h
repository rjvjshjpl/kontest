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

#ifndef __KONTEST_H__
#define __KONTEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Schedulers
 */

typedef unsigned KontestSchedulerID;

typedef struct {
  unsigned step;
  void *userData;
} KontestSchedulerState;

typedef void* (*KontestSchedulerInit)(void);
typedef int (*KontestRulePre)(KontestSchedulerState);
typedef void (*KontestRuleAction)(KontestSchedulerState);
typedef void (*KontestMonitor)(KontestSchedulerState);

typedef struct {
  char *name;
  KontestRulePre precondition;    // optional
  KontestRuleAction action;
} KontestRule;

// initializer optional
KontestSchedulerID kontest_create_scheduler(KontestSchedulerInit initializer,
                                            unsigned numRules, KontestRule *rules,
                                            unsigned numMonitors, KontestMonitor *monitors);

// hint optional; fails on deadlock or init failure
void kontest_run_scheduler(KontestSchedulerID s, unsigned steps, unsigned *hint);

#ifdef __cplusplus
}
#endif

#endif // __KONTEST_H__
