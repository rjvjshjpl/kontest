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

#ifndef KONTEST_UTILS_H
#define KONTEST_UTILS_H

#define kontest_error(fmt, ...) kontest_error_inner(__FILE__, __LINE__, fmt, __VA_ARGS__)

__attribute__((noreturn)) __attribute__((format(printf, 3, 4)))
void kontest_error_inner(const char *file, int line,
                         const char *fmt, ...);

#endif // KONTEST_UTILS_H
