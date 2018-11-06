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
// Created by dfremont on 8/4/17.
//

// This class is needed because for some reason std::priority_queue
// does not support const iteration.

#ifndef KLEE_ITERABLEPRIORITYQUEUE_H
#define KLEE_ITERABLEPRIORITYQUEUE_H

namespace klee {
  template< typename ValueType, typename Compare = std::less<ValueType> >
  class IterablePriorityQueue {
  private:
    const Compare &comparator;
    using Container = std::vector<ValueType>;
    Container values;

  public:
    explicit IterablePriorityQueue(const Compare &comp = Compare()) : comparator(comp) {}

    void push(ValueType &&v) {
      values.push_back(std::forward<ValueType>(v));
      std::push_heap(values.begin(), values.end(), comparator);
    }

    ValueType pop() {
      std::pop_heap(values.begin(), values.end(), comparator);
      ValueType v = std::move(values.back());
      values.pop_back();
      return v;
    }

    void clear() {
      values.clear();
    }

    bool empty() const {
      return values.empty();
    }

    typename Container::size_type size() const {
      return values.size();
    }

    // In a bit of abuse of notation, we define the const iteration functions
    // but call them begin/end instead of cbegin/cend; this is necessary for
    // range-for to work
    typename Container::const_iterator begin() const {
      return values.cbegin();
    }

    typename Container::const_iterator end() const {
      return values.cend();
    }
  };
};

#endif //KLEE_ITERABLEPRIORITYQUEUE_H
