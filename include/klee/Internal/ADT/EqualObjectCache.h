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
// Created by dfremont on 8/7/17.
//

// Stores unique representatives of objects related by ==.
// The representatives are permanent, so objects cannot be removed
// but pointers to elements are always valid.

#ifndef KLEE_EQUALOBJECTCACHE_H
#define KLEE_EQUALOBJECTCACHE_H

#include <unordered_set>

namespace klee {
  template< typename T, typename ObjectHash = std::hash<T> >
  class EqualObjectCache {
  private:
    // The container used here must have the property that a reference to one
    // of its elements remains valid as long as the element is not removed.
    using Container = std::unordered_set<T, ObjectHash>;
    Container objects;

  public:
    EqualObjectCache() {}

    const T *insert(const T &o) {
      std::pair<typename Container::iterator, bool> res = objects.insert(o);
      return &(*(res.first));
    }

    const T *insert(T &&o) {
      std::pair<typename Container::iterator, bool> res = objects.insert(o);
      return &(*(res.first));
    }

    template<class... Args>
    const T *emplace(Args&&... args) {
      std::pair<typename Container::iterator, bool> res = objects.emplace(std::forward<Args>(args)...);
      return &(*(res.first));
    }

    const T *find(const T &o) const {
      typename Container::const_iterator it = objects.find(o);
      if (it == objects.cend())
        return 0;
      else
        return &(*it);
    }
  };
};

#endif //KLEE_EQUALOBJECTCACHE_H
