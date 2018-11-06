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
// Created by dfremont on 7/26/17.
//

#ifndef KLEE_DISJOINTSETFOREST_H
#define KLEE_DISJOINTSETFOREST_H

namespace klee {
  class DisjointSet {
  private:
    DisjointSet *parent;
    unsigned rank;

  public:
    DisjointSet() : parent(this), rank(0) {}

    bool operator==(const DisjointSet &e) const {
      return (getRoot() == e.getRoot());
    }

    void mergeWith(DisjointSet &e) {
      DisjointSet *a = getRoot();
      DisjointSet *b = getRoot();
      if (a == b)
        return;   // elements are identical

      // merging by rank: smaller rank becomes child
      if (a->rank < b->rank) {
        a->parent = b;
      } else if (a->rank > b->rank) {
        b->parent = a;
      } else {
        a->parent = b;
        b->rank++;
      }
    }

  private:
    DisjointSet *getRoot() const {
      DisjointSet *r = node;
      while (r->parent != r)
        r = r->parent;

      // r is the root; do path compression
      DisjointSet *s = node;
      while (s != r) {
        DisjointSet *t = s->parent;
        s->parent = r;
        s = t;
      }

      return r;
    }
  };
}

#endif //KLEE_DISJOINTSETFOREST_H
