//
// Created by dfremont on 7/14/17.
//

#ifndef KLEE_REVERSETRIE_H
#define KLEE_REVERSETRIE_H

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>

namespace klee {
  template<typename ValueType>
  class ReverseTrie {
  private:
    class Node {
    private:
      mutable unsigned refCount;

    public:
      const ValueType value;
      const Node *parent;

      Node(ValueType value, const Node *parent) : refCount(0), value(value), parent(parent) {}
      explicit Node(const Node *parent) : Node(ValueType(), parent) {}

      // The following reference-counting mechanism must only be used for Nodes
      // allocated individually with new (except the root, which is never freed)
      void claim() const {
        ++refCount;
      }

      void release() const {
        const Node *node = this;
        // avoiding recursion here so that deep tries don't cause stack overflows
        while (node) {
          const Node *parent = node->parent;
          if (--node->refCount == 0)
            delete node;
          else
            break;
          node = parent;
        }
      }
    };

  public:
    class Path;
    class Element {
      friend Path;
      friend ReverseTrie;

    private:
      const Node *node;
      unsigned depth;

      Element(const Node *node, unsigned depth) : node(node), depth(depth) {
        node->claim();
      }

    public:
      Element(const Element &e) : Element(e.node, e.depth) {}

      Element(Element &&e) {
        node = e.node;
        depth = e.depth;
        node->claim();    // the destructor for e will release the node
      }

      Element &operator=(Element &&e) {
        node->release();
        node = e.node;
        depth = e.depth;
        node->claim();    // as above
        return *this;
      }

      const ValueType& head() const {
        return node->value;
      }

      unsigned size() const {
        return depth;
      }

      void push_back(ValueType value) {
        // reference to old node inherited by new node
        node = new const Node(value, node);
        node->claim();   // reference to new node
        depth++;
      }

      void extendAlong(const Path &path) {
        assert(path.size() > depth && "extended ReverseTrie Element along too short Path!");
        const Node *next = path.nodes[depth];
        assert(node != next && "extended ReverseTrie Element to same node!");
        node->release();
        node = next;
        node->claim();
        depth++;
      }

      void dump() const {
        const Node *s = node;
        for (; s->parent ; s = s->parent) fprintf(stderr, "(%u,%u) -> ", (unsigned) s->value, s->refCount);
        fprintf(stderr, "[%" PRIxPTR "]\n", (uintptr_t) s);
      }

      ~Element() {
        node->release();
      }
    };

    class Path {
      friend Element;
    private:
      const Node *root;
      const Node **nodes;
      unsigned length;

    public:
      Path() : nodes(0), length(0) {}

      Path(const Element &e) {
        const Node *curNode = e.node;
        length = e.depth;
        unsigned curDepth = length;
        nodes = new const Node*[curDepth];
        while (curDepth > 0) {
          assert(curNode && "corrupted ReverseTrie element!");
          nodes[--curDepth] = curNode;
          curNode = curNode->parent;
        }
        assert(curNode->parent == 0 && "corrupted ReverseTrie element!");
        root = curNode;
      }

      Path(const Path &p) = delete;   // cannot be copied

      Path(Path &&p) {
        root = p.root;
        nodes = p.nodes;
        p.nodes = nullptr;    // so ~Path does not delete nodes
        length = p.length;
      }

      Path &operator=(Path &&p) {
        if (nodes)
          delete[] nodes;
        root = p.root;
        nodes = p.nodes;
        p.nodes = nullptr;    // as above
        length = p.length;
        return *this;
      }

      const ValueType &operator[](unsigned index) const {
        return nodes[index]->value;
      }

      unsigned size() const {
        return length;
      }

      Element prefix(unsigned depth) const {
        assert(depth <= length && "prefix cannot be longer than Path!");
        if (depth == 0) {
          return Element(root, 0);
        } else {
          return Element(nodes[depth-1], depth);
        }
      }

      ~Path() {
        if (nodes)
          delete[] nodes;
      }
    };

  private:
    const Node root;

  public:
    ReverseTrie() : root(0) {
      root.claim();    // root node is never deleted
    };

    Element getRoot() const {
      return Element(&root, 0);
    }
  };
};

#endif //KLEE_REVERSETRIE_H
