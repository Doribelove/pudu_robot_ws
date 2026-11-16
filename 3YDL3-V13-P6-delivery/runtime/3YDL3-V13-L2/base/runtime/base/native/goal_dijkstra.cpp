// Goal-stop CSR Dijkstra. Heap ordering follows SciPy 1.8.0's Fibonacci
// heap to preserve the existing equal-cost predecessor choice. The heap
// helpers are adapted from scipy/sparse/csgraph/_shortest_path.pyx, under
// the SciPy BSD license in SCIPY_LICENSE.txt. See THIRD_PARTY.md.
#include <cstdint>
#include <cmath>
#include <limits>
#include <new>
#include <vector>

enum State { SCANNED, NOT_IN_HEAP, IN_HEAP };
struct Node {
    unsigned int index, rank, source;
    State state;
    double val;
    Node *parent, *left, *right, *children;
};
struct Heap { Node *minimum; Node *ranks[100]; };

static Node *leftmost(Node *n) { while (n->left) n=n->left; return n; }
static Node *rightmost(Node *n) { while (n->right) n=n->right; return n; }
static void sibling(Node *n, Node *s) {
    Node *t=rightmost(n); t->right=s; s->left=t; s->right=nullptr;
    s->parent=n->parent; if (s->parent) ++s->parent->rank;
}
static void child(Node *n, Node *c) {
    c->parent=n;
    if (n->children) sibling(n->children,c);
    else { n->children=c; c->right=c->left=nullptr; n->rank=1; }
}
static void detach(Node *n) {
    if (n->parent) {
        --n->parent->rank;
        n->parent->children=n->left ? n->left : n->right;
    }
    if (n->left) n->left->right=n->right;
    if (n->right) n->right->left=n->left;
    n->left=n->right=n->parent=nullptr;
}
static void insert(Heap *h, Node *n) {
    if (h->minimum) {
        sibling(h->minimum,n);
        if (n->val<h->minimum->val) h->minimum=n;
    } else h->minimum=n;
}
static void decrease(Heap *h, Node *n, double value) {
    n->val=value;
    if (n->parent && n->parent->val>=value) { detach(n); insert(h,n); }
    else if (h->minimum->val>n->val) h->minimum=n;
}
static void link(Heap *h, Node *n) {
    if (!h->ranks[n->rank]) h->ranks[n->rank]=n;
    else {
        Node *other=h->ranks[n->rank]; h->ranks[n->rank]=nullptr;
        if (n->val<other->val || n==h->minimum) {
            detach(other); child(n,other); link(h,n);
        } else { detach(n); child(other,n); link(h,other); }
    }
}
static Node *pop(Heap *h) {
    Node *t;
    if (h->minimum->children) {
        t=leftmost(h->minimum->children);
        while (t) { Node *next=t->right; detach(t); sibling(h->minimum,t); t=next; }
        h->minimum->children=nullptr;
    }
    t=leftmost(h->minimum);
    if (t==h->minimum) {
        if (h->minimum->right) t=h->minimum->right;
        else { Node *out=h->minimum; h->minimum=nullptr; return out; }
    }
    Node *out=h->minimum; detach(out); h->minimum=t;
    for (auto &r:h->ranks) r=nullptr;
    while (t) {
        if (t->val<h->minimum->val) h->minimum=t;
        Node *next=t->right; link(h,t); t=next;
    }
    return out;
}

// Inputs are a trusted, sorted CSR compiled by FreeSpaceTopology; weight
// values are float32 as in r0, promoted individually to double. No fast-math.
// Returns 0=found, 1=unreachable, -1=allocation, -2=invalid endpoints.
extern "C" int goal_dijkstra(int n, const int32_t *indptr,
    const int32_t *indices, const float *weights, int src, int dst,
    int32_t *pred, int64_t *settled, double *distance) noexcept {
    *settled=0; *distance=std::numeric_limits<double>::infinity();
    if (n<=0 || src<0 || dst<0 || src>=n || dst>=n) return -2;
    try {
        std::vector<Node> nodes(n);
        for (int k=0;k<n;++k) {
            nodes[k]={static_cast<unsigned>(k),0,static_cast<unsigned>(-9999),
                      NOT_IN_HEAP,0.,nullptr,nullptr,nullptr,nullptr};
            pred[k]=-9999;
        }
        Heap heap{}; insert(&heap,&nodes[src]);
        while (heap.minimum) {
            Node *v=pop(&heap); v->state=SCANNED; ++*settled;
            if (v->index==static_cast<unsigned>(dst)) { *distance=v->val; return 0; }
            for (int j=indptr[v->index];j<indptr[v->index+1];++j) {
                Node *current=&nodes[indices[j]];
                if (current->state==SCANNED) continue;
                const double next=v->val+static_cast<double>(weights[j]);
                if (current->state==NOT_IN_HEAP) {
                    current->state=IN_HEAP; current->val=next; insert(&heap,current);
                    pred[indices[j]]=v->index;
                } else if (current->val>next) {
                    decrease(&heap,current,next); pred[indices[j]]=v->index;
                }
            }
        }
        return 1;
    } catch (const std::bad_alloc &) { return -1; }
    catch (...) { return -2; }
}
