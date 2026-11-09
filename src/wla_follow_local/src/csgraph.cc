// Port of scipy 1.11.4 scipy/sparse/csgraph/_shortest_path.pyx (directed
// Dijkstra with its Fibonacci heap) and of coo_matrix.tocsr(). The heap is
// copied operation for operation: which of several equal-cost paths becomes
// the predecessor depends on its pop order.
#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "numpy_ops.h"

namespace wla::follow::detail {

namespace {

enum class FibonacciState : std::uint8_t
{
    Scanned,
    NotInHeap,
    InHeap,
};

struct FibonacciNode
{
    unsigned int m_index{0};
    unsigned int m_rank{0};
    FibonacciState m_state{FibonacciState::NotInHeap};
    double m_val{0.};
    FibonacciNode* m_parent{nullptr};
    FibonacciNode* m_leftSibling{nullptr};
    FibonacciNode* m_rightSibling{nullptr};
    FibonacciNode* m_children{nullptr};
};

struct FibonacciHeap
{
    FibonacciNode* m_minNode{nullptr};
    std::array<FibonacciNode*, 100> m_rootsByRank{};
};

FibonacciNode* LeftmostSibling(FibonacciNode* node)
{
    FibonacciNode* temp = node;
    while (temp->m_leftSibling)
    {
        temp = temp->m_leftSibling;
    }
    return temp;
}

void AddSibling(FibonacciNode* node, FibonacciNode* new_sibling)
{
    if (node->m_rightSibling)
    {
        node->m_rightSibling->m_leftSibling = new_sibling;
    }
    new_sibling->m_rightSibling = node->m_rightSibling;
    new_sibling->m_leftSibling = node;
    node->m_rightSibling = new_sibling;
    new_sibling->m_parent = node->m_parent;
    if (new_sibling->m_parent)
    {
        new_sibling->m_parent->m_rank += 1;
    }
}

void AddChild(FibonacciNode* node, FibonacciNode* new_child)
{
    new_child->m_parent = node;
    if (node->m_children)
    {
        AddSibling(node->m_children, new_child);
    }
    else
    {
        node->m_children = new_child;
        new_child->m_rightSibling = nullptr;
        new_child->m_leftSibling = nullptr;
        node->m_rank = 1;
    }
}

void Remove(FibonacciNode* node)
{
    if (node->m_parent)
    {
        node->m_parent->m_rank -= 1;
        if (node->m_parent->m_children == node)
        {
            node->m_parent->m_children = node->m_rightSibling;
        }
    }
    if (node->m_leftSibling)
    {
        node->m_leftSibling->m_rightSibling = node->m_rightSibling;
    }
    if (node->m_rightSibling)
    {
        node->m_rightSibling->m_leftSibling = node->m_leftSibling;
    }
    node->m_leftSibling = nullptr;
    node->m_rightSibling = nullptr;
    node->m_parent = nullptr;
}

void InsertNode(FibonacciHeap& heap, FibonacciNode* node)
{
    if (heap.m_minNode)
    {
        if (node->m_val < heap.m_minNode->m_val)
        {
            node->m_leftSibling = nullptr;
            node->m_rightSibling = heap.m_minNode;
            heap.m_minNode->m_leftSibling = node;
            heap.m_minNode = node;
        }
        else
        {
            AddSibling(heap.m_minNode, node);
        }
    }
    else
    {
        heap.m_minNode = node;
    }
}

void DecreaseVal(FibonacciHeap& heap, FibonacciNode* node, double newval)
{
    node->m_val = newval;
    if (node->m_parent && (node->m_parent->m_val >= newval))
    {
        Remove(node);
        InsertNode(heap, node);
    }
    else if (heap.m_minNode->m_val > node->m_val)
    {
        Remove(node);
        node->m_rightSibling = heap.m_minNode;
        heap.m_minNode->m_leftSibling = node;
        heap.m_minNode = node;
    }
}

void Link(FibonacciHeap& heap, FibonacciNode* node)
{
    if (heap.m_rootsByRank[node->m_rank] == nullptr)
    {
        heap.m_rootsByRank[node->m_rank] = node;
    }
    else
    {
        FibonacciNode* linknode = heap.m_rootsByRank[node->m_rank];
        heap.m_rootsByRank[node->m_rank] = nullptr;
        if (node->m_val < linknode->m_val || node == heap.m_minNode)
        {
            Remove(linknode);
            AddChild(node, linknode);
            Link(heap, node);
        }
        else
        {
            Remove(node);
            AddChild(linknode, node);
            Link(heap, linknode);
        }
    }
}

FibonacciNode* RemoveMin(FibonacciHeap& heap)
{
    FibonacciNode* temp = heap.m_minNode->m_children;
    while (temp)
    {
        FibonacciNode* temp_right = temp->m_rightSibling;
        Remove(temp);
        AddSibling(heap.m_minNode, temp);
        temp = temp_right;
    }
    FibonacciNode* out = heap.m_minNode;
    temp = heap.m_minNode->m_rightSibling;
    Remove(heap.m_minNode);
    heap.m_minNode = temp;
    if (temp == nullptr)
    {
        return out;
    }
    heap.m_rootsByRank.fill(nullptr);
    while (temp)
    {
        if (temp->m_val < heap.m_minNode->m_val)
        {
            heap.m_minNode = temp;
        }
        FibonacciNode* temp_right = temp->m_rightSibling;
        Link(heap, temp);
        temp = temp_right;
    }
    temp = LeftmostSibling(heap.m_minNode);
    if (heap.m_minNode != temp)
    {
        Remove(heap.m_minNode);
        heap.m_minNode->m_rightSibling = temp;
        temp->m_leftSibling = heap.m_minNode;
    }
    return out;
}

} // namespace

Csr BuildCsr(int n, const std::vector<int>& src, const std::vector<int>& dst, const std::vector<double>& weights)
{
    if (src.size() != dst.size() || src.size() != weights.size() || n < 0)
    {
        throw std::invalid_argument("invalid_graph");
    }
    std::vector<std::size_t> order(src.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    for (std::size_t k = 0; k < src.size(); ++k)
    {
        if (src[k] < 0 || src[k] >= n || dst[k] < 0 || dst[k] >= n)
        {
            throw std::invalid_argument("invalid_graph_edge");
        }
    }
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return src[a] != src[b] ? src[a] < src[b] : dst[a] < dst[b];
    });
    Csr csr;
    csr.m_indptr.assign(static_cast<std::size_t>(n) + 1, 0);
    int last_src = -1, last_dst = -1;
    for (std::size_t k : order)
    {
        if (src[k] == last_src && dst[k] == last_dst)
        {
            csr.m_weights.back() += weights[k];  // sum_duplicates
            continue;
        }
        csr.m_indices.push_back(dst[k]);
        csr.m_weights.push_back(weights[k]);
        last_src = src[k];
        last_dst = dst[k];
        csr.m_indptr[static_cast<std::size_t>(src[k]) + 1] += 1;
    }
    for (std::size_t r = 0; r < static_cast<std::size_t>(n); ++r)
    {
        csr.m_indptr[r + 1] += csr.m_indptr[r];
    }
    return csr;
}

void Dijkstra(const Csr& graph, int source, std::vector<double>& distance, std::vector<int>& predecessor)
{
    const std::size_t n = graph.m_indptr.size() - 1;
    if (source < 0 || static_cast<std::size_t>(source) >= n)
    {
        throw std::invalid_argument("indices out of range 0...N");
    }
    distance.assign(n, std::numeric_limits<double>::infinity());
    predecessor.assign(n, -9999);
    std::vector<FibonacciNode> nodes(n);
    for (std::size_t k = 0; k < n; ++k)
    {
        nodes[k].m_index = static_cast<unsigned int>(k);
    }
    distance[static_cast<std::size_t>(source)] = 0.;
    FibonacciHeap heap;
    InsertNode(heap, &nodes[static_cast<std::size_t>(source)]);
    while (heap.m_minNode)
    {
        FibonacciNode* v = RemoveMin(heap);
        v->m_state = FibonacciState::Scanned;
        for (int j = graph.m_indptr[v->m_index]; j < graph.m_indptr[v->m_index + 1]; ++j)
        {
            const auto j_current = static_cast<std::size_t>(graph.m_indices[static_cast<std::size_t>(j)]);
            FibonacciNode* current = &nodes[j_current];
            if (current->m_state != FibonacciState::Scanned)
            {
                const double next_val = v->m_val + graph.m_weights[static_cast<std::size_t>(j)];
                if (next_val <= std::numeric_limits<double>::infinity())
                {
                    if (current->m_state == FibonacciState::NotInHeap)
                    {
                        current->m_state = FibonacciState::InHeap;
                        current->m_val = next_val;
                        InsertNode(heap, current);
                        predecessor[j_current] = static_cast<int>(v->m_index);
                    }
                    else if (current->m_val > next_val)
                    {
                        DecreaseVal(heap, current, next_val);
                        predecessor[j_current] = static_cast<int>(v->m_index);
                    }
                }
            }
        }
        distance[v->m_index] = v->m_val;
    }
}

} // namespace wla::follow::detail
