#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

class KDTree {
public:
    explicit KDTree(const std::vector<Eigen::Vector3d>& points)
        : points_(points)
    {
        std::vector<int> indices(points.size());
        for (size_t i = 0; i < points.size(); ++i)
            indices[i] = static_cast<int>(i);
        root_ = build(indices, 0);
    }

    int nearest(const Eigen::Vector3d& query) const {
        int best_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        search(root_.get(), query, best_idx, best_dist);
        return best_idx;
    }

    std::vector<int> k_nearest(const Eigen::Vector3d& query, int k) const {
        // Max-heap of (distance, index) — top element is the farthest neighbor
        std::vector<std::pair<double, int>> heap;
        heap.reserve(k);
        search_k(root_.get(), query, k, heap);
        std::vector<int> result(heap.size());
        for (size_t i = 0; i < heap.size(); ++i)
            result[i] = heap[i].second;
        return result;
    }

private:
    struct Node {
        int index;
        int axis;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    const std::vector<Eigen::Vector3d>& points_;
    std::unique_ptr<Node> root_;

    std::unique_ptr<Node> build(std::vector<int>& indices, int depth) {
        if (indices.empty()) return nullptr;

        int axis = depth % 3;
        std::sort(indices.begin(), indices.end(),
                  [&](int a, int b) { return points_[a][axis] < points_[b][axis]; });

        size_t mid = indices.size() / 2;
        auto node = std::make_unique<Node>();
        node->index = indices[mid];
        node->axis = axis;

        std::vector<int> left(indices.begin(), indices.begin() + mid);
        std::vector<int> right(indices.begin() + mid + 1, indices.end());

        node->left = build(left, depth + 1);
        node->right = build(right, depth + 1);

        return node;
    }

    void search_k(const Node* node, const Eigen::Vector3d& query,
                  int k, std::vector<std::pair<double, int>>& heap) const
    {
        if (!node) return;

        double d = (points_[node->index] - query).squaredNorm();

        if (static_cast<int>(heap.size()) < k) {
            heap.push_back({d, node->index});
            std::push_heap(heap.begin(), heap.end());
        } else if (d < heap.front().first) {
            std::pop_heap(heap.begin(), heap.end());
            heap.back() = {d, node->index};
            std::push_heap(heap.begin(), heap.end());
        }

        double diff = query[node->axis] - points_[node->index][node->axis];
        const Node* first = diff < 0 ? node->left.get() : node->right.get();
        const Node* second = diff < 0 ? node->right.get() : node->left.get();

        search_k(first, query, k, heap);

        double worst = static_cast<int>(heap.size()) < k
                           ? std::numeric_limits<double>::max()
                           : heap.front().first;
        if (diff * diff < worst)
            search_k(second, query, k, heap);
    }

    void search(const Node* node, const Eigen::Vector3d& query,
                int& best_idx, double& best_dist) const
    {
        if (!node) return;

        double d = (points_[node->index] - query).squaredNorm();
        if (d < best_dist) {
            best_dist = d;
            best_idx = node->index;
        }

        double diff = query[node->axis] - points_[node->index][node->axis];
        const Node* first = diff < 0 ? node->left.get() : node->right.get();
        const Node* second = diff < 0 ? node->right.get() : node->left.get();

        search(first, query, best_idx, best_dist);

        // Only search other subtree if splitting plane is closer than current best
        if (diff * diff < best_dist)
            search(second, query, best_idx, best_dist);
    }
};
