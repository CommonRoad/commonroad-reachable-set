#pragma once

#include "reachset/utility/shared_include.hpp"

namespace reach {
/// Possible states of nodes (segments).
enum class TreeNodeStatus {
    /// segment fully activated (all within [low, high])
    ACTIVE,
    /// segment partially activated (subset of [low, high])
    PARTIAL,
    /// segment not activated (none within [low, high])
    NONACTIVE
};

/// Each node (with active counters) represents a segment on the axis.
/// Each segment is defined with an interval [low, high]. The counter represents the number of times it has been
/// activated. Its potential left and right children have intervals [low, mid] and [mid, high], respectively.
struct CounterTreeNode {
    double low;
    double high;
    double mid;
    int counter;
    TreeNodeStatus status;
    std::shared_ptr<CounterTreeNode> child_left;
    std::shared_ptr<CounterTreeNode> child_right;

    /// Constructor of CounterTreeNode.
    /// @param low lower bound of the node
    /// @param high upper bound of the node
    CounterTreeNode(double const& low, double const& high);

    bool operator==(CounterTreeNode const& other) const;

    void create_left_child();

    void create_right_child();

    /// Increments the counter by 1.
    void activate();

    /// Decrements the counter by 1.
    void deactivate();

    /// Updates the status of the node.
    void update_status();

    /// Returns true if both the left and right children are active.
    bool all_children_are_active() const;

    /// Returns true if both the left and right children are non-active.
    bool all_children_are_non_active() const;

    /// Returns true if the node is fully enclosed in [low, high].
    bool enclosed_by_interval(double const& low, double const& high) const;
};

/// Segment tree to hold nodes (segments) of intervals, with nodes being CounterTreeNode.
/// Each leaf in this tree has a counter of how many times it has been activated.
class CounterSegmentTree {
private:
    std::shared_ptr<CounterTreeNode> node_root;

    /// Activates a node (segment).
    /// When activating a node with an interval [low, high], if it fully covers the node, the node's counter gets
    /// incremented by 1; if it does not fully covers the node, the child nodes are created (if needed), whose counter
    /// will be incremented.
    void activate_node(double const& low, double const& high, std::shared_ptr<CounterTreeNode> const& node);

    /// Creates (if necessary) and activates the child nodes.
    void create_and_activate_children(
            double const& low, double const& high, std::shared_ptr<CounterTreeNode> const& node);

    /// Deactivation is the opposite operation of activation.
    void deactivate_node(double const& low, double const& high, std::shared_ptr<CounterTreeNode> const& node);

    /// Deactivates the child nodes.
    void deactivate_children(double const& low, double const& high, std::shared_ptr<CounterTreeNode> const& node);

    /// Returns a list of active nodes.
    std::vector<std::shared_ptr<CounterTreeNode>> get_active_nodes(std::shared_ptr<CounterTreeNode> const& node) const;

public:
    CounterSegmentTree() = default;

    CounterSegmentTree(double const& low, double const& high);

    /// Activates the root node.
    void activate(double const& low, double const& high);

    /// Deactivates the root node.
    void deactivate(double const& low, double const& high);

    /// Returns the list of nodes whose status is ACTIVE.
    std::vector<std::shared_ptr<CounterTreeNode>> get_active_nodes() const;

    /// Returns the stack of currently active intervals.
    /// Stack is in the form of [low1, high1, low2, high2, ..]. When pushing into
    /// the stack, intervals are merged where application (e.g. low2 == high1).\n
    /// [5, 10] => push TreeNode(low = 10, high = 15) => [5 ,15]
    std::stack<double> get_stack_of_active_intervals() const;

    /// Prints active nodes.
    void print_active_nodes() const;

    void visit_node(std::shared_ptr<CounterTreeNode> const& node) const;

    /// Returns the non-active intervals starting from the root node.
    std::stack<double> get_stack_of_non_active_intervals(double const& low, double const& high) const;

    /// Returns the complement interval in the specified range.
    /// A query returns intervals within [low, high] that have no active (incremented) segment. This is done by
    /// searching the non-active & fully enclosed nodes, and the not-yet existing child nodes. Combined with sweep line
    /// algorithm, this function is used to obtain the vertical segments of merged rectangles: This function returns the
    /// non-active segment upon an event, which is essentially the desired vertical segment.
    ///
    /// *Example*: \n
    /// tree = CounterSegmentTree(0, 20) // tree ranges from 0 to 20\n\n
    /// tree.get_stack_of_non_active_intervals(0, 10) => stack = [0, 10]\n\n
    /// tree.activate(0, 10) // segments within [0, 10] gets incremented
    /// tree.get_stack_of_non_active_intervals(0, 20) => stack = [10, 20]\n\n
    /// tree.activate(5, 15) // segments within [5, 15] gets incremented
    /// tree.get_stack_of_non_active_intervals(0, 18) => stack = [15, 18]\n\n
    /// tree.deactivate(0, 10) // segments within [0, 10] gets decremented
    /// tree.get_stack_of_non_active_intervals(0, 18) => stack = [0, 5, 15, 18]
    void get_stack_of_non_active_intervals(double const& low, double const& high,
                                           std::shared_ptr<CounterTreeNode> const& node,
                                           std::stack<double>& stack) const;

    /// Updates the stack containing the complement intervals.
    /// The intervals are merged.
    ///
    /// *Example*:\n
    /// stack = []\n
    /// called with (5, 10) -> stack = [5, 10]\n
    /// called with (10, 15) -> stack = [5, 15]\n
    /// called with (18, 20) -> stack = [5, 15, 18, 20]
    static void add_node_interval_to_stack(double const& low, double const& high, std::stack<double>& stack);

    /// Returns the stacks of complement interval from the child nodes.
    void
    non_active_intervals_in_children(double const& low, double const& high,
                                     std::shared_ptr<CounterTreeNode> const& sharedPtr,
                                     std::stack<double>& stack) const;
};

/// Each node (with a state) represents a segment on the axis.
/// Each segment is defined with an interval [low, high]. The node only has a state, which is not based on counter.
/// Its potential left and right children have intervals [low, mid] and [mid, high], respectively.
struct ToggleTreeNode {
    double low;
    double high;
    double mid;
    TreeNodeStatus status;
    std::shared_ptr<ToggleTreeNode> child_left;
    std::shared_ptr<ToggleTreeNode> child_right;

    ToggleTreeNode(double const& low, double const& high, TreeNodeStatus const& status = TreeNodeStatus::NONACTIVE);

    bool operator==(ToggleTreeNode const& other) const;

    void create_left_child(TreeNodeStatus const& status);

    void create_right_child(TreeNodeStatus const& status);

    /// Toggles the state of the node.
    void toggle();

    /// Updates the status of the node.
    void update_status();

    /// Returns true if both the left and right children are active.
    bool all_children_are_active() const;

    /// Returns true if both the left and right children are non-active.
    bool all_children_are_non_active() const;

    /// Returns true if the node is fully enclosed in [low, high].
    bool enclosed_by_interval(double const& low, double const& high) const;

};

/// Segment tree to hold nodes (segments) of intervals.
/// The states of leaves in this tree can be toggled.
class ToggleSegmentTree {
private:
    std::shared_ptr<ToggleTreeNode> node_root;

    void toggle_node(double const& low, double const& high, std::shared_ptr<ToggleTreeNode> const& node);

    /// Creates (if necessary) and toggles the child nodes.
    void create_and_toggle_children(double const& low, double const& high, std::shared_ptr<ToggleTreeNode> const& node);

    /// Returns a list of active nodes.
    std::vector<std::shared_ptr<ToggleTreeNode>> get_active_nodes(std::shared_ptr<ToggleTreeNode> const& node) const;

public:
    ToggleSegmentTree() = default;

    ToggleSegmentTree(double const& low, double const& high);

    /// Toggles the status of a node (segment).
    void toggle(double const& low, double const& high);

    /// Returns the list of nodes whose status is ACTIVE.
    std::vector<std::shared_ptr<ToggleTreeNode>> get_active_nodes() const;

    /// Returns the stack of currently active intervals. Stack is in the form of [low1, high1, low2, high2, ..].
    /// When pushing into the stack, intervals are merged where application (e.g. low2 == high1).\n
    /// [5, 10] => push TreeNode2(low = 10, high = 15) => [5 ,15]
    std::stack<double> get_stack_of_active_intervals() const;

    /// Prints active nodes.
    void print_active_nodes() const;

    void visit_node(std::shared_ptr<ToggleTreeNode> const& node) const;
};

using CounterTreeNodePtr = std::shared_ptr<CounterTreeNode>;
using ToggleTreeNodePtr = std::shared_ptr<ToggleTreeNode>;
}
