#include "reachset/data_structure/segment_tree.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

CounterTreeNode::CounterTreeNode(double const& low, double const& high) :
        low(low), high(high), mid((low + high) / 2), counter(0), status(TreeNodeStatus::NONACTIVE), child_left(nullptr),
        child_right(nullptr) {
    if (low > high)
        throw std::logic_error("<CounterTreeNode> Low is greater than high.");
}

bool CounterTreeNode::operator==(CounterTreeNode const& other) const {
    if (this->low == other.low and this->high == other.high)
        return true;

    return false;
}

void CounterTreeNode::create_left_child() {
    child_left = make_shared<CounterTreeNode>(low, mid);
}

void CounterTreeNode::create_right_child() {
    child_right = make_shared<CounterTreeNode>(mid, high);
}

void CounterTreeNode::activate() {
    counter++;
}

void CounterTreeNode::deactivate() {
    counter--;
    if (counter < 0)
        throw std::logic_error("<CounterTreeNode> Node has negative counter.");
}

void CounterTreeNode::update_status() {
    if (counter or all_children_are_active())
        status = TreeNodeStatus::ACTIVE;
    else if (all_children_are_non_active())
        status = TreeNodeStatus::NONACTIVE;
    else
        status = TreeNodeStatus::PARTIAL;
}

bool CounterTreeNode::all_children_are_active() const {
    return (child_left != nullptr and child_left->status == TreeNodeStatus::ACTIVE) and
           (child_right != nullptr and child_right->status == TreeNodeStatus::ACTIVE);
}

bool CounterTreeNode::all_children_are_non_active() const {
    return (child_left == nullptr or child_left->status == TreeNodeStatus::NONACTIVE) and
           (child_right == nullptr or child_right->status == TreeNodeStatus::NONACTIVE);
}

bool CounterTreeNode::enclosed_by_interval(double const& low, double const& high) const {
    return low <= this->low and this->high <= high;
}

CounterSegmentTree::CounterSegmentTree(double const& low, double const& high) {
    if (low > high)
        throw std::logic_error("<CounterSegmentTree> Low is greater than high.");

    node_root = make_shared<CounterTreeNode>(low, high);
}

void CounterSegmentTree::activate(double const& low, double const& high) {
    if (low > high)
        throw std::logic_error("<CounterSegmentTree> Low is greater than high.");

    activate_node(low, high, node_root);
}

void CounterSegmentTree::activate_node(double const& low, double const& high, CounterTreeNodePtr const& node) {
    if (node->enclosed_by_interval(low, high))
        node->activate();
    else
        create_and_activate_children(low, high, node);

    node->update_status();
}

void CounterSegmentTree::create_and_activate_children(double const& low, double const& high,
                                                      CounterTreeNodePtr const& node) {
    if (low < node->mid) {
        if (node->child_left == nullptr)
            node->create_left_child();
        activate_node(low, high, node->child_left);
    }

    if (node->mid < high) {
        if (node->child_right == nullptr)
            node->create_right_child();
        activate_node(low, high, node->child_right);
    }

}

void CounterSegmentTree::deactivate(double const& low, double const& high) {
    deactivate_node(low, high, node_root);
}

void CounterSegmentTree::deactivate_node(double const& low, double const& high, CounterTreeNodePtr const& node) {
    if (node->enclosed_by_interval(low, high))
        node->deactivate();
    else
        deactivate_children(low, high, node);

    node->update_status();
}

void CounterSegmentTree::deactivate_children(double const& low, double const& high, CounterTreeNodePtr const& node) {
    if (low < node->mid and node->child_left)
        deactivate_node(low, high, node->child_left);

    if (node->mid < high and node->child_right)
        deactivate_node(low, high, node->child_right);
}

vector<CounterTreeNodePtr> CounterSegmentTree::get_active_nodes() const {
    return get_active_nodes(node_root);
}

vector<CounterTreeNodePtr>
CounterSegmentTree::get_active_nodes(CounterTreeNodePtr const& node) const {
    if (node->status == TreeNodeStatus::ACTIVE)
        return {node};

    else if (node->status == TreeNodeStatus::NONACTIVE)
        return {};

    else {
        vector<CounterTreeNodePtr> vec_nodes_active_left, vec_nodes_active_right;

        if (node->child_left != nullptr)
            vec_nodes_active_left = get_active_nodes(node->child_left);

        if (node->child_right != nullptr)
            vec_nodes_active_right = get_active_nodes(node->child_right);

        vec_nodes_active_left.insert(vec_nodes_active_left.end(),
                                     std::make_move_iterator(vec_nodes_active_right.begin()),
                                     std::make_move_iterator(vec_nodes_active_right.end()));

        return vec_nodes_active_left;
    }
}

stack<double> CounterSegmentTree::get_stack_of_active_intervals() const {
    stack<double> stack_intervals;
    auto vec_nodes_active = get_active_nodes();

    for (auto const& node: vec_nodes_active) {
        if (not stack_intervals.empty() and stack_intervals.top() == node->low) {
            // the node tot be pushed has a low value as the last value in the stack
            stack_intervals.pop();
        } else stack_intervals.emplace(node->low);

        stack_intervals.emplace(node->high);
    }

    return stack_intervals;
}

void CounterSegmentTree::print_active_nodes() const {
    visit_node(node_root);
}

void CounterSegmentTree::visit_node(CounterTreeNodePtr const& node) const {
    if (node->status == TreeNodeStatus::ACTIVE)
        cout << "Active Node: [" << node->low << ", " << node->high << "]" << endl;
    else {
        if (node->child_left != nullptr)
            visit_node(node->child_left);

        if (node->child_right != nullptr)
            visit_node(node->child_right);
    }
}

stack<double> CounterSegmentTree::get_stack_of_non_active_intervals(double const& low, double const& high) const {
    stack<double> stack_intervals;
    get_stack_of_non_active_intervals(low, high, node_root, stack_intervals);

    return stack_intervals;
}

void CounterSegmentTree::get_stack_of_non_active_intervals(double const& low, double const& high,
                                                           CounterTreeNodePtr const& node, stack<double>& stack) const {
    if (node->status == TreeNodeStatus::ACTIVE) return;

    else if (node->status == TreeNodeStatus::NONACTIVE and node->enclosed_by_interval(low, high))
        add_node_interval_to_stack(node->low, node->high, stack);

    else
        non_active_intervals_in_children(low, high, node, stack);

}

void CounterSegmentTree::add_node_interval_to_stack(double const& low, double const& high, stack<double>& stack) {
    if (not stack.empty() and stack.top() == low)
        stack.pop();
    else
        stack.emplace(low);

    stack.emplace(high);
}

void CounterSegmentTree::non_active_intervals_in_children(double const& low, double const& high,
                                                          CounterTreeNodePtr const& node,
                                                          stack<double>& stack) const {
    if (low < node->mid) {
        if (node->child_left != nullptr) {
            get_stack_of_non_active_intervals(low, high, node->child_left, stack);
        } else {
            // child not defined, so it is definitely empty
            auto low_interval = std::max(low, node->low);
            auto high_interval = std::min(node->mid, high);
            add_node_interval_to_stack(low_interval, high_interval, stack);
        }

    }

    if (node->mid < high) {
        if (node->child_right != nullptr) {
            get_stack_of_non_active_intervals(low, high, node->child_right, stack);
        } else {
            // child not defined, so it is definitely empty
            auto low_interval = std::max(low, node->mid);
            auto high_interval = std::min(node->high, high);
            add_node_interval_to_stack(low_interval, high_interval, stack);
        }
    }

}


ToggleTreeNode::ToggleTreeNode(double const& low, double const& high, TreeNodeStatus const& status) :
        low(low), high(high), mid((low + high) / 2), status(status), child_left(nullptr), child_right(nullptr) {
    if (low > high)
        throw std::logic_error("<ToggleTreeNode> Low is greater than high.");
}

bool ToggleTreeNode::operator==(ToggleTreeNode const& other) const {
    if (this->low == other.low and this->high == other.high)
        return true;

    return false;
}

void ToggleTreeNode::create_left_child(TreeNodeStatus const& status) {
    child_left = make_shared<ToggleTreeNode>(low, mid, status);
}

void ToggleTreeNode::create_right_child(TreeNodeStatus const& status) {
    child_right = make_shared<ToggleTreeNode>(mid, high, status);
}

void ToggleTreeNode::toggle() {
    if (status == TreeNodeStatus::ACTIVE)
        status = TreeNodeStatus::NONACTIVE;
    else if (status == TreeNodeStatus::NONACTIVE)
        status = TreeNodeStatus::ACTIVE;

    if (child_left != nullptr)
        child_left->toggle();
    if (child_right != nullptr)
        child_right->toggle();
}

void ToggleTreeNode::update_status() {
    if (status == TreeNodeStatus::ACTIVE or all_children_are_active())
        status = TreeNodeStatus::ACTIVE;
    else if (all_children_are_non_active())
        status = TreeNodeStatus::NONACTIVE;
    else
        status = TreeNodeStatus::PARTIAL;
}

bool ToggleTreeNode::all_children_are_active() const {
    return (child_left != nullptr and child_left->status == TreeNodeStatus::ACTIVE) and
           (child_right != nullptr and child_right->status == TreeNodeStatus::ACTIVE);
}

bool ToggleTreeNode::all_children_are_non_active() const {
    return (child_left == nullptr or child_left->status == TreeNodeStatus::NONACTIVE) and
           (child_right == nullptr or child_right->status == TreeNodeStatus::NONACTIVE);
}

bool ToggleTreeNode::enclosed_by_interval(double const& low, double const& high) const {
    return low <= this->low and this->high <= high;
}

ToggleSegmentTree::ToggleSegmentTree(double const& low, double const& high) :
        node_root(make_shared<ToggleTreeNode>(low, high)) {}

void ToggleSegmentTree::toggle(double const& low, double const& high) {
    if (low > high)
        throw std::logic_error("<ToggleSegmentTree> Low is greater than high.");

    toggle_node(low, high, node_root);
}

void
ToggleSegmentTree::toggle_node(double const& low, double const& high, std::shared_ptr<ToggleTreeNode> const& node) {
    if (node->enclosed_by_interval(low, high))
        node->toggle();
    else
        create_and_toggle_children(low, high, node);

    node->update_status();
}

void ToggleSegmentTree::create_and_toggle_children(double const& low, double const& high,
                                                   std::shared_ptr<ToggleTreeNode> const& node) {
    if (low >= node->high or high <= node->low) return;
    else {
        if (node->child_left == nullptr)
            node->create_left_child(node->status);
        toggle_node(low, high, node->child_left);

        if (node->child_right == nullptr)
            node->create_right_child(node->status);
        toggle_node(low, high, node->child_right);

        if (node->status == TreeNodeStatus::ACTIVE)
            node->status = TreeNodeStatus::NONACTIVE;
    }

}

std::vector<std::shared_ptr<ToggleTreeNode>> ToggleSegmentTree::get_active_nodes() const {
    return get_active_nodes(node_root);
}

std::vector<std::shared_ptr<ToggleTreeNode>>
ToggleSegmentTree::get_active_nodes(std::shared_ptr<ToggleTreeNode> const& node) const {
    if (node->status == TreeNodeStatus::ACTIVE)
        return {node};

    else if (node->status == TreeNodeStatus::NONACTIVE)
        return {};

    else {
        vector<ToggleTreeNodePtr> vec_nodes_active_left, vec_nodes_active_right;

        if (node->child_left != nullptr)
            vec_nodes_active_left = get_active_nodes(node->child_left);

        if (node->child_right != nullptr)
            vec_nodes_active_right = get_active_nodes(node->child_right);

        vec_nodes_active_left.insert(vec_nodes_active_left.end(),
                                     std::make_move_iterator(vec_nodes_active_right.begin()),
                                     std::make_move_iterator(vec_nodes_active_right.end()));

        return vec_nodes_active_left;
    }
}

std::stack<double> ToggleSegmentTree::get_stack_of_active_intervals() const {
    stack<double> stack_intervals;
    auto vec_nodes_active = get_active_nodes();

    for (auto const& node: vec_nodes_active) {
        if (not stack_intervals.empty() and stack_intervals.top() == node->low)
            stack_intervals.pop();
        else
            stack_intervals.emplace(node->low);

        stack_intervals.emplace(node->high);
    }

    return stack_intervals;
}

void ToggleSegmentTree::print_active_nodes() const {
    visit_node(node_root);
}

void ToggleSegmentTree::visit_node(std::shared_ptr<ToggleTreeNode> const& node) const {
    if (node->status == TreeNodeStatus::ACTIVE)
        cout << "Active Node: [" << node->low << ", " << node->high << "]" << endl;
    else {
        if (node->child_left != nullptr)
            visit_node(node->child_left);

        if (node->child_right != nullptr)
            visit_node(node->child_right);
    }
}
