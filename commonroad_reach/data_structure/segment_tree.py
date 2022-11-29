from enum import Enum, auto
from typing import List, Optional


class TreeNodeStatus(Enum):
    """
    Possible states of nodes representing segments.

    - ACTIVE: segment fully activated (entire [low, high])
    - PARTIAL: segment partially activated (subset of [low, high])
    - NONACTIVE: segment not activated (none within [low, high])
    """
    ACTIVE = auto()
    PARTIAL = auto()
    NONACTIVE = auto()


class CounterTreeNode:
    """
    Each node represents a segment on the axis.

    Each segment is defined with an interval [low, high]. The counter represents the number of times it has
    been activated. Its potential left and right children have intervals [low, mid] and [mid, high], respectively.
    """

    def __init__(self, low: int, high: int):
        if low > high:
            raise Exception("<CounterTreeNode> Low is greater than high.")

        self.low = low
        self.high = high
        self.mid = (low + high) // 2
        self.count = 0
        self.status = TreeNodeStatus.NONACTIVE
        self.child_left: Optional[CounterTreeNode] = None
        self.child_right: Optional[CounterTreeNode] = None

    def __repr__(self) -> str:
        return f"TreeNode(low={self.low}, high={self.high}, status={self.status})"

    def __eq__(self, other: object) -> bool:
        if isinstance(other, CounterTreeNode):
            if self.low == other.low and self.high == other.high:
                return True

        return False

    def create_left_child(self):
        return CounterTreeNode(self.low, self.mid)

    def create_right_child(self):
        return CounterTreeNode(self.mid, self.high)

    def activate(self):
        self.count += 1

    def deactivate(self):
        self.count -= 1
        if self.count < 0:
            raise Exception("<CounterTreeNode> Node has negative count.")

    def update_status(self):
        if self.count or self.all_children_active():
            self.status = TreeNodeStatus.ACTIVE

        elif self.all_children_non_active():
            self.status = TreeNodeStatus.NONACTIVE

        else:
            self.status = TreeNodeStatus.PARTIAL

    def all_children_active(self) -> bool:
        return (self.child_left and self.child_left.status == TreeNodeStatus.ACTIVE) and (
                self.child_right and self.child_right.status == TreeNodeStatus.ACTIVE)

    def all_children_non_active(self) -> bool:
        return (not self.child_left or self.child_left.status == TreeNodeStatus.NONACTIVE) and (
                not self.child_right or self.child_right.status == TreeNodeStatus.NONACTIVE)

    def enclosed_by_interval(self, low: int, high: int) -> bool:
        """
        Returns whether the node is fully enclosed by the interval [low, high].
        """
        return low <= self.low and self.high <= high


class CounterSegmentTree:
    """
    Segment tree to hold nodes (segments) of intervals.

    Each leaf in this tree has a counter of times it has been activated.
    """

    def __init__(self, low: int, high: int):
        if low > high:
            raise Exception("<CounterSegmentTree> Low is greater than high.")

        self.node_root = CounterTreeNode(low, high)

    def activate(self, low: int, high: int):
        """
        Activates the root node for the input interval [low, high].
        """
        if low > high:
            raise Exception("<CounterSegmentTree> Low is greater than high.")

        self._activate_node(low, high, self.node_root)

    def _activate_node(self, low: int, high: int, node: CounterTreeNode):
        """
        Activates a node.

        When activating a node with an interval [low, high], if the interval fully encloses the node, the node's counter
        gets incremented by 1; if it does not fully enclose the node, child nodes are created (if needed),
        whose counter will be incremented.
        """
        if node.enclosed_by_interval(low, high):
            node.activate()

        else:
            self._create_and_activate_children(low, high, node)

        node.update_status()

    def _create_and_activate_children(self, low: int, high: int, node: CounterTreeNode):
        """
        Creates (if necessary) and activates the child nodes.
        """
        if low < node.mid:
            if not node.child_left:
                node.child_left = node.create_left_child()

            self._activate_node(low, high, node.child_left)

        if node.mid < high:
            if not node.child_right:
                node.child_right = node.create_right_child()

            self._activate_node(low, high, node.child_right)

    def deactivate(self, low: int, high: int):
        """
        Deactivates the root node for the input interval [low, high].
        """
        self._deactivate_node(low, high, self.node_root)

    def _deactivate_node(self, low: int, high: int, node: CounterTreeNode):
        """
        Deactivates a node.

        Deactivation is the opposite operation of activation.
        """
        if node.enclosed_by_interval(low, high):
            node.deactivate()

        else:
            self._deactivate_children(low, high, node)

        node.update_status()

    def _deactivate_children(self, low: int, high: int, node: CounterTreeNode):
        if low < node.mid and node.child_left:
            self._deactivate_node(low, high, node.child_left)

        if node.mid < high and node.child_right:
            self._deactivate_node(low, high, node.child_right)

    def get_active_nodes(self):
        """
        Returns the list of nodes whose status is ACTIVE.
        """
        return self._get_active_nodes(self.node_root)

    def _get_active_nodes(self, node: CounterTreeNode):
        """
        Returns a list of active nodes.
        """
        if node.status == TreeNodeStatus.ACTIVE:
            return [node]

        elif node.status == TreeNodeStatus.NONACTIVE:
            return []

        else:
            list_nodes_active_left = list()
            list_nodes_active_right = list()

            if node.child_left:
                list_nodes_active_left = self._get_active_nodes(node.child_left)

            if node.child_right:
                list_nodes_active_right = self._get_active_nodes(node.child_right)

            return list_nodes_active_left + list_nodes_active_right

    def get_stack_of_active_intervals(self):
        """
        Returns the stack of currently active intervals.

        .. note::
            Stack is in the form of [low1, high1, low2, high2, ..]. When pushing into the stack, intervals are
            merged where applicable (e.g. low2 == high1).

            **Example:**

            - [5, 10] => push TreeNode(low = 10, high = 15) => [5 ,15]
        """
        stack_intervals = []
        list_nodes_active = self.get_active_nodes()

        for node in list_nodes_active:
            # the node to be pushed has a low value as the last value in stack
            if stack_intervals and stack_intervals[-1] == node.low:
                stack_intervals.pop()

            else:
                stack_intervals.append(node.low)

            stack_intervals.append(node.high)

        return stack_intervals

    def print_active_nodes(self):
        def visit_node(node: CounterTreeNode):
            if node.status == TreeNodeStatus.ACTIVE:
                print(f"Active Node: [{node.low}, {node.high}]")

            else:
                if node.child_left:
                    visit_node(node.child_left)

                if node.child_right:
                    visit_node(node.child_right)

        visit_node(self.node_root)

    def get_non_active_intervals(self, low: int, high: int):
        """
        Returns the non-active intervals starting from the root node.
        """
        stack_intervals = []
        self._get_non_active_intervals(low, high, self.node_root, stack_intervals)

        return stack_intervals

    def _get_non_active_intervals(self, low: int, high: int, node: CounterTreeNode, stack: List):
        """
        Returns the complement interval in the specified range.

        A query returns intervals within [low, high] that have no active
        (incremented) segment.

        Example:
            tree = SegmentTree(0, 20) // tree ranges from 0 to 20

            tree.get_non_active_intervals(0, 10) => stack = [0, 10]

            tree.increment(0, 10) // segments within [0, 10] gets incremented
            tree.get_non_active_intervals(0, 20) => stack = [10, 20]

            tree.increment(5, 15) // segments within [5, 15] gets incremented
            tree.get_non_active_intervals(0, 18) => stack = [15, 18]

            tree.decrement(0, 10) // segments within [0, 10] gets decremented
            tree.get_non_active_intervals(0, 18) => stack = [0, 5, 15, 18]

        This is done by searching the non-active & fully enclosed nodes, and the
        not yet existing child nodes. Combined with sweep line algorithm, this
        function is used to obtain the vertical segments of merged rectangles:
        This function returns the non-active segment upon an event, which is
        essentially the desired vertical segment.
        """
        if node.status == TreeNodeStatus.ACTIVE:
            return

        elif node.status == TreeNodeStatus.NONACTIVE and node.enclosed_by_interval(low, high):
            self.add_node_interval_to_stack(node.low, node.high, stack)

        else:
            self._get_non_active_intervals_in_children(low, high, node, stack)

    def _get_non_active_intervals_in_children(self, low: int, high: int, node: CounterTreeNode, stack: List):
        """
        Stacks the complement interval from the child nodes.
        """
        if low < node.mid:
            if node.child_left:
                self._get_non_active_intervals(low, high, node.child_left, stack)

            else:
                # child not defined, thus it is definitely empty
                low_interval = max(low, node.low)
                high_interval = min(node.mid, high)
                self.add_node_interval_to_stack(low_interval, high_interval, stack)

        if node.mid < high:
            if node.child_right:
                self._get_non_active_intervals(low, high, node.child_right, stack)

            else:
                # child not defined, thus it is definitely empty
                low_interval = max(low, node.mid)
                high_interval = min(node.high, high)
                self.add_node_interval_to_stack(low_interval, high_interval, stack)

    @staticmethod
    def add_node_interval_to_stack(low: int, high: int, stack: List):
        """
        Updates the stack containing the complement intervals.

        .. note::
            The intervals are merged.

            **Example:**

                #. stack = []
                #. called with (5, 10) -> stack = [5, 10]
                #. called with (10, 15) -> stack = [5, 15]
                #. called with (18, 20) -> stack = [5, 15, 18, 20]
        """
        if stack and stack[-1] == low:
            # if new low equals last high, pop last high and don't push new low
            stack.pop()

        else:
            stack.append(low)

        stack.append(high)


class ToggleTreeNode:
    """
    Each node represents a segment on the axis.

    Each segment is defined with an interval [low, high]. The node only has a state, which is not based on
    counters. Its potential left and right children have intervals [low, mid] and [mid, high], respectively.
    """

    def __init__(self, low: int, high: int, status=TreeNodeStatus.NONACTIVE):
        self.low = low
        self.high = high
        self.mid = (low + high) // 2
        self.status = status
        self.child_left: Optional[ToggleTreeNode] = None
        self.child_right: Optional[ToggleTreeNode] = None

    def __repr__(self) -> str:
        return f"TreeNode(low={self.low}, high={self.high}, status={self.status})"

    def __eq__(self, other: object) -> bool:
        if isinstance(other, ToggleTreeNode):
            if self.low == other.low and self.high == other.high:
                return True

        return False

    def create_left_child(self, status):
        return ToggleTreeNode(self.low, self.mid, status)

    def create_right_child(self, status):
        return ToggleTreeNode(self.mid, self.high, status)

    def toggle(self):
        """
        Toggles the state of the node.
        """
        if self.status == TreeNodeStatus.ACTIVE:
            self.status = TreeNodeStatus.NONACTIVE

        elif self.status == TreeNodeStatus.NONACTIVE:
            self.status = TreeNodeStatus.ACTIVE

        if self.child_left:
            self.child_left.toggle()

        if self.child_right:
            self.child_right.toggle()

    def update_status(self):
        if self.status == TreeNodeStatus.ACTIVE or self.all_children_active():
            self.status = TreeNodeStatus.ACTIVE

        elif self.all_children_non_active():
            self.status = TreeNodeStatus.NONACTIVE

        else:
            self.status = TreeNodeStatus.PARTIAL

    def all_children_non_active(self) -> bool:
        return (not self.child_left or self.child_left.status == TreeNodeStatus.NONACTIVE) and (
                not self.child_right or self.child_right.status == TreeNodeStatus.NONACTIVE)

    def all_children_active(self) -> bool:
        return (self.child_left and self.child_left.status == TreeNodeStatus.ACTIVE) and (
                self.child_right and self.child_right.status == TreeNodeStatus.ACTIVE)

    def enclosed_by_interval(self, low: int, high: int) -> bool:
        """
        Returns whether the node is fully enclosed by the interval [low, high].
        """
        return low <= self.low and self.high <= high


class ToggleSegmentTree:
    """
    Segment tree to hold nodes (segments) of intervals.

    The states of leaves in this tree can be toggled.
    """

    def __init__(self, low: int, high: int):
        self.node_root = ToggleTreeNode(low, high)

    def toggle(self, low: int, high: int):
        """
        toggles the root node.
        """
        self._toggle_node(low, high, self.node_root)

    def _toggle_node(self, low: int, high, node: ToggleTreeNode):
        if node.enclosed_by_interval(low, high):
            node.toggle()

        else:
            self._create_and_toggle_children(low, high, node)

        node.update_status()

    def _create_and_toggle_children(self, low: int, high: int, node: ToggleTreeNode):
        """
        Creates (if necessary) and toggles the child nodes.
        """
        if low >= node.high or high <= node.low:
            return

        else:
            if not node.child_left:
                node.child_left = node.create_left_child(node.status)

            self._toggle_node(low, high, node.child_left)

            if not node.child_right:
                node.child_right = node.create_right_child(node.status)

            self._toggle_node(low, high, node.child_right)

            if node.status == TreeNodeStatus.ACTIVE:
                node.status = TreeNodeStatus.NONACTIVE

    def get_active_nodes(self):
        """
        Returns the list of nodes whose status is ACTIVE.
        """
        return self._get_active_nodes(self.node_root)

    def _get_active_nodes(self, node: ToggleTreeNode):
        """
        Returns a list of active nodes.
        """
        if node.status == TreeNodeStatus.ACTIVE:
            return [node]

        elif node.status == TreeNodeStatus.NONACTIVE:
            return []

        else:
            list_nodes_active_left = list()
            list_nodes_full_right = list()

            if node.child_left:
                list_nodes_active_left = self._get_active_nodes(node.child_left)

            if node.child_right:
                list_nodes_full_right = self._get_active_nodes(node.child_right)

            return list_nodes_active_left + list_nodes_full_right

    def get_stack_of_active_intervals(self):
        """
        Returns the stack of currently active intervals.

        .. note::
            Stack is in the form of [low1, high1, low2, high2, ..]. When pushing into the stack, intervals are
            merged where applicable (e.g. low2 == high1).

            **Example:**

            - [5, 10] => push TreeNode(low = 10, high = 15) => [5 ,15]
        """
        stack_intervals = []
        list_nodes_active = self.get_active_nodes()

        for node in list_nodes_active:
            # the node to be pushed has a low value as the last value in stack
            if stack_intervals and stack_intervals[-1] == node.low:
                stack_intervals.pop()

            else:
                stack_intervals.append(node.low)

            stack_intervals.append(node.high)

        return stack_intervals

    def print_active_nodes(self):
        def visit_node(node: ToggleTreeNode):
            if node.status == TreeNodeStatus.ACTIVE:
                print(f"Active Node: [{node.low}, {node.high}]")

            else:
                if node.child_left:
                    visit_node(node.child_left)

                if node.child_right:
                    visit_node(node.child_right)

        visit_node(self.node_root)
