from collections import defaultdict
from enum import Enum, auto
from functools import cmp_to_key
from typing import Dict, List, Tuple, Union

from commonroad_reach.data_structure.reach.reach_line import ReachLine
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.segment_tree import CounterSegmentTree, ToggleSegmentTree


class EventType(Enum):
    """
    Possible types of events in the sweep line algorithm.
    """

    ENTER = auto()
    EXIT = auto()


class Event:
    """
    Event in the sweep line algorithm.

    p_lon = x (CART) = s (CVLN), p_lat = y (CART) = d (CVLN)

    It is assumed that the line is swept from left to right. The left edge of a rectangle
    is typed 'ENTER', and the right edge 'EXIT'.
    """

    def __init__(self, type_vent, p_lon, p_lat_low, p_lat_high):
        self.type = type_vent
        self.p_lon = p_lon
        self.p_lat_low = p_lat_low
        self.p_lat_high = p_lat_high

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Event):
            if self.type == other.type and self.p_lon == other.p_lon and \
                    self.p_lat_low == other.p_lat_low and self.p_lat_high == other.p_lat_high:
                return True

        return False

    def __repr__(self) -> str:
        return f"Event(type={self.type}, p_lon={self.p_lon}, " \
               f"p_lat_low={self.p_lat_low}, p_lat_high={self.p_lat_high})"


class SweepLine:
    """
    Class performing sweep line algorithms.
    """
    tree: Union[CounterSegmentTree, ToggleSegmentTree]

    @classmethod
    def obtain_vertical_segments_from_rectangles(cls, list_rectangles: List[ReachPolygon], ) -> List[ReachLine]:
        """
        Returns a list of vertical segments from the input rectangles.

        Steps:
            1. create a segment tree with min/max lateral position of rectangles
            2. create events of line sweeping from rectangles: left edge = ENTER, right edge = EXIT
            3. create vertical segments with the events
        """
        if not list_rectangles:
            return []

        p_lat_min_rectangles, p_lat_max_rectangles = \
            cls.compute_extremum_lateral_positions_of_rectangles(list_rectangles)
        cls.tree = CounterSegmentTree(p_lat_min_rectangles, p_lat_max_rectangles)

        list_events = cls.create_event_list(list_rectangles)

        list_segments_vertical = cls.create_vertical_segments_from_events(list_events)

        return list_segments_vertical

    @staticmethod
    def compute_extremum_lateral_positions_of_rectangles(list_rectangles: List[ReachPolygon]) -> Tuple[int, int]:
        """
        Returns the minimum and maximum lateral positions of the given list of rectangles.
        """
        p_lat_min_rectangles = min([rectangle.p_lat_min for rectangle in list_rectangles])
        p_lat_max_rectangles = max([rectangle.p_lat_max for rectangle in list_rectangles])

        return p_lat_min_rectangles, p_lat_max_rectangles

    @classmethod
    def create_event_list(cls, list_rectangles: List[ReachPolygon]) -> List[Event]:
        """
        Creates a list of sorted events with the given list of rectangles.
        """
        list_events = []

        for rectangle in list_rectangles:
            list_events.append(Event(EventType.ENTER, rectangle.p_lon_min,
                                     rectangle.p_lat_min, rectangle.p_lat_max))
            list_events.append(Event(EventType.EXIT, rectangle.p_lon_max,
                                     rectangle.p_lat_min, rectangle.p_lat_max))

        list_events = cls.sort_events(list_events)

        return list_events

    @classmethod
    def sort_events(cls, list_events: List[Event]) -> List[Event]:
        return sorted(list_events, key=cmp_to_key(cls.compare_events))

    @classmethod
    def compare_events(cls, event1: Event, event2: Event):
        """
        Custom comparison function for events.

        Events are ordered in the following order:
            1. longitudinal position of rectangles
            2. type of the event
            3. lower lateral position
        """
        if event1.p_lon < event2.p_lon:
            return -1

        elif event1.p_lon > event2.p_lon:
            return 1

        if event1.type == EventType.ENTER and event2.type == EventType.EXIT:
            return -1

        elif event1.type == EventType.EXIT and event2.type == EventType.ENTER:
            return 1

        if event1.p_lat_low < event2.p_lat_low:
            return -1

        elif event1.p_lat_low > event2.p_lat_low:
            return 1

        return 0

    @classmethod
    def create_vertical_segments_from_events(cls, list_events: List[Event]) -> List[ReachLine]:
        """
        Creates a list of vertical segments from the list of events.
        """
        list_segments_vertical = []

        for event in list_events:
            if event.type == EventType.ENTER:
                list_segments_vertical += cls.create_vertical_segments_from_event(event)
                cls.tree.activate(event.p_lat_low, event.p_lat_high)

            else:
                cls.tree.deactivate(event.p_lat_low, event.p_lat_high)
                list_segments_vertical += cls.create_vertical_segments_from_event(event)

        return list_segments_vertical

    @classmethod
    def create_vertical_segments_from_event(cls, event: Event) -> List[ReachLine]:
        """
        Returns a list of vertical segments with the tree and event.

        For each event, query the tree to get the nonactive intervals, which is the desired vertical segment.
        """
        list_segments = []

        stack_intervals_lat = cls.tree.get_non_active_intervals(event.p_lat_low, event.p_lat_high)
        while stack_intervals_lat:
            p_lat_high = stack_intervals_lat.pop()
            p_lat_low = stack_intervals_lat.pop()
            p_lon = event.p_lon

            segment = ReachLine(p_lon, p_lat_low, p_lon, p_lat_high)
            list_segments.append(segment)

        return list_segments

    @classmethod
    def create_rectangles_from_vertical_segments(cls, list_segments: List[ReachLine]) -> List[ReachPolygon]:
        """
        Returns a list of rectangles from the given vertical segments.
            
        Step:
            1. Create a segment tree with the list of segments.
            2. Create a dictionary that maps p_lon to a list of rectangles whose left edge is aligned with p_lon.
            3. Merge rectangles that share the same coordinates of p_lat.
        """
        cls.tree = cls.create_tree_from_segments(list_segments)

        dict_p_lon_to_list_rectangles = cls.create_p_lon_to_rectangles_dictionary(list_segments)

        list_rectangles_final = cls.merge_rectangles_with_same_lateral_coordinates(dict_p_lon_to_list_rectangles)

        return list_rectangles_final

    @staticmethod
    def create_tree_from_segments(list_segments: List[ReachLine]) -> ToggleSegmentTree:
        """
        Creates a ToggleSegmentTree from the list of given segments.
        """
        p_lat_min_segments = min([segment.p_lat_min for segment in list_segments])
        p_lat_max_segments = max([segment.p_lat_max for segment in list_segments])

        tree = ToggleSegmentTree(int(p_lat_min_segments), int(p_lat_max_segments))

        return tree

    @classmethod
    def create_p_lon_to_rectangles_dictionary(cls, list_segments: List[ReachLine]) -> Dict[int, List[ReachPolygon]]:
        """
        Create a dictionary that maps p_lon to a list of rectangles whose left edge is aligned with p_lon.

        Steps:
            1. Create a dictionary that maps p_lon to list of tuples of p_lat from segments
            2. Iterate through p_lon, retrieve relevant tuples of p_lat and toggle the status of segments between
               these p_lat in the segment tree. Get intervals of active segments and create rectangles.
        """
        if not list_segments:
            return {}

        dict_p_lon_to_list_rectangles = defaultdict(list)

        dict_p_lon_to_list_tuples_p_lat = defaultdict(list)
        for segment in list_segments:
            p_lon_min, p_lat_min, _, p_lat_max = segment.bounds
            dict_p_lon_to_list_tuples_p_lat[p_lon_min].append((p_lat_min, p_lat_max))

        list_p_lon = list(dict_p_lon_to_list_tuples_p_lat.keys())
        for p_lon_min, p_lon_max in zip(list_p_lon[:-1], list_p_lon[1:]):
            list_tuples_p_lat = dict_p_lon_to_list_tuples_p_lat[p_lon_min]
            for tuple_p_lat in list_tuples_p_lat:
                cls.tree.toggle(tuple_p_lat[0], tuple_p_lat[1])

            stack_interval_active = cls.tree.get_stack_of_active_intervals()
            while stack_interval_active:
                p_lat_max = stack_interval_active.pop()
                p_lat_min = stack_interval_active.pop()

                dict_p_lon_to_list_rectangles[p_lon_min].append(
                    ReachPolygon.from_rectangle_vertices(p_lon_min, p_lat_min, p_lon_max, p_lat_max))

        return dict_p_lon_to_list_rectangles

    @classmethod
    def merge_rectangles_with_same_lateral_coordinates(
            cls, dict_p_lon_to_list_rectangles: Dict[int, List[ReachPolygon]]) -> List[ReachPolygon]:
        """
        Return a list of rectangles with possible merging.

        Iterate through pairs of lists of rectangles, if there is a right rectangle with the same lateral coordinates,
        then do not add to list. Instead, the right rectangle is popped and replaced by the merged one.
        """
        list_rectangles_merged = []

        list_p_lon = list(dict_p_lon_to_list_rectangles)
        for p_lon_left, p_lon_right in zip(list_p_lon[:-1], list_p_lon[1:]):
            list_rectangles_left = dict_p_lon_to_list_rectangles[p_lon_left]
            list_rectangles_right = dict_p_lon_to_list_rectangles[p_lon_right]

            for rectangle_left in list_rectangles_left:
                add_to_list = True

                for rectangle_right in list_rectangles_right:
                    if cls.rectangles_have_same_p_lat(rectangle_left, rectangle_right) and \
                            rectangle_left.p_lon_max >= rectangle_right.p_lon_min:
                        list_rectangles_right.remove(rectangle_right)
                        list_rectangles_right.append(
                            ReachPolygon.from_rectangle_vertices(rectangle_left.p_lon_min, rectangle_left.p_lat_min,
                                                                 rectangle_right.p_lon_max, rectangle_left.p_lat_max))
                        add_to_list = False
                        break

                if add_to_list:
                    list_rectangles_merged.append(rectangle_left)

        # add rectangles from the last list in to final list
        list_rectangles_merged += dict_p_lon_to_list_rectangles[list_p_lon[-1]]

        return list_rectangles_merged

    @staticmethod
    def rectangles_have_same_p_lat(rectangle1: ReachPolygon, rectangle2: ReachPolygon) -> bool:
        """
        Returns True if the two input rectangles have the same lateral positions.
        """
        return rectangle1.p_lat_min == rectangle2.p_lat_min and rectangle1.p_lat_max == rectangle2.p_lat_max
