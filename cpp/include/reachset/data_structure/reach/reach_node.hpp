#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reach_polygon.hpp"

namespace reach {
/// Node within the reachability graph, also used in the reachable set computation.
/// Each node is a Cartesian product of polygon_lon and polygon_lat. In curvilinear coordinate system, polygon_lon
/// is a polygon in the longitudinal p-v domain, and polygon_lat is a polygon in the lateral p-v domain; In Cartesian
/// coordinate system, they represent polygons in the x-v and y-v domains, respectively. ReachNode inherits ReachBaseSet.
/// In addition, it has a step, an ID, and lists of parent nodes and child nodes.
class ReachNode {
private:
    static int cnt_id;
    std::vector<std::weak_ptr<ReachNode>> _vec_nodes_parent;
    std::vector<std::weak_ptr<ReachNode>> _vec_nodes_child;

    /// Check whether it is necessary to perform a halfspace intersection given current and target bounds.
    /// @param current current bound of the polygon
    /// @param target target bound of the polygon
    /// @param is_max if true, current and target bound are treated as maximal bounds, otherwise as minimal bounds.
    /// @returns true iff the halfspace intersection is necessary.
    static inline bool _is_halfspace_intersection_necessary(double current, double target, bool is_max) {
        return is_max ? target < current : target > current;
    };

public:
    ReachNode() = default;

    /// Constructor of ReachNode.
    /// @param step step of the node
    /// @param polygon_lon longitudinal polygon of the node
    /// @param polygon_lat lateral polygon of the node
    ReachNode(int const& step, ReachPolygonPtr  polygon_lon, ReachPolygonPtr  polygon_lat);

    int id{};
    int step{};
    ReachPolygonPtr polygon_lon;
    ReachPolygonPtr polygon_lat;
    // vector of nodes from which the current node is originated
    std::vector<std::shared_ptr<ReachNode>> vec_nodes_source{};


    inline std::shared_ptr<ReachNode> clone() const {
        auto node_clone =
                std::make_shared<ReachNode>(step, polygon_lon->clone(), polygon_lat->clone());
        node_clone->vec_nodes_child() = vec_nodes_child();
        node_clone->vec_nodes_parent() = vec_nodes_parent();
        node_clone->vec_nodes_source = vec_nodes_source;

        return node_clone;
    }

    inline std::tuple<double, double, double, double> box_lon() const{ return polygon_lon->bounding_box();}

    inline std::tuple<double, double, double, double> box_lat() const{ return polygon_lat->bounding_box();}

    inline double p_lon_min() const { return std::get<0>(this->box_lon()); }

    inline double p_lon_max() const { return std::get<2>(this->box_lon()); }

    inline double v_lon_min() const { return std::get<1>(this->box_lon()); }

    inline double v_lon_max() const { return std::get<3>(this->box_lon()); }

    inline double p_lat_min() const { return std::get<0>(this->box_lat()); }

    inline double p_lat_max() const { return std::get<2>(this->box_lat()); }

    inline double v_lat_min() const { return std::get<1>(this->box_lat()); }

    inline double v_lat_max() const { return std::get<3>(this->box_lat()); }

    inline bool is_empty() const { return polygon_lon->empty() || polygon_lat->empty(); }

    inline std::vector<std::shared_ptr<ReachNode>> vec_nodes_parent() const { 
        std::vector<std::shared_ptr<ReachNode>> vec_nodes_parent_shared;
        for (auto const& elem : this->_vec_nodes_parent)
        {
            vec_nodes_parent_shared.emplace_back(elem.lock());
        }
        return vec_nodes_parent_shared;
    };

    inline std::vector<std::shared_ptr<ReachNode>> vec_nodes_child() const { 
        std::vector<std::shared_ptr<ReachNode>> vec_nodes_child_shared;
        for (auto const& elem : this->_vec_nodes_child)
        {
            vec_nodes_child_shared.emplace_back(elem.lock());
        }
        return vec_nodes_child_shared;
    };

    /// Rectangle representing the projection of the node onto the position domain.
    inline ReachPolygonPtr position_rectangle() const {
        return std::make_shared<ReachPolygon>(p_lon_min(), p_lat_min(),
                                              p_lon_max(), p_lat_max());
    }

    inline void assign_parent_nodes(std::vector<std::weak_ptr<ReachNode>> const& vec_nodes_parent) {
        _vec_nodes_parent = vec_nodes_parent;
    }

    void assign_child_nodes(std::vector<std::weak_ptr<ReachNode>> const& vec_nodes_child) {
        _vec_nodes_child = vec_nodes_child;
    }

    inline void clear_parent_nodes() {
        _vec_nodes_parent.clear();
    }

    inline void clear_child_nodes() {
        _vec_nodes_child.clear();
    }

    /// Increases the ID of the node by 1.
    inline void increment_id() {
        id = ReachNode::cnt_id++;
    }

    /// Intersects with the given rectangle in position domain.
    inline void intersect_in_position_domain(double const& p_lon_min=-std::numeric_limits<double>::infinity(), double const& p_lat_min=-std::numeric_limits<double>::infinity(),
                                             double const& p_lon_max=std::numeric_limits<double>::infinity(), double const& p_lat_max=std::numeric_limits<double>::infinity()) {
        if (_is_halfspace_intersection_necessary(this->p_lon_max(), p_lon_max, true)) {
            polygon_lon->intersect_halfspace(1, 0, p_lon_max);
        }
        if (_is_halfspace_intersection_necessary(this->p_lon_min(), p_lon_min, false)) {
            polygon_lon->intersect_halfspace(-1, 0, -p_lon_min);
        }
        if (_is_halfspace_intersection_necessary(this->p_lat_max(), p_lat_max, true)) {
            polygon_lat->intersect_halfspace(1, 0, p_lat_max);
        }
        if (_is_halfspace_intersection_necessary(this->p_lat_min(), p_lat_min, false)) {
            polygon_lat->intersect_halfspace(-1, 0, -p_lat_min);
        }
    }

    /// Intersects with the given velocities in velocity domain.
    inline void intersect_in_velocity_domain(double const& v_lon_min=-std::numeric_limits<double>::infinity(), double const& v_lat_min=-std::numeric_limits<double>::infinity(),
                                             double const& v_lon_max=std::numeric_limits<double>::infinity(), double const& v_lat_max=std::numeric_limits<double>::infinity()) {
        if (_is_halfspace_intersection_necessary(this->v_lon_max(), v_lon_max, true)) {
            polygon_lon->intersect_halfspace(0, 1, v_lon_max);
        }
        if (_is_halfspace_intersection_necessary(this->v_lon_min(), v_lon_min, false)) {
            polygon_lon->intersect_halfspace(0, -1, -v_lon_min);
        }
        if (_is_halfspace_intersection_necessary(this->v_lat_max(), v_lat_max, true)) {
            polygon_lat->intersect_halfspace(0, 1, v_lat_max);
        }
        if (_is_halfspace_intersection_necessary(this->v_lat_min(), v_lat_min, false)) {
            polygon_lat->intersect_halfspace(0, -1, -v_lat_min);
        }
    }

    bool add_parent_node(std::shared_ptr<ReachNode> const& node_parent);

    bool remove_parent_node(std::shared_ptr<ReachNode> const& node_parent);

    bool add_child_node(std::shared_ptr<ReachNode> const& node_child);

    bool remove_child_node(std::shared_ptr<ReachNode> const& node_child);

    /// Resets the ID counter of nodes to zero.
    inline static void reset_id_counter() {
        ReachNode::cnt_id = 0;
    }
};

using ReachNodePtr = std::shared_ptr<ReachNode>;
}