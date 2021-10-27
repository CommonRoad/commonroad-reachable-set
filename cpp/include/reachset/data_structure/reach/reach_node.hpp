#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reach_polygon.hpp"

namespace reach {
/// Node within the reachability graph, also used in the reachable set computation.
/// Each node is a Cartesian product of polygon_lon and polygon_lat. In Curvilinear coordinate system, polygon_lon
/// is a polygon in the longitudinal p-v domain, and polygon_lat is a polygon in the lateral p-v domain; In Cartesian
/// coordinate system, they represent polygons in the x-v and y-v domains, respectively. It stores a
/// ReachNode inherits ReachBaseSet. In addition, it has a time step, an ID, and lists of parent nodes and child nodes.
class ReachNode {
private:
    static int cnt_id;
    std::vector<std::shared_ptr<ReachNode>> _vec_nodes_parent;
    std::vector<std::shared_ptr<ReachNode>> _vec_nodes_child;

public:
    int id{};
    int time_step{};
    ReachPolygonPtr polygon_lon;
    ReachPolygonPtr polygon_lat;
    std::tuple<double, double, double, double> box_lon;
    std::tuple<double, double, double, double> box_lat;
    // vector of nodes from which the current node is originated
    std::vector<std::shared_ptr<ReachNode>> vec_nodes_source{};
    std::set<std::string> set_propositions;

    ReachNode() = default;

    /// Constructor of ReachNode
    /// @param time_step time step of the node
    /// @param polygon_lon longitudinal polygon of the node
    /// @param polygon_lat lateral polygon of the node
    ReachNode(int const& time_step, ReachPolygonPtr const& polygon_lon, ReachPolygonPtr const& polygon_lat,
              std::set<std::string> const& set_propositions = {});

    std::shared_ptr<ReachNode> clone() const;

    inline double p_lon_min() const { return std::get<0>(this->box_lon); }

    inline double p_lon_max() const { return std::get<2>(this->box_lon); }

    inline double v_lon_min() const { return std::get<1>(this->box_lon); }

    inline double v_lon_max() const { return std::get<3>(this->box_lon); }

    inline double p_lat_min() const { return std::get<0>(this->box_lat); }

    inline double p_lat_max() const { return std::get<2>(this->box_lat); }

    inline double v_lat_min() const { return std::get<1>(this->box_lat); }

    inline double v_lat_max() const { return std::get<3>(this->box_lat); }

    inline std::vector<std::shared_ptr<ReachNode>> vec_nodes_parent() const { return this->_vec_nodes_parent; };

    inline std::vector<std::shared_ptr<ReachNode>> vec_nodes_child() const { return this->_vec_nodes_child; };

    /// Rectangle representing the projection of the node onto the position domain.
    ReachPolygonPtr position_rectangle() const;

    bool add_parent_node(std::shared_ptr<ReachNode> const& node_parent);

    bool remove_parent_node(std::shared_ptr<ReachNode> const& node_parent);

    bool add_child_node(std::shared_ptr<ReachNode> const& node_child);

    bool remove_child_node(std::shared_ptr<ReachNode> const& node_child);

    void assign_parent_nodes(std::vector<std::shared_ptr<ReachNode>> const& vec_nodes_parent);

    void assign_child_nodes(std::vector<std::shared_ptr<ReachNode>> const& vec_nodes_child);

    void clear_parent_nodes();

    void clear_child_nodes();

    /// Increases the ID of the node by 1
    inline void increment_id() {
        id = ReachNode::cnt_id++;
    }

    /// Resets the ID counter of nodes to zero.
    static void reset_id_counter();

    /// Intersects with the given rectangle in position domain
    void intersect_in_position_domain(double const& p_lon_min, double const& p_lat_min,
                                      double const& p_lon_max, double const& p_lat_max);

    /// Intersects with the given velocities in velocity domain
    void intersect_in_velocity_domain(double const& v_lon_min, double const& v_lat_min,
                                      double const& v_lon_max, double const& v_lat_max);

};

using ReachNodePtr = std::shared_ptr<ReachNode>;
}