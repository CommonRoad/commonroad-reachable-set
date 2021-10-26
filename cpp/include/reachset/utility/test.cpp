vector<ReachNodePtr>
reach::create_reachable_set_nodes_continuous(int const& time_step, vector<ReachNodePtr> const& vec_base_sets_adapted,
                                             int const& num_threads) {
    vector<ReachNodePtr> vec_nodes_reachable_set_new;
    vec_nodes_reachable_set_new.reserve(vec_base_sets_adapted.size());

    omp_lock_t lock;
    omp_init_lock(&lock);
#pragma omp parallel num_threads(num_threads)
    {
        vector<ReachNodePtr> vec_nodes_reachable_set_thread;
        vec_nodes_reachable_set_thread.reserve(vec_base_sets_adapted.size());

#pragma omp for nowait
        for (auto& node_child: vec_base_sets_adapted) {
            node_child->time_step = time_step;

            // update parent-child relationship
            for (auto& node_parent: node_child->vec_nodes_source) {
                node_child->add_parent_node(node_parent);
                omp_set_lock(&lock);
                node_parent->add_child_node(node_child);
                omp_unset_lock(&lock);
            }
            vec_nodes_reachable_set_thread.emplace_back(node_child);
        }
#pragma omp critical
vec_nodes_reachable_set_new.insert(vec_nodes_reachable_set_new.end(),
                                   std::make_move_iterator(vec_nodes_reachable_set_thread.begin()),
                                   std::make_move_iterator(vec_nodes_reachable_set_thread.end()));
    }
    omp_destroy_lock(&lock);

    return vec_nodes_reachable_set_new;
}