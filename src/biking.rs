//Written by C. Wen

#[allow(dead_code)]
mod contraction_hierarchies {
    use crate::graph::routing::*;
    use std::{
        cmp::Reverse,
        collections::{BinaryHeap, HashMap},
    };

    pub struct ContractedGraph {
        pub ordered_nodes: HashMap<i64, u32>, //id, order number
    }

    impl ContractedGraph {
        pub fn new() -> ContractedGraph {
            ContractedGraph {
                ordered_nodes: HashMap::new(),
            }
        }

        pub fn compute_random_node_ordering(&mut self, graph: &mut Dijkstra, length: usize) {
            //self.ordered_nodes_to_contract.insert(0);
            while self.ordered_nodes.len() < length {
                self.ordered_nodes
                    .insert(graph.get_random_node_id().unwrap_or_default(), 0);
            }
            graph.reset_all_flags(true);
        }

        pub fn contract_node(
            &mut self,
            node_id: i64,
            graph: &mut Dijkstra,
            compute_edge_diff_only: bool,
        ) -> (u8, i8) {
            //(#shortcuts, edge difference = #shortcuts - arcs incident to node)
            let mut num_shortcuts: u8 = 0;
            let mut edge_diff: i8 = 0;
            let mut costs_of_uv = HashMap::new();
            let mut costs_of_vw = HashMap::new();
            if let Some(edgelist) = graph.graph.edges.get_mut(&node_id) {
                //if Some
                for (w, (cost, flag)) in edgelist {
                    if *flag {
                        costs_of_vw.insert(*w, *cost);
                        *flag = false;
                        edge_diff -= 1;
                    }
                }
            }

            for (&w, &cost) in costs_of_vw.iter() {
                graph
                    .graph
                    .edges
                    .get_mut(&w)
                    .unwrap()
                    .get_mut(&node_id)
                    .unwrap()
                    .1 = false;
                costs_of_uv.insert(w, cost);
            }

            /*for (u, edgelist) in graph.graph.edges.iter_mut() { //for arcs, undirected
                for (v, (cost, flag)) in edgelist.iter_mut() {
                    if *v == node_id {
                        *flag = false;
                        costs_of_uv.insert(*u, *cost);
                    }
                }
            }*/

            graph.set_cost_upper_bound(
                //costs_of_uv.clone().into_values().max().unwrap_or_default() +
                2 * costs_of_vw.clone().into_values().max().unwrap_or_default(),
            );

            let mut temp_shortcuts = HashMap::new();
            for (u, cost_uv) in costs_of_uv.iter() {
                graph.dijkstra(*u, -1, &None, true);
                for (w, cost_vw) in costs_of_vw.iter() {
                    if w == u {
                        continue;
                    }
                    let path_via_uvw = cost_uv + cost_vw;
                    let &dist_w = graph
                        .visited_nodes
                        .get(w)
                        .unwrap_or(temp_shortcuts.get(&(u, w)).unwrap_or(&u64::MAX));
                    if dist_w > path_via_uvw
                        && temp_shortcuts.insert((u, w), path_via_uvw).is_none()
                        && temp_shortcuts.insert((w, u), path_via_uvw).is_none()
                    {
                        edge_diff += 1;
                        num_shortcuts += 1;
                    }
                }
            }
            if compute_edge_diff_only {
                for (&w, &_) in costs_of_vw.iter() {
                    graph
                        .graph
                        .edges
                        .get_mut(&w)
                        .unwrap()
                        .get_mut(&node_id)
                        .unwrap()
                        .1 = true;
                    graph
                        .graph
                        .edges
                        .get_mut(&node_id)
                        .unwrap()
                        .get_mut(&w)
                        .unwrap()
                        .1 = true;
                }
            } else {
                for ((u, w), path_via_uvw) in temp_shortcuts {
                    graph
                        .graph
                        .edges
                        .get_mut(u)
                        .unwrap()
                        .insert(*w, (path_via_uvw, true));
                }
            }

            (num_shortcuts, edge_diff)
        }

        pub fn ch_precompute(&mut self, graph: &mut Dijkstra) -> u64 {
            let mut total_shortcuts = 0;
            graph.set_max_settled_nodes(20);

            //step 1: calculate initial e_d order --> PQ with (k: e_d, v: node)
            let mut priority_queue: BinaryHeap<Reverse<(i8, i64)>> = BinaryHeap::new();
            for &n in graph.graph.raw_nodes.clone().iter() {
                let (_, e_d) = self.contract_node(n, graph, true);
                priority_queue.push(Reverse((e_d, n)));
            }

            //step 2: contract all nodes in order of ed, recalculate lazy heuristic
            //Lazy: If current node does not have smallest ED, pick next, recompute its ED, repeat until find smallest
            let mut index = 0;
            while !priority_queue.is_empty() {
                let (_, node_id) = priority_queue.pop().unwrap().0;
                let (_, e_d) = self.contract_node(node_id, graph, true);

                if !priority_queue.is_empty() {
                    let (next_ed, _) = priority_queue.peek().unwrap().0;
                    if e_d > next_ed {
                        priority_queue.push(Reverse((e_d, node_id)));
                        continue;
                    }
                }

                //establish contraction order
                self.ordered_nodes.insert(node_id, index);
                total_shortcuts += self.contract_node(node_id, graph, false).0 as u64;
                index += 1;
            }
            //step 3: reset arc flags, only flags of arc(u, v) with u.index < v.index == true
            graph.reset_all_flags(false);
            for (u, edgelist) in graph.graph.edges.iter_mut() {
                for (v, (_, flag)) in edgelist {
                    if self.ordered_nodes.get(u) < self.ordered_nodes.get(v) {
                        *flag = true;
                    }
                }
            }
            graph.set_max_settled_nodes(u64::MAX);
            graph.set_cost_upper_bound(31536000);
            total_shortcuts
        }

        pub fn bidirectional_compute(
            graph: &mut Dijkstra,
            source_id: i64,
            target_id: i64,
        ) -> (u64, i64) {
            graph.dijkstra(source_id, target_id, &None, true);
            let dist_source_to_u = graph.visited_nodes.clone();
            graph.dijkstra(target_id, source_id, &None, true);
            let dist_target_to_u = &graph.visited_nodes;

            let mut distances_s_t = Vec::new();
            for (u, dist_s) in dist_source_to_u {
                if let Some(dist_t) = dist_target_to_u.get(&u) {
                    distances_s_t.push((dist_t + dist_s, u));
                }
            }
            *distances_s_t.iter().min().unwrap_or(&(0, 0))
        }
    }
}
#[cfg(test)]
mod tests {
    use crate::biking::contraction_hierarchies::*;
    use crate::graph::graph_construction::*;
    use crate::graph::routing::*;
    use std::time::Instant;

    #[test]
    fn contraction_hierarchies_test() {
        //std::env::set_var("RUST_BACKTRACE", "1");
        //let path = "bw.pbf";
        //let path = "uci.pbf";
        let path = "saarland.pbf";

        //need to write this replacement!
        let mut roads = RoadNetwork::new();
        print!(
            "{} Base Graph Nodes: {}, Edges: {}\t\t",
            path,
            roads.nodes.len(),
            roads.edges.len()
        );
        let now = Instant::now();
        let mut time = now.elapsed().as_millis() as f32 * 0.001;
        println!(
            "time: {}, reduced map nodes: {}, edges: {}",
            time,
            roads.nodes.len(),
            /* roads
                .edges
                .iter()
                .map(|(_, edges)| edges.len()) */
            roads
                .edges.values().map(|edges| edges.len())
                .sum::<usize>()
                / 2
        );

        let mut graph = Dijkstra::new(&roads);
        let mut ch_algo = ContractedGraph::new();
        let now = Instant::now();

        graph.reset_all_flags(true);
        graph.set_max_settled_nodes(20);

        ch_algo.compute_random_node_ordering(&mut graph, 1000); //here
        let mut contraction_time = Vec::new();
        let mut shortcut_hg = vec![0, 0, 0, 0, 0];
        let mut edge_diff_hg = vec![0, 0, 0, 0, 0];
        
        //for (&n, _) in ch_algo.ordered_nodes.clone().iter() {
        for &n in ch_algo.ordered_nodes.clone().keys() {
            let now = Instant::now();
            let (num_shortcut, num_edge_diff) = ch_algo.contract_node(n, &mut graph, false);
            time = now.elapsed().as_micros() as f32;
            contraction_time.push(time);

            if num_shortcut == 0 {
                shortcut_hg[0] += 1;
            } else if num_shortcut == 1 {
                shortcut_hg[1] += 1;
            } else if num_shortcut == 2 {
                shortcut_hg[2] += 1;
            } else if num_shortcut == 3 {
                shortcut_hg[3] += 1;
            } else if num_shortcut >= 4 {
                shortcut_hg[4] += 1;
            }

            if num_edge_diff <= -3 {
                edge_diff_hg[0] += 1;
            } else if num_edge_diff == -2 {
                edge_diff_hg[1] += 1;
            //} else if num_edge_diff >= -1 && num_edge_diff <= 1 {
            } else if (-1..=1).contains(&num_edge_diff) {
                edge_diff_hg[2] += 1;
            } else if num_edge_diff == 2 {
                edge_diff_hg[3] += 1;
            } else if num_edge_diff >= 3 {
                edge_diff_hg[4] += 1;
            }
        }

        println!(
            "average contraction time in micros {}",
            contraction_time.iter().sum::<f32>() / contraction_time.len() as f32
        );

        println!("shortcut histogram {:?}", shortcut_hg);

        println!("edge difference histogram {:?}", edge_diff_hg);

        graph.reset_all_flags(true);

        let total_shortcuts = ch_algo.ch_precompute(&mut graph);
        time = now.elapsed().as_millis() as f32 * 0.001;
        println!("precomp seconds: {}", time);
        println!("total shortcuts: {}", total_shortcuts);

        let mut shortest_path_costs = Vec::new();
        let mut query_time = Vec::new();

        for _ in 0..1000 {
            //1000
            let source = graph.get_random_node_id().unwrap();
            let target = graph.get_random_node_id().unwrap();
            let now = Instant::now();
            let result = ContractedGraph::bidirectional_compute(&mut graph, source, target);
            time = now.elapsed().as_millis() as f32 * 0.001;
            query_time.push(time);
            shortest_path_costs.push(result.0);
        }

        println!(
            "average cost mm:ss {}:{} \n\t eg {}, {}, {}",
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 60,
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 % 60,
            shortest_path_costs.get(250).unwrap(),
            shortest_path_costs.get(500).unwrap(),
            shortest_path_costs.get(750).unwrap()
        );
        println!(
            "average query time in seconds {}",
            query_time.iter().sum::<f32>() / query_time.len() as f32
        );
    }
}
