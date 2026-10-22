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
    use crate::graph::graph_construction::*;
    //use crate::landmark_algo::*;
    use crate::biking::contraction_hierarchies::*;
    use crate::graph::routing::*;
    //use std::collections::HashMap;
    use std::time::Instant;

    #[test]
    fn contraction_hierarchies_test() {
        //std::env::set_var("RUST_BACKTRACE", "1");
        //let path = "bw.pbf";
        //let path = "uci.pbf";
        let path = "saarland.pbf";

        //need to write this replacement!
        let data = RoadNetwork::read_from_osm_file(path).unwrap();
        let mut roads = RoadNetwork::new(data.0, data.1);
        print!(
            "{} Base Graph Nodes: {}, Edges: {}\t\t",
            path,
            roads.nodes.len(),
            roads.edges.len()
        );
        let now = Instant::now();
        roads = roads.reduce_to_largest_connected_component();
        let mut time = now.elapsed().as_millis() as f32 * 0.001;
        println!(
            "time: {}, reduced map nodes: {}, edges: {}",
            time,
            roads.nodes.len(),
            roads
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
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
        for (&n, _) in ch_algo.ordered_nodes.clone().iter() {
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
            } else if num_edge_diff >= -1 && num_edge_diff <= 1 {
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

    /*
    #[test]
    fn dijkstras_test() {

        let mut shortest_path_costs = Vec::new();
        let mut query_time = Vec::new();
        let mut settled_nodes = Vec::new();
        let heuristics = None;

        //let precompute = landmark_heuristic_precompute(&mut graph, 42);
        let arc_flag_thing = ArcFlags::new(49.20, 49.25, 6.95, 7.05); //saar
        //let arc_flag_thing = ArcFlags::new(47.95, 48.05, 7.75, 7.90); //ba-wu
        //let arc_flag_thing = ArcFlags::new(33.63, 33.64, -117.84, -117.83); //uci
        arc_flag_thing.arc_flags_precompute(&mut graph);
        time = now.elapsed().as_millis() as f32 * 0.001;
        println!("pre done {} \n", time);


        for _ in 0..100 {
            let source = graph.get_random_node_id().unwrap();
            //let target = graph.get_random_node_id().unwrap();
            let target = graph.get_random_node_area_id(49.20, 49.25, 6.95, 7.05); //saar
            //let target = graph.get_random_node_area_id(47.95, 48.05, 7.75, 7.90); //ba-wu
            //let target = graph.get_random_node_area_id(33.63, 33.64, -117.84, -117.83); //uci
            //heuristics = a_star_heuristic(&roads, target);
            //heuristics = landmark_heuristic(&precompute, &graph, target);
            let now = Instant::now();
            let result = graph.dijkstra(source, target, &heuristics, true);
            time = now.elapsed().as_millis() as f32 * 0.001;
            query_time.push(time);

            if let Some(cost) = result.0 {
                shortest_path_costs.push(cost.get_path().1);
            } else {
                shortest_path_costs.push(0);
            }
            settled_nodes.push(graph.visited_nodes.len() as u64);
        }

        println!(
            "average cost in minutes {}",
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 60
        );
        println!(
            "average query time in seconds {}",
            query_time.iter().sum::<f32>() / query_time.len() as f32
        );
        println!(
            "average settle node number {}",
            settled_nodes.iter().sum::<u64>() / settled_nodes.len() as u64
        );
    }*/

    /*
    #[test]
    fn test() {
        let _node0 = Node {
            id: 0,
            lat: 490000000,
            lon: 65000000,
        };
        let node1 = Node {
            id: 1,
            lat: 491000000,
            lon: 65100000,
        };
        let node2 = Node {
            id: 2,
            lat: 495000000,
            lon: 70000000,
        };
        let node3 = Node {
            id: 3,
            lat: 493500000,
            lon: 71250000,
        };
        let node4 = Node {
            id: 4,
            lat: 492500000,
            lon: 72500000,
        };
        let node5 = Node {
            id: 5,
            lat: 497500000,
            lon: 72500000,
        };
        let node6 = Node {
            id: 6,
            lat: 0,
            lon: 0
        };
        let node7 = Node {
            id: 7,
            lat: 0,
            lon: 0
        };
        let node8 = Node {
            id: 8,
            lat: 0,
            lon: 0
        };
        let node9 = Node {
            id: 9,
            lat: 0,
            lon: 0
        };
        let node10 = Node {
            id: 10,
            lat: 0,
            lon: 0
        };
        let node11 = Node {
            id: 11,
            lat: 0,
            lon: 0
        };
        let node12 = Node {
            id: 12,
            lat: 0,
            lon: 0
        };
        let node13 = Node {
            id: 13,
            lat: 0,
            lon: 0
        };
        //based on Professor Bast's example graph from CH lecture 1 slide 16
        let roads = RoadNetwork {
            nodes: HashMap::from([ (1, node1), (2, node2), (3, node3), (4, node4), (5, node5), (6, node6), (7, node7), (8, node8), (9, node9), (10, node10), (11, node11), (12, node12), (13, node13)]),
            edges: HashMap::from([
                (1, HashMap::from([ (2, (3, false)), (13, (4, false)), (6, (7, false)) ])),
                (2, HashMap::from([ (1, (3, false)), (8, (2, false)), (13, (5, false)) ])),
                (3, HashMap::from([ (5, (5, false)), (12, (2, false)), (4, (4, false)) ])),
                (4, HashMap::from([ (3, (4, false)), (12, (3, false)), (7, (4, false)) ])),
                (5, HashMap::from([ (6, (6, false)), (11, (3, false)), (3, (5, false)) ])),
                (6, HashMap::from([ (1, (7, false)), (10, (4, false)), (5, (6, false)) ])),
                (7, HashMap::from([ (9, (7, false)), (11, (3, false)), (4, (4, false)) ])),
                (8, HashMap::from([ (2, (2, false)), (13, (2, false)), (9, (5, false)) ])),
                (9, HashMap::from([ (8, (5, false)), (10, (3, false)), (7, (7, false)) ])),
                (10, HashMap::from([ (9, (3, false)), (11, (1, false)), (6, (4, false)), (13, (1, false)) ])),
                (11, HashMap::from([ (7, (3, false)), (12, (1, false)), (5, (3, false)), (10, (1, false)) ])),
                (12, HashMap::from([ (4, (3, false)), (3, (2, false)), (11, (1, false)) ])),
                (13, HashMap::from([ (8, (2, false)), (10, (1, false)), (1, (4, false)), (2, (5, false)) ])),
            ]),
            raw_ways: vec![Way{id:0,speed:0, refs:vec![0,0]}],
            raw_nodes: vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
        };
        let mut graph = Dijkstra::new(&roads);
        let mut ch_algo = ContractedGraph::new();
        graph.reset_all_flags(true);

        //(#shortcuts, edge difference = #shortcuts - arcs incident to node)
        print!("edge diff only{:?}\t", ch_algo.contract_node(2, &mut graph, true));

        print!("contract node {:?}\t", ch_algo.contract_node(2, &mut graph, false));

        let mut graph = Dijkstra::new(&roads);
        let mut ch_algo = ContractedGraph::new();
        graph.reset_all_flags(true);

        print!("total shortcuts: {}\n", ch_algo.ch_precompute(&mut graph));
        print!("{:?}\n", graph.graph.edges);
        print!("{:?}\n", ch_algo.ordered_nodes);
        let result = ContractedGraph::bidirectional_compute(&mut graph, 1, 4);
        print!("cost: {}    joint: {}\n", result.0, result.1);
    }
    */

    /*
    #[test]
    fn tnr_test_custom() {
        let node0 = Node { id: 0, lat: 0, lon: 0 };
        let node1 = Node { id: 1, lat: 0, lon: 0 };
        let node2 = Node { id: 2, lat: 0, lon: 0 };
        let node3 = Node { id: 3, lat: 0, lon: 0 };
        let node4 = Node { id: 4, lat: 0, lon: 0 };
        let node5 = Node { id: 5, lat: 0, lon: 0 };
        let node6 = Node { id: 6, lat: 0, lon: 0 };
        let node7 = Node { id: 7, lat: 0, lon: 0 };
        let node8 = Node { id: 8, lat: 0, lon: 0 };
        let node9 = Node { id: 9, lat: 0, lon: 0 };

        let roads = RoadNetwork {
            nodes: HashMap::from([ (0, node0), (1, node1), (2, node2), (3, node3), (4, node4), (5, node5), (6, node6), (7, node7), (8, node8), (9, node9)]),
            edges: HashMap::from([
                (0, HashMap::from([ (1, (3, false)) ])),
                (1, HashMap::from([ (2, (3, false)), (3, (3, false)), (8, (3, false)), (0, (3, false)) ])),
                (2, HashMap::from([ (1, (3, false)), (4, (3, false)), (5, (3, false)) ])),
                (3, HashMap::from([ (1, (3, false)), (6, (3, false)), (7, (3, false)) ])),
                (4, HashMap::from([ (2, (3, false)) ])),
                (5, HashMap::from([ (2, (3, false)) ])),
                (6, HashMap::from([ (3, (3, false)) ])),
                (7, HashMap::from([ (3, (3, false)) ])),
                (8, HashMap::from([ (1, (3, false)), (9, (3, false)) ])),
                (9, HashMap::from([ (8, (3, false)) ]))
            ]),
            raw_ways: vec![Way{id:0,speed:0, refs:vec![0,0]}],
            raw_nodes: vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        };

        let mut graph = Dijkstra::new(&roads);

        graph.reset_all_flags(true);
        let transit_nodes = ch_based_precompute(&mut graph, 3);
        //let transit_nodes = HashSet::from([2, 3, 8]);
        println!("{:?}", transit_nodes);

        for _ in 0..1 {
            let source_id = graph.get_random_node_id().unwrap();
            let target_id = graph.get_random_node_id().unwrap();
            println!("s: {}, t: {}", source_id, target_id);
            let access_nodes = compute_access_nodes(source_id, &mut graph, &transit_nodes);
            print!("n:  {:?}\t", access_nodes);
            let distance_sets = access_node_distance_generator(source_id, target_id, access_nodes, &mut graph);
            let result = compute_shortest_path_tnr(distance_sets);

            println!("{}", result)
        }

    }
    */
}
