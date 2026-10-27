//Written by C. Wen, inspired by K. Chin
pub mod graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::graph::routing::*;
    use core::num;
    use std::{collections::{HashMap, HashSet}, hash::Hash, ops::Index}; 
    use osr_ffi::{Algorithm, Profile, Direction, ffi};
    use serde::{Deserialize, Serialize};
    use serde_json::{Value, json};
    use std::{
        env,
        time::Instant,
    };

    #[derive(Debug, Clone, Serialize)]
    struct GraphInfo {
    node_count: u64,
    way_count: u64,
    has_elevations: bool,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct RoadNetwork {
        //graph struct that will be used to route
        pub nodes: HashSet<u32>, // OSR node id
        pub raw_ways: Vec<u32>, // OSR way id
        pub edges: HashMap<u32, HashMap<u32, (u64, bool)>>, // tail.id, <head.id, (cost, arcflag)>
    }
    
    impl RoadNetwork {
        //need way to construct this from OSR stuff kyler wrote
        pub fn new() -> Self {
            let path = "/path";
            let router =  ffi::open_router(path).expect("OSR returned null router");
            let graph = GraphInfo {
                node_count: router.node_count(),
                way_count: router.way_count(),
                has_elevations: router.has_elevations(),
            };
            //init new RoadNetwork based on results from reading .pbf file
            let mut edges: HashMap<u32, HashMap<u32, (u64, bool)>> = HashMap::new();
            let mut nodes = HashSet::new();
            let mut raw_ways = Vec::new();
            Self { nodes,raw_ways, edges }
        }
    }

    impl Default for RoadNetwork {
        fn default() -> Self {
            Self::new()
        }
    }
    
}

#[allow(unused)]
pub mod routing {
    //routing algorithms and helper functiions
    use crate::graph::graph_construction::*;
    use rand::Rng;
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::hash::Hash;
    use std::path;
    use std::rc::Rc;
    use std::time::Instant;

    pub struct Dijkstra {
        //handle dijkstra calculations
        pub graph: RoadNetwork,
        pub visited_nodes: HashMap<u32, u64>,
        cost_upper_bound: u64,
        max_settled_nodes: u64,
    }

    #[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord, Hash)]
    pub struct PathedNode {
        //node that references parent nodes, used to create path from goal node to start node
        pub node_self: u32, //nodeid
        pub distance_from_start: u64,
        pub parent_node: Option<Rc<PathedNode>>,
    }

    impl PathedNode {
        pub fn extract_parent<PathedNode: std::clone::Clone>(
            //returns parent of a pathed node
            last_elem: Rc<PathedNode>,
        ) -> PathedNode {
            let inner: PathedNode = Rc::unwrap_or_clone(last_elem);
            inner
        }

        pub fn get_path(self) -> (Vec<u32>, u64) { //node id
            //uses reference to find the source node with parent_node == None
            let mut shortest_path: Vec<u32> = Vec::new();  //nodeid
            let mut total_distance: u64 = self.distance_from_start;
            let mut current = self;
            while let Some(previous_node) = current.parent_node {
                shortest_path.push(current.node_self);
                current = PathedNode::extract_parent(previous_node);
            }
            shortest_path.push(current.node_self);
            (shortest_path, total_distance)
        }
    }

    impl Dijkstra {
        //implementation of dijkstra's shortest path algorithm
        pub fn new(graph: &RoadNetwork) -> Self {
            let visited_nodes = HashMap::new();
            Self {
                graph: graph.clone(),
                visited_nodes,
                cost_upper_bound: u64::MAX,
                max_settled_nodes: u64::MAX,
            }
        }

        pub fn set_cost_upper_bound(&mut self, upper_bound: u64) {
            self.cost_upper_bound = upper_bound;
        }

        pub fn set_max_settled_nodes(&mut self, max_settled: u64) {
            self.max_settled_nodes = max_settled;
        }

        pub fn get_neighbors(
            &mut self,
            current: &PathedNode,
            consider_arc_flags: bool,
        ) -> Vec<(u32, u64)> {  //nodeid
            //return node id of neighbors
            let mut paths = Vec::new();
            let mut next_node_edges = HashMap::new();
            //need some case to handle neighbor to parent instead of just parent to neighbor
            if let Some(connections) = self.graph.edges.get_mut(&current.node_self) {
                next_node_edges.clone_from(connections);
            }
            for path in next_node_edges {
                if self.visited_nodes.contains_key(&path.0) {
                    continue;
                }
                if (consider_arc_flags && !path.1.1) {
                    continue;
                }
                paths.push((*self.graph.nodes.get(&path.0).unwrap(), path.1.0));
            }
            paths
        }

         pub fn dijkstra(
            &mut self,
            source_id: u32,
            target_id: Option<u32>,
            heuristics: &Option<HashMap<u32, u64>>,
            consider_arc_flags: bool,
        ) -> (Option<PathedNode>, HashMap<u32, u32>) {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
            let mut previous_nodes = HashMap::new();

            //set target (-1) for all-node-settle rather than just target settle or smth
            self.visited_nodes.clear();

            let source = *self
                .graph
                .nodes
                .get(&source_id)
                .unwrap_or_else(|| panic!("source node not found"));

            let source_node: PathedNode = PathedNode {
                node_self: (source),
                distance_from_start: 0,
                parent_node: (None),
            };

            //stores distances of node relative to target
            let mut gscore: HashMap<u32, u64> = HashMap::new();
            gscore.insert(source_id, 0);

            priority_queue.push(Reverse((0, source_node.clone())));

            let mut target = 0;

            if let Some (targ_id) = target_id{
                target = *self
                    .graph
                    .nodes
                    .get(&targ_id)
                    .unwrap_or_else(|| panic!("target node not found"));
            }

            let mut counter = 1;
            let mut cost = 0;
            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0.1; //.0 "unwraps" from Reverse()
                cost = pathed_current_node.distance_from_start;
                let idx = pathed_current_node.node_self;

                self.visited_nodes.insert(idx, cost);

                #[allow(clippy::collapsible_if)]
                //found target node
                if let Some(targ_id) = &target_id {
                    if idx.eq(targ_id) {
                        return (Some(pathed_current_node), previous_nodes);
                    }
                }

                //stop conditions
                //cost or # of settled nodes goes over limit
                if cost > self.cost_upper_bound
                    || self.visited_nodes.len() > self.max_settled_nodes as usize
                {
                    return (None, previous_nodes);
                }

                //cost is higher than current path (not optimal)
                if cost > *gscore.get(&idx).unwrap_or(&u64::MAX) {
                    continue;
                }

                for neighbor in self.get_neighbors(&pathed_current_node, consider_arc_flags) {
                    let temp_distance = pathed_current_node.distance_from_start + neighbor.1;
                    let next_distance = *gscore.get(&neighbor.0).unwrap_or(&u64::MAX);

                    if temp_distance < next_distance {
                        gscore.insert(neighbor.0, temp_distance);
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
                            node_self: neighbor.0,
                            distance_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };
                        let h;
                        if let Some(heuristic) = heuristics {
                            h = heuristic.get(&neighbor.0).unwrap_or(&0);
                        } else {
                            h = &0;
                        }
                        //fscore = temp_distance (gscore) + h (hscore)
                        priority_queue.push(Reverse((temp_distance + h, tentative_new_node)));
                        previous_nodes.insert(neighbor.0, pathed_current_node.node_self);
                    }
                }
                counter += 1;
            }
            (None, previous_nodes)
        } 

        pub fn get_random_node_id(&mut self) -> Option<u32> {
            //returns ID of a random valid node from a graph
            let mut full_node_list = &self.graph.nodes;
            full_node_list.iter().next().copied()
        }

        pub fn get_unvisted_node_id(
            //returns the first unvisted node that function parses upon (used to find largest connected component)
            &mut self,
            other_located_nodes: &HashMap<u32, i32>,
        ) -> Option<u32> {
            if other_located_nodes.len() == self.graph.nodes.len() {
                println!("all nodes visted");
                return None;
            }
            let other_located_nodes = other_located_nodes
                .iter()
                .filter(|(id, count)| **count > 0)
                .map(|(id, _)| id)
                .collect::<Vec<&u32>>();

            for node in &self.graph.nodes {
                if !other_located_nodes.contains(&node) {
                    return Some(*node);
                }
            }
            None
        }

        pub fn reset_all_flags(&mut self, state: bool) {
            //for (_, edgelist) in self.graph.edges.iter_mut() {
            for edgelist in self.graph.edges.values_mut() {
                for edge in edgelist.iter_mut() {
                    edge.1.1 = state;
                }
            }
        }
    }
}
