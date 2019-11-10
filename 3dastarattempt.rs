/// remember, the a* algorithm is: F(N) = G(N) + H(N), G(N) = cost to move from start to node, H(N) is cost from node to end 
/// our goal is to find the shortest, most cost efficient path using a* pathfinding

//from docs.rs pathfinding, need these for algorithm
use indexmap::map::Entry::{Occupied, Vacant};
use indexmap::IndexMap;
use num_traits::Zero;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};
use std::hash::Hash;
use std::usize;

use super::reverse_path;


pub fn astar<N, C, FN, IN, FH, FS>(
    start: &N,              //this would be the starting node
    mut successors: FN,     //successors are nodes adjacent to the node we are evaluating, F(N) is this cost associated with moving
    mut heuristic: FH,      //this would be the H(N) from the a* algorithm
    mut success: FS,        //checks to see if we reached the end node!
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Clone,   // from Eq, this relationship makes it so a node won't be used twice in our path
    C: Zero + Ord + Copy,   // using this to evaluate the cost of our path later
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,  //checks the success of the path, so we use bool

{
    let mut to_see = BinaryHeap::new(); //using binaryheap bc our data structure will take the shape of a binary tree when we find a path
    to_see.push(SmallestCostHolder {
        estimated_cost: heuristic(start),
        cost: Zero::zero(),
        index: 0,
    });

    //gonna try to get the successors in 3d
    let mut parents: IndexMap<N, (usize, C)> = IndexMap::new();
    parents.insert(start.clone(), (usize::max_value(), Zero::zero()));
    while let Some(SmallestCostHolder { cost, index, .. }) = to_see.pop() {
        let successors = {
            let (node, &(_, c)) = parents.get_index(index).unwrap();
            let x = parents.getindex(lat);      //x component of the node
            let y = parents.getindex(long);     //y component of the node
            let z = parents.getindex(height);   //z componenet of the node

            //these will check which successor node we will pick from the parent, i will need to fix syntax for sure though
            if(node[x-1] && node[x-1][y] && node[x-1][y][z] && !node[x-1][y][z+1]) {    //for the node down one from the starting node
                successor.push(node[x-1][y][z]);
            }

            if(node[x+1] && node[x+1][y] && node[x+1][y][z] && !node[x+1][y][z+1]) {    //up one node
                successor.push(node[x+1][y][z]);
            }

            if(node[x] && node[x][y-1] && node[x][y-1][z] && !node[x][y-1][z+1]) {      //left one node
                successor.push(node[x][y-1][z]);
            }
            if(node[x] && node[x][y+1] && node[x][y+1][z] && !node[x][y+1][z+1]) {      //right one node
                successor.push(node[x][y+1][z]);
            }

            if(node[x-1] && node[x-1][y] && node[x-1][y][z-1] && !node[x-1][y][z] && !node[x-1][y][z+1]) { //bottom down node
                successor.push(node[x-1][y][z-1]);
            }

            if(node[x+1] && node[x+1][y] && node[x+1][y][z-1] && !node[x+1][y][z] && !node[x+1][y][z+1]) { //bottom up node
                successor.push(node[x+1][y][z-1]);
            }

            if(node[x] && node[x][y-1] && node[x][y-1][z-1] && !node[x][y-1][z] && !node[x][y-1][z+1]) { //bottom left node
                successor.push(node[x][y-1][z-1]);
            }

            if(node[x] && node[x][y+1] && node[x][y+1][z-1] && !node[x][y+1][z] && !node[x][y+1][z+1]) { //bottom right node
                successor.push(node[x][y+1][z-1]);
            }

            if(node[x-1] && node[x-1][y] && node[x-1][y][z+1] && !node[x-1][y][z+2] && !node[x][y][z+2]) {  //node that is top and down
                successor.push(node[x-1][y][z+1]);
            }

            if(node[x+1] && node[x+1][y] && node[x+1][y][z+1] && !node[x+1][y][z+2] && !node[x][y][z+2]) { //top up node
                successor.push(node[x+1][y][z+1]);
            }

            if(node[x] && node[x][y-1] && node[x][y-1][z+1] && !node[x][y-1][z+2] && !node[x][y][z+2]) {  //top left node
                successor.push(node[x][y-1][z+1]);
            }

            if(node[x] && node[x][y+1] && node[x][y+1][z+1] && !node[x][y+1][z+2] && !node[x][y][z+2]) {  //top right node
                successor.push(node[x][y+1][z+1]);
            }
        
            // gotta account for diagonal nodes from parent as well...

            if success(node) {                  //if any of the nodes reach the end, return the path and cost
                let path = reverse_path(&parents, |&(p, _)| p, index);
                return Some((path, cost));      //can insert a node multi. time into the binary heap, ensures that we are currently dealing with the best path
            }
    
            if cost > c {
                continue;
            }
            successors(node)
        };

        for (successor, move_cost) in successors {
            let new_cost = cost + move_cost;
            let h;          // heuristic!
            let n;          // index for successor node
            match parents.entry(successor) {
                Vacant(e) => {
                    h = heuristic(e.key());
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = heuristic(e.key());
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_see.push(SmallestCostHolder {
                estimated_cost: new_cost + h,
                cost: new_cost,
                index: n,
            });
        }
    }
    None
}

/// Iterator structure to get the best path (hopefully)
#[derive(Clone)]
pub struct AstarSolution<N> {
    sinks: Vec<usize>,
    parents: Vec<(N, Vec<usize>)>, //parent nodes
    current: Vec<Vec<usize>>,      //current nodes
    terminated: bool,               // determines if something is to be cut from the path
}

impl<N: Clone + Eq + Hash> AstarSolution<N> { 
    fn complete(&mut self) {
        loop {
            let ps = match self.current.last() {
                None => self.sinks.clone(),
                Some(last) => {
                    let &top = last.last().unwrap();
                    self.parents(top).clone()
                }
            };
            if ps.is_empty() {
                break;
            }
            self.current.push(ps);
        }
    }

    fn next_vec(&mut self) {
        while self.current.last().map(Vec::len) == Some(1) {
            self.current.pop();
        }
        self.current.last_mut().map(Vec::pop);
    }

    fn node(&self, i: usize) -> &N {
        &self.parents[i].0
    }

    fn parents(&self, i: usize) -> &Vec<usize> {
        &self.parents[i].1
    }
}

impl<N: Clone + Eq + Hash> Iterator for AstarSolution<N> {
    type Item = Vec<N>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.terminated {
            return None;
        }
        self.complete();
        let path = self
            .current
            .iter()
            .rev()
            .map(|v| v.last().cloned().unwrap())
            .map(|i| self.node(i).clone())
            .collect::<Vec<_>>();
        self.next_vec();
        self.terminated = self.current.is_empty();
        Some(path)          //returns some path, which is the most efficient path!
    }
}