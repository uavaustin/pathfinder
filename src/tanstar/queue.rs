use super::*;

use std::collections::BinaryHeap;

// Simple wrapper around heap and set for efficient data retrival
pub struct Queue {
    heap: BinaryHeap<Wrapper<Vertex>>, // Efficiently get min
    set: HashSet<i32>,                 // Efficiently check of existence
}

impl Queue {
    pub fn new() -> Self {
        Queue {
            heap: BinaryHeap::new(),
            set: HashSet::new(),
        }
    }

    // Insert to queue
    pub fn push(&mut self, vertex: Wrapper<Vertex>) {
        {
            let _vertex = vertex.lock();
            self.set.insert(_vertex.borrow().index);
        }
        self.heap.push(vertex);
    }

    // Return min from queue
    pub fn pop(&mut self) -> Option<Wrapper<Vertex>> {
        self.heap.pop()
    }

    pub fn contains(&self, vertex: &Wrapper<Vertex>) -> bool {
        let _vertex = vertex.lock();
        let index = _vertex.borrow().index;
        self.set.contains(&index)
    }
}
