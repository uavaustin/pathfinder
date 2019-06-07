use super::*;

// Simple wrapper around heap and set for efficient data retrival
pub struct Queue {
    heap: BinaryHeap<Arc<RefCell<Vertex>>>, // Efficiently get min
    set: HashSet<i32>,                      // Efficiently check of existence
}

impl Queue {
    pub fn new() -> Self {
        Queue {
            heap: BinaryHeap::new(),
            set: HashSet::new(),
        }
    }

    // Insert to queue
    pub fn push(&mut self, vertex: Arc<RefCell<Vertex>>) {
        self.set.insert(vertex.borrow().index);
        self.heap.push(vertex);
    }

    // Return min from queue
    pub fn pop(&mut self) -> Option<Arc<RefCell<Vertex>>> {
        self.heap.pop()
    }

    pub fn contains(&self, vertex: &Arc<RefCell<Vertex>>) -> bool {
        self.set.contains(&vertex.borrow().index)
    }
}
