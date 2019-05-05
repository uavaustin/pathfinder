use super::*;

impl Queue {
    pub fn new() -> Self {
        Queue {
            heap: BinaryHeap::new(),
            set: HashSet::new(),
        }
    }

    // Insert to queue
    pub fn push(&mut self, vertex: Rc<RefCell<Vertex>>) {
        self.set.insert(vertex.borrow().index);
        self.heap.push(vertex);
    }

    // Return min from queue
    pub fn pop(&mut self) -> Option<Rc<RefCell<Vertex>>> {
        self.heap.pop()
    }

    pub fn contains(&self, vertex: &Rc<RefCell<Vertex>>) -> bool {
        self.set.contains(&vertex.borrow().index)
    }
}
