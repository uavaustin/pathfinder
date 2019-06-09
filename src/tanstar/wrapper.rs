extern crate parking_lot;

use self::parking_lot::{ReentrantMutex, ReentrantMutexGuard};
use std::cell::RefCell;
use std::cmp::Ordering;
use std::sync::Arc;

#[derive(Debug)]
pub struct Wrapper<T>(Arc<ReentrantMutex<RefCell<T>>>);

impl<T: Eq> Eq for Wrapper<T> {}

impl<T: PartialEq> PartialEq for Wrapper<T> {
    fn eq(&self, other: &Self) -> bool {
        // *self.lock().borrow().eq(*other.lock().borrow())
        let _self = self.lock();
        let _other = other.lock();
        let eq = T::eq(&_self.borrow(), &_other.borrow());
        eq
    }
}

impl<T: Ord> Ord for Wrapper<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        let _self = self.lock();
        let _other = other.lock();
        let order = T::cmp(&_self.borrow(), &_other.borrow());
        order
    }
}

impl<T: PartialOrd> PartialOrd for Wrapper<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let _self = self.lock();
        let _other = other.lock();
        let p_order = T::partial_cmp(&_self.borrow(), &_other.borrow());
        p_order
    }
}

impl<T> Clone for Wrapper<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T> Wrapper<T> {
    pub fn new(n: T) -> Self {
        Self(Arc::new(ReentrantMutex::new(RefCell::new(n))))
    }

    pub fn lock(&self) -> ReentrantMutexGuard<RefCell<T>> {
        self.0.lock()
    }
}
