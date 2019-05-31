// lib.rs
// contains exposed functionality of the library
#![allow(dead_code)]
#![allow(unused_variables)]

pub mod obj;
pub mod tanstar;

pub mod algorithm;

use private::Sealed;
pub use obj::{Location, Obstacle, Plane, Waypoint};
pub use tanstar::{TConfig, Tanstar};
pub use algorithm::{Algorithm, AlgorithmTy, AlgorithmConstructor, AlgorithmFields, AlgorithmAdjustPath, AlgorithmAdjustPathQualified};

use std::collections::LinkedList;

#[macro_use]
extern crate cfg_if;

/// Marker Trait used for 'sealed' Traits (this cannot be implemented outside
/// of this trait and thus any Trait that uses Sealed as a supertrait is
/// sealed - all it's implementations are within this crate).
mod private { pub trait Sealed { } }

cfg_if! {
    if #[cfg(feature = "restrict-algorithm-types")] {
        #[macro_use]
        extern crate lazy_static;
        use std::any::TypeId;
        use std::collections::HashMap;

        // macro_rules! blessed_algorithms {
        //     () => {};
        // }

        lazy_static! {
            pub static ref ALGORITHMS: HashMap<&'static str, TypeId> = {
                let mut m = HashMap::new();
                m.insert("tanstar", TypeId::of::<Tanstar>());
                m
            };
        }

        pub trait TypeContainer {
            type Type: Algorithm;
        }

        pub struct TS; impl TypeContainer for TS { type Type = Tanstar; }

        pub enum Algos {
            TanStar(TS),
        }

        pub fn string_to_type(s: String) -> Algos {
            match s.as_str() {
                "tanstar" => Algos::TanStar(TS),
                _ => Algos::TanStar(TS),
            }
        }


        pub fn string_to_ctor(s: String) -> impl Algorithm {
            match s.as_str() {
                "tanstar" => Tanstar::new(),
                _ => Tanstar::new(),
            }
        }
    } else {
        // If we don't care about having an authoritative list of Algorithm
        // implementations, blanket impl Sealed for all the implementors that
        // match the criteria. In other words, allow all implementors of the
        // Algorithm traits, internal and external, to implement Algorithm.
        impl<A: AlgorithmFields + AlgorithmConstructor + AlgorithmAdjustPath> Sealed for A { }
        // impl<C: Default, A: AlgorithmFields<C> + AlgorithmConstructor<C> + AlgorithmAdjustPath<C>> Sealed for A { }

    }
}

pub struct Pathfinder<A: Algorithm> {
    algo: A,
}

impl<C: Default, A: AlgorithmTy<C> + Algorithm<Config = C>> Pathfinder<A> {
    pub fn new(
        mut algo: A,
        config: <A as Algorithm>::Config,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        algo.init(config, flyzones, obstacles);
        Self { algo: algo }
    }

    pub fn get_adjust_path<T>(
        &mut self,
        plane: Plane,
        mut wp_list: LinkedList<Waypoint<T>>,
    ) -> LinkedList<Waypoint<T>> {
        let mut new_wp_list = LinkedList::new();
        let mut current_wp: Waypoint<T>;
        let mut current_loc = plane.location;

        loop {
            match wp_list.pop_front() {
                Some(wp) => current_wp = wp,
                None => break,
            }

            let next_loc = current_wp.location.clone();

            if let Some(mut path) = self.algo.adjust_path::<T>(current_loc, next_loc) {
                println!("appending");
                new_wp_list.append(&mut path);
            } else {
                println!("no path");
                break;
            }

            current_loc = current_wp.location;
            new_wp_list.push_back(current_wp);
            // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint
            // self.wp_list.push_back(Waypoint::from_degrees(0, 30.69, -97.69, 100f32, 10f32));
        }

        new_wp_list
    }

    pub fn set_config(&mut self, config: <A as AlgorithmFields>::Config) {
        self.algo.set_config(config);
    }

    pub fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>) {
        self.algo.set_flyzone(flyzone);
    }

    pub fn set_obstacles(&mut self, obstacles: Vec<Obstacle>) {
        self.algo.set_obstacles(obstacles);
    }

    pub fn get_config(&self) -> &<A as AlgorithmFields>::Config {
        self.algo.get_config()
    }

    pub fn get_flyzone(&mut self) -> &Vec<Vec<Location>> {
        self.algo.get_flyzone()
    }

    pub fn get_obstacle(&self) -> &Vec<Obstacle> {
        self.algo.get_obstacles()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn tanstar_invalid_flyzones_test() {
        Pathfinder::new(Tanstar::new(), TConfig::default(), vec![], Vec::new());
    }

    #[test]
    #[should_panic]
    fn tanstar_invalid_flyzone_test() {
        Pathfinder::new(Tanstar::new(), TConfig::default(), vec![vec![]], Vec::new());
    }
}
