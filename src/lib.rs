//! This library provides a framework for obstacle avoidance and implements the tan* algorithm.

#![allow(dead_code)]
#![allow(unused_variables)]
#![deny(missing_debug_implementations)]

pub mod algorithm;
pub mod obj;
pub mod pathfinder;
pub mod tanstar;

pub use algorithm::{
    Algorithm, AlgorithmAdjustPath, AlgorithmAdjustPathQualified, AlgorithmConfig,
    AlgorithmConstructor, AlgorithmFields,
};
pub use obj::{Location, Obstacle, Plane, Waypoint};
pub use pathfinder::Pathfinder;
pub use tanstar::{TConfig, Tanstar};

use private::Sealed;

#[macro_use]
extern crate cfg_if;

/// Marker Trait used for 'sealed' Traits (this cannot be implemented outside
/// of this trait and thus any Trait that uses Sealed as a supertrait is
/// sealed - all it's implementations are within this crate).
mod private {
    pub trait Sealed {}
}

cfg_if! {
    if #[cfg(feature = "restrict-algorithm-types")] {
        #[macro_use]
        extern crate lazy_static;
        use std::any::TypeId;
        use std::collections::HashMap;

        macro_rules! blessed_algorithm {
            ($m: ident, $algo: ty, $alias: tt) => {
                impl private::Sealed for $algo { }
                $m.insert(stringify!($alias), TypeId::of::<$algo>());
            };
        }

        lazy_static! {
            pub static ref ALGORITHMS: HashMap<&'static str, TypeId> = {
                let mut m = HashMap::new();

                blessed_algorithm!(m, Tanstar, "tanstar");

                m
            };
        }

        // pub trait TypeContainer {
        //     type Type: Algorithm;
        // }

        // pub struct TS; impl TypeContainer for TS { type Type = Tanstar; }

        // pub enum Algos {
        //     TanStar(TS),
        // }

        // pub fn string_to_type(s: String) -> Algos {
        //     match s.as_str() {
        //         "tanstar" => Algos::TanStar(TS),
        //         _ => Algos::TanStar(TS),
        //     }
        // }

        // pub fn string_to_ctor(s: String) -> impl Algorithm {
        //     match s.as_str() {
        //         "tanstar" => Tanstar::new(),
        //         _ => Tanstar::new(),
        //     }
        // }
    } else {
        // If we don't care about having an authoritative list of Algorithm
        // implementations, blanket impl Sealed for all the implementors that
        // match the criteria. In other words, allow all implementors of the
        // Algorithm traits, internal and external, to implement Algorithm.
        impl<A: AlgorithmConfig + AlgorithmFields + AlgorithmConstructor + AlgorithmAdjustPath> Sealed for A { }
    }
}
