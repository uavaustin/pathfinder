// algorithm.rs
// trait that defines minimum requirement for a pathfinding algorithm

//! The Algorithm traits have become a little weird and deserve explanation.
//!
//! At this point we have five Algorithm traits:
//!   - Algorithm
//!   - AlgorithmConstructor
//!   - AlgorithmAdjustPath
//!   - AlgorithmAdjustPathQualified
//!   - AlgorithmFields
//!
//! If you're looking to create an Algorithm, implement Algorithm{Constructor,
//! AdjustPath, Fields} and add your Algorithm to the list in lib.rs and you
//! should be good to go. You'll get an Algorithm implementation for free and
//! you'll never have to worry about the stuff below.
//!
//! _Why_ we have all these traits is a little more complicated, but it's almost entirely because of Trait Objects and the Object Safety rules.
//!
//! Trait Objects come into play because there are situations in which it's
//! desirable to be able to pick a particular Algorithm implementation _at
//! runtime_. Allowing this means that we don't know the exact type of the
//! Algorithm implementation being used at compile time which means we have to
//! use Trait Objects.
//!
//! The Object Safety Rules state that we aren't allowed to return Self in any
//! trait methods (`new` violates this) and that we aren't allowed to have any
//! generic methods in the trait (`adjust_path` violates this). So, to get
//! around this we broke Algorithm up into separate traits, created a
//! qualified version of `adjust_path` (you have to specify the Waypoint's
//! associated data type as a trait type parameter) with a blanket impl, and
//! then did some stuff to make sure that writing Algorithms and using them
//! isn't too cumbersome.
//!
//! We also did some stuff to provide guarantees the list of Algorithms this
//! crate knows about (and provides when the "restricted-algorithm-types"
//! feature is enabled) is an accurate list.
//!
//! Details are below.
//!
//! We're trying to ensure two things:
//!   - that all impls of Algorithm have impls of Algorithm{Constructor,
//!    AdjustPath, Fields} with the same Config type
//!   - that Algorithm is a sealed trait (i.e. no type outside of this crate
//!    can implement it)
//!
//! The following, unfortunately, does not work (infinitely recurses):
//! ```compile_fail
//! pub trait Algorithm
//! where
//!     Self: AlgorithmConstructor<Config = <Self as Algorithm>::Config>,
//!     Self: AlgorithmAdjustPath<Config = <Self as Algorithm>::Config>,
//!     Self: AlgorithmFields<Config = <Self as Algorithm>::Config> {}
//! ```
//!
//! We can ensure that all impls that come from our blanket impl meet the first requirement:
//! ```
//! impl<A, C: Default> Algorithm for A
//! where
//!     A: AlgorithmConstructor<Config = C>,
//!     A: AlgorithmAdjustPath<Config = C>,
//!     A: AlgorithmFields<Config = C>
//! {
//!     type Config = C;
//! }
//! ```
//! but this doesn't prevent us from going and implementing Algorithm (within
//! this crate) for mismatched Config types (afaik, there's no way to restrict
//! impls of a trait to just blanket impls or anything like that).
//!
//! Defaults on Trait type parameters may save us:
//! ```
//! pub trait Algorithm<C: Default = <Self as AlgorithmFields>::Config>
//! where
//!     Self: AlgorithmConstructor<Config = C>,
//!     Self: AlgorithmAdjustPath<Config = C>,
//!     Self: AlgorithmFields<Config = C>,
//! {
//!     type Config: Default;
//!     // Unfortunately Associated Type Defaults are unstable (#29661) so we
//!     // can't do:
//!     // `type Config: Default = C;`
//!     // Where clauses on Associated Types are also unstable (#44265 - GATs) so
//!     // we also can't do this (this is what we really want):
//!     // `type Conf: Default where Self: AlgorithmFields<Config = <Self as Algorithm>::Config>;`
//!     //
//!     // This means that we can't ensure that the associated Config type matches
//!     // the Config type of the three other Algorithm traits. This isn't great
//!     // but it's still livable since:
//!     //   - really no one should implement Algorithm (blanket impl)
//!     //   - you'll still get type errors if you rely on Algorithm::Config and there's a mismatch
//!     //   - Algorithm::Config is mostly there for compatibility reasons
//! }
//! ```
//!
//! Another option is to ditch the type parameter on Algorithm and use transitivity:
//! ```
//! pub trait Algorithm
//! where
//!     Self: AlgorithmAdjustPath + AlgorithmConstructor + AlgorithmFields,
//!     // A == B == C -> (A == B, B == C)
//!     Self: AlgorithmAdjustPath<Config = <Self as AlgorithmConstructor>::Config>,
//!     Self: AlgorithmConstructor<Config = <Self as AlgorithmFields>::Config>,
//!     {
//!     type Config: Default;
//! }
//! ```
//! In practice this shouldn't really be different than the above, but let's
//! go with the above (default generic type parameter) for now - it's a little
//! cleaner.
//!
//! The 2nd requirement is easy enough to satisfy:
//! ```
//! pub(crate) trait Sealed { }
//!
//! // Blessed Algorithms:
//! impl Sealed for SomeAlgorithm { }
//!
//! // The above plus an additional super trait bound on Algorithm:
//! pub trait Algorithm<C: Default = <Self as AlgorithmFields>::Config>
//! where
//!     Self: private::Sealed
//!     <snipped>
//! ```
//! This is nearly verbatim the pattern described [here](https://rust-lang-nursery.github.io/api-guidelines/future-proofing.html#sealed-traits-protect-against-downstream-implementations-c-sealed).
//! It requires us to explicitly 'bless' Algorithms within this crate, which
//! isn't great does make it so we have an authoritative list of all the
//! existing Algorithm implementations.
//!
//! The blessing is done in lib.rs. To ease the pain a little bit, we only
//! require the blessing when the "restrict-algorithm-types" feature is
//! enabled. When it isn't, foreign Algorithm implementations are possible.

use super::*;

// pub trait AlgorithmConstructor<C: Default> {
//     fn new() -> Self;
// }

// pub trait AlgorithmAdjustPath<C: Default> {
//     fn adjust_path<T>(&mut self, start: Location, end: Location)
//         -> Option<LinkedList<Waypoint<T>>>;
// }

// pub trait AlgorithmAdjustPathQualified<C: Default, T> {
//     fn adjust_path(&mut self, start: Location, end: Location)
//         -> Option<LinkedList<Waypoint<T>>>;
// }

// impl<C: Default, A: AlgorithmAdjustPath<C>, T> AlgorithmAdjustPathQualified<C, T> for A {
//     fn adjust_path(&mut self, start: Location, end: Location)
//         -> Option<LinkedList<Waypoint<T>>>
//         {
//             <A as AlgorithmAdjustPath>::adjust_path::<T>(self, start, end)
//         }
// }

// pub trait AlgorithmFields<C: Default> {
//     fn init(
//         &mut self,
//         config: C,
//         flyzones: Vec<Vec<Location>>,
//         obstacles: Vec<Obstacle>,
//     );

//     // Getters
//     fn get_config(&self) -> &C;
//     fn get_flyzone(&self) -> &Vec<Vec<Location>>;
//     fn get_obstacles(&self) -> &Vec<Obstacle>;

//     // Setters
//     fn set_config(&mut self, config: C);
//     fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>);
//     fn set_obstacles(&mut self, obstacles: Vec<Obstacle>);
// }


pub trait AlgorithmConstructor {
    type Config: Default;

    fn new() -> Self;
}

pub trait AlgorithmAdjustPath {
    type Config: Default;

    fn adjust_path<T>(&mut self, start: Location, end: Location)
        -> Option<LinkedList<Waypoint<T>>>;
}

pub trait AlgorithmAdjustPathQualified<T> {
    type Config: Default;

    fn adjust_path(&mut self, start: Location, end: Location)
        -> Option<LinkedList<Waypoint<T>>>;
}

impl<A: AlgorithmAdjustPath, T> AlgorithmAdjustPathQualified<T> for A {
    type Config = A::Config;

    fn adjust_path(&mut self, start: Location, end: Location)
        -> Option<LinkedList<Waypoint<T>>>
        {
            <A as AlgorithmAdjustPath>::adjust_path::<T>(self, start, end)
        }
}

pub trait AlgorithmFields {
    type Config: Default;

    fn init(
        &mut self,
        config: <Self as AlgorithmFields>::Config,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    );

    // Getters
    fn get_config(&self) -> &<Self as AlgorithmFields>::Config;
    fn get_flyzone(&self) -> &Vec<Vec<Location>>;
    fn get_obstacles(&self) -> &Vec<Obstacle>;

    // Setters
    fn set_config(&mut self, config: <Self as AlgorithmFields>::Config);
    fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>);
    fn set_obstacles(&mut self, obstacles: Vec<Obstacle>);
}


// trait AlgorithmConfig<C: Default = <Self as AlgorithmConstructor>::Config>
// where
//     Self: AlgorithmConstructor<Config = C>,
//     Self: AlgorithmAdjustPath<Config = C>,
//     Self: AlgorithmFields<Config = C> {}

// impl<A, C: Default> AlgorithmConfig<C> for A
// where
//     A: AlgorithmConstructor<Config = C>,
//     A: AlgorithmAdjustPath<Config = C>,
//     A: AlgorithmFields<Config = C> {}


// pub trait Algorithm<C: Default = <Self as AlgorithmConstructor>::Config>
// where
//     Self: AlgorithmAdjustPath + AlgorithmConstructor + AlgorithmFields,
//     Self: AlgorithmAdjustPath<Config = C>,
//     Self: AlgorithmConstructor<Config = C>,
//     Self: AlgorithmFields<Config = C>,
//     Self: Sealed,
// {
//     type Config: Default;
//     // Unfortunately Associated Type Defaults are unstable (#29661) so we
//     // can't do:
//     // `type Config: Default = C;`
//     // Where clauses on Associated Types are also unstable (#44265 - GATs) so
//     // we also can't do this (this is what we really want):
//     // `type Config: Default where Self: AlgorithmFields<Config = <Self as Algorithm>::Config>;`
//     //
//     // This means that we can't ensure that the associated Config type matches
//     // the Config type of the three other Algorithm traits. This isn't great
//     // but it's still livable since:
//     //   - really no one should implement Algorithm (blanket impl)
//     //   - you'll still get type errors if you rely on Algorithm::Config and there's a mismatch
//     //   - Algorithm::Config is mostly there for compatibility reasons
// }

pub trait Algorithm
where
    Self: AlgorithmAdjustPath + AlgorithmConstructor + AlgorithmFields,
    // A == B == C -> (A == B, B == C)
    Self: AlgorithmAdjustPath<Config = <Self as AlgorithmConstructor>::Config>,
    Self: AlgorithmConstructor<Config = <Self as AlgorithmFields>::Config>,
    Self: Sealed,
    // Self: AlgorithmConstructor<Config = <Self as Algorithm>::Config>,
    // Self: AlgorithmAdjustPath<Config = <Self as Algorithm>::Config>,
    // Self: AlgorithmFields<Config = <Self as Algorithm>::Config>
// where
    // Self: AlgorithmConfig<<Self as Algorithm>::Config>,
    {
    type Config: Default;
    // type Config: Default where Self: AlgorithmFields<Config = <Self as Algorithm>::Config>;
}

impl<A, C: Default> Algorithm for A
where
    A: AlgorithmConstructor<Config = C>,
    A: AlgorithmAdjustPath<Config = C>,
    A: AlgorithmFields<Config = C>,
    A: Sealed,
{
    type Config = C;
}

pub trait AlgorithmTy<C: Default>: Algorithm<Config = C> + AlgorithmFields<Config = C> + AlgorithmAdjustPath<Config = C> {
    type Config: Default;
}

impl<A, C: Default> AlgorithmTy<C> for A
where
    A: AlgorithmConstructor + AlgorithmAdjustPath + AlgorithmFields + Algorithm,
    A: AlgorithmConstructor<Config = C>,
    A: AlgorithmAdjustPath<Config = C>,
    A: AlgorithmFields<Config = C>,
    A: Algorithm<Config = C>,
{
    type Config = C;
}

// pub trait Algorithm
// where
//     // Self: AlgorithmAdjustPath + AlgorithmConstructor + AlgorithmFields,
//     // A == B == C -> (A == B, B == C)
//     // Self: AlgorithmAdjustPath<Config = <Self as AlgorithmConstructor>::Config>,
//     // Self: AlgorithmConstructor<Config = <Self as AlgorithmFields>::Config>,
//     Self: AlgorithmConstructor<<Self as Algorithm>::Config>,
//     Self: AlgorithmAdjustPath<<Self as Algorithm>::Config>,
//     Self: AlgorithmFields<<Self as Algorithm>::Config>
// // where
//     // Self: AlgorithmConfig<<Self as Algorithm>::Config>,
//     {
//     type Config: Default;
//     // type Config: Default where Self: AlgorithmFields<Config = <Self as Algorithm>::Config>;
// }

// impl<A, C: Default> Algorithm for A
// where
//     A: AlgorithmConstructor<C>,
//     A: AlgorithmAdjustPath<C>,
//     A: AlgorithmFields<C>,
//     A: Sealed,
// {
//     type Config = C;
// }


// pub trait AlgorithmP {
//     type Config: Default;

//     fn new() -> Self;

//     fn init(
//         &mut self,
//         config: Self::Config,
//         flyzones: Vec<Vec<Location>>,
//         obstacles: Vec<Obstacle>,
//     );
//     fn adjust_path<T>(&mut self, start: Location, end: Location)
//         -> Option<LinkedList<Waypoint<T>>>;

//     // Getters
//     fn get_config(&self) -> &Self::Config;
//     fn get_flyzone(&self) -> &Vec<Vec<Location>>;
//     fn get_obstacles(&self) -> &Vec<Obstacle>;

//     // Setters
//     fn set_config(&mut self, config: Self::Config);
//     fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>);
//     fn set_obstacles(&mut self, obstacles: Vec<Obstacle>);
// }
