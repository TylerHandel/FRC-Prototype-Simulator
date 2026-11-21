//! CAD in (PLAN.md D8, D9, D20): reading a cadpack, Onshape, proposals, re-import.
pub mod cadpack;
pub mod hull;
pub mod import;
pub mod onshape;
pub mod propose;

pub use cadpack::Cadpack;
