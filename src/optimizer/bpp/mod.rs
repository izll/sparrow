//! Bin Packing Problem (BPP) pipeline.
//!
//! Mirrors the strip packing (SPP) pipeline in [`crate::optimizer`], but packs the items into
//! copies of fixed-size bins ([`jagua_rs::probs::bpp`]) instead of a single strip of variable width.
//!
//! The container-agnostic core (sampling, evaluation, collision quantification) is shared with the
//! SPP path: [`crate::sample::search::search_placement`], [`crate::eval::sep_evaluator::SeparationEvaluator`]
//! and [`crate::quantify::tracker::CollisionTracker`] are reused as-is. The only structural difference
//! is that a BPP solution consists of *multiple* layouts, so the separator keeps **one collision
//! tracker per open layout** and the "total loss" is the sum over all of them.
//!
//! Phase 1 (this module) contains the foundation:
//! * [`lbf::BPLBFBuilder`] — constructive first solution,
//! * [`separator::BPSeparator`] — the separation loop (SPP Algorithm 9 over all layouts),
//! * [`worker::BPSeparatorWorker`] — the parallel move workers (SPP Algorithm 5 per layout).
//!
//! The bin-count reduction loop (`explore`), the remainder consolidation (`compress`) and the
//! `optimize_bpp()` orchestration are phase 2.

pub mod lbf;
pub mod separator;
pub mod worker;

#[doc(inline)]
pub use lbf::BPLBFBuilder;
#[doc(inline)]
pub use separator::{BPSeparator, BPSnapshot};
#[doc(inline)]
pub use worker::BPSeparatorWorker;
