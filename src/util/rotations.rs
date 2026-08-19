//! **The one rotation grid.** Every place in the code base that has to answer "which rotations can
//! this item actually be placed at?" asks this module, so that the answers cannot disagree.
//!
//! They used to. [`UniformBBoxSampler`](crate::sample::uniform_sampler::UniformBBoxSampler) — the
//! only thing that ever *proposes* a rotation to the engine — samples a continuously rotatable item
//! on a **16-step** grid, while the packability pre-check
//! ([`crate::util::packability::items_too_tall_for_strip`]) and the walled-mode pre-check
//! ([`crate::optimizer::sheets::items_too_wide_for_sheet`]) each kept their own private **24-step**
//! copy. Two grids that share only the multiples of 45° is not a rounding difference, it is a
//! different set of allowed placements, and the pre-checks are *rejection* gates: whatever they
//! cannot fit, the program refuses to solve at all.
//!
//! The audited failure: a 100 x 10 rectangle pre-rotated by 22.5°, in a 10.2 mm strip. The sampler's
//! grid contains -22.5° (= 337.5°), which un-rotates it into a 10.0 mm tall box that fits. The
//! 24-step grid's nearest sample is 7.5° away, so the pre-check computed a minimum height of 23 mm
//! and killed the run with `the strip is only 10.2 mm high, but item 0 (23.0 mm) does not fit` —
//! an input the engine solves without trouble, and did solve before the pre-check existed.
//!
//! Hence: [`candidate_rotations`] is *the* grid, the sampler uses it, and the pre-checks use it too.
//! A pre-check that reasons about a rotation the sampler never proposes is answering the wrong
//! question in both directions — it can reject the solvable, and it can pass what cannot be reached.

use jagua_rs::entities::Item;
use jagua_rs::geometry::geo_enums::RotationRange;
use std::f32::consts::PI;

/// Number of rotations sampled for a continuously rotatable item.
///
/// Shared by the sampler and every pre-check; changing it changes both at once, which is the whole
/// point of it living here.
pub const ROT_N_SAMPLES: usize = 16;

/// The rotations (radians) an item may actually be **placed at** by this engine.
///
/// * [`RotationRange::None`] ⇒ just `0.0`.
/// * [`RotationRange::Discrete`] ⇒ exactly the declared set, in declaration order.
/// * [`RotationRange::Continuous`] ⇒ the [`ROT_N_SAMPLES`]-step uniform grid over `[0, 2π)`. Note
///   that this is a *sample* of a continuum: the engine can place such an item at any angle it is
///   handed (see [`crate::sample::uniform_sampler::convert_sample_to_closest_feasible`]), but the
///   only angles it ever proposes on its own are these. A pre-check must therefore never treat a
///   continuous item as *restricted* to this grid — see [`fits_when_continuous`].
pub fn candidate_rotations(item: &Item) -> impl Iterator<Item = f32> + '_ {
    enum Rots<'a> {
        Fixed(std::iter::Once<f32>),
        Discrete(std::slice::Iter<'a, f32>),
        Grid(std::ops::Range<usize>),
    }
    impl Iterator for Rots<'_> {
        type Item = f32;
        fn next(&mut self) -> Option<f32> {
            match self {
                Rots::Fixed(it) => it.next(),
                Rots::Discrete(it) => it.next().copied(),
                Rots::Grid(r) => r.next().map(|i| i as f32 * (2.0 * PI) / ROT_N_SAMPLES as f32),
            }
        }
    }

    match &item.allowed_rotation {
        RotationRange::None => Rots::Fixed(std::iter::once(0.0)),
        RotationRange::Discrete(r) => Rots::Discrete(r.iter()),
        RotationRange::Continuous => Rots::Grid(0..ROT_N_SAMPLES),
    }
}

/// Whether an item is **continuously** rotatable, i.e. whether [`candidate_rotations`] is a sample
/// of a continuum rather than the complete set of legal angles.
///
/// The rejection pre-checks use this to stay on the safe side: for such an item the grid proves
/// *fitting* (a sampled rotation that fits is a rotation the engine can reach) but never proves
/// *non-fitting*, because an angle between two samples may still fit.
pub fn is_continuous(item: &Item) -> bool {
    matches!(item.allowed_rotation, RotationRange::Continuous)
}

/// Convenience for the pre-checks: `true` when the item is continuous, so a "does not fit" verdict
/// derived from the sampled grid must not be turned into a hard rejection.
pub fn fits_when_continuous(item: &Item) -> bool {
    is_continuous(item)
}

/// Smallest positive difference between `a` and `b` **as angles**, in radians (result in `[0, π]`).
pub fn angle_delta(a: f32, b: f32) -> f32 {
    let two_pi = 2.0 * PI;
    let d = (a - b).rem_euclid(two_pi);
    d.min(two_pi - d)
}

/// Whether `rotation` (radians) is one of the item's **allowed** rotations, modulo 2π, within
/// `tol_rad`.
///
/// This is the question the export gate asks, and it is deliberately *not* the same question as
/// [`candidate_rotations`]: a continuous item may legally sit at any angle, including one that is
/// nowhere near the sampling grid (the separator's moves and
/// [`convert_sample_to_closest_feasible`](crate::sample::uniform_sampler::convert_sample_to_closest_feasible)
/// both produce such angles).
pub fn rotation_is_allowed(item: &Item, rotation: f32, tol_rad: f32) -> bool {
    match &item.allowed_rotation {
        RotationRange::Continuous => rotation.is_finite(),
        RotationRange::None => rotation.is_finite() && angle_delta(rotation, 0.0) <= tol_rad,
        RotationRange::Discrete(allowed) => {
            rotation.is_finite() && allowed.iter().any(|&a| angle_delta(rotation, a) <= tol_rad)
        }
    }
}

/// Whether an **external** (JSON-level) rotation in *degrees* is allowed by an item's
/// `allowed_orientations` field, using **exactly** jagua-rs' import semantics
/// ([`jagua_rs::io::import`]):
///
/// | `allowed_orientations` | meaning |
/// |---|---|
/// | absent / `null`        | `RotationRange::Continuous` — any angle |
/// | `[]`                   | `RotationRange::None` — fixed at 0° |
/// | `[0.0]`                | `RotationRange::None` — fixed at 0° |
/// | `[a, b, ...]`          | `RotationRange::Discrete` — one of those angles |
///
/// The empty list mapping to "fixed 0°" rather than "nothing allowed" is jagua's, not a guess: an
/// item that may be placed at no angle at all could never be packed, so the only reading that makes
/// an instance solvable is the one jagua implements. `scripts/validate_solution.py` reads the same
/// table.
///
/// The comparison is modulo 360°, because the engine exports the angle it happens to hold: an item
/// declared `[0, 180]` is routinely written out as `-180`.
pub fn ext_orientation_ok(rotation_deg: f32, allowed_deg: Option<&[f32]>, tol_deg: f32) -> bool {
    if !rotation_deg.is_finite() {
        return false;
    }
    match allowed_deg {
        None => true,
        Some([]) => angle_delta_deg(rotation_deg, 0.0) <= tol_deg,
        Some([a]) if *a == 0.0 => angle_delta_deg(rotation_deg, 0.0) <= tol_deg,
        Some(list) => list.iter().any(|&a| angle_delta_deg(rotation_deg, a) <= tol_deg),
    }
}

/// [`angle_delta`] in degrees.
pub fn angle_delta_deg(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(360.0);
    d.min(360.0 - d)
}

/// Human-readable description of an item's allowed rotations, in **degrees**, for error messages.
pub fn describe_allowed(item: &Item) -> String {
    match &item.allowed_rotation {
        RotationRange::Continuous => "any (continuous rotation)".to_string(),
        RotationRange::None => "0° only".to_string(),
        RotationRange::Discrete(a) => a
            .iter()
            .map(|r| format!("{:.3}°", r.to_degrees()))
            .collect::<Vec<_>>()
            .join(", "),
    }
}
