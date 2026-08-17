use crate::quantify::circles_soa::{CirclesSoA, SOA_LANES};
use jagua_rs::geometry::fail_fast::SPSurrogate;
use jagua_rs::geometry::geo_traits::DistanceTo;
use std::f32::consts::PI;

/// Calculates a proxy for the overlap area between two simple polygons (using poles).
/// Algorithm 3 from https://doi.org/10.48550/arXiv.2509.13329
#[inline(always)]
pub fn overlap_area_proxy(sp1: &SPSurrogate, sp2: &SPSurrogate, epsilon: f32) -> f32 {
    let mut total_overlap = 0.0;
    for p1 in &sp1.poles {
        for p2 in &sp2.poles {
            // Penetration depth between the two poles (circles)
            let pd = (p1.radius + p2.radius) - p1.center.distance_to(&p2.center);

            let pd_decay = match pd >= epsilon {
                true => pd,
                false => epsilon.powi(2) / (-pd + 2.0 * epsilon),
            };

            total_overlap += pd_decay * f32::min(p1.radius, p2.radius);
        }
    }
    total_overlap *= PI;
    debug_assert!(total_overlap.is_normal());
    
    total_overlap
}

/// Same as [`overlap_area_proxy`], but the poles of the second surrogate are provided in SoA layout (`p2`).
///
/// The inner loop is written branch-free over fixed-size chunks of [`SOA_LANES`] with independent per-lane accumulators,
/// which allows LLVM to auto-vectorize it on stable Rust (no `-ffast-math`/reassociation needed since the
/// lanes are never summed until the very end). With `target-cpu=native` (AVX2) this yields 8-wide `vsqrtps`/`vdivps`.
///
/// `p2` must have been loaded from `sp2.poles` (see [`CirclesSoA::load`]).
#[inline(always)]
pub fn overlap_area_proxy_soa(sp1: &SPSurrogate, p2: &CirclesSoA, epsilon: f32) -> f32 {
    let e_sq = epsilon * epsilon;
    let two_e = 2.0 * epsilon;

    // Fixed-size chunk views, so all indexing inside the lane loop is bounds-check free.
    let (xs, xr) = p2.x.as_chunks::<SOA_LANES>();
    let (ys, yr) = p2.y.as_chunks::<SOA_LANES>();
    let (rs, rr) = p2.r.as_chunks::<SOA_LANES>();
    debug_assert!(xr.is_empty() && yr.is_empty() && rr.is_empty(), "SoA buffers must be padded to a multiple of SOA_LANES");
    debug_assert!(xs.len() == ys.len() && ys.len() == rs.len());
    let n_chunks = xs.len().min(ys.len()).min(rs.len());

    let mut acc = [0.0f32; SOA_LANES];

    for p1 in sp1.poles.iter() {
        let x1 = p1.center.0;
        let y1 = p1.center.1;
        let r1 = p1.radius;

        for c in 0..n_chunks {
            let (x2, y2, r2) = (&xs[c], &ys[c], &rs[c]);
            for l in 0..SOA_LANES {
                let dx = x1 - x2[l];
                let dy = y1 - y2[l];
                // Penetration depth between the two poles (circles)
                let pd = (r1 + r2[l]) - (dx * dx + dy * dy).sqrt();
                // Decay function for (near) non-penetrating poles
                let pd_decay = if pd >= epsilon { pd } else { e_sq / (two_e - pd) };
                let min_r = if r1 < r2[l] { r1 } else { r2[l] };
                acc[l] += pd_decay * min_r;
            }
        }
    }

    let total_overlap = acc.iter().sum::<f32>() * PI;

    debug_assert!(total_overlap.is_normal());
    total_overlap
}
