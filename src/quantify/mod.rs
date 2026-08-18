use crate::consts::OVERLAP_PROXY_EPSILON_DIAM_RATIO;
use crate::quantify::circles_soa::CirclesSoA;
use crate::quantify::overlap_proxy::{overlap_area_proxy, overlap_area_proxy_soa};
use jagua_rs::geometry::geo_traits::DistanceTo;
use jagua_rs::geometry::primitives::{Rect, SPolygon};

pub mod circles_soa;
pub mod overlap_proxy;
mod pair_matrix;
pub mod tracker;
#[cfg(feature = "simd")]
pub mod simd;

/// Quantifies a collision between two simple polygons.
/// Algorithm 4 from https://doi.org/10.48550/arXiv.2509.13329
#[inline(always)]
pub fn quantify_collision_poly_poly(s1: &SPolygon, s2: &SPolygon) -> f32 {
    let epsilon = f32::max(s1.diameter, s2.diameter) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;

    let overlap_proxy = overlap_area_proxy(s1.surrogate(), s2.surrogate(), epsilon) + epsilon.powi(2);

    debug_assert!(overlap_proxy.is_normal());

    let penalty = calc_shape_penalty(s1, s2);

    overlap_proxy.sqrt() * penalty
}

/// Same as [`quantify_collision_poly_poly`], but with the poles of `s2` provided in SoA layout (`poles2`),
/// enabling the (auto)vectorized overlap proxy. Use this when the same `s2` is quantified against many `s1`.
#[inline(always)]
pub fn quantify_collision_poly_poly_soa(s1: &SPolygon, s2: &SPolygon, poles2: &CirclesSoA) -> f32 {
    debug_assert!(poles2.n == s2.surrogate().poles.len(), "SoA poles must match the poles of s2");
    let epsilon = f32::max(s1.diameter, s2.diameter) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;

    let overlap_proxy = overlap_area_proxy_soa(s1.surrogate(), poles2, epsilon) + epsilon.powi(2);

    debug_assert!(overlap_proxy.is_normal());
    debug_assert!(
        float_cmp::approx_eq!(f32, overlap_proxy, overlap_area_proxy(s1.surrogate(), s2.surrogate(), epsilon) + epsilon.powi(2), epsilon = overlap_proxy * 1e-3),
        "SoA and sequential overlap proxies do not match"
    );

    let penalty = calc_shape_penalty(s1, s2);

    overlap_proxy.sqrt() * penalty
}

pub fn calc_shape_penalty(s1: &SPolygon, s2: &SPolygon) -> f32 {
    // The shape-based penalty between two shapes is defined as the geometric mean of the square roots of their convex hull areas.
    let p1 = f32::sqrt(s1.surrogate().convex_hull_area);
    let p2 = f32::sqrt(s2.surrogate().convex_hull_area);
    (p1 * p2).sqrt()
}

/// Quantifies a collision between a simple polygon and a **hole** of the container
/// (a quality-0 zone; in the multi-sheet mode: a sheet wall).
///
/// Deliberately **bbox-overlap based** rather than pole based, for two reasons:
///
/// * hole shapes are container geometry and never get a pole surrogate generated (only *items* do,
///   see [`jagua_rs::entities::Item`]), so the pole proxy is not even available for them;
/// * a wall is a long, thin, axis-aligned rectangle — the exact overlap area of two bounding boxes
///   is both cheaper and a *better* gradient here than a pole approximation of a sliver would be:
///   it decreases strictly monotonically as the item is pushed off the wall, all the way to zero.
///
/// Mirrors the structure of [`quantify_collision_poly_container`] (same shape penalty, same
/// `sqrt` scaling), so hole losses are directly comparable to container and pair losses and the
/// shared GLS weighting works unchanged.
#[inline(always)]
pub fn quantify_collision_poly_hole(s: &SPolygon, hole_bbox: Rect) -> f32 {
    let s_bbox = s.bbox;
    let overlap = match Rect::intersection(s_bbox, hole_bbox) {
        Some(r) => {
            // The item overlaps the hole: the penetrated area (+ epsilon so it is never exactly zero)
            r.area() + 0.0001 * s_bbox.area()
        }
        None => {
            // The bounding boxes do not overlap, but the exact shapes were detected as colliding
            // (possible for non-convex shapes vs. a rotated bbox). Fall back to a small positive
            // value so the loss stays strictly positive, as the tracker requires.
            0.0001 * s_bbox.area()
        }
    };
    debug_assert!(overlap.is_normal());

    let penalty = calc_shape_penalty(s, s);

    2.0 * overlap.sqrt() * penalty
}

/// Quantifies a collision between a simple polygon and the exterior of the container.
#[inline(always)]
pub fn quantify_collision_poly_container(s: &SPolygon, c_bbox: Rect) -> f32 {
    let s_bbox = s.bbox;
    let overlap = match Rect::intersection(s_bbox, c_bbox) {
        Some(r) => {
            //intersection exist, calculate the area of the intersection (+ a small value to ensure it is never zero)
            (s_bbox.area() - r.area()) + 0.0001 * s_bbox.area()
        }
        None => {
            //no intersection, guide towards intersection with container
            s_bbox.area() + s_bbox.centroid().distance_to(&c_bbox.centroid())
        }
    };
    debug_assert!(overlap.is_normal());

    let penalty = calc_shape_penalty(s, s);

    2.0 * overlap.sqrt() * penalty
}
