use super::types::{Vec2, Real};
use super::scene::EPSILON;
use cgmath::dot;

pub fn cross_real_vector(a: Real, v: Vec2) -> Vec2 {
    Vec2::new( -a * v.y, a * v.x )
}

// Cross( const Vec2& a, const Vec2& b )
pub fn cross_vectors(a: Vec2, b: Vec2) -> Real {
    a.x * b.y - a.y * b.x
}

// LenSqr
pub fn len_sqr(v: Vec2) -> Real {
    v.x.powi(2) + v.y.powi(2)
}

pub fn dist_sqr(a: Vec2, b: Vec2) -> Real {
    let c = a - b;
    return dot( c, c );
}

// Equal
pub fn float_cmp(a: Real, b: Real) -> bool {
    (a - b).abs() <= EPSILON
}