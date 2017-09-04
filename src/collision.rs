use super::types::{Vec2, Real, Mat2};
use super::scene::BodyIndex;
use super::Body;
use super::scene::{GRAVITY, EPSILON};
use super::body::PolygonShapeVertex;
use super::operations::{cross_real_vector, len_sqr, dist_sqr, float_cmp};
use cgmath::{dot, Matrix, InnerSpace};

pub struct ManifoldData {
    pub pair: (BodyIndex, BodyIndex),
    pub penetration: Real,
    pub normal: Vec2,
    pub contacts: Vec<Vec2>,
}

pub struct Manifold {
    pub pair: (BodyIndex, BodyIndex),
    pub penetration: Real,
    pub normal: Vec2,
    pub contacts: Vec<Vec2>,
    pub e: Real,
    pub df: Real,
    pub sf: Real,
}

impl ManifoldData {
    pub fn initialize(&self, delta: Real, body_a: &Body, body_b: &Body) -> Manifold {
        // Calculate average restitution
        let mut e = body_a.restitution.min(body_b.restitution);

        // Calculate static and dynamic friction
        let sf = body_a.static_friction.powi(2).sqrt();
        let df = body_a.dynamic_friction.powi(2).sqrt();

        for contact in &self.contacts {
            let ra = contact - body_a.position;
            let rb = contact - body_b.position;

            let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                     body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

            if len_sqr(rv) < len_sqr(Vec2::new(GRAVITY[0] * delta, GRAVITY[1] * delta)) + EPSILON {
                e = 0.0;
            }
        }
        
        Manifold {
            pair: self.pair,
            penetration: self.penetration,
            normal: self.normal,
            contacts: self.contacts.clone(),
            e: e,
            df: df,
            sf: sf
        }
    }
}

// CircletoCircle
pub fn circle_circle(
        (i_a, radius_a, body_a): (BodyIndex, Real, &Body), 
        (i_b, radius_b, body_b): (BodyIndex, Real, &Body)) 
    -> Option<ManifoldData> {
    // Calculate translational vePolygonShapeVertexctor, which is normal

    let normal = body_b.position - body_a.position;
    let dist_sqr = len_sqr(normal);
    let radius = radius_a + radius_b;

    if dist_sqr >= radius.powi(2) {
        return None
    }
    let distance = dist_sqr.sqrt();

    if distance == 0.0 {
        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: radius_a,
            normal: Vec2::new(1.0, 0.0),
            contacts: vec![body_a.position]
        })
    } else {
        let normal_over_distance = normal / distance;
        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: radius - distance,
            normal: normal_over_distance,
            contacts: vec![normal_over_distance * radius_a + body_a.position],
        })
    }
}

pub fn circle_polygon(
        (i_a, radius_a, body_a): (BodyIndex, Real, &Body), 
        (i_b, orientation_b, vertices_b, body_b): (BodyIndex, &Mat2, &Vec<PolygonShapeVertex>, &Body)) 
    -> Option<ManifoldData> {

    let pos_a = body_a.position;
    let pos_b = body_b.position;

    let mut center = pos_a;
    center = orientation_b.transpose() * (center - pos_b);

    let mut separation = ::std::f32::MIN;
    let mut face_normal = 0;
    for (i, vertex) in vertices_b.iter().enumerate() {
        let s = dot(vertex.normal, center - vertex.position);

        if s > radius_a {
            return None
        }

        if s > separation {
            separation = s;
            face_normal = i;
        }
    }

    let mut v1 = vertices_b[face_normal].clone();
    let i2 = if face_normal + 1 < vertices_b.len() { face_normal + 1 } else { 0 };
    let mut v2 = vertices_b[i2].clone();

    if separation < EPSILON {
        let normal = -(orientation_b * v1.normal);

        return Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: radius_a,
            normal: normal,
            contacts: vec![normal * radius_a + pos_a],
        })
    }

    let dot1 = dot(center - v1.position, v2.position - v1.position);
    let dot2 = dot(center - v2.position, v1.position - v2.position);
    let penetration = radius_a - separation;

    if dot1 <= 0.0 {
        if dist_sqr(center, v1.position) > radius_a.powi(2) {
            return None
        }

        let mut n = v1.position - center;
        n = orientation_b * n;
        if !float_cmp(n.magnitude(), 0.0) {
            n = n.normalize();
        }
        v1.position = orientation_b * v1.position + pos_b;

        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: penetration,
            normal: n,
            contacts: vec![v1.position]
        })
    } else if dot2 <= 0.0 {
        if dist_sqr(center, v2.position) > radius_a.powi(2) {
            return None
        }

        let mut n = v2.position - center;
        n = orientation_b * n;
        if !float_cmp(n.magnitude(), 0.0) {
            n = n.normalize();
        }
        
        v2.position = orientation_b * v2.position + pos_b;

        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: penetration,
            normal: n,
            contacts: vec![v2.position]
        })
    } else {
        let mut n = v1.normal;

        if dot(center - v1.position, n) > radius_a {
            return None
        }

        n = -(orientation_b * n);

        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: penetration,
            normal: n,
            contacts: vec![n * radius_a + pos_a]
        })
    }
}