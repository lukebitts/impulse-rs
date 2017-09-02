use cgmath::Rad;
use std::f32::consts::PI;

use super::types::{Vec2, Mat2, Real};

pub struct PolygonShapeVertex {
    position: Vec2,
    normal: Vec2,
}

pub enum Shape {
    Polygon {
        orientation: Mat2,
        vertices: Vec<PolygonShapeVertex>,
    },
    Circle {
        radius: Real
    },
}

// Cross( const Vec2& a, const Vec2& b )
fn cross_vectors(a: Vec2, b: Vec2) -> Real {
    // return a.x * b.y - a.y * b.x;
    a.x * b.y - a.y * b.x
}

struct MassData {
    pub moment_inertia: Real,
    pub inv_inertia: Real,
    pub mass: Real,
    pub inv_mass: Real,
}

// Circle::ComputeMass, PolygonShape::ComputeMass
fn compute_mass(shape: &mut Shape, density: Real) -> MassData {
    match shape {
        &mut Shape::Circle{ radius } => {
            // body->m = PI * radius * radius * density;
            // body->im = (body->m) ? 1.0f / body->m : 0.0f;
            // body->I = body->m * radius * radius;
            // body->iI = (body->I) ? 1.0f / body->I : 0.0f;

            let m = PI * radius * radius * density;
            let i = m * radius * radius;

            MassData {
                moment_inertia: m,
                inv_inertia: if m != 0.0 { 1.0 / m } else { 0.0 },
                mass: i,
                inv_mass: if i != 0.0 { 1.0 / i } else { 0.0 },
            }
        }
        &mut Shape::Polygon{ ref mut vertices, .. } => {
            // Calculate centroid and moment of interia
            let mut c = Vec2::new(0.0, 0.0); // centroid
            let mut area = 0.0;
            let mut i = 0.0;
            let k_inv3 = 1.0 / 3.0;

            //for(uint32 i1 = 0; i1 < m_vertexCount; ++i1)
            for edge in vertices.chunks(2) {
                // Triangle vertices, third vertex implied as (0, 0)
                //Vec2 p1( m_vertices[i1] );
                let p1 = edge[0].position;
                let p2 = if edge.len() == 2 { edge[1].position } else { edge[0].position };
                
                let d = cross_vectors(p1, p2);
                let triangle_area = 0.5 * d;

                area += triangle_area;

                // Use area to weight the centroid average, not just vertex position
                c += triangle_area * k_inv3 * (p1 + p2);

                let intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
                let inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
                i += (0.25 * k_inv3 * d) * (intx2 + inty2);
            }

            c *= 1.0 / area;

            // Translate vertices to centroid (make the centroid (0, 0)
            // for the polygon in model space)
            for vertex in vertices {
               vertex.position -= c;
            }

            let m = density * area;
            let i = i * density;

            MassData {
                moment_inertia: m,
                inv_inertia: if m != 0.0 { 1.0 / m } else { 0.0 },
                mass: i,
                inv_mass: if i != 0.0 { 1.0 / i } else { 0.0 },
            }
        }
    }
}

pub struct Body {
    shape: Shape,

    position: Vec2,
    orient: Rad<Real>,
    velocity: Vec2,
    angular_velocity: Real,
    torque: Real,
    force: Vec2,
    static_friction: Real,
    dynamic_friction: Real,
    restitution: Real,

    moment_inertia: Real,
    inv_inertia: Real,
    mass: Real,
    inv_mass: Real,
}

impl Body {
    // Body::Body, Shape::Initialize
    pub fn new(mut shape: Shape, position: Vec2) -> Self {
        let mass_data = compute_mass(&mut shape, 1.0);

        Body {
            shape: shape,

            position: position,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            torque: 0.0,
            orient: Rad(0.0),
            force: Vec2::new(0.0, 0.0),
            static_friction: 0.5,
            dynamic_friction: 0.3,
            restitution: 0.2,

            moment_inertia: mass_data.moment_inertia,
            inv_inertia: mass_data.inv_inertia,
            mass: mass_data.mass,
            inv_mass: mass_data.inv_mass,
        }
    }

    // Body::ApplyForce
    pub fn apply_force(&mut self, force: Vec2) {
        // force += f;
        self.force += force;
    }

    // Body::ApplyImpulse
    pub fn apply_impulse(&mut self, impulse: Vec2, contact_vector: Vec2) {
        // velocity += im * impulse;
        // angularVelocity += iI * Cross( contactVector, impulse );
        self.velocity += self.inv_mass * impulse;
        self.angular_velocity += self.inv_inertia * cross_vectors(contact_vector, impulse);
    }

    // Body::SetStatic
    pub fn set_static(&mut self) {
        // I = 0.0f;
        // iI = 0.0f;
        // m = 0.0f;
        // im = 0.0f;
        self.moment_inertia = 0.0;
        self.inv_inertia = 0.0;
        self.mass = 0.0;
        self.inv_mass = 0.0;
    }

    pub fn set_orient<T: Into<Rad<Real>>>(&mut self, radians: T) {
        let radians = radians.into();
        self.orient = radians;
        match self.shape {
            Shape::Circle{ .. } => (),
            Shape::Polygon{ ref mut orientation, .. } => {
                fn set(radians: Rad<Real>) -> Mat2
                {
                    let c = radians.0.cos();
                    let s = radians.0.sin();
                    //m00 = c; m01 = -s;
                    //m10 = s; m11 =  c;

                    Mat2::new(c, -s, s, c)
                }

                *orientation = set(radians);
            }
        }
    }
}