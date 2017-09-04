use cgmath::Rad;
use super::scene::GRAVITY;
use super::types::{Vec2, Mat2, Real, PI};
use super::operations::cross_vectors;

#[derive(Clone)]
pub struct PolygonShapeVertex {
    pub position: Vec2,
    pub normal: Vec2,
}

#[derive(Clone)]
pub enum Shape {
    Circle {
        radius: Real
    },
    Polygon {
        orientation: Mat2,
        vertices: Vec<PolygonShapeVertex>,
    },
}

impl Shape {
    pub fn rect(h: Vec2) -> Shape {
        Shape::Polygon {
            orientation: Mat2::new(1.0, 0.0, 0.0, 1.0),
            vertices: vec![
                PolygonShapeVertex { position: Vec2::new(-h.x, -h.y), normal: Vec2::new( 0.0, -1.0) },
                PolygonShapeVertex { position: Vec2::new( h.x, -h.y), normal: Vec2::new( 1.0,  0.0) },
                PolygonShapeVertex { position: Vec2::new( h.x,  h.y), normal: Vec2::new( 0.0,  1.0) },
                PolygonShapeVertex { position: Vec2::new(-h.x,  h.y), normal: Vec2::new(-1.0,  0.0) },
            ],
        }
    }
}

struct MassData {
    pub moment_inertia: Real,
    pub inv_inertia: Real,
    pub mass: Real,
    pub inv_mass: Real,
}

// Circle::ComputeMass
fn compute_mass(shape: &mut Shape, density: Real) -> MassData {
    match shape {
        &mut Shape::Circle{ radius } => {
            let m = PI * radius * radius * density;
            let i = m * radius * radius;

            MassData {
                moment_inertia: i,
                inv_inertia: if i != 0.0 { 1.0 / i } else { 0.0 },
                mass: m,
                inv_mass: if m != 0.0 { 1.0 / m } else { 0.0 },
            }
        }
        &mut Shape::Polygon { ref mut vertices, .. } => {
            let mut c = Vec2::new(0.0, 0.0); // centroid
            let mut area = 0.0;
            let mut i = 0.0;
            let k_inv3 = 1.0 / 3.0;

            for edge in vertices.chunks(2) {
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

#[derive(Clone)]
pub struct Body {
    pub shape: Shape,

    pub position: Vec2,
    pub orient: Rad<Real>,
    pub velocity: Vec2,
    pub angular_velocity: Real,
    pub torque: Real,
    pub force: Vec2,
    pub static_friction: Real,
    pub dynamic_friction: Real,
    pub restitution: Real,

    pub moment_inertia: Real,
    pub inv_inertia: Real,
    pub mass: Real,
    pub inv_mass: Real,
}

impl Body {
    // Body::Body, Shape::Initialize
    pub fn with_density(mut shape: Shape, position: Vec2, density: Real) -> Self {
        let mass_data = compute_mass(&mut shape, density);

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

    pub fn new(shape: Shape, position: Vec2) -> Self {
        Self::with_density(shape, position, 1.0)
    }

    // Body::ApplyForce
    pub fn apply_force(&mut self, force: Vec2) {
        // force += f;
        self.force += force;
    }

    // Body::ApplyImpulse
    pub fn apply_impulse(&mut self, impulse: Vec2, contact_vector: Vec2) {
        self.velocity += self.inv_mass * impulse;
        self.angular_velocity += self.inv_inertia * cross_vectors(contact_vector, impulse);
    }

    // Body::SetStatic
    pub fn set_static(&mut self) {
        self.moment_inertia = 0.0;
        self.inv_inertia = 0.0;
        self.mass = 0.0;
        self.inv_mass = 0.0;
    }

    // Body::SetOrient, Circle::SetOrient
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

                    Mat2::new(c, -s, s, c)
                }

                *orientation = set(-radians);
            }
        }
    }

    // IntegrateForces
    pub fn integrate_forces(&mut self, delta: Real) {
        if self.inv_mass == 0.0 {
            return
        }
        let safe_gravity = [GRAVITY[0], GRAVITY[1]];
        self.velocity += (self.force * self.inv_mass + Vec2::from(safe_gravity)) * (delta / 2.0);
        self.angular_velocity += self.torque * self.inv_inertia * (delta / 2.0);
    }

    // IntegrateVelocity
    pub fn integrate_velocity(&mut self, delta: Real) {
        if self.inv_mass == 0.0 {
            return
        }
        self.position += self.velocity * delta;
        let orient = {
            self.orient = Rad(self.orient.0 + self.angular_velocity * delta);
            self.orient
        };
        self.set_orient(orient);

        self.integrate_forces(delta);
    }
}