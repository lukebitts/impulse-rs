use cgmath::Rad;
use super::scene::GRAVITY;
use super::types::{Vec2, UnsafeVec2, Real, PI};
use noisy_float::types::n32;

#[derive(Debug)]
pub enum Shape {
    Circle {
        radius: Real
    },
}

// Cross( const Vec2& a, const Vec2& b )
pub fn cross_vectors(a: Vec2, b: Vec2) -> Real {
    a.x * b.y - a.y * b.x
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
            let m = n32(PI) * radius * radius * density;
            let i = m * radius * radius;

            MassData {
                moment_inertia: i,
                inv_inertia: if i != 0.0 { n32(1.0) / i } else { n32(0.0) },
                mass: m,
                inv_mass: if m != 0.0 { n32(1.0) / m } else { n32(0.0) },
            }
        }
    }
}

//#[derive(Debug)]
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
    pub fn new(mut shape: Shape, position: Vec2) -> Self {
        let mass_data = compute_mass(&mut shape, n32(1.0));

        Body {
            shape: shape,

            position: position,
            velocity: Vec2::new(n32(0.0), n32(0.0)),
            angular_velocity: n32(0.0),
            torque: n32(0.0),
            orient: Rad(n32(0.0)),
            force: Vec2::new(n32(0.0), n32(0.0)),
            static_friction: n32(0.5),
            dynamic_friction: n32(0.3),
            restitution: n32(0.2),

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
        let unsafe_delta_velocity = self.inv_mass.raw() * UnsafeVec2::new(impulse.x.raw(), impulse.y.raw());
        let unsafe_velocity = UnsafeVec2::new(self.velocity.x.raw(), self.velocity.y.raw()) + unsafe_delta_velocity;

        self.velocity = Vec2::new(n32(unsafe_velocity.x), n32(unsafe_velocity.y));
        self.angular_velocity += self.inv_inertia * cross_vectors(contact_vector, impulse);
    }

    // Body::SetStatic
    pub fn set_static(&mut self) {
        self.moment_inertia = n32(0.0);
        self.inv_inertia = n32(0.0);
        self.mass = n32(0.0);
        self.inv_mass = n32(0.0);
    }

    // Body::SetOrient, Circle::SetOrient
    pub fn set_orient<T: Into<Rad<Real>>>(&mut self, radians: T) {
        let radians = radians.into();
        self.orient = radians;
        match self.shape {
            Shape::Circle{ .. } => (),
        }
    }

    // IntegrateForces
    pub fn integrate_forces(&mut self, delta: Real) {
        if self.inv_mass == 0.0 {
            return
        }
        let safe_gravity = [n32(GRAVITY[0]), n32(GRAVITY[1])];
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