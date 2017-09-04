use super::types::{Real, UnsafeVec2, Vec2};
use super::{Body, Shape};
use super::collision::{self, cross_real_vector, Manifold, ManifoldData};
use super::body::cross_vectors;
use cgmath::{dot, InnerSpace};
use noisy_float::types::n32;

pub static GRAVITY : [f32; 2] = [0.0, 500.0];
pub static EPSILON : f32 = 0.0001;
pub static FRAME_TIME: f32 = 1.0/60.0;

pub struct Scene {
    delta: Real,
    iterations: u32,
    pub bodies: Vec<Body>,
}

pub type BodyIndex = usize;

// Equal
fn float_cmp(a: Real, b: Real) -> bool {
    n32((a - b).raw().abs()) <= EPSILON
}

impl Scene {
    pub fn new() -> Self {
        Scene {
            delta: n32(0.0),
            iterations: 10,
            bodies: vec![],
        }
    }

    // Scene::Add
    pub fn add(&mut self, body: Body) {
        self.bodies.push(body);
    }

    // Scene::Step
    pub fn step(&mut self, delta: Real) {
        let contact_data = self.generate_contact_list();

        for body in &mut self.bodies {
            body.integrate_forces(delta);
        }

        let mut contacts = Vec::new();
        for data in &contact_data {
            let contact = data.initialize(delta, &self.bodies[data.pair.0], &self.bodies[data.pair.1]);
            contacts.push(contact);
        }

        for _ in 0..self.iterations {
            for contact in &contacts {
                self.apply_impulse(&contact);
            }
        }

        for body in &mut self.bodies {
            body.integrate_velocity(delta);
        }

        for contact in &mut contacts {
            self.positional_correct(&contact);
        }

        for body in &mut self.bodies {
            body.force = Vec2::new(n32(0.0), n32(0.0));
            body.torque = n32(0.0);
        }
    }

    fn generate_contact_list(&self) -> Vec<ManifoldData> {
        let mut ret = Vec::new();
        for (i, body_a) in self.bodies.iter().enumerate() {
            for (j, body_b) in self.bodies.iter().enumerate().skip(i + 1) {
                if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
                    continue
                }
                match (&body_a.shape, &body_b.shape) {
                    (&Shape::Circle { radius: r1 }, &Shape::Circle { radius: r2 }) => {
                        if let Some(manifold_data) = collision::circle_circle(
                            (i, r1, body_a),
                            (j, r2, body_b)
                        ) {
                            ret.push(manifold_data);
                        }
                    }
                }
            }
        }
        ret
    }

    fn get_two_mut(&mut self, i_a: BodyIndex, i_b: BodyIndex) -> (&mut Body, &mut Body) {
        assert!(i_a != i_b); // Can't borrow the same value twice
        assert!(i_a < self.bodies.len());
        assert!(i_b < self.bodies.len());
        if i_a < i_b {
            let (start, end) = self.bodies.split_at_mut(i_a + 1);
            let start_len = start.len();    
            (&mut start[i_a], &mut end[i_b - start_len])
        }
        else {
            let (start, end) = self.bodies.split_at_mut(i_b + 1);
            let start_len = start.len();
            (&mut end[i_a - start_len], &mut start[i_b])
        }
    }

    // Manifold::ApplyImpulse
    fn apply_impulse(&mut self, m: &Manifold) {
        let (i_a, i_b) = m.pair;
        let (body_a, body_b) = self.get_two_mut(i_a, i_b);

        if float_cmp(body_a.inv_mass + body_b.inv_mass, n32(0.0)) {
            //InfiniteMassCorrection
            body_a.velocity = Vec2::new(n32(0.0), n32(0.0));
            body_b.velocity = Vec2::new(n32(0.0), n32(0.0));

            return
        }

        for contact in &m.contacts {
            let ra = contact - body_a.position;
            let rb = contact - body_b.position;
            
            let (inv_mass_sum, j, impulse) = {
                let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                         body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

                let unsafe_rv = UnsafeVec2::new(rv.x.raw(), rv.y.raw());
                let unsafe_normal = UnsafeVec2::new(m.normal.x.raw(), m.normal.y.raw());

                let contact_vel = n32(dot(unsafe_rv, unsafe_normal));
                if contact_vel > 0.0 {
                    return
                }

                let ra_cross_n = cross_vectors(ra, m.normal);
                let rb_cross_n = cross_vectors(rb, m.normal);
                
                let inv_mass_sum = 
                    body_a.inv_mass + body_b.inv_mass +
                    n32(ra_cross_n.raw().powi(2)) * body_a.inv_inertia +
                    n32(rb_cross_n.raw().powi(2)) * body_b.inv_inertia;
                
                let j = -(n32(1.0) + m.e) * contact_vel / inv_mass_sum / n32(m.contacts.len() as f32);
                (inv_mass_sum, j, m.normal * j)
            };

            body_a.apply_impulse(-impulse, ra);
            body_b.apply_impulse( impulse, rb);

            let tangent_impulse = {
                // rv = B->velocity + Cross( B->angularVelocity, rb ) -
                //      A->velocity - Cross( A->angularVelocity, ra );
                let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                         body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

                // Vec2 t = rv - (normal * Dot( rv, normal ));
                // t.Normalize( );
                let unsafe_rv = UnsafeVec2::new(rv.x.raw(), rv.y.raw());
                let unsafe_normal = UnsafeVec2::new(m.normal.x.raw(), m.normal.y.raw());
                
                let mut unsafe_t = unsafe_rv - (unsafe_normal * dot(unsafe_rv, unsafe_normal));
                let len_t = unsafe_t.magnitude();
                //if len_t is too small we can't normalize the vector, since it would divide by zero
                if !float_cmp(n32(len_t), n32(0.0)) {
                    unsafe_t = unsafe_t.normalize();
                }
                let t = Vec2::new(n32(unsafe_t.x), n32(unsafe_t.y));

                let unsafe_rv_dot_unsafe_t = dot(unsafe_rv, unsafe_t);
                let jt = -n32(unsafe_rv_dot_unsafe_t) / inv_mass_sum / n32(m.contacts.len() as f32);

                if float_cmp(jt, n32(0.0)) {
                    return
                }

                if n32(jt.raw().abs()) < j * m.sf {
                    t * jt
                } else {
                    t * -j * m.df
                }
            };

            body_a.apply_impulse(-tangent_impulse, ra);
            body_b.apply_impulse( tangent_impulse, rb);
        }
    }

    // Manifold::PositionalCorrect
    fn positional_correct(&mut self, m: &Manifold) {
        let (body_a, body_b) = self.get_two_mut(m.pair.0, m.pair.1);

        let k_slop = 0.05;
        let percent = 0.4;

        let unsafe_normal = UnsafeVec2::new(m.normal.x.raw(), m.normal.y.raw());
        let unsafe_correction = ((m.penetration - k_slop).raw().max(0.0) / (body_a.inv_mass.raw() + body_b.inv_mass.raw())) * unsafe_normal * percent;
        let correction = Vec2::new(n32(unsafe_correction.x), n32(unsafe_correction.y));

        body_a.position -= correction * body_a.inv_mass;
        body_b.position += correction * body_b.inv_mass;
    }
}


