extern crate cgmath;
extern crate ggez;
extern crate noisy_float;

mod types;
use types::{Vec2, Real};

mod body;
use body::{Body, Shape};

mod collision;
mod scene;
use scene::Scene;

use ggez::conf;
use ggez::event;
use ggez::{Context, GameResult};
use ggez::graphics;
use ggez::graphics::{DrawMode, Point};
use std::time::Duration;
use noisy_float::types::n32;

struct MainState {
    scene: Scene,
    time: Duration,
    dt: Duration,
}

impl MainState {
    fn new(_: &mut Context) -> GameResult<MainState> {
        let s = MainState {
            scene: Scene::new(),
            time: Duration::from_secs(0),
            dt: Duration::from_secs(0)
        };

        Ok(s)
    }
}

fn create_circle(radius: Real, x: Real, y: Real, dynamic: bool) -> Body {
    let shape = Shape::Circle{ radius };
    let mut b = Body::new(shape, Vec2::new(x, y));
    if !dynamic { b.set_static() }
    b
}

impl event::EventHandler for MainState {
    fn update(&mut self, ctx: &mut Context, dt: Duration) -> GameResult<()> {
        self.dt = dt;
        self.time += dt;
        let frame_time = Duration::from_millis((n32(scene::FRAME_TIME) * n32(1000.0)).raw() as u64);
        let dt_float = n32(scene::FRAME_TIME);
        while self.time >= frame_time {
            self.time -= frame_time;

            self.scene.step(dt_float);

            break
        }
        println!("{:?}", dt.subsec_nanos() as f32 / 1_000_000f32);
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx);

        for body in &self.scene.bodies {
            match &body.shape {
                &Shape::Circle { radius } => {
                    if body.inv_mass == 0.0 {
                        graphics::set_color(ctx, graphics::WHITE)?;
                    } else {
                        graphics::set_color(ctx, graphics::BLACK)?;
                    }
                    graphics::circle(ctx, DrawMode::Fill, Point { x: body.position.x.raw(), y: body.position.y.raw() }, radius.raw(), 32)?
                } 
            }
        }
        graphics::present(ctx);
        Ok(())
    }
}

pub fn main() {
    let mut c = conf::Conf::new();
    c.window_width = 1024;
    c.window_height = 768;

    let ctx = &mut Context::load_from_conf("impulse engine", "*", c).unwrap();
    println!("{}", graphics::get_renderer_info(ctx).unwrap());

    let state = &mut MainState::new(ctx).unwrap();

    for i in 1..80 {
        for j in 1..10 {
            state.scene.add(create_circle(n32(5.0), n32(25. + (12. * i as f32)), n32(25.  + (12. * j as f32)), true));
        }
    }
    
    fn x(v: i32) -> f32 {
        (v as f32-5.)/2.
    }

    for i in 1..10 {
        state.scene.add(create_circle(n32(100.0), n32(100.0 * i as f32), n32(600.0 + 50. * x(i)), false));
    }
    state.scene.add(create_circle(n32(100.0), n32(100.0 * 0  as f32), n32(500.0), false));
    state.scene.add(create_circle(n32(100.0), n32(100.0 * 10 as f32), n32(450.0), false));


    if let Err(e) = event::run(ctx, state) {
        println!("Error encountered: {}", e);
    } else {
        println!("Game exited cleanly.");
    }
}
