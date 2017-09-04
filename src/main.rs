extern crate cgmath;
extern crate ggez;
extern crate rayon;

mod types;
use types::{Vec2, Mat2, Real};

mod body;
use body::{Body, Shape};

mod operations;
mod collision;
mod scene;
use scene::Scene;

use ggez::conf;
use ggez::event;
use ggez::{Context, GameResult};
use ggez::graphics;
use ggez::graphics::{DrawMode, Point};
use std::time::Duration;
use cgmath::{Rad, Deg};

struct MainState {
    scene: Scene,
    time: Duration,
    spawn_timer: Duration,
    spawn_count: u32,
    dt: Duration,
}

impl MainState {
    fn new(_: &mut Context) -> GameResult<MainState> {
        let s = MainState {
            scene: Scene::new(),
            time: Duration::from_secs(0),
            dt: Duration::from_secs(0),
            spawn_timer: Duration::from_secs(0),
            spawn_count: 0,
        };

        Ok(s)
    }
}

fn create_circle_with_density(radius: Real, x: Real, y: Real, dynamic: bool, density: Real) -> Body {
    let shape = Shape::Circle{ radius };
    let mut b = Body::with_density(shape, Vec2::new(x, y), density);
    if !dynamic { b.set_static() }
    b
}

fn create_circle(radius: Real, x: Real, y: Real, dynamic: bool) -> Body {
    let shape = Shape::Circle{ radius };
    let mut b = Body::new(shape, Vec2::new(x, y));
    if !dynamic { b.set_static() }
    b
}

fn create_rect<T: Into<Rad<f32>>>(hw: Real, hh: Real, x: Real, y: Real, orientation: T, dynamic: bool) -> Body {
    let shape = Shape::rect(Vec2::new(hw, hh));
    let mut b = Body::new(shape, Vec2::new(x, y));
    b.set_orient(orientation);
    if !dynamic { b.set_static() }
    b
}

impl event::EventHandler for MainState {
    fn update(&mut self, ctx: &mut Context, dt: Duration) -> GameResult<()> {
        self.dt = dt;
        self.time += dt;
        self.spawn_timer += dt;

        if self.spawn_timer >= Duration::from_millis(200) /*&& self.spawn_count < 100*/ {
            self.spawn_timer -= Duration::from_millis(200);
            self.spawn_count += 1;
            self.scene.add(create_circle(15.0, 147.0, 77., true));
        }

        if self.spawn_count % 20 == 0 {
            self.spawn_count += 1;
            let mut circle = create_circle_with_density(30.0, 1200., 200., true, 0.5);
            circle.apply_impulse(Vec2::new(-20000.0, 0.0), Vec2::new(50.0, 50.0));
            self.scene.add(circle);
        }

        if self.spawn_count % 4 == 0 /*&& self.spawn_count < 100*/ {
            self.spawn_count += 1;
            let mut circle = create_circle(10.0, 1800., 1000., true);
            circle.apply_impulse(Vec2::new(-500000.0, -510000.0), Vec2::new(0.0, 0.0));
            self.scene.add(circle);
        }

        let frame_time = Duration::from_millis((scene::FRAME_TIME * 1000.0) as u64);
        let dt_float = scene::FRAME_TIME;
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
                        match radius {
                            10.0 => graphics::set_color(ctx, graphics::Color::new(200./255.,101./255.,230./255.,200./255.))?,
                            30.0 => graphics::set_color(ctx, graphics::Color::new(255./255.,0./255.,0./255.,150./255.))?,
                            _ => graphics::set_color(ctx, graphics::Color::new(100./255.,101./255.,230./255.,200./255.))?,
                        }
                        //graphics::circle(ctx, DrawMode::Fill, Point { x: body.position.x, y: body.position.y }, radius, 6)?;
                    }
                    graphics::circle(ctx, DrawMode::Fill, Point { x: body.position.x, y: body.position.y }, radius, 32)?;
                }
                &Shape::Polygon { ref vertices, .. } => {
                    let a = -body.orient.0;
                    let rotation_matrix = Mat2::new(a.cos(), -a.sin(), a.sin(), a.cos());

                    graphics::polygon(ctx, DrawMode::Fill, 
                        vertices.iter()
                        .map(|v| v.position)
                        .map(|v| rotation_matrix * v)
                        .map(|v| v + body.position)
                        .map(|v| Point { x: v.x, y: v.y })
                        .collect::<Vec<_>>().as_ref())?;
                }
            }
        }
        graphics::present(ctx);
        Ok(())
    }
}

pub fn main() {
    let mut c = conf::Conf::new();
    c.window_width = 1920;
    c.window_height = 1060;

    let ctx = &mut Context::load_from_conf("impulse engine", "*", c).unwrap();
    println!("{}", graphics::get_renderer_info(ctx).unwrap());

    //graphics::set_background_color(ctx, graphics::BLACK);
    graphics::set_background_color(ctx, graphics::Color::new(120./255.,255./255.,141./255.,255./255.));

    let state = &mut MainState::new(ctx).unwrap();


    /*for i in 1..20 {
        for j in 1..5 {
            state.scene.add(create_circle(15.0, 115. + (32. * i as f32), 45.  + (32. * j as f32), true));
            //state.scene.add(create_rect(10.0, 10.0, 25. + (12. * i as f32), 25.  + (12. * j as f32), Deg(80.0), true));
        }
    }*/
    
    fn x(v: i32) -> f32 {
        -(v as f32-5.).powi(2)/4.0
    }

    for i in 1..10 {
        //if i == 5 { continue }
        state.scene.add(create_circle(70.0, 100.0 * i as f32, 1000.0 + 50. * x(i), false));
    }
    //state.scene.add(create_circle(100.0, 100.0 * 0.0  as f32, 500.0, false));
    //state.scene.add(create_circle(100.0, 100.0 * 10.0 as f32, 450.0, false));

    //state.scene.add(create_rect(200.0, 20.0, 450.0, 400.0, Deg(80.0), false));

    state.scene.add(create_rect(300.0, 20.0, 300.0, 300.0, Deg(10.0), false));
    //state.scene.add(create_rect(150.0, 20.0, 650.0, 500.0, Deg(-20.0), false));

    state.scene.add(create_rect(20.0, 400.0, 1700.0, 600.0, Deg(0.0), false));
    state.scene.add(create_rect(20.0, 400.0, 1880.0, 600.0, Deg(0.0), false));

    state.scene.add(create_rect(400.0, 20.0, 1100.0, 600.0, Deg(-10.0), false));


    if let Err(e) = event::run(ctx, state) {
        println!("Error encountered: {}", e);
    } else {
        println!("Game exited cleanly.");
    }
}
