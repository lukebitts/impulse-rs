#![allow(dead_code)]
#![allow(unused)]

extern crate cgmath;

mod types;
use types::{Vec2};

mod body;
use body::{Body, Shape};


fn create_circle() -> Body {
    let shape = Shape::Circle{ radius: 25.0 };
    Body::new(shape, Vec2::new(50.0, 50.0))
}

fn main() {
    
    let bodies : Vec<_> = (0..10).map(|_| create_circle()).collect();

    

}
