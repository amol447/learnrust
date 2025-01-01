pub mod vanillafilter;

extern crate approx; // For the macro assert_relative_eq!
extern crate derive_more;
extern crate nalgebra as na;

#[derive(Debug)]
struct Test {
    x: isize,
    y: String,
}
impl Test {
    fn new(x: isize, y: String) -> Test {
        Self { x, y }
    }
}
fn main() {
    println!("Hello, world!");
    let s = Test::new(1, "1".to_string());
    println!("{:?}", s);
}
