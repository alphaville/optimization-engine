use super::*;

#[test]
fn t_rectangle_closed() {
    let xmin = vec![2.0; 5];
    let xmax = vec![4.5; 5];
    let rectangle = Rectangle::new(&xmin[..], &xmax[..]);
    let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];

    rectangle.project(&mut x);

    println!("x = {:?}", x);
}

#[test]
fn t_rectangle_only_xmin() {
    let xmin = vec![2.0; 5];
    let rectangle = Rectangle::new_only_xmin(&xmin[..]);
    let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];

    rectangle.project(&mut x);

    println!("x = {:?}", x);
}

#[test]
fn t_ball_at_origin() {
    let radius = 1.0;
    let mut x = [1.0, 1.0];
    let ball = Ball2::new_at_origin_with_radius(radius);

    ball.project(&mut x);

    println!("x = {:?}", x);
}

#[test]
fn t_ball_elsewhere() {
    let radius = 1.0;
    let centre = [1.0, 1.0];
    let mut x = [2.0, 2.0];
    let ball = Ball2::new(&centre[..], radius);

    ball.project(&mut x);

    println!("x = {:?}", x);
}

#[test]
fn t_no_constraints() {
    let mut x = [1.0, 2.0, 3.0];
    let whole_space = NoConstraints::new();

    whole_space.project(&mut x);

    assert_eq!([1., 2., 3.], x);
}
