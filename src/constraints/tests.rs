use super::*;

#[test]
fn t_rectangle_bounded() {
    let xmin = vec![2.0; 5];
    let xmax = vec![4.5; 5];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];

    rectangle.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &[2.0, 2.0, 3.0, 4.0, 4.5],
        &x,
        1e-8,
        1e-8,
        "projection on bounded rectangle",
    );
}

#[test]
fn t_rectangle_bounded_negative_entries() {
    let xmin = [-5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
    let xmax = [-1.0, -2.0, -1.0, 2.0, 1.0, 0.0, 4.0, 6.0, 9.0, 100.0, 500.0];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [-6.0, -3.0, 0.0, 3.0, -5.0, 1.0, 2.0, 3.0, -1.0, 0.0, 0.0];

    rectangle.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &[-5.0, -3.0, -1.0, 2.0, -1.0, 0.0, 2.0, 3.0, 3.0, 4.0, 5.0],
        &x,
        1e-8,
        1e-8,
        "projection on bounded rectangle v2",
    );
}

#[test]
fn t_rectangle_only_xmin() {
    let xmin = vec![2.0; 5];
    let rectangle = Rectangle::new(Some(&xmin[..]), None);
    let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];

    rectangle.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &[2.0, 2.0, 3.0, 4.0, 5.0],
        &x,
        1e-8,
        1e-8,
        "projection on halfspace (xmin)",
    );
}

#[test]
fn t_rectangle_only_xmax() {
    let xmax = vec![-3.0; 5];
    let rectangle = Rectangle::new(None, Some(&xmax[..]));
    let mut x = [-10.0, -20.0, 0.0, 5.0, 3.0];

    rectangle.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &[-10.0, -20.0, -3.0, -3.0, -3.0],
        &x,
        1e-8,
        1e-8,
        "projection",
    );
}

#[test]
fn t_ball_at_origin() {
    let radius = 1.0;
    let mut x = [1.0, 1.0];
    let ball = Ball2::new(None, radius);

    ball.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &[0.7071067811865476, 0.7071067811865476],
        &x,
        1e-8,
        1e-8,
        "projection on ball centered at origin",
    );
}

#[test]
fn t_ball_elsewhere() {
    let radius = 1.0;
    let center = [1.0, 1.0];
    let mut x = [2.0, 2.0];
    let ball = Ball2::new(Some(&center[..]), radius);

    ball.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &[1.7071067811865476, 1.7071067811865476],
        &x,
        1e-8,
        1e-8,
        "projection on ball centered at [1, 1]",
    );
}

#[test]
fn t_no_constraints() {
    let mut x = [1.0, 2.0, 3.0];
    let whole_space = NoConstraints::new();

    whole_space.project(&mut x);

    assert_eq!([1., 2., 3.], x);
}
