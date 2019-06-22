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
#[should_panic]
fn t_rectangle_incompatible_dims() {
    let xmin = vec![1.0; 5];
    let xmax = vec![2.0; 4];
    let _rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
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

#[test]
#[should_panic]
fn cartesian_product_constraints_incoherent_indices() {
    let ball1 = Ball2::new(None, 1.0);
    let ball2 = Ball2::new(None, 0.5);
    let mut cart_prod = CartesianProduct::new();
    cart_prod.add_constraint(3, &ball1);
    cart_prod.add_constraint(2, &ball2);
}

#[test]
#[should_panic]
fn cartesian_product_constraints_wrong_vector_dim() {
    let ball1 = Ball2::new(None, 1.0);
    let ball2 = Ball2::new(None, 0.5);
    let mut cart_prod = CartesianProduct::new();
    cart_prod.add_constraint(3, &ball1);
    cart_prod.add_constraint(10, &ball2);
    let mut x = [0.0; 30];
    cart_prod.project(&mut x);
}

#[test]
fn cartesian_product_constraints() {
    let radius1 = 1.0;
    let radius2 = 0.5;
    let idx1 = 3;
    let idx2 = 5;
    let ball1 = Ball2::new(None, radius1);
    let ball2 = Ball2::new(None, radius2);
    let mut cart_prod = CartesianProduct::new();
    cart_prod.add_constraint(idx1, &ball1);
    cart_prod.add_constraint(idx2, &ball2);
    let mut x = [3.0, 4.0, 5.0, 2.0, 1.0];
    cart_prod.project(&mut x);
    let r1 = crate::matrix_operations::norm2(&x[0..idx1]);
    let r2 = crate::matrix_operations::norm2(&x[idx1..idx2]);
    unit_test_utils::assert_nearly_equal(r1, radius1, 1e-8, 1e-12, "r1 is wrong");
    unit_test_utils::assert_nearly_equal(r2, radius2, 1e-8, 1e-12, "r2 is wrong");
}

#[test]
fn cartesian_product_ball_and_rectangle() {
    /* Rectangle 1 */
    let xmin1 = vec![-1.0; 3];
    let xmax1 = vec![1.0; 3];
    let rectangle1 = Rectangle::new(Some(&xmin1), Some(&xmax1));

    /* Ball */
    let radius = 1.0;
    let ball = Ball2::new(None, radius);

    /* Rectangle 2 */
    let xmin2 = vec![-0.5; 2];
    let xmax2 = vec![0.5; 2];
    let rectangle2 = Rectangle::new(Some(&xmin2), Some(&xmax2));

    /* Cartesian product */
    let mut cart_prod = CartesianProduct::new();
    cart_prod.add_constraint(3, &rectangle1);
    cart_prod.add_constraint(7, &ball);
    cart_prod.add_constraint(9, &rectangle2);

    /* Projection */
    let mut x = [-10.0, 0.5, 10.0, 0.01, -0.01, 0.1, 10.0, -1.0, 1.0];
    cart_prod.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(
        &x[0..3],
        &[-1.0, 0.5, 1.0],
        1e-8,
        1e-10,
        "wrong projection on rectangle 1",
    );

    let r = crate::matrix_operations::norm2(&x[3..7]);
    unit_test_utils::assert_nearly_equal(r, radius, 1e-8, 1e-12, "r is wrong");

    unit_test_utils::assert_nearly_equal_array(
        &x[7..9],
        &[-0.5, 0.5],
        1e-8,
        1e-10,
        "wrong projection on rectangle 2",
    );
}
