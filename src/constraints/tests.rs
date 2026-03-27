use crate::{matrix_operations, numeric::cast};

use super::*;
use num::{Float, ToPrimitive};
use rand;
use rand::RngExt;
use rand_distr::{Distribution, Gamma};

#[test]
fn t_zero_set() {
    let zero = Zero::new();
    let mut x = [1.0, 2.0, 3.0];
    let x_projection = [0.0; 3];
    zero.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x_projection,
        &x,
        1e-12,
        1e-12,
        "wrong projection on zero set",
    );
}

#[test]
fn t_zero_set_f32() {
    let zero = Zero::new();
    let mut x = [1.0_f32, -2.0, 3.5];
    zero.project(&mut x);
    assert_eq!([0.0_f32, 0.0, 0.0], x);
}

#[test]
fn t_hyperplane() {
    let normal_vector = [1.0, 2.0, 3.0];
    let offset = 1.0;
    let hyperplane = Hyperplane::new(&normal_vector, offset);
    let mut x = [-1., 3., 5.];
    let x_proj_expected = [
        -2.357_142_857_142_857,
        0.285_714_285_714_286,
        0.928_571_428_571_429,
    ];
    hyperplane.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x,
        &x_proj_expected,
        1e-8,
        1e-14,
        "halfspace projection failed",
    );
}

#[test]
#[should_panic]
fn t_hyperplane_zero_normal() {
    let normal_vector = [0.0, 0.0];
    let _hyperplane = Hyperplane::new(&normal_vector, 1.0);
}

#[test]
#[should_panic]
fn t_hyperplane_wrong_dimension() {
    let normal_vector = [1.0, 2.0, 3.0];
    let hyperplane = Hyperplane::new(&normal_vector, 1.0);
    let mut x = [1.0, 2.0];
    hyperplane.project(&mut x).unwrap();
}

#[test]
fn t_halfspace_project_inside() {
    let normal_vector = [1., 2.];
    let offset = 5.0;
    let halfspace = Halfspace::new(&normal_vector, offset);
    let mut x = [-1., 3.];
    let x_expected = [-1., 3.];
    halfspace.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x,
        &x_expected,
        1e-10,
        1e-14,
        "halfspace projection failed (inside)",
    );
}

#[test]
fn t_halfspace_project_outside() {
    let normal_vector = [1., 2.];
    let offset = 1.0;
    let halfspace = Halfspace::new(&normal_vector, offset);
    let mut x = [-1., 3.];
    let x_expected = [-1.8, 1.4];
    halfspace.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x,
        &x_expected,
        1e-8,
        1e-14,
        "halfspace projection failed (outside)",
    );
}

#[test]
#[should_panic]
fn t_finite_set_inconsistent_dimensions() {
    let x1 = vec![1.0; 2];
    let x2 = vec![0.0; 3];
    let data: &[&[f64]] = &[&x1, &x2];
    let _f = FiniteSet::new(data);
}

#[test]
#[should_panic]
fn t_finite_set_empty_data() {
    let mut _data: &[&[f64]] = &[];
    let _f = FiniteSet::new(_data);
}

#[test]
fn t_finite_set() {
    let data: &[&[f64]] = &[&[0.0, 0.0], &[1.0, 1.0], &[0.0, 1.0], &[1.0, 0.0]];
    let finite_set = FiniteSet::new(data);
    let mut x = [0.7, 0.6];
    finite_set.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &[1.0, 1.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [1,1])",
    );
    x = [-0.1, 0.2];
    finite_set.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &[0.0, 0.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [0,0])",
    );
    x = [0.48, 0.501];
    finite_set.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &[0.0, 1.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [0,1])",
    );
    x = [0.7, 0.2];
    finite_set.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &[1.0, 0.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [1,0])",
    );
}

#[test]
fn t_finite_set_f32() {
    let data: &[&[f32]] = &[&[0.0_f32, 0.0], &[1.0, 1.0], &[0.0, 1.0], &[1.0, 0.0]];
    let finite_set = FiniteSet::new(data);
    let mut x = [0.7_f32, 0.2];
    finite_set.project(&mut x);
    assert_eq!([1.0_f32, 0.0], x);
}

#[test]
#[should_panic]
fn t_finite_set_project_wrong_dimension() {
    let data: &[&[f64]] = &[&[0.0, 0.0], &[1.0, 1.0]];
    let finite_set = FiniteSet::new(data);
    let mut x = [0.5, 0.5, 0.5];
    finite_set.project(&mut x).unwrap();
}

#[test]
fn t_rectangle_bounded() {
    let xmin = [2.0; 5];
    let xmax = [4.5; 5];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];

    rectangle.project(&mut x).unwrap();

    unit_test_utils::assert_nearly_equal_array(
        &[2.0, 2.0, 3.0, 4.0, 4.5],
        &x,
        1e-8,
        1e-8,
        "projection on bounded rectangle",
    );
}

#[test]
fn t_rectangle_bounded_f32() {
    let xmin = [2.0_f32; 3];
    let xmax = [4.5_f32; 3];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [1.0_f32, 3.0, 5.0];

    rectangle.project(&mut x);

    assert_eq!([2.0_f32, 3.0, 4.5], x);
}

#[test]
fn t_rectangle_infinite_bounds() {
    let xmin = [-1.0, 2.0, f64::NEG_INFINITY];
    let xmax = [1.0, f64::INFINITY, 5.0];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [-2.0, 3.0, 1.0];

    rectangle.project(&mut x).unwrap();

    unit_test_utils::assert_nearly_equal_array(
        &[-1.0, 3.0, 1.0],
        &x,
        1e-8,
        1e-8,
        "projection on unbounded rectangle",
    );
}

#[test]
#[should_panic]
fn t_rectangle_incompatible_dims() {
    let xmin = [1.0; 5];
    let xmax = [2.0; 4];
    let _rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
}

#[test]
#[should_panic]
fn t_rectangle_inconsistent_bounds() {
    let xmin = [1.0, 3.0];
    let xmax = [2.0, 2.5];
    let _rectangle = Rectangle::new(Some(&xmin), Some(&xmax));
}

#[test]
fn t_rectangle_bounded_negative_entries() {
    let xmin = [-5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
    let xmax = [-1.0, -2.0, -1.0, 2.0, 1.0, 0.0, 4.0, 6.0, 9.0, 100.0, 500.0];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [-6.0, -3.0, 0.0, 3.0, -5.0, 1.0, 2.0, 3.0, -1.0, 0.0, 0.0];

    rectangle.project(&mut x).unwrap();

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
    let xmin = [2.0; 5];
    let rectangle = Rectangle::new(Some(&xmin[..]), None);
    let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];

    rectangle.project(&mut x).unwrap();

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
    let xmax = [-3.0; 5];
    let rectangle = Rectangle::new(None, Some(&xmax[..]));
    let mut x = [-10.0, -20.0, 0.0, 5.0, 3.0];

    rectangle.project(&mut x).unwrap();

    unit_test_utils::assert_nearly_equal_array(
        &[-10.0, -20.0, -3.0, -3.0, -3.0],
        &x,
        1e-8,
        1e-8,
        "projection",
    );
}

#[test]
fn t_ball2_at_origin() {
    let radius = 1.0;
    let mut x = [1.0, 1.0];
    let ball = Ball2::new(None, radius);

    ball.project(&mut x).unwrap();

    unit_test_utils::assert_nearly_equal_array(
        &[
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        ],
        &x,
        1e-8,
        1e-8,
        "projection on ball centered at origin",
    );
}

#[test]
fn t_ball2_at_origin_f32() {
    let radius = 1.0_f32;
    let mut x = [1.0_f32, 1.0];
    let ball = Ball2::new(None, radius);

    ball.project(&mut x);

    let expected = std::f32::consts::FRAC_1_SQRT_2;
    assert!((x[0] - expected).abs() < 1e-6);
    assert!((x[1] - expected).abs() < 1e-6);
}

#[test]
fn t_ball2_at_origin_different_radius_outside() {
    let radius = 0.8;
    let mut x = [1.0, 1.0];
    let ball = Ball2::new(None, radius);
    ball.project(&mut x).unwrap();
    let norm_proj_x = crate::matrix_operations::norm2(&x);
    unit_test_utils::assert_nearly_equal(radius, norm_proj_x, 1e-10, 1e-12, "wrong norm");
}

#[test]
fn t_ball2_at_origin_different_radius_inside() {
    let radius = 0.8;
    let mut x = [-0.2, 0.15];
    let ball = Ball2::new(None, radius);
    ball.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&x, &[-0.2, 0.15], 1e-10, 1e-12, "wrong");
}

#[test]
fn t_ball2_at_center_different_radius_outside() {
    let radius = 1.2;
    let mut x = [1.0, 1.0];
    let center = [-0.8, -1.1];
    let ball = Ball2::new(Some(&center), radius);
    ball.project(&mut x).unwrap();
    let norm_x_minus_c = crate::matrix_operations::norm2_squared_diff(&x, &center).sqrt();
    unit_test_utils::assert_nearly_equal(radius, norm_x_minus_c, 1e-10, 1e-12, "wrong norm");
}

#[test]
fn t_ball2_at_center_different_radius_inside() {
    let radius = 1.2;
    let mut x = [-0.9, -0.85];
    let center = [-0.8, -1.1];
    let ball = Ball2::new(Some(&center), radius);
    ball.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[-0.9, -0.85], &x, 1e-10, 1e-12, "wrong result");
}

#[test]
fn t_ball2_elsewhere() {
    let radius = 1.0;
    let center = [1.0, 1.0];
    let mut x = [2.0, 2.0];
    let ball = Ball2::new(Some(&center[..]), radius);

    ball.project(&mut x).unwrap();

    let expected_proj_element = std::f64::consts::FRAC_1_SQRT_2 + 1.;
    unit_test_utils::assert_nearly_equal_array(
        &[expected_proj_element, expected_proj_element],
        &x,
        1e-8,
        1e-8,
        "projection on ball centered at [1, 1]",
    );
}

#[test]
fn t_ball2_boundary_no_change() {
    let radius = 2.0;
    let mut x = [0.0, 2.0];
    let x_expected = x;
    let ball = Ball2::new(None, radius);
    ball.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&x_expected, &x, 1e-12, 1e-12, "wrong result");
}

#[test]
fn t_no_constraints() {
    let mut x = [1.0, 2.0, 3.0];
    let whole_space = NoConstraints::new();

    whole_space.project(&mut x).unwrap();

    unit_test_utils::assert_nearly_equal_array(&[1., 2., 3.], &x, 1e-10, 1e-15, "x is wrong");
}

#[test]
fn t_no_constraints_f32() {
    let mut x = [1.0_f32, 2.0, 3.0];
    let whole_space = NoConstraints::new();

    whole_space.project(&mut x);

    assert_eq!([1.0_f32, 2.0, 3.0], x);
}

#[test]
#[should_panic]
fn t_cartesian_product_constraints_incoherent_indices() {
    let ball1 = Ball2::new(None, 1.0);
    let ball2 = Ball2::new(None, 0.5);
    let _cart_prod = CartesianProduct::new()
        .add_constraint(3, ball1)
        .add_constraint(2, ball2);
}

#[test]
#[should_panic]
fn t_cartesian_product_constraints_wrong_vector_dim() {
    let ball1 = Ball2::new(None, 1.0);
    let ball2 = Ball2::new(None, 0.5);
    let cart_prod = CartesianProduct::new()
        .add_constraint(3, ball1)
        .add_constraint(10, ball2);
    let mut x = [0.0; 30];
    cart_prod.project(&mut x).unwrap();
}

#[test]
fn t_cartesian_product_constraints() {
    let radius1 = 1.0;
    let radius2 = 0.5;
    let idx1 = 3;
    let idx2 = 5;
    let ball1 = Ball2::new(None, radius1);
    let ball2 = Ball2::new(None, radius2);
    let cart_prod = CartesianProduct::new()
        .add_constraint(idx1, ball1)
        .add_constraint(idx2, ball2);
    let mut x = [3.0, 4.0, 5.0, 2.0, 1.0];
    cart_prod.project(&mut x).unwrap();
    let r1 = crate::matrix_operations::norm2(&x[0..idx1]);
    let r2 = crate::matrix_operations::norm2(&x[idx1..idx2]);
    unit_test_utils::assert_nearly_equal(r1, radius1, 1e-8, 1e-12, "r1 is wrong");
    unit_test_utils::assert_nearly_equal(r2, radius2, 1e-8, 1e-12, "r2 is wrong");
}

#[test]
fn t_cartesian_product_ball_and_rectangle() {
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
    let cart_prod = CartesianProduct::new()
        .add_constraint(3, rectangle1)
        .add_constraint(7, ball)
        .add_constraint(9, rectangle2);

    /* Projection */
    let mut x = [-10.0, 0.5, 10.0, 0.01, -0.01, 0.1, 10.0, -1.0, 1.0];
    cart_prod.project(&mut x).unwrap();

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

#[test]
fn t_cartesian_product_ball_and_rectangle_f32() {
    let xmin1 = vec![-1.0_f32; 2];
    let xmax1 = vec![1.0_f32; 2];
    let rectangle1 = Rectangle::new(Some(&xmin1), Some(&xmax1));

    let radius = 1.0_f32;
    let ball = Ball2::new(None, radius);

    let cart_prod = CartesianProduct::new()
        .add_constraint(2, rectangle1)
        .add_constraint(5, ball);

    let mut x = [-4.0_f32, 0.25_f32, 2.0_f32, -1.0_f32, 2.0_f32];
    cart_prod.project(&mut x);

    assert_eq!([-1.0_f32, 0.25_f32], x[..2]);
    let ball_norm = crate::matrix_operations::norm2(&x[2..5]);
    assert!((ball_norm - radius).abs() < 1e-5_f32);
}

#[test]
fn t_second_order_cone_case_i() {
    let soc = SecondOrderCone::new(1.0);
    let mut x = vec![1.0, 1.0, 1.42];
    let x_copy = x.clone();
    soc.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&x, &x_copy, 1e-10, 1e-12, "x has been modified");
}

#[test]
fn t_second_order_cone_case_ii() {
    let alpha = 0.5;
    let soc = SecondOrderCone::new(alpha);
    let mut x = vec![1.0, 1.0, -0.71];
    soc.project(&mut x).unwrap();
    let expected = vec![0.0; 3];
    unit_test_utils::assert_nearly_equal_array(
        &x,
        &expected,
        1e-10,
        1e-12,
        "wrong result (should be zero)",
    );
}

#[test]
fn t_second_order_cone_case_iii() {
    let alpha = 1.5;
    let soc = SecondOrderCone::new(alpha);
    let mut x = vec![1.0, 1.0, 0.1];
    soc.project(&mut x).unwrap();
    // make sure the new `x` is in the cone
    let norm_z: f64 = crate::matrix_operations::norm2(&x[..=1]);
    assert!(norm_z <= alpha * x[2]);
    // in fact the projection should be on the boundary of the cone
    assert!((norm_z - alpha * x[2]).abs() <= 1e-7);
}

#[test]
fn t_second_order_cone_case_iii_f32() {
    let alpha = 1.5_f32;
    let soc = SecondOrderCone::new(alpha);
    let mut x = vec![1.0_f32, 1.0_f32, 0.1_f32];
    soc.project(&mut x);
    let norm_z = crate::matrix_operations::norm2(&x[..=1]);
    assert!(norm_z <= alpha * x[2] + 1e-5_f32);
    assert!((norm_z - alpha * x[2]).abs() <= 1e-4_f32);
}

#[test]
#[should_panic]
fn t_second_order_cone_illegal_alpha_i() {
    let alpha = 0.0;
    let _soc = SecondOrderCone::new(alpha);
}

#[test]
#[should_panic]
fn t_second_order_cone_illegal_alpha_ii() {
    let alpha = -1.0;
    let _soc = SecondOrderCone::new(alpha);
}

#[test]
#[should_panic]
fn t_second_order_cone_short_vector() {
    let alpha = 1.0;
    let soc = SecondOrderCone::new(alpha);
    let mut _x = vec![1.0];
    soc.project(&mut _x).unwrap();
}

#[test]
fn t_cartesian_product_dimension() {
    let data: &[&[f64]] = &[&[0.0, 0.0], &[1.0, 1.0]];
    let finite_set = FiniteSet::new(data);
    let finite_set_2 = finite_set;
    let ball = Ball2::new(None, 1.0);
    let no_constraints = NoConstraints::new();
    let cartesian = CartesianProduct::new_with_capacity(4)
        .add_constraint(2, finite_set)
        .add_constraint(4, finite_set_2)
        .add_constraint(7, no_constraints)
        .add_constraint(10, ball);
    assert!(10 == cartesian.dimension());

    // let's do a projection to make sure this works
    // Note: we've used the same set (finite_set), twice
    let mut x = [-0.5, 1.1, 0.45, 0.55, 10.0, 10.0, -500.0, 1.0, 1.0, 1.0];
    cartesian.project(&mut x).unwrap();
    println!("X = {:#?}", x);
    let sqrt_3_over_3 = 3.0_f64.sqrt() / 3.;
    unit_test_utils::assert_nearly_equal_array(
        &x,
        &[
            0.0,
            0.0,
            1.0,
            1.0,
            10.0,
            10.0,
            -500.0,
            sqrt_3_over_3,
            sqrt_3_over_3,
            sqrt_3_over_3,
        ],
        1e-10,
        1e-12,
        "wrong projection on cartesian product",
    );
}

#[test]
fn t_cartesian_ball_no_constraint() {
    let xc = [1., 0., 0.];
    let radius = 1.0;
    let ball2 = Ball2::new(Some(&xc), radius);
    let no_constraints = NoConstraints::new();
    let cartesian = CartesianProduct::new_with_capacity(4)
        .add_constraint(2, no_constraints)
        .add_constraint(5, ball2)
        .add_constraint(8, no_constraints)
        .add_constraint(9, no_constraints);
    assert_eq!(9, cartesian.dimension());
    let mut x = [100., -200., 0.5, 1.5, 3.5, 1000., 5., -500., 2_000_000.];
    cartesian.project(&mut x).unwrap();
    let x_proj_ball = [0.869811089019176, 0.390566732942472, 0.911322376865767];
    unit_test_utils::assert_nearly_equal_array(
        &x[0..=1],
        &[100., -200.],
        1e-10,
        1e-15,
        "projection on no constraints is wrong",
    );
    unit_test_utils::assert_nearly_equal_array(&x[2..=4], &x_proj_ball, 1e-8, 1e-15, "haha");
    unit_test_utils::assert_nearly_equal_array(
        &x[5..=8],
        &[1000., 5., -500., 2_000_000.],
        1e-10,
        1e-5,
        "projection on no constraints is wrong",
    );
}

#[test]
fn t_ball_inf_origin() {
    let ball_inf = BallInf::new(None, 1.0);
    let mut x = [0.0, -0.5, 0.5, 1.5, 3.5, 0.8, 1.1, -5.0, -10.0];
    let x_correct = [0.0, -0.5, 0.5, 1.0, 1.0, 0.8, 1.0, -1.0, -1.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x_correct,
        &x,
        1e-10,
        1e-12,
        "projection on ball inf",
    );
    println!("x = {:#?}", x);
}

#[test]
fn t_ball_inf_center() {
    let xc = [5.0, -6.0];
    let ball_inf = BallInf::new(Some(&xc), 1.5);
    let mut x = [11.0, -0.5];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[6.5, -4.5], &x, 1e-10, 1e-12, "upper right");

    let mut x = [3.0, -7.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[3.5, -7.0], &x, 1e-10, 1e-12, "left");

    let mut x = [800.0, -5.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[6.5, -5.0], &x, 1e-10, 1e-12, "right");

    let mut x = [9.0, -10.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[6.5, -7.5], &x, 1e-10, 1e-12, "down right");

    let mut x = [3.0, 0.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[3.5, -4.5], &x, 1e-10, 1e-12, "top left");

    let mut x = [6.0, -5.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[6.0, -5.0], &x, 1e-10, 1e-12, "inside");

    let mut x = [5.0, -6.0];
    ball_inf.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(&[5.0, -6.0], &x, 1e-10, 1e-12, "centre");
}

#[test]
fn t_ball_inf_boundary_no_change() {
    let ball_inf = BallInf::new(None, 1.0);
    let mut x = [-1.0, 0.2, 1.0];
    let x_expected = x;
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&x_expected, &x, 1e-12, 1e-12, "wrong result");
}

#[test]
fn t_is_convex_ball_inf() {
    let ball_inf = BallInf::new(None, 1.5);
    assert!(ball_inf.is_convex());
}

#[test]
fn t_is_convex_ball2() {
    let ball_2 = Ball2::new(None, 1.0);
    assert!(ball_2.is_convex());
}

#[test]
fn t_is_convex_finite_set() {
    let finite = FiniteSet::new(&[&[1.0, 2.0, 3.0]]);
    assert!(finite.is_convex());

    let finite_noncvx = FiniteSet::new(&[&[1.0, 2.0], &[3.0, 4.0]]);
    assert!(!finite_noncvx.is_convex());
}

#[test]
fn t_is_convex_soc() {
    let soc = SecondOrderCone::new(2.0);
    assert!(soc.is_convex());
}

#[test]
fn t_is_convex_zero() {
    let zero = Zero::new();
    assert!(<Zero as Constraint<f64>>::is_convex(&zero));
}

#[test]
fn t_is_convex_halfspace() {
    let normal_vector = vec![1.0, 2.0, 4.0];
    let offset = 1.0;
    let halfspace = Halfspace::new(&normal_vector, offset);
    assert!(halfspace.is_convex());
}

#[test]
fn t_is_convex_cartesian_product() {
    let ball_2 = Ball2::new(None, 1.0);
    let ball_inf = BallInf::new(None, 1.5);
    let finite = FiniteSet::new(&[&[1.0, 2.0, 3.0]]);
    let cartesian_product = CartesianProduct::new()
        .add_constraint(4, ball_2)
        .add_constraint(6, ball_inf)
        .add_constraint(9, finite);
    assert!(cartesian_product.is_convex());

    let finite_noncvx = FiniteSet::new(&[&[1.0, 2.0], &[3.0, 4.0]]);
    let cartesian_product = cartesian_product.add_constraint(10, finite_noncvx);
    assert!(!cartesian_product.is_convex());
}

#[test]
fn t_hyperplane_is_convex() {
    let normal_vector = [1.0, 2.0, 3.0];
    let offset = 1.0;
    let hyperplane = Hyperplane::new(&normal_vector, offset);
    assert!(hyperplane.is_convex());
}

#[test]
fn t_simplex_projection() {
    let mut x = [1.0, 2.0, 3.0];
    let alpha = 3.0;
    let my_simplex = Simplex::new(alpha);
    my_simplex.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal(
        crate::matrix_operations::sum(&x),
        alpha,
        1e-8,
        1e-10,
        "sum of projected vector not equal to alpha",
    );
}

#[test]
fn t_simplex_projection_f32() {
    let mut x = [1.0_f32, 2.0, 3.0];
    let alpha = 3.0_f32;
    let simplex = Simplex::new(alpha);
    simplex.project(&mut x);

    let sum = x[0] + x[1] + x[2];
    assert!((sum - alpha).abs() < 1e-5);
    assert!(x.iter().all(|&xi| xi >= -1e-6));
}

#[test]
fn t_halfspace_boundary_no_change() {
    let normal_vector = [1.0, 2.0];
    let offset = 5.0;
    let halfspace = Halfspace::new(&normal_vector, offset);
    let mut x = [1.0, 2.0];
    let x_expected = x;
    halfspace.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&x_expected, &x, 1e-12, 1e-12, "wrong result");
}

#[test]
fn t_simplex_projection_random_spam() {
    let n = 10;
    let n_trials = 1000;
    for _ in 0..n_trials {
        let mut x = vec![0.0; n];
        let scale = 10.;
        x.iter_mut()
            .for_each(|xi| *xi = scale * (2. * rand::random::<f64>() - 1.));
        let alpha_scale = 20.;
        let alpha = alpha_scale * rand::random::<f64>();
        let simplex = Simplex::new(alpha);
        simplex.project(&mut x).unwrap();
        println!("x = {:?}", x);
        assert!(x.iter().all(|&xi| xi >= -1e-12));
        unit_test_utils::assert_nearly_equal(
            crate::matrix_operations::sum(&x),
            alpha,
            1e-8,
            1e-10,
            "sum of projected vector not equal to alpha",
        );
    }
}

#[test]
fn t_simplex_projection_random_optimality() {
    for n in (10..=60).step_by(10) {
        for _ in 0..10 * n {
            let mut z = vec![0.0; n];
            let scale = 1000.;
            z.iter_mut()
                .for_each(|xi| *xi = scale * (2. * rand::random::<f64>() - 1.));
            let alpha_scale = 100.;
            let alpha = alpha_scale * rand::random::<f64>();
            let simplex = Simplex::new(alpha);
            let y = z.clone();
            simplex.project(&mut z).unwrap();
            for j in 0..n {
                let w = alpha * (y[j] - z[j]) - crate::matrix_operations::inner_product(&z, &y)
                    + crate::matrix_operations::norm2_squared(&z);
                let norm_z = crate::matrix_operations::norm_inf(&z);
                let norm_diff_y_z = crate::matrix_operations::norm_inf_diff(&y, &z);
                assert!(
                    w <= 1e-9 * (1. + f64::max(norm_z, norm_diff_y_z)),
                    "optimality conditions failed for simplex"
                );
            }
        }
    }
}

#[test]
#[should_panic]
fn t_simplex_alpha_zero() {
    let _ = Simplex::new(0.);
}

#[test]
#[should_panic]
fn t_simplex_alpha_negative() {
    let _ = Simplex::new(-1.);
}

#[test]
#[should_panic]
fn t_simplex_empty_vector() {
    let simplex = Simplex::new(1.0);
    let mut x = [];
    simplex.project(&mut x).unwrap();
}

#[test]
fn t_ball1_random_optimality_conditions() {
    for n in (10..=60).step_by(10) {
        let n_trials = 1000;
        for _ in 0..n_trials {
            let mut x = vec![0.0; n];
            let mut x_star = vec![0.0; n];
            let scale = 20.;
            x_star
                .iter_mut()
                .for_each(|xi| *xi = scale * (2. * rand::random::<f64>() - 1.));
            x.copy_from_slice(&x_star);
            let radius = 5. * rand::random::<f64>();
            let ball1 = Ball1::new(None, radius);
            ball1.project(&mut x_star).unwrap();
            // make sure |x|_1 <= radius
            assert!(
                crate::matrix_operations::norm1(&x_star) <= radius * (1. + 1e-9),
                "norm(x, 1) > radius"
            );
            // check the optimality conditions
            for j in 0..n {
                let w = radius * (x[j] - x_star[j])
                    - crate::matrix_operations::inner_product(&x, &x_star)
                    + crate::matrix_operations::norm2_squared(&x_star);
                let norm_x_star = crate::matrix_operations::norm_inf(&x_star);
                let norm_diff_x_x_star = crate::matrix_operations::norm_inf_diff(&x, &x_star);
                assert!(
                    w <= 1e-10 * (1. + f64::max(norm_x_star, norm_diff_x_x_star)),
                    "optimality conditions failed for ball1"
                );
            }
            // and of course...
            for j in 0..n {
                let w = -radius * (x[j] - x_star[j])
                    - crate::matrix_operations::inner_product(&x, &x_star)
                    + crate::matrix_operations::norm2_squared(&x_star);
                let norm_x_star = crate::matrix_operations::norm_inf(&x_star);
                let norm_diff_x_x_star = crate::matrix_operations::norm_inf_diff(&x, &x_star);
                assert!(
                    w <= 1e-10 * (1. + f64::max(norm_x_star, norm_diff_x_x_star)),
                    "optimality conditions failed for ball1"
                );
            }
        }
    }
}

#[test]
fn t_ball1_projection_f32() {
    let ball1 = Ball1::new(None, 1.0_f32);
    let mut x = [2.0_f32, -1.0_f32, 0.0_f32];
    ball1.project(&mut x);
    assert!((x[0] - 1.0_f32).abs() < 1e-6_f32);
    assert!(x[1].abs() < 1e-6_f32);
    assert!(x[2].abs() < 1e-6_f32);
    assert!(crate::matrix_operations::norm1(&x) <= 1.0_f32 + 1e-6_f32);
}

#[test]
fn t_ball1_random_optimality_conditions_centered() {
    for n in (10..=60).step_by(10) {
        let n_trials = 1000;
        for _ in 0..n_trials {
            let mut x = vec![0.0; n];
            let mut xc = vec![0.0; n];
            let scale = 50.;
            let scale_xc = 10.;
            x.iter_mut()
                .for_each(|xi| *xi = scale * (2. * rand::random::<f64>() - 1.));
            xc.iter_mut()
                .for_each(|xi| *xi = scale_xc * (2. * rand::random::<f64>() - 1.));
            let radius = 5. * rand::random::<f64>();
            let ball1 = Ball1::new(Some(&xc), radius);
            ball1.project(&mut x).unwrap();
            // x = x - xc
            x.iter_mut()
                .zip(xc.iter())
                .for_each(|(xi, &xci)| *xi -= xci);
            assert!(
                crate::matrix_operations::norm1(&x) <= radius * (1. + 1e-9),
                "norm(x - xc, 1) > radius"
            );
        }
    }
}

#[test]
#[should_panic]
fn t_ball1_wrong_dimensions() {
    let xc = vec![1.0, 2.0];
    let mut x = vec![3.0, 4.0, 5.0];
    let radius = 1.0;
    let ball1 = Ball1::new(Some(&xc), radius);
    ball1.project(&mut x).unwrap();
}

#[test]
fn t_sphere2_no_center() {
    let radius = 0.9;
    let mut x_out = [1.0, 1.0];
    let mut x_in = [-0.3, -0.2];
    let unit_sphere = Sphere2::new(None, radius);
    unit_sphere.project(&mut x_out).unwrap();
    unit_sphere.project(&mut x_in).unwrap();
    let norm_out = crate::matrix_operations::norm2(&x_out);
    let norm_in = crate::matrix_operations::norm2(&x_in);
    unit_test_utils::assert_nearly_equal(radius, norm_out, 1e-10, 1e-12, "norm_out is not 1.0");
    unit_test_utils::assert_nearly_equal(radius, norm_in, 1e-10, 1e-12, "norm_in is not 1.0");
}

#[test]
fn t_sphere2_no_center_projection_of_zero() {
    let radius = 0.9;
    let mut x = [0.0, 0.0];
    let unit_sphere = Sphere2::new(None, radius);
    unit_sphere.project(&mut x).unwrap();
    let norm_result = crate::matrix_operations::norm2(&x);
    unit_test_utils::assert_nearly_equal(radius, norm_result, 1e-10, 1e-12, "norm_out is not 1.0");
}

#[test]
fn t_sphere2_center() {
    let radius = 1.3;
    let center = [-3.0, 5.0];
    let mut x = [1.0, 1.0];
    let unit_sphere = Sphere2::new(Some(&center), radius);

    unit_sphere.project(&mut x).unwrap();
    let mut x_minus_c = [0.0; 2];
    x.iter()
        .zip(center.iter())
        .zip(x_minus_c.iter_mut())
        .for_each(|((a, b), c)| {
            *c = a - b;
        });

    let norm_out = crate::matrix_operations::norm2(&x_minus_c);
    unit_test_utils::assert_nearly_equal(radius, norm_out, 1e-10, 1e-12, "norm_out is not 1.0");
}

#[test]
fn t_sphere2_center_projection_of_center() {
    let radius = 1.3;
    let center = [-3.0, 5.0];
    let mut x = [-3.0, 5.0];
    let unit_sphere = Sphere2::new(Some(&center), radius);

    unit_sphere.project(&mut x).unwrap();
    let mut x_minus_c = [0.0; 2];
    x.iter()
        .zip(center.iter())
        .zip(x_minus_c.iter_mut())
        .for_each(|((a, b), c)| {
            *c = a - b;
        });

    let norm_out = crate::matrix_operations::norm2(&x_minus_c);
    unit_test_utils::assert_nearly_equal(radius, norm_out, 1e-10, 1e-12, "norm_out is not 1.0");
}

#[test]
#[should_panic]
fn t_sphere2_empty_vector() {
    let radius = 1.0;
    let unit_sphere = Sphere2::new(None, radius);
    let mut x = [];
    unit_sphere.project(&mut x).unwrap();
}

#[test]
#[should_panic]
fn t_sphere2_center_wrong_dimension() {
    let radius = 1.0;
    let center = [1.0, 2.0, 3.0];
    let unit_sphere = Sphere2::new(Some(&center), radius);
    let mut x = [1.0, 2.0];
    unit_sphere.project(&mut x).unwrap();
}

#[test]
#[should_panic]
fn t_ball1_alpha_negative() {
    let _ = Ball1::new(None, -1.);
}

#[test]
fn t_epigraph_squared_norm_inside() {
    let epi = EpigraphSquaredNorm::new();
    let mut x = [1., 2., 10.];
    epi.project(&mut x).unwrap();
    let x_correct = x;
    unit_test_utils::assert_nearly_equal_array(
        &x_correct,
        &x,
        1e-12,
        1e-14,
        "wrong projection on epigraph of squared norm",
    );
}

#[test]
fn t_epigraph_squared_norm_boundary_no_change() {
    let epi = EpigraphSquaredNorm::new();
    let mut x = [1.0, 2.0, 5.0];
    let x_expected = x;
    epi.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&x_expected, &x, 1e-12, 1e-12, "wrong result");
}

#[test]
fn t_epigraph_squared_norm() {
    let epi = EpigraphSquaredNorm::new();
    for i in 0..100 {
        let t = 0.01 * i as f64;
        let mut x = [1., 2., 3., t];
        epi.project(&mut x).unwrap();
        let err = (matrix_operations::norm2_squared(&x[..3]) - x[3]).abs();
        assert!(err < 1e-10, "wrong projection on epigraph of squared norm");
    }
}

#[test]
fn t_epigraph_squared_norm_correctness() {
    let epi = EpigraphSquaredNorm::new();
    let mut x = [1., 2., 3., 4.];
    let x_correct = [
        0.560142228903570,
        1.120_284_457_807_14,
        1.680426686710711,
        4.392630432414829,
    ];
    epi.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x_correct,
        &x,
        1e-12,
        1e-14,
        "wrong projection on epigraph of squared norm",
    );
}

#[test]
fn t_epigraph_squared_norm_f32() {
    let epi = EpigraphSquaredNorm::new();
    let mut x = [1.0_f32, 0.0, 0.0];
    epi.project(&mut x);
    let err = (matrix_operations::norm2_squared(&x[..2]) - x[2]).abs();
    assert!(err < 1e-4);
}

#[test]
fn t_affine_space() {
    let a = vec![
        0.5, 0.1, 0.2, -0.3, -0.6, 0.3, 0., 0.5, 1.0, 0.1, -1.0, -0.4,
    ];
    let b = vec![1., 2., -0.5];
    let affine_set = AffineSpace::new(a, b);
    let mut x = [1., -2., -0.3, 0.5];
    affine_set.project(&mut x).unwrap();
    let x_correct = [
        1.888564346697095,
        5.629857182200888,
        1.796_204_902_230_79,
        2.888362906715977,
    ];
    unit_test_utils::assert_nearly_equal_array(
        &x_correct,
        &x,
        1e-10,
        1e-12,
        "projection on affine set is wrong",
    );
}

#[test]
fn t_affine_space_f32() {
    let a = vec![
        0.5_f32, 0.1, 0.2, -0.3, -0.6, 0.3, 0.0, 0.5, 1.0, 0.1, -1.0, -0.4,
    ];
    let b = vec![1.0_f32, 2.0, -0.5];
    let affine_set = AffineSpace::new(a.clone(), b.clone());
    let mut x = [1.0_f32, -2.0, -0.3, 0.5];
    affine_set.project(&mut x);

    let x_correct = [
        1.888_564_3_f32,
        5.629_857_f32,
        1.796_204_9_f32,
        2.888_363_f32,
    ];
    assert!((x[0] - x_correct[0]).abs() < 1e-4_f32);
    assert!((x[1] - x_correct[1]).abs() < 1e-4_f32);
    assert!((x[2] - x_correct[2]).abs() < 1e-4_f32);
    assert!((x[3] - x_correct[3]).abs() < 1e-4_f32);

    for (row, bi) in a.chunks_exact(4).zip(b.iter()) {
        let ax_i = row
            .iter()
            .zip(x.iter())
            .fold(0.0_f32, |sum, (aij, xj)| sum + (*aij) * (*xj));
        assert!((ax_i - *bi).abs() < 1e-4_f32);
    }
}

#[test]
fn t_affine_space_projection_feasibility() {
    let a = vec![
        0.5, 0.1, 0.2, -0.3, -0.6, 0.3, 0., 0.5, 1.0, 0.1, -1.0, -0.4,
    ];
    let b = vec![1., 2., -0.5];
    let affine_set = AffineSpace::new(a.clone(), b.clone());
    let mut x = [1., -2., -0.3, 0.5];
    affine_set.project(&mut x);
    let residual = [
        a[0] * x[0] + a[1] * x[1] + a[2] * x[2] + a[3] * x[3] - b[0],
        a[4] * x[0] + a[5] * x[1] + a[6] * x[2] + a[7] * x[3] - b[1],
        a[8] * x[0] + a[9] * x[1] + a[10] * x[2] + a[11] * x[3] - b[2],
    ];
    assert!(
        crate::matrix_operations::norm_inf(&residual) <= 1e-10,
        "projection does not satisfy Ax = b"
    );
}

#[test]
fn t_affine_space_larger() {
    let a = vec![
        1.0f64, 1., 1., 0., 0., 0., 1., 1., 1., 0., 0., 0., 1., 1., 1., -1., 4., -1., 0., 2.,
    ];
    let b = vec![1., -2., 3., 4.];
    let affine_set = AffineSpace::new(a, b);
    let mut x = [10., 11., -9., 4., 5.];
    affine_set.project(&mut x).unwrap();
    let x_correct = [
        9.238095238095237,
        -0.714285714285714,
        -7.523809523809524,
        6.238095238095238,
        4.285714285714288,
    ];
    unit_test_utils::assert_nearly_equal_array(
        &x_correct,
        &x,
        1e-10,
        1e-12,
        "projection on affine set is wrong",
    );
}

#[test]
fn t_affine_space_single_row() {
    let a = vec![1., 1., 1., 1.];
    let b = vec![1.];
    let affine_set = AffineSpace::new(a, b);
    let mut x = [5., 6., 10., 25.];
    affine_set.project(&mut x).unwrap();
    let s = x.iter().sum();
    unit_test_utils::assert_nearly_equal(1., s, 1e-12, 1e-14, "wrong sum");
}

#[test]
fn t_affine_space_try_new() {
    let a = vec![
        0.5, 0.1, 0.2, -0.3, -0.6, 0.3, 0., 0.5, 1.0, 0.1, -1.0, -0.4,
    ];
    let b = vec![1., 2., -0.5];
    let affine_set = AffineSpace::try_new(a, b);
    assert!(affine_set.is_ok(), "try_new should succeed on valid data");
}

#[test]
fn t_affine_space_try_new_empty_b() {
    let a = vec![1.0, 2.0];
    let b = vec![];
    let affine_set = AffineSpace::<f64>::try_new(a, b);
    assert!(matches!(affine_set, Err(AffineSpaceError::EmptyB)));
}

#[test]
fn t_affine_space_try_new_wrong_dimensions() {
    let a = vec![0.5, 0.1, 0.2, -0.3, -0.6, 0.3, 0., 0.5, 1.0, 0.1, -1.0];
    let b = vec![1., 2., -0.5];
    let affine_set = AffineSpace::try_new(a, b);
    assert!(matches!(
        affine_set,
        Err(AffineSpaceError::IncompatibleDimensions)
    ));
}

#[test]
fn t_affine_space_try_new_rank_deficient() {
    let a = vec![1.0, 2.0, 2.0, 4.0];
    let b = vec![1.0, 2.0];
    let affine_set = AffineSpace::try_new(a, b);
    assert!(matches!(affine_set, Err(AffineSpaceError::NotFullRowRank)));
}

#[test]
#[should_panic]
fn t_affine_space_wrong_dimensions() {
    let a = vec![0.5, 0.1, 0.2, -0.3, -0.6, 0.3, 0., 0.5, 1.0, 0.1, -1.0];
    let b = vec![1., 2., -0.5];
    let _ = AffineSpace::new(a, b);
}

/// Sample `n_points` random points on the `l_p` sphere in `R^dim`
///
/// Assumes:
/// - `p > 0.0`
/// - `radius > 0.0`
/// - `dim > 0`
///
/// Returns a vector of points, each point being a `Vec<f64>` of length `dim`.
pub fn sample_lp_sphere(n_points: usize, dim: usize, p: f64, radius: f64) -> Vec<Vec<f64>> {
    assert!(n_points > 0);
    assert!(dim > 0);
    assert!(p > 0.0);
    assert!(radius > 0.0);

    let mut rng = rand::rng();

    // Y_i ~ Gamma(shape=1/p, scale=1)
    let gamma = Gamma::new(1.0 / p, 1.0).unwrap();

    let mut points = Vec::with_capacity(n_points);

    for _ in 0..n_points {
        let mut g = Vec::with_capacity(dim);

        for _ in 0..dim {
            let y: f64 = gamma.sample(&mut rng);
            let sign = if rng.random_bool(0.5) { 1.0 } else { -1.0 };
            let value = sign * y.powf(1.0 / p);
            g.push(value);
        }

        let norm_p = g
            .iter()
            .map(|x: &f64| x.abs().powf(p))
            .sum::<f64>()
            .powf(1.0 / p);

        let point: Vec<f64> = g.into_iter().map(|x| radius * x / norm_p).collect();

        points.push(point);
    }

    points
}

#[test]
fn t_sample_lp_sphere_points_have_correct_norm() {
    let n_points = 10_000;
    let dim = 5;
    let p = 3.2;
    let radius = 0.9;

    let samples = sample_lp_sphere(n_points, dim, p, radius);

    assert_eq!(samples.len(), n_points);

    for point in samples.iter() {
        assert_eq!(point.len(), dim);

        let norm_p = point
            .iter()
            .map(|xi| xi.abs().powf(p))
            .sum::<f64>()
            .powf(1.0 / p);

        unit_test_utils::assert_nearly_equal(
            radius,
            norm_p,
            1e-10,
            1e-12,
            "sample_lp_sphere produced a point with incorrect p-norm",
        );
    }
}

/// Check if a given vector `x_candidate_proj` is actually the projection
/// of `x` onto the p-norm ball centered at the origin with a given radius
///
/// This is based on taking `sample_points` on the sphere of the lp-ball.
///
/// Note that this test is stochastic, so it only guarantees the correctness
/// of the projection in probability.
fn is_norm_p_projection(
    x: &[f64],
    x_candidate_proj: &[f64],
    p: f64,
    radius: f64,
    sample_points: usize,
) -> bool {
    let n = x.len();
    assert_eq!(n, x_candidate_proj.len());

    // Make sure ||x||_p ≤ radius + ε, where ε is a small tolerance
    let feasibility_tol = 1e-10;
    let inner_prod_tol = 1e-10;
    let norm_proj = x_candidate_proj
        .iter()
        .map(|xi| xi.abs().powf(p))
        .sum::<f64>()
        .powf(1.0 / p);
    if norm_proj > radius + feasibility_tol {
        return false;
    }

    // e = x - x_candidate_proj
    let e: Vec<f64> = x
        .iter()
        .zip(x_candidate_proj.iter())
        .map(|(xi, yi)| xi - yi)
        .collect();
    let samples = sample_lp_sphere(sample_points, n, p, radius);
    for xi in samples.iter() {
        // w = x_candidate_proj - xi
        let w: Vec<f64> = x_candidate_proj
            .iter()
            .zip(xi.iter())
            .map(|(xproj_i, xi_i)| xproj_i - xi_i)
            .collect();
        let inner = matrix_operations::inner_product(&w, &e);
        if inner < inner_prod_tol {
            return false;
        }
    }
    true
}

fn is_norm_p_projection_with_tol(
    x: &[f64],
    x_candidate_proj: &[f64],
    p: f64,
    radius: f64,
    sample_points: usize,
    feasibility_tol: f64,
    inner_prod_tol: f64,
) -> bool {
    let n = x.len();
    assert_eq!(n, x_candidate_proj.len());

    let norm_proj = x_candidate_proj
        .iter()
        .map(|xi| xi.abs().powf(p))
        .sum::<f64>()
        .powf(1.0 / p);
    if norm_proj > radius + feasibility_tol {
        return false;
    }

    let e: Vec<f64> = x
        .iter()
        .zip(x_candidate_proj.iter())
        .map(|(xi, yi)| xi - yi)
        .collect();
    let samples = sample_lp_sphere(sample_points, n, p, radius);
    for xi in samples.iter() {
        let w: Vec<f64> = x_candidate_proj
            .iter()
            .zip(xi.iter())
            .map(|(xproj_i, xi_i)| xproj_i - xi_i)
            .collect();
        let inner = matrix_operations::inner_product(&w, &e);
        if inner < -inner_prod_tol {
            return false;
        }
    }
    true
}

fn as_f64_vec<T: ToPrimitive>(x: &[T]) -> Vec<f64> {
    x.iter()
        .map(|xi| {
            xi.to_f64()
                .expect("test float values must be convertible to f64")
        })
        .collect()
}

fn lp_norm_generic<T: Float>(x: &[T], p: T) -> T {
    x.iter()
        .map(|xi| xi.abs().powf(p))
        .fold(T::zero(), |sum, xi| sum + xi)
        .powf(T::one() / p)
}

fn random_vec<T: Float>(rng: &mut impl rand::Rng, len: usize, lower: f64, upper: f64) -> Vec<T> {
    (0..len)
        .map(|_| cast::<T>(rng.random_range(lower..upper)))
        .collect()
}

fn run_ballp_random_properties<T>()
where
    T: Float + ToPrimitive,
{
    let mut rng = rand::rng();
    let solver_tol = if T::epsilon() > cast::<T>(1e-10) {
        cast::<T>(1e-5)
    } else {
        cast::<T>(1e-12)
    };
    let feasibility_tol = if T::epsilon() > cast::<T>(1e-10) {
        cast::<T>(5e-3)
    } else {
        cast::<T>(1e-8)
    };
    let idempotence_tol = if T::epsilon() > cast::<T>(1e-10) {
        cast::<T>(2e-4)
    } else {
        cast::<T>(1e-10)
    };
    let inner_prod_tol = if T::epsilon() > cast::<T>(1e-10) {
        5e-3
    } else {
        1e-8
    };

    for &(dim, p_f64, radius_f64, with_center) in &[
        (3_usize, 1.7_f64, 1.1_f64, false),
        (4_usize, 2.5_f64, 0.9_f64, true),
        (5_usize, 3.4_f64, 1.4_f64, true),
    ] {
        for _ in 0..40 {
            let center = with_center.then(|| random_vec::<T>(&mut rng, dim, -1.5, 1.5));
            let mut x = random_vec::<T>(&mut rng, dim, -4.0, 4.0);
            let x_before = x.clone();
            let p = cast::<T>(p_f64);
            let radius = cast::<T>(radius_f64);
            let ball = BallP::new(center.as_deref(), radius, p, solver_tol, 300);
            ball.project(&mut x);

            let shifted_projection: Vec<T> = if let Some(center) = center.as_ref() {
                x.iter()
                    .zip(center.iter())
                    .map(|(xi, ci)| *xi - *ci)
                    .collect()
            } else {
                x.clone()
            };
            let proj_norm = lp_norm_generic(&shifted_projection, p);
            assert!(
                proj_norm <= radius + feasibility_tol,
                "projected point is not feasible for BallP"
            );

            let mut reproj = x.clone();
            ball.project(&mut reproj);
            let max_reproj_diff = reproj
                .iter()
                .zip(x.iter())
                .fold(T::zero(), |acc, (a, b)| acc.max((*a - *b).abs()));
            assert!(
                max_reproj_diff <= idempotence_tol,
                "BallP projection is not idempotent within tolerance"
            );

            let shifted_x_before: Vec<f64> = if let Some(center) = center.as_ref() {
                x_before
                    .iter()
                    .zip(center.iter())
                    .map(|(xi, ci)| {
                        (*xi - *ci)
                            .to_f64()
                            .expect("test float values must be convertible to f64")
                    })
                    .collect()
            } else {
                as_f64_vec(&x_before)
            };
            let shifted_projection_f64 = as_f64_vec(&shifted_projection);
            assert!(
                is_norm_p_projection_with_tol(
                    &shifted_x_before,
                    &shifted_projection_f64,
                    p_f64,
                    radius_f64,
                    500,
                    feasibility_tol
                        .to_f64()
                        .expect("test float values must be convertible to f64"),
                    inner_prod_tol,
                ),
                "BallP projection failed sampled optimality check"
            );
        }
    }
}

fn run_epigraph_squared_norm_random_properties<T>()
where
    T: Float + roots::FloatType + std::iter::Sum<T> + ToPrimitive,
{
    let mut rng = rand::rng();
    let feasibility_tol = if T::epsilon() > cast::<T>(1e-10) {
        cast::<T>(2e-4)
    } else {
        cast::<T>(1e-10)
    };
    let idempotence_tol = if T::epsilon() > cast::<T>(1e-10) {
        cast::<T>(2e-4)
    } else {
        cast::<T>(1e-10)
    };
    let vi_tol = if T::epsilon() > cast::<T>(1e-10) {
        2e-3
    } else {
        1e-8
    };
    let epi = EpigraphSquaredNorm::new();

    for dim in 2..=5 {
        for _ in 0..50 {
            let mut x = random_vec::<T>(&mut rng, dim, -3.0, 3.0);
            x.push(cast::<T>(rng.random_range(-2.0..4.0)));
            let x_before = as_f64_vec(&x);

            epi.project(&mut x);

            let z = &x[..dim];
            let t = x[dim];
            let norm_z_sq = matrix_operations::norm2_squared(z);
            assert!(
                norm_z_sq <= t + feasibility_tol,
                "Epigraph projection is not feasible"
            );

            let mut reproj = x.clone();
            epi.project(&mut reproj);
            let max_reproj_diff = reproj
                .iter()
                .zip(x.iter())
                .fold(T::neg_infinity(), |acc, (a, b)| {
                    acc.max(num::Float::abs(*a - *b))
                });
            assert!(
                max_reproj_diff <= idempotence_tol,
                "Epigraph projection is not idempotent within tolerance"
            );

            let proj_f64 = as_f64_vec(&x);
            let residual: Vec<f64> = x_before
                .iter()
                .zip(proj_f64.iter())
                .map(|(xb, xp)| xb - xp)
                .collect();

            for _ in 0..150 {
                let z_feasible: Vec<f64> = (0..dim).map(|_| rng.random_range(-3.0..3.0)).collect();
                let norm_z_sq_feasible = z_feasible.iter().map(|zi| zi * zi).sum::<f64>();
                let t_feasible = norm_z_sq_feasible + rng.random_range(0.0..3.0);
                let mut y = z_feasible;
                y.push(t_feasible);
                let diff: Vec<f64> = proj_f64
                    .iter()
                    .zip(y.iter())
                    .map(|(xp, yi)| xp - yi)
                    .collect();
                let inner = matrix_operations::inner_product(&diff, &residual);
                assert!(
                    inner >= -vi_tol,
                    "Epigraph projection failed sampled variational inequality"
                );
            }
        }
    }
}

fn assert_projection_idempotent<C: Constraint>(constraint: &C, x0: &[f64], message: &'static str) {
    let mut once = x0.to_vec();
    let mut twice = x0.to_vec();
    constraint.project(&mut once);
    constraint.project(&mut twice);
    constraint.project(&mut twice);
    unit_test_utils::assert_nearly_equal_array(&once, &twice, 1e-10, 1e-12, message);
}

#[test]
fn t_ballp_at_origin_projection() {
    let radius = 0.8;
    let mut x = [1.0, -1.0, 6.0];
    let x0 = x;
    let p = 3.;
    let tol = 1e-16;
    let max_iters: usize = 200;
    let ball = BallP::new(None, radius, p, tol, max_iters);
    ball.project(&mut x).unwrap();
    assert!(is_norm_p_projection(&x0, &x, p, radius, 10_000));
}

#[test]
fn t_ballp_at_origin_projection_preserves_signs() {
    let radius = 0.9;
    let mut x = [1.0, -3.0, 2.5, -0.7];
    let x0 = x;
    let ball = BallP::new(None, radius, 3.0, 1e-14, 200);
    ball.project(&mut x);
    for (proj, original) in x.iter().zip(x0.iter()) {
        assert!(proj.abs() <= original.abs() + 1e-12);
        if *original != 0.0 {
            assert_eq!(proj.signum(), original.signum());
        }
    }
}

#[test]
fn t_ballp_zero_coordinates_branch() {
    let radius = 0.7;
    let p = 3.5;
    let mut x = [0.0, -2.0, 0.0, 1.5];
    let x0 = x;
    let ball = BallP::new(None, radius, p, 1e-14, 300);
    ball.project(&mut x);
    assert_eq!(x[0], 0.0);
    assert_eq!(x[2], 0.0);
    assert!(is_norm_p_projection(&x0, &x, p, radius, 10_000));
}

#[test]
fn t_ballp_outside_projection_lands_on_boundary_for_multiple_p() {
    let test_cases = [
        (1.1, [2.0, -1.0, 0.5]),
        (1.5, [1.0, -2.0, 3.0]),
        (2.5, [3.0, -4.0, 1.0]),
        (10.0, [1.2, -0.7, 2.1]),
    ];
    let radius = 0.8;

    for (p, x_init) in test_cases {
        let mut x = x_init;
        let ball = BallP::new(None, radius, p, 1e-14, 400);
        ball.project(&mut x);
        let norm_p = x
            .iter()
            .map(|xi| xi.abs().powf(p))
            .sum::<f64>()
            .powf(1.0 / p);
        unit_test_utils::assert_nearly_equal(
            radius,
            norm_p,
            1e-9,
            1e-11,
            "projection should lie on the boundary",
        );
    }
}

#[test]
fn t_ballp_boundary_no_change() {
    let radius = 1.0;
    let p = 4.0;
    let mut x = [1.0, 0.0];
    let x_expected = x;
    let ball = BallP::new(None, radius, p, 1e-14, 200);
    ball.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&x_expected, &x, 1e-12, 1e-12, "wrong result");
}

#[test]
fn t_ballp_translated_projection_multiple_p_values() {
    let center = [1.0, -2.0, 0.5];
    let radius = 0.9;
    let cases = [
        (1.1, [3.0, -4.0, 2.0]),
        (1.5, [2.5, -0.5, 1.8]),
        (2.5, [4.0, -3.5, -1.0]),
        (10.0, [1.8, 0.5, 3.0]),
    ];

    for (p, x_init) in cases {
        let mut x = x_init;
        let ball = BallP::new(Some(&center), radius, p, 1e-14, 400);
        ball.project(&mut x);
        let norm_p = x
            .iter()
            .zip(center.iter())
            .map(|(xi, ci)| (xi - ci).abs().powf(p))
            .sum::<f64>()
            .powf(1.0 / p);
        unit_test_utils::assert_nearly_equal(
            radius,
            norm_p,
            1e-9,
            1e-11,
            "translated lp projection should lie on the boundary",
        );
    }
}

#[test]
fn t_halfspace_projection_is_idempotent() {
    let normal_vector = [1.0, 2.0];
    let halfspace = Halfspace::new(&normal_vector, 1.0);
    assert_projection_idempotent(
        &halfspace,
        &[-1.0, 3.0],
        "halfspace projection not idempotent",
    );
}

#[test]
fn t_rectangle_projection_is_idempotent() {
    let xmin = [-1.0, 0.0, -2.0];
    let xmax = [1.0, 2.0, 0.5];
    let rectangle = Rectangle::new(Some(&xmin), Some(&xmax));
    assert_projection_idempotent(
        &rectangle,
        &[-10.0, 1.5, 3.0],
        "rectangle projection not idempotent",
    );
}

#[test]
fn t_ball2_projection_is_idempotent() {
    let center = [0.5, -1.0];
    let ball = Ball2::new(Some(&center), 0.8);
    assert_projection_idempotent(&ball, &[3.0, 2.0], "ball2 projection not idempotent");
}

#[test]
fn t_ball_inf_projection_is_idempotent() {
    let center = [2.0, -3.0];
    let ball_inf = BallInf::new(Some(&center), 1.2);
    assert_projection_idempotent(&ball_inf, &[10.0, 1.0], "ballinf projection not idempotent");
}

#[test]
fn t_affine_space_projection_is_idempotent() {
    let a = vec![1.0, 1.0, 0.0, 1.0, -1.0, 2.0];
    let b = vec![1.0, 0.5];
    let affine_set = AffineSpace::new(a, b);
    assert_projection_idempotent(
        &affine_set,
        &[3.0, -2.0, 4.0],
        "affine-space projection not idempotent",
    );
}

#[test]
fn t_sphere2_projection_is_idempotent() {
    let center = [1.0, 1.0, -1.0];
    let sphere = Sphere2::new(Some(&center), 2.0);
    assert_projection_idempotent(
        &sphere,
        &[4.0, -2.0, 3.0],
        "sphere projection not idempotent",
    );
}

#[test]
fn t_ballp_projection_is_idempotent() {
    let center = [0.0, 1.0, -1.0];
    let ball = BallP::new(Some(&center), 0.75, 3.0, 1e-14, 300);
    assert_projection_idempotent(&ball, &[2.0, -3.0, 1.5], "ballp projection not idempotent");
}

#[test]
fn t_ballp_at_origin_x_already_inside() {
    let radius = 1.5;
    let mut x = [0.5, -0.2, 0.1];
    let x0 = x;
    let p = 3.;
    let tol = 1e-16;
    let max_iters: usize = 1200;
    let ball = BallP::new(None, radius, p, tol, max_iters);
    ball.project(&mut x).unwrap();
    unit_test_utils::assert_nearly_equal_array(
        &x0,
        &x,
        1e-12,
        1e-12,
        "wrong projection on lp-ball",
    );
}

#[test]
fn t_ballp_at_xc_projection() {
    let radius = 0.8;
    let mut x = [0.0, 0.1];
    let x_center = [1.0, 3.0];
    let p = 4.;
    let tol = 1e-16;
    let max_iters: usize = 200;
    let ball = BallP::new(Some(&x_center), radius, p, tol, max_iters);
    ball.project(&mut x).unwrap();

    let nrm: f64 = (x.iter().zip(x_center.iter()).fold(0.0_f64, |s, (x, y)| {
        let diff: f64 = *x - *y;
        diff.abs().powf(p) + s
    }))
    .powf(1.0_f64 / p);
    unit_test_utils::assert_nearly_equal(radius, nrm, 1e-10, 1e-12, "wrong distance to lp-ball");

    let proj_expected = [0.5178727276722618, 2.2277981662325224];
    unit_test_utils::assert_nearly_equal_array(
        &proj_expected,
        &x,
        1e-12,
        1e-12,
        "wrong projection on lp-ball centered at xc != 0",
    );
}

#[test]
fn t_ballp_at_xc_projection_f32() {
    let radius = 0.8_f32;
    let mut x = [0.0_f32, 0.1];
    let x_center = [1.0_f32, 3.0];
    let p = 4.0_f32;
    let tol = 1e-6_f32;
    let max_iters: usize = 200;
    let ball = BallP::new(Some(&x_center), radius, p, tol, max_iters);
    ball.project(&mut x);

    let nrm = x
        .iter()
        .zip(x_center.iter())
        .fold(0.0_f32, |s, (xi, yi)| s + (*xi - *yi).abs().powf(p))
        .powf(1.0_f32 / p);
    assert!((radius - nrm).abs() < 1e-4_f32);

    let proj_expected = [0.517_872_75_f32, 2.227_798_2_f32];
    assert!((x[0] - proj_expected[0]).abs() < 1e-4_f32);
    assert!((x[1] - proj_expected[1]).abs() < 1e-4_f32);
}

#[test]
fn t_ballp_random_properties_f64() {
    run_ballp_random_properties::<f64>();
}

#[test]
fn t_ballp_random_properties_f32() {
    run_ballp_random_properties::<f32>();
}

#[test]
fn t_epigraph_squared_norm_random_properties_f64() {
    run_epigraph_squared_norm_random_properties::<f64>();
}

#[test]
fn t_epigraph_squared_norm_random_properties_f32() {
    run_epigraph_squared_norm_random_properties::<f32>();
}

#[test]
#[should_panic]
fn t_rectangle_no_bounds() {
    let _rectangle = Rectangle::<f64>::new(None, None);
}

#[test]
#[should_panic]
fn t_rectangle_only_xmin_wrong_dimension() {
    let xmin = [1.0, 2.0, 3.0];
    let rectangle = Rectangle::new(Some(&xmin), None);
    let mut x = [0.0, 1.0];
    rectangle.project(&mut x);
}

#[test]
#[should_panic]
fn t_rectangle_only_xmax_wrong_dimension() {
    let xmax = [1.0, 2.0, 3.0];
    let rectangle = Rectangle::new(None, Some(&xmax));
    let mut x = [0.0, 1.0];
    rectangle.project(&mut x);
}

#[test]
#[should_panic]
fn t_halfspace_wrong_dimension() {
    let normal_vector = [1.0, 2.0, 3.0];
    let halfspace = Halfspace::new(&normal_vector, 1.0);
    let mut x = [1.0, 2.0];
    halfspace.project(&mut x);
}

#[test]
#[should_panic]
fn t_ball2_wrong_dimensions() {
    let center = [1.0, 2.0];
    let ball = Ball2::new(Some(&center), 1.0);
    let mut x = [1.0, 2.0, 3.0];
    ball.project(&mut x);
}

#[test]
#[should_panic]
fn t_ball2_nonpositive_radius() {
    let _ball = Ball2::new(None, 0.0);
}

#[test]
#[should_panic]
fn t_ball_inf_wrong_dimensions() {
    let center = [1.0, 2.0];
    let ball_inf = BallInf::new(Some(&center), 1.0);
    let mut x = [1.0, 2.0, 3.0];
    ball_inf.project(&mut x);
}

#[test]
#[should_panic]
fn t_ball_inf_nonpositive_radius() {
    let _ball_inf = BallInf::new(None, 0.0);
}

#[test]
#[should_panic]
fn t_epigraph_squared_norm_short_vector() {
    let epi = EpigraphSquaredNorm::new();
    let mut x = [1.0];
    epi.project(&mut x);
}

#[test]
#[should_panic]
fn t_affine_space_empty_b() {
    let _affine_set = AffineSpace::new(vec![1.0, 2.0], vec![]);
}

#[test]
#[should_panic]
fn t_affine_space_project_wrong_dimension() {
    let a = vec![1.0, 0.0, 0.0, 1.0];
    let b = vec![0.0, 0.0];
    let affine_set = AffineSpace::new(a, b);
    let mut x = [1.0];
    affine_set.project(&mut x);
}

#[test]
#[should_panic]
fn t_affine_space_rank_deficient_matrix() {
    let a = vec![1.0, 2.0, 1.0, 2.0];
    let b = vec![1.0, 1.0];
    let _affine_set = AffineSpace::new(a, b);
}

#[test]
fn t_is_convex_sphere2() {
    let sphere = Sphere2::new(None, 1.0);
    assert!(!sphere.is_convex());
}

#[test]
fn t_is_convex_no_constraints() {
    let whole_space = NoConstraints::new();
    assert!(<NoConstraints as Constraint<f64>>::is_convex(&whole_space));
}

#[test]
fn t_is_convex_rectangle() {
    let xmin = [-1.0, -2.0];
    let xmax = [1.0, 2.0];
    let rectangle = Rectangle::new(Some(&xmin), Some(&xmax));
    assert!(rectangle.is_convex());
}

#[test]
fn t_is_convex_simplex() {
    let simplex = Simplex::new(1.0);
    assert!(simplex.is_convex());
}

#[test]
fn t_is_convex_ball1() {
    let ball1 = Ball1::new(None, 1.0);
    assert!(ball1.is_convex());
}

#[test]
fn t_is_convex_ballp() {
    let ballp = BallP::new(None, 1.0, 3.0, 1e-12, 100);
    assert!(ballp.is_convex());
}

#[test]
fn t_is_convex_epigraph_squared_norm() {
    let epi = EpigraphSquaredNorm::new();
    assert!(<EpigraphSquaredNorm as Constraint<f64>>::is_convex(&epi));
}

#[test]
fn t_is_convex_affine_space() {
    let a = vec![1.0, 0.0, 0.0, 1.0];
    let b = vec![1.0, -1.0];
    let affine_set = AffineSpace::new(a, b);
    assert!(affine_set.is_convex());
}

#[test]
#[should_panic]
fn t_ballp_nonpositive_radius() {
    let _ballp = BallP::new(None, 0.0, 2.0, 1e-12, 100);
}

#[test]
#[should_panic]
fn t_ballp_exponent_too_small() {
    let _ballp = BallP::new(None, 1.0, 1.0, 1e-12, 100);
}

#[test]
#[should_panic]
fn t_ballp_nonfinite_exponent() {
    let _ballp = BallP::new(None, 1.0, f64::INFINITY, 1e-12, 100);
}

#[test]
#[should_panic]
fn t_ballp_nonpositive_tolerance() {
    let _ballp = BallP::new(None, 1.0, 2.0, 0.0, 100);
}

#[test]
#[should_panic]
fn t_ballp_zero_max_iters() {
    let _ballp = BallP::new(None, 1.0, 2.0, 1e-12, 0);
}

#[test]
#[should_panic]
fn t_ballp_wrong_dimensions() {
    let center = [1.0, 2.0];
    let ballp = BallP::new(Some(&center), 1.0, 3.0, 1e-12, 100);
    let mut x = [1.0, 2.0, 3.0];
    ballp.project(&mut x);
}
