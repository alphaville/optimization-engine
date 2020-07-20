use super::*;

#[test]
fn t_zero_set() {
    let zero = Zero::new();
    let mut x = [1.0, 2.0, 3.0];
    let x_projection = [0.0; 3];
    zero.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(
        &x_projection,
        &x,
        1e-12,
        1e-12,
        "wrong projection on zero set",
    );
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
    hyperplane.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(
        &x,
        &x_proj_expected,
        1e-8,
        1e-14,
        "halfspace projection failed",
    );
}

#[test]
fn t_halfspace_project_inside() {
    let normal_vector = [1., 2.];
    let offset = 5.0;
    let halfspace = Halfspace::new(&normal_vector, offset);
    let mut x = [-1., 3.];
    let x_expected = [-1., 3.];
    halfspace.project(&mut x);
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
    halfspace.project(&mut x);
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
    finite_set.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(
        &[1.0, 1.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [1,1])",
    );
    x = [-0.1, 0.2];
    finite_set.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(
        &[0.0, 0.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [0,0])",
    );
    x = [0.48, 0.501];
    finite_set.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(
        &[0.0, 1.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [0,1])",
    );
    x = [0.7, 0.2];
    finite_set.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(
        &[1.0, 0.0],
        &x,
        1e-10,
        1e-10,
        "projection is wrong (should be [1,0])",
    );
}

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
fn t_rectangle_infinite_bounds() {
    let xmin = [-1.0, 2.0, std::f64::NEG_INFINITY];
    let xmax = [1.0, std::f64::INFINITY, 5.0];
    let rectangle = Rectangle::new(Some(&xmin[..]), Some(&xmax[..]));
    let mut x = [-2.0, 3.0, 1.0];

    rectangle.project(&mut x);

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
fn t_ball_elsewhere() {
    let radius = 1.0;
    let center = [1.0, 1.0];
    let mut x = [2.0, 2.0];
    let ball = Ball2::new(Some(&center[..]), radius);

    ball.project(&mut x);

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
fn t_no_constraints() {
    let mut x = [1.0, 2.0, 3.0];
    let whole_space = NoConstraints::new();

    whole_space.project(&mut x);

    unit_test_utils::assert_nearly_equal_array(&[1., 2., 3.], &x, 1e-10, 1e-15, "x is wrong");
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
    cart_prod.project(&mut x);
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
    cart_prod.project(&mut x);
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

#[test]
fn t_second_order_cone_case_i() {
    let soc = SecondOrderCone::new(1.0);
    let mut x = vec![1.0, 1.0, 1.42];
    let x_copy = x.clone();
    soc.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&x, &x_copy, 1e-10, 1e-12, "x has been modified");
}

#[test]
fn t_second_order_cone_case_ii() {
    let alpha = 0.5;
    let soc = SecondOrderCone::new(alpha);
    let mut x = vec![1.0, 1.0, -0.71];
    soc.project(&mut x);
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
    soc.project(&mut x);
    // make sure the new `x` is in the cone
    let norm_z = crate::matrix_operations::norm2(&x[..=1]);
    assert!(norm_z <= alpha * x[2]);
    // in fact the projection should be on the boundary of the cone
    assert!((norm_z - alpha * x[2]).abs() <= 1e-7);
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
    soc.project(&mut _x);
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
    cartesian.project(&mut x);
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
    let radius = 0.5;
    let ball2 = Ball2::new(Some(&xc), radius);
    let no_constraints = NoConstraints::new();
    let cartesian = CartesianProduct::new_with_capacity(4)
        .add_constraint(2, no_constraints)
        .add_constraint(5, ball2)
        .add_constraint(8, no_constraints)
        .add_constraint(9, no_constraints);
    assert_eq!(9, cartesian.dimension());
    let mut x = [100., -200., 0.5, 1.5, 3.5, 1000., 5., -500., 2_000_000.];
    cartesian.project(&mut x);
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
    ball_inf.project(&mut x);
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
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[6.5, -4.5], &x, 1e-10, 1e-12, "upper right");

    let mut x = [3.0, -7.0];
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[3.5, -7.0], &x, 1e-10, 1e-12, "left");

    let mut x = [800.0, -5.0];
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[6.5, -5.0], &x, 1e-10, 1e-12, "right");

    let mut x = [9.0, -10.0];
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[6.5, -7.5], &x, 1e-10, 1e-12, "down right");

    let mut x = [3.0, 0.0];
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[3.5, -4.5], &x, 1e-10, 1e-12, "top left");

    let mut x = [6.0, -5.0];
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[6.0, -5.0], &x, 1e-10, 1e-12, "inside");

    let mut x = [5.0, -6.0];
    ball_inf.project(&mut x);
    unit_test_utils::assert_nearly_equal_array(&[5.0, -6.0], &x, 1e-10, 1e-12, "centre");
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
    assert!(zero.is_convex());
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
