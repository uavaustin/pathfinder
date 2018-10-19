use super::Node;

// Creates a vector from v to u
#[inline]
pub fn node_vector(v: &Node, u: &Node) -> (f32, f32) {
    ((u.x - v.x) as f32, (u.y - v.y) as f32)
}
#[inline]
pub fn cross_product(v: (f32, f32), u: (f32, f32)) -> f32 {
    v.0 * u.1 - u.0 * v.1
}
#[inline]
pub fn dot_product(v: (f32, f32), u: (f32, f32)) -> f32 {
    v.0 * u.0 + v.1 * u.1
}
// Calculates the distance between two nodes
#[inline]
pub fn node_distance(v: &Node, u: &Node) -> f32 {
    let (dx, dy) = node_vector(v, u);
    dx.hypot(dy)
}
#[inline]
// Using v as reference, calculate new vector u' such that the
// angle measurement between v and u' is half the angle between v and u.
pub fn halve_vector(v: (f32, f32), u: (f32, f32)) -> (f32, f32) {
    let mut angle = (dot_product(v, u) / (v.0.hypot(v.1) * u.0.hypot(u.1))).acos() / 2f32;
    println!("angle: {:?}", angle);
    if cross_product(v, u) < 0f32 {
        angle = -angle;
    }
    (
        angle.cos() * v.0 - angle.sin() * v.1,
        angle.sin() * v.0 + angle.cos() * v.1,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::{self, consts};

    macro_rules! assert_eq_vector {
        ($v:expr, $u:expr) => {{
            match (&$v, &$u) {
                ((v_x, v_y), (u_x, u_y)) => {
                    if !((v_x - u_x).abs() < f32::EPSILON && (v_y - u_y).abs() < f32::EPSILON) {
                        panic!(
                            r#"assertion failed: `(left ~= right)`
                     left: `{:?}`,
                    right: `{:?}`"#,
                            (v_x, v_y),
                            (u_x, u_y)
                        )
                    }
                }
            }
        }};
    }

    #[test]
    fn halve_vector_test_identity() {
        let v = (1f32, 0f32);
        let u = (1f32, 0f32);
        assert_eq_vector!(v, halve_vector(v, u));
    }

    #[test]
    fn halve_vector_test_left() {
        let v = (1f32, 0f32);
        let u = (0f32, 1f32);
        assert_eq_vector!(
            (consts::SQRT_2 / 2f32, consts::SQRT_2 / 2f32),
            halve_vector(v, u)
        );
        let u = (0.5, 3f32.sqrt() / 2f32);
        assert_eq_vector!((3f32.sqrt() / 2f32, 0.5), halve_vector(v, u));
        let v = (0.5, -3f32.sqrt() / 2f32);
        assert_eq_vector!((1f32, 0f32), halve_vector(v, u));
    }

    #[test]
    fn halve_vector_test_right() {
        let v = (1f32, 0f32);
        let u = (0f32, -1f32);
        assert_eq_vector!(
            (consts::SQRT_2 / 2f32, -consts::SQRT_2 / 2f32),
            halve_vector(v, u)
        );
        let u = (0.5, -3f32.sqrt() / 2f32);
        assert_eq_vector!((3f32.sqrt() / 2f32, -0.5), halve_vector(v, u));
        let v = (0.5, 3f32.sqrt() / 2f32);
        assert_eq_vector!((1f32, 0f32), halve_vector(v, u));
    }

    #[test]
    fn halve_vector_test_full() {
        let v = (1f32, 0f32);
        let u = (-1f32, 0f32);
        assert_eq_vector!((0f32, 1f32), halve_vector(v, u));
    }
}
