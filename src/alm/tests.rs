use crate::alm::*;
use crate::core::panoc::*;

#[test]
fn t_create_alm_cache() {
    let tolerance = 1e-8;
    let nx = 10;
    let n1 = 5;
    let n2 = 0;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let alm_cache = AlmCache::new(panoc_cache, n1, n2);
    assert!(alm_cache.iteration == 0, "iter != 0");
    if let Some(y) = alm_cache.y {
        assert!(y.len() == n1);
    } else {
        panic!("No y allocated");
    }
    if let Some(y) = alm_cache.y_plus {
        assert!(y.len() == n1);
    } else {
        panic!("No y_plus allocated");
    }
    assert!(
        alm_cache.w_pm.is_none(),
        "Memory for w_pm should not have been allocated"
    );
}
