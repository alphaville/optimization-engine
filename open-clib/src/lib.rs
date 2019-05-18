use libc::{c_double, c_int};
use optimization_engine::{constraints::*, panoc::*, *};
use std::num::NonZeroUsize;

pub struct PanocInstance {
    //cache: panoc::PANOCCache,
//optimizer: panoc::PANOCOptimizer,
}

impl PanocInstance {
    pub fn new() -> Self {
        PanocInstance {}
    }
}

#[no_mangle]
pub extern "C" fn panoc_new() -> *mut PanocInstance {
    // Add impl
    Box::into_raw(Box::new(PanocInstance::new()))
}

#[no_mangle]
pub extern "C" fn panoc_solve(ptr: *mut PanocInstance) {
    // Add impl
    if ptr.is_null() {
        return;
    }

    let p: &mut PanocInstance = unsafe { &mut *ptr };
}

#[no_mangle]
pub extern "C" fn panoc_free(ptr: *mut PanocInstance) {
    // Add impl
    if ptr.is_null() {
        return;
    }

    unsafe {
        Box::from_raw(ptr);
    }
}
