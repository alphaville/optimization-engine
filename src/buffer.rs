use num::{Float, Zero};
use std::ops::Fn;
use std::fmt;

// type definition F: Fn(T) -> T to be updated
// this is still work in progress
pub struct Buffer<T, F>
    where T: Float + Zero,
          F: Fn(T) -> T
{
    current_location : Vec<T>,
    f_current        : T,
    df_current       : Vec<T>,
    f_new            : T,
    df_new           : Vec<T>,
    precomputed      : bool,
    casadi_fun_f_df  : F
}


impl<T, F> Buffer<T, F> 
    where T : Float + Zero,
          F:  Fn(T) -> T {
    pub fn new(x_length: usize, casadi : F) -> Buffer<T, F>{
        Buffer {
            current_location : vec![T::zero(); x_length],
            f_current        : T::zero(),
            df_current       : vec![T::zero(); x_length],
            f_new            : T::zero(),
            df_new           : vec![T::zero(); x_length],
            precomputed      : false,
            casadi_fun_f_df  : casadi
        }
    }

    pub fn set_new_location_as_current(&mut self)  {
        self.f_current = self.f_new;
        // Note: maybe this is not such a good idea...
        // I'm cloning to avoid  "cannot move out of borrowed content"
        self.df_current = (self.df_new).clone(); 
    }    

    pub fn evaluate_new_location(&self, x : T) -> T{
        let f_casadi = &self.casadi_fun_f_df;
        f_casadi(x)
    }

    pub fn invalidate(&mut self) {
        self.precomputed = false;
    }

    // ------------------------------------------------------------------------
    // GETTERS go here (returning references to fields)
    // ------------------------------------------------------------------------
    pub fn get_f_current(&self) -> &T {
        &self.f_current
    }

    pub fn get_f_new(&self) -> &T {
        &self.f_new
    }

    pub fn get_df_currect(&self) ->  &Vec<T> {
        &self.df_current
    }

    pub fn get_df_new(&self) ->  &Vec<T> {
        &self.df_new
    }

    pub fn get_current_location(&self) ->  &Vec<T> {
        &self.current_location
    }
}

impl<T, F> fmt::Display for Buffer<T, F> 
where T: Float + Zero + std::fmt::Debug,
          F: Fn(T) -> T
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, 
"f_current   : {:?},
f_new       : {:?}, 
df_current  : {:?},
df_new      : {:?}
precomputed : {}
current_loc : {:?}", 
            self.f_current,
            self.f_new,
            self.df_current,
            self.df_new,
            self.precomputed,
            self.current_location)
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn make_new_buffer() {
       let t = |s| s + 1.0; 
       let _ : super::Buffer<f64, _> = super::Buffer::new(10, t);

       let mut x : Vec<i32>= vec![1,2,3];
       let y : Vec<i32> = vec![5,6,7];
       x = y;
    }

    #[test]
    fn get_f_current() {
        let t = |s| s + 1.0; 
        let buff : super::Buffer<f64, _> = super::Buffer::new(10, t);
        let f_current = buff.get_f_current();
        assert_eq!(f_current, &0.0_f64)
    }

    #[test]
    fn set_new_location_as_current() {
        let t = |s| s + 1.0; 
        let mut buff : super::Buffer<f64, _> = super::Buffer::new(10, t);
        buff.set_new_location_as_current();
    }

    #[test]
    fn consume_casadi_function() {
        let t = |s| s + 1.0; 
        let mut buff : super::Buffer<f64, _> = super::Buffer::new(10, t);
        let result = buff.evaluate_new_location(5.0_f64);
        assert_eq!(result, 6.0_f64);
        println!("{}", buff);
    }

}