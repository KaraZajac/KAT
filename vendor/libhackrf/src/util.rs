use num_complex::Complex;

pub trait ToComplexI8 {
    fn to_i8(self) -> Complex<i8>;
}

pub trait ToComplexF32 {
    fn to_f32(self) -> Complex<f32>;
}

impl ToComplexI8 for Complex<f32> {
    fn to_i8(self) -> Complex<i8> {
        Complex::new((self.re * 127.0) as i8, (self.im * 127.0) as i8)
    }
}

impl ToComplexF32 for Complex<i8> {
    fn to_f32(self) -> Complex<f32> {
        Complex::new(self.re as f32 / 127.0, self.im as f32 / 127.0)
    }
}
