use num::{Float, ToPrimitive};

/// Convert a numeric literal or integer index into the target float type.
#[inline]
pub(crate) fn cast<T>(value: impl ToPrimitive) -> T
where
    T: Float,
{
    T::from(value).expect("numeric constant must be representable")
}
