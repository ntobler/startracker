pub fn as_slice_of_arrays<T, const L: usize>(slice: &[T]) -> Option<&[[T; L]]> {
    match slice.len() % L {
        0 => Some(unsafe {
            std::slice::from_raw_parts(slice.as_ptr() as *const [T; L], slice.len() / L)
        }),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_as_vec_of_arrays_valid_length() {
        let data: &[f32] = &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let array_ref = as_slice_of_arrays::<f32, 3>(data).expect("Expected valid array reference");
        assert_eq!(array_ref, &[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    }

    #[test]
    fn test_as_vec_of_arrays_invalid_length() {
        let data: &[f32] = &[1.0, 2.0, 3.0, 4.0, 5.0];
        assert!(
            as_slice_of_arrays::<f32, 3>(data).is_none(),
            "Expected None for invalid length"
        );
    }

    #[test]
    fn test_as_vec_of_arrays_empty_slice() {
        let data: &[f32] = &[];
        let array_ref =
            as_slice_of_arrays::<f32, 3>(data).expect("Expected valid array reference for empty slice");
        assert_eq!(array_ref.len(), 0);
    }
}
