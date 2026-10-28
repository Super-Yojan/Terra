//! Depth-image decoding shared by the phone apps.
use crate::ControllerError;

/// ARCore raw-depth confidence (0–255) below which a pixel is dropped.
pub const DEFAULT_MIN_DEPTH_CONFIDENCE: u8 = 128;

#[uniffi::export]
/// [`DEFAULT_MIN_DEPTH_CONFIDENCE`] for the apps.
pub fn default_min_depth_confidence() -> u8 {
    DEFAULT_MIN_DEPTH_CONFIDENCE
}

#[uniffi::export]
/// Decode a 16-bit little-endian millimetre depth image (ARCore
/// `acquireRawDepthImage16Bits` / `acquireDepthImage16Bits`) into row-major
/// metres. Zero depth, and confidence below `min_confidence` when an 8-bit
/// confidence image of the same size is given, become NaN.
///
/// # Errors
/// `InvalidInput` when a buffer is too short for its size and row stride.
pub fn depth16_to_metres(
    depth: Vec<u8>,
    depth_row_stride: u32,
    confidence: Option<Vec<u8>>,
    confidence_row_stride: u32,
    width: u32,
    height: u32,
    min_confidence: u8,
) -> Result<Vec<f32>, ControllerError> {
    let (w, h) = (width as usize, height as usize);
    let fits = |len: usize, stride: usize, bytes_per_pixel: usize| {
        h == 0
            || (stride >= w * bytes_per_pixel
                && (h - 1)
                    .checked_mul(stride)
                    .and_then(|n| n.checked_add(w * bytes_per_pixel))
                    .is_some_and(|n| n <= len))
    };
    let depth_stride = depth_row_stride as usize;
    let confidence_stride = confidence_row_stride as usize;
    if !fits(depth.len(), depth_stride, 2)
        || confidence
            .as_ref()
            .is_some_and(|c| !fits(c.len(), confidence_stride, 1))
    {
        return Err(ControllerError::InvalidInput {
            message: "depth image buffer is shorter than its size".into(),
        });
    }
    let mut out = Vec::with_capacity(w * h);
    for row in 0..h {
        for col in 0..w {
            let at = row * depth_stride + col * 2;
            let millimetres = u16::from_le_bytes([depth[at], depth[at + 1]]);
            let trusted = confidence
                .as_ref()
                .is_none_or(|c| c[row * confidence_stride + col] >= min_confidence);
            out.push(if millimetres == 0 || !trusted {
                f32::NAN
            } else {
                f32::from(millimetres) / 1000.0
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Strided rows decode, and zero or low-confidence pixels drop out.
    fn decodes_with_stride_and_confidence() {
        // 2×2 image, depth row stride 6 bytes, confidence row stride 3.
        let depth = vec![0xE8, 0x03, 0, 0, 9, 9, 0xD0, 0x07, 0xB8, 0x0B, 9, 9];
        let confidence = vec![255, 255, 0, 10, 255, 0];
        let plain = depth16_to_metres(depth.clone(), 6, None, 0, 2, 2, 128).unwrap();
        assert_eq!(plain[0], 1.0);
        assert!(plain[1].is_nan());
        assert_eq!((plain[2], plain[3]), (2.0, 3.0));
        let gated = depth16_to_metres(depth.clone(), 6, Some(confidence), 3, 2, 2, 128).unwrap();
        assert_eq!(gated[0], 1.0);
        assert!(gated[2].is_nan());
        assert_eq!(gated[3], 3.0);
        assert!(depth16_to_metres(depth, 6, Some(vec![0; 4]), 3, 2, 2, 128).is_err());
        assert!(depth16_to_metres(vec![0; 7], 4, None, 0, 2, 2, 0).is_err());
    }
}
