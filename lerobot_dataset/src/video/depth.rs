//! Depth quantization matching lerobot 0.6's `depth_utils.quantize_depth`.
//!
//! A camera sends raw `z16` codes; `depth_unit_m` (metres per LSB, from the
//! camera) converts them to millimetres, then lerobot's log-space
//! quantization maps millimetres to a 12-bit code that is stored as a
//! single-channel `gray12le` HEVC-lossless video. Decode is the exact
//! inverse (see the compliance harness), so metres round-trip to within the
//! 12-bit step. All internal math is done in millimetres, exactly as lerobot
//! does for integer depth input.

use crate::config::DepthQuantization;

/// The 12-bit code ceiling (`DEPTH_QMAX`).
pub const DEPTH_QMAX: f64 = 4095.0;
const MM_PER_METRE: f64 = 1000.0;

/// One z16 code (interpreted little-endian) converted to millimetres.
#[inline]
fn code_to_mm(code: u16, depth_unit_m: f64) -> f64 {
    // lerobot casts the depth array to float32 before quantizing; match that
    // precision so the produced codes agree bit-for-bit.
    (code as f32 * (depth_unit_m as f32) * (MM_PER_METRE as f32)) as f64
}

/// The `(log_min, log_span)` (or linear `(min, span)`) constants for one
/// quantization, in millimetres.
fn norm_constants(q: &DepthQuantization) -> (f64, f64) {
    let min_mm = q.depth_min_m * MM_PER_METRE;
    let max_mm = q.depth_max_m * MM_PER_METRE;
    let shift_mm = q.shift_m * MM_PER_METRE;
    if q.use_log {
        let log_min = (min_mm + shift_mm).ln();
        let log_max = (max_mm + shift_mm).ln();
        (log_min, log_max - log_min)
    } else {
        (min_mm, max_mm - min_mm)
    }
}

fn quantize_mm(mm: f64, q: &DepthQuantization, shift_mm: f64, base: f64, span: f64) -> u16 {
    let norm = if q.use_log {
        ((mm + shift_mm).ln() - base) / span
    } else {
        (mm - base) / span
    };
    // np.rint rounds half to even; match it so codes agree with lerobot.
    (norm * DEPTH_QMAX).round_ties_even().clamp(0.0, DEPTH_QMAX) as u16
}

/// Converts a little-endian z16 buffer to a little-endian `gray12le` buffer of
/// 12-bit codes ready for the ffmpeg depth encoder. `z16` length must be even.
pub fn z16_to_gray12le(z16: &[u8], depth_unit_m: f64, q: &DepthQuantization) -> Vec<u8> {
    debug_assert_eq!(z16.len() % 2, 0);
    let shift_mm = q.shift_m * MM_PER_METRE;
    let (base, span) = norm_constants(q);
    let mut out = Vec::with_capacity(z16.len());
    for chunk in z16.chunks_exact(2) {
        let code = u16::from_le_bytes([chunk[0], chunk[1]]);
        let mm = code_to_mm(code, depth_unit_m);
        out.extend_from_slice(&quantize_mm(mm, q, shift_mm, base, span).to_le_bytes());
    }
    out
}

/// Millimetre values of a z16 buffer, for stats (lerobot computes depth stats
/// over the raw millimetre values, not the quantized codes).
pub fn z16_to_mm(z16: &[u8], depth_unit_m: f64) -> impl Iterator<Item = f64> + '_ {
    z16.chunks_exact(2)
        .map(move |c| code_to_mm(u16::from_le_bytes([c[0], c[1]]), depth_unit_m))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_q() -> DepthQuantization {
        DepthQuantization::default()
    }

    /// The reference codes from lerobot's own quantize_depth for a few
    /// millimetre inputs at the default log quantization (depth_unit = 1 mm),
    /// captured from the compliance env.
    #[test]
    fn matches_lerobot_reference_codes() {
        // mm -> expected 12-bit code (from lerobot quantize_depth, log, mm).
        let cases = [(500u16, 397u16), (800, 617), (1100, 822), (2000, 1365)];
        let q = default_q();
        for (mm, expected) in cases {
            let bytes = mm.to_le_bytes();
            let out = z16_to_gray12le(&bytes, 0.001, &q);
            let code = u16::from_le_bytes([out[0], out[1]]);
            assert!(
                code.abs_diff(expected) <= 1,
                "mm {mm}: got code {code}, expected {expected}"
            );
        }
    }

    #[test]
    fn clamps_out_of_range() {
        let q = default_q();
        // 0 mm is below depth_min; far-past-max saturates at the ceiling.
        let below = z16_to_gray12le(&0u16.to_le_bytes(), 0.001, &q);
        assert_eq!(u16::from_le_bytes([below[0], below[1]]), 0);
        let above = z16_to_gray12le(&60000u16.to_le_bytes(), 0.001, &q);
        assert_eq!(u16::from_le_bytes([above[0], above[1]]), DEPTH_QMAX as u16);
    }

    #[test]
    fn depth_unit_scales_to_mm() {
        let q = default_q();
        // z16=1000 at 1mm/LSB and z16=500 at 2mm/LSB both mean 1000 mm.
        let a = z16_to_gray12le(&1000u16.to_le_bytes(), 0.001, &q);
        let b = z16_to_gray12le(&500u16.to_le_bytes(), 0.002, &q);
        assert_eq!(a, b);
        assert_eq!(
            z16_to_mm(&500u16.to_le_bytes(), 0.002).next().unwrap(),
            1000.0
        );
    }
}
