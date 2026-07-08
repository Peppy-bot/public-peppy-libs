//! Per-episode and aggregated feature statistics.
//!
//! Mirrors lerobot's `compute_stats.py`: population std, and quantiles
//! (q01/q10/q50/q90/q99) from a 5000-bin histogram with intra-bin linear
//! interpolation rather than exact quantiles. Vector features feed the
//! histogram in one batch over `[min - 1e-10, max + 1e-10]`, matching
//! lerobot's single-update path. Image features deviate deliberately:
//! lerobot samples frames after the fact from PNGs on disk, which a
//! streaming writer cannot do, so images accumulate over every frame
//! (stride-downsampled like lerobot) against a fixed [0, 255] bin range,
//! with `count` = frames seen. Values are normalized to [0, 1] on output
//! exactly like lerobot; the difference from lerobot's frame sampling is
//! bounded by the histogram bin width and verified by the compliance
//! harness.

pub const QUANTILE_KEYS: [&str; 5] = ["q01", "q10", "q50", "q90", "q99"];
pub const QUANTILES: [f64; 5] = [0.01, 0.10, 0.50, 0.90, 0.99];
const NUM_BINS: usize = 5000;
const EDGE_PADDING: f64 = 1e-10;

/// Statistics for one feature over one episode or the whole dataset.
///
/// `min/max/mean/std` and each quantile hold one value per dimension
/// (vector dims, or 3 RGB channels for images).
#[derive(Debug, Clone, PartialEq)]
pub struct FeatureStats {
    pub min: Vec<f64>,
    pub max: Vec<f64>,
    pub mean: Vec<f64>,
    pub std: Vec<f64>,
    pub count: u64,
    pub quantiles: [Vec<f64>; 5],
}

/// One histogram per dimension over a fixed value range.
struct Histogram {
    edges_start: f64,
    edges_end: f64,
    counts: Vec<Vec<u64>>,
    total: u64,
}

impl Histogram {
    fn new(dims: usize, start: f64, end: f64) -> Self {
        Self {
            edges_start: start,
            edges_end: end,
            counts: vec![vec![0; NUM_BINS]; dims],
            total: 0,
        }
    }

    fn edge(&self, i: usize) -> f64 {
        if i == NUM_BINS {
            return self.edges_end;
        }
        let step = (self.edges_end - self.edges_start) / NUM_BINS as f64;
        self.edges_start + i as f64 * step
    }

    /// Bin assignment matching `np.histogram` with explicit edges:
    /// `searchsorted(edges, v, side="right") - 1`, with values equal to the
    /// last edge counted in the last bin. Out-of-range values cannot occur
    /// (edges are padded beyond the data range) but are clamped defensively.
    fn add(&mut self, dim: usize, value: f64) {
        if value >= self.edges_end {
            self.counts[dim][NUM_BINS - 1] += 1;
            return;
        }
        let (mut lo, mut hi) = (0usize, NUM_BINS + 1);
        while lo < hi {
            let mid = (lo + hi) / 2;
            if self.edge(mid) > value {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        let bin = (lo as i64 - 1).clamp(0, NUM_BINS as i64 - 1) as usize;
        self.counts[dim][bin] += 1;
    }

    /// lerobot's `_compute_single_quantile`: left-searchsorted on the
    /// cumulative counts, then linear interpolation inside the target bin.
    fn quantile(&self, dim: usize, q: f64) -> f64 {
        let target = q * self.total as f64;
        let counts = &self.counts[dim];
        let mut cumulative = 0u64;
        for (i, &c) in counts.iter().enumerate() {
            cumulative += c;
            if cumulative as f64 >= target {
                if i == 0 {
                    return self.edge(0);
                }
                let count_before = (cumulative - c) as f64;
                let fraction = (target - count_before) / c as f64;
                return self.edge(i) + fraction * (self.edge(i + 1) - self.edge(i));
            }
        }
        self.edge(NUM_BINS)
    }
}

/// Stats over a fully buffered episode of vector rows, single-batch like
/// lerobot's `get_feature_stats(axis=0)`. Rows are f64 because numpy
/// promotes int64 columns to float64 and float32 converts exactly.
pub fn vector_stats(rows: &[Vec<f64>], dims: usize) -> FeatureStats {
    assert!(rows.iter().all(|r| r.len() == dims));
    let count = rows.len() as u64;
    if count == 0 {
        return FeatureStats {
            min: vec![0.0; dims],
            max: vec![0.0; dims],
            mean: vec![0.0; dims],
            std: vec![0.0; dims],
            count: 0,
            quantiles: std::array::from_fn(|_| vec![0.0; dims]),
        };
    }

    let mut min = vec![f64::INFINITY; dims];
    let mut max = vec![f64::NEG_INFINITY; dims];
    let mut sum = vec![0.0f64; dims];
    let mut sum_sq = vec![0.0f64; dims];
    for row in rows {
        for (d, &v) in row.iter().enumerate() {
            min[d] = min[d].min(v);
            max[d] = max[d].max(v);
            sum[d] += v;
            sum_sq[d] += v * v;
        }
    }
    let mean: Vec<f64> = sum.iter().map(|s| s / count as f64).collect();
    let std: Vec<f64> = sum_sq
        .iter()
        .zip(&mean)
        .map(|(sq, m)| (sq / count as f64 - m * m).max(0.0).sqrt())
        .collect();

    // Fewer than 2 samples: lerobot's basic path sets quantiles to the mean.
    if count < 2 {
        return FeatureStats {
            min,
            max,
            mean: mean.clone(),
            std,
            count,
            quantiles: std::array::from_fn(|_| mean.clone()),
        };
    }

    let mut quantiles: [Vec<f64>; 5] = std::array::from_fn(|_| Vec::with_capacity(dims));
    for d in 0..dims {
        let mut hist = Histogram::new(1, min[d] - EDGE_PADDING, max[d] + EDGE_PADDING);
        for row in rows {
            hist.add(0, row[d]);
        }
        hist.total = count;
        for (qi, &q) in QUANTILES.iter().enumerate() {
            quantiles[qi].push(hist.quantile(0, q));
        }
    }

    FeatureStats {
        min,
        max,
        mean,
        std,
        count,
        quantiles,
    }
}

/// Streaming per-channel RGB stats over every (downsampled) frame of an episode.
pub struct ImageStatsAccumulator {
    min: [f64; 3],
    max: [f64; 3],
    sum: [f64; 3],
    sum_sq: [f64; 3],
    pixel_count: u64,
    frame_count: u64,
    hist: Histogram,
}

impl Default for ImageStatsAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl ImageStatsAccumulator {
    pub fn new() -> Self {
        Self {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
            sum: [0.0; 3],
            sum_sq: [0.0; 3],
            pixel_count: 0,
            frame_count: 0,
            hist: Histogram::new(3, 0.0 - EDGE_PADDING, 255.0 + EDGE_PADDING),
        }
    }

    /// Adds one frame's pixels (interleaved RGB, already downsampled).
    pub fn add_frame(&mut self, rgb: &[u8]) {
        assert_eq!(rgb.len() % 3, 0);
        for pixel in rgb.chunks_exact(3) {
            for (c, &v) in pixel.iter().enumerate() {
                let v = v as f64;
                self.min[c] = self.min[c].min(v);
                self.max[c] = self.max[c].max(v);
                self.sum[c] += v;
                self.sum_sq[c] += v * v;
                self.hist.add(c, v);
            }
        }
        self.pixel_count += rgb.len() as u64 / 3;
        self.frame_count += 1;
    }

    /// Final stats normalized to [0, 1]; `count` = frames seen.
    pub fn finish(mut self) -> FeatureStats {
        let n = self.pixel_count as f64;
        if self.pixel_count == 0 {
            return FeatureStats {
                min: vec![0.0; 3],
                max: vec![0.0; 3],
                mean: vec![0.0; 3],
                std: vec![0.0; 3],
                count: 0,
                quantiles: std::array::from_fn(|_| vec![0.0; 3]),
            };
        }
        self.hist.total = self.pixel_count;
        let mean: Vec<f64> = self.sum.iter().map(|s| s / n / 255.0).collect();
        let std: Vec<f64> = self
            .sum_sq
            .iter()
            .zip(&self.sum)
            .map(|(sq, s)| {
                let m = s / n;
                (sq / n - m * m).max(0.0).sqrt() / 255.0
            })
            .collect();
        let quantiles = std::array::from_fn(|qi| {
            (0..3)
                .map(|c| self.hist.quantile(c, QUANTILES[qi]) / 255.0)
                .collect()
        });
        FeatureStats {
            min: self.min.iter().map(|v| v / 255.0).collect(),
            max: self.max.iter().map(|v| v / 255.0).collect(),
            mean,
            std,
            count: self.frame_count,
            quantiles,
        }
    }
}

/// lerobot's `aggregate_feature_stats`: count-weighted mean, parallel
/// (Chan) variance, elementwise min/max, count-weighted quantile average.
pub fn aggregate(stats: &[&FeatureStats]) -> FeatureStats {
    assert!(!stats.is_empty());
    let dims = stats[0].mean.len();
    let total_count: u64 = stats.iter().map(|s| s.count).sum();
    let n = total_count as f64;

    let mut min = vec![f64::INFINITY; dims];
    let mut max = vec![f64::NEG_INFINITY; dims];
    let mut mean = vec![0.0f64; dims];
    for s in stats {
        for d in 0..dims {
            min[d] = min[d].min(s.min[d]);
            max[d] = max[d].max(s.max[d]);
            mean[d] += s.mean[d] * s.count as f64 / n;
        }
    }
    let mut variance = vec![0.0f64; dims];
    for s in stats {
        for d in 0..dims {
            let delta = s.mean[d] - mean[d];
            variance[d] += (s.std[d] * s.std[d] + delta * delta) * s.count as f64 / n;
        }
    }
    let std = variance.iter().map(|v| v.sqrt()).collect();
    let quantiles = std::array::from_fn(|qi| {
        (0..dims)
            .map(|d| {
                stats
                    .iter()
                    .map(|s| s.quantiles[qi][d] * s.count as f64 / n)
                    .sum()
            })
            .collect()
    });

    FeatureStats {
        min,
        max,
        mean,
        std,
        count: total_count,
        quantiles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rows(values: &[&[f64]]) -> Vec<Vec<f64>> {
        values.iter().map(|r| r.to_vec()).collect()
    }

    #[test]
    fn vector_stats_match_hand_computed() {
        let stats = vector_stats(&rows(&[&[1.0, 10.0], &[2.0, 20.0], &[3.0, 30.0]]), 2);
        assert_eq!(stats.min, vec![1.0, 10.0]);
        assert_eq!(stats.max, vec![3.0, 30.0]);
        assert_eq!(stats.mean, vec![2.0, 20.0]);
        let expected_std = (2.0f64 / 3.0).sqrt();
        assert!((stats.std[0] - expected_std).abs() < 1e-12);
        assert!((stats.std[1] - expected_std * 10.0).abs() < 1e-11);
        assert_eq!(stats.count, 3);
    }

    #[test]
    fn quantiles_interpolate_within_bins() {
        // 100 evenly spread values: quantile estimates track the value range
        // within one bin width.
        let values: Vec<Vec<f64>> = (0..100).map(|i| vec![i as f64]).collect();
        let stats = vector_stats(&values, 1);
        let bin_width = (99.0 + 2.0 * EDGE_PADDING) / NUM_BINS as f64;
        assert!((stats.quantiles[2][0] - 49.5).abs() < 1.0 + bin_width);
        assert!(stats.quantiles[0][0] < 2.0);
        assert!(stats.quantiles[4][0] > 97.0);
    }

    #[test]
    fn single_sample_sets_quantiles_to_mean() {
        let stats = vector_stats(&rows(&[&[5.0]]), 1);
        assert_eq!(stats.std, vec![0.0]);
        for q in &stats.quantiles {
            assert_eq!(q, &vec![5.0]);
        }
    }

    #[test]
    fn image_stats_normalized_and_counted_in_frames() {
        let mut acc = ImageStatsAccumulator::new();
        acc.add_frame(&[0, 128, 255, 0, 128, 255]);
        acc.add_frame(&[0, 128, 255, 0, 128, 255]);
        let stats = acc.finish();
        assert_eq!(stats.count, 2);
        assert_eq!(stats.min, vec![0.0, 128.0 / 255.0, 1.0]);
        assert_eq!(stats.max, vec![0.0, 128.0 / 255.0, 1.0]);
        assert!((stats.mean[1] - 128.0 / 255.0).abs() < 1e-12);
        assert!(stats.std.iter().all(|s| *s == 0.0));
    }

    #[test]
    fn aggregate_matches_pooled_computation() {
        let a = vector_stats(&rows(&[&[1.0], &[2.0], &[3.0]]), 1);
        let b = vector_stats(&rows(&[&[10.0], &[20.0]]), 1);
        let pooled = vector_stats(&rows(&[&[1.0], &[2.0], &[3.0], &[10.0], &[20.0]]), 1);
        let agg = aggregate(&[&a, &b]);
        assert_eq!(agg.count, 5);
        assert_eq!(agg.min, pooled.min);
        assert_eq!(agg.max, pooled.max);
        assert!((agg.mean[0] - pooled.mean[0]).abs() < 1e-12);
        assert!((agg.std[0] - pooled.std[0]).abs() < 1e-12);
    }
}
