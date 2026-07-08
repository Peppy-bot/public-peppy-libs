pub mod episodes;
pub mod info;
pub mod stats;
pub mod tasks;

use std::path::Path;

use serde_json::{Map, Value, json};

use crate::error::Error;
use crate::layout::{INFO_PATH, STATS_PATH};
use crate::meta::episodes::FeatureStatsEntry;

fn stat_value(entry: &FeatureStatsEntry, values: &[f64]) -> Value {
    if entry.is_image {
        // Shape (3,1,1), matching lerobot's channel-first keepdims layout.
        Value::Array(values.iter().map(|v| json!([[v]])).collect())
    } else {
        json!(values)
    }
}

/// `meta/stats.json`: aggregated stats per feature.
pub fn build_stats_json(entries: &[FeatureStatsEntry]) -> Value {
    let mut features = Map::new();
    for entry in entries {
        let stats = &entry.stats;
        let mut object = Map::new();
        object.insert("min".into(), stat_value(entry, &stats.min));
        object.insert("max".into(), stat_value(entry, &stats.max));
        object.insert("mean".into(), stat_value(entry, &stats.mean));
        object.insert("std".into(), stat_value(entry, &stats.std));
        object.insert("count".into(), json!([stats.count]));
        for (key, values) in stats::QUANTILE_KEYS.iter().zip(&stats.quantiles) {
            object.insert((*key).into(), stat_value(entry, values));
        }
        features.insert(entry.key.clone(), Value::Object(object));
    }
    Value::Object(features)
}

pub fn write_json(root: &Path, relative: &str, value: &Value) -> Result<(), Error> {
    let path = root.join(relative);
    let text = serde_json::to_string_pretty(value).expect("json values always serialize");
    crate::atomic::write_atomic(&path, text.as_bytes())
}

pub fn write_info(root: &Path, info: &Value) -> Result<(), Error> {
    write_json(root, INFO_PATH, info)
}

pub fn write_stats(root: &Path, stats: &Value) -> Result<(), Error> {
    write_json(root, STATS_PATH, stats)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meta::stats::FeatureStats;

    #[test]
    fn image_stats_serialize_as_3_1_1() {
        let entry = FeatureStatsEntry {
            key: "observation.images.cam".into(),
            is_image: true,
            stats: FeatureStats {
                min: vec![0.0, 0.1, 0.2],
                max: vec![1.0; 3],
                mean: vec![0.5; 3],
                std: vec![0.2; 3],
                count: 7,
                quantiles: std::array::from_fn(|_| vec![0.5; 3]),
            },
        };
        let value = build_stats_json(std::slice::from_ref(&entry));
        let min = &value["observation.images.cam"]["min"];
        assert_eq!(min, &json!([[[0.0]], [[0.1]], [[0.2]]]));
        assert_eq!(value["observation.images.cam"]["count"], json!([7]));
    }
}
