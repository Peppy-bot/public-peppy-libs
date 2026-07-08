//! `meta/episodes/`: one row per episode with file locations, row offsets,
//! video time offsets, and flattened per-episode stats. Rows are addressed
//! positionally by the loader, so episodes append in index order.

use std::path::Path;
use std::sync::Arc;

use arrow::array::builder::{Float64Builder, Int64Builder, ListBuilder, StringBuilder};
use arrow::array::{ArrayRef, Int64Array, RecordBatch};
use arrow::datatypes::{DataType, Field, Schema};

use crate::error::Error;
use crate::layout::{FileSlot, episodes_path};
use crate::meta::stats::{FeatureStats, QUANTILE_KEYS};

/// Where one episode's video landed inside a camera's shared mp4 sequence.
#[derive(Debug, Clone, Copy)]
pub struct VideoLocation {
    pub slot: FileSlot,
    pub from_timestamp: f64,
    pub to_timestamp: f64,
}

/// Per-episode stats for one feature; images serialize as shape (3,1,1).
#[derive(Debug, Clone)]
pub struct FeatureStatsEntry {
    pub key: String,
    pub is_image: bool,
    pub stats: FeatureStats,
}

pub struct EpisodeRow {
    pub episode_index: i64,
    pub task: String,
    pub length: i64,
    pub data_slot: FileSlot,
    pub dataset_from_index: i64,
    pub dataset_to_index: i64,
    /// Per camera in config order: (camera key, location).
    pub videos: Vec<(String, VideoLocation)>,
    /// Canonical feature order: vectors, cameras, then default columns.
    pub stats: Vec<FeatureStatsEntry>,
    pub episodes_slot: FileSlot,
}

fn element(data_type: DataType) -> Field {
    Field::new("element", data_type, true)
}

fn list_of(data_type: DataType) -> DataType {
    DataType::List(Arc::new(element(data_type)))
}

fn image_stat_type() -> DataType {
    list_of(list_of(list_of(DataType::Float64)))
}

pub fn episodes_schema(row: &EpisodeRow) -> Schema {
    let mut fields = vec![
        Field::new("episode_index", DataType::Int64, true),
        Field::new("tasks", list_of(DataType::Utf8), true),
        Field::new("length", DataType::Int64, true),
        Field::new("data/chunk_index", DataType::Int64, true),
        Field::new("data/file_index", DataType::Int64, true),
        Field::new("dataset_from_index", DataType::Int64, true),
        Field::new("dataset_to_index", DataType::Int64, true),
    ];
    for (key, _) in &row.videos {
        for suffix in ["chunk_index", "file_index"] {
            fields.push(Field::new(
                format!("videos/{key}/{suffix}"),
                DataType::Int64,
                true,
            ));
        }
        for suffix in ["from_timestamp", "to_timestamp"] {
            fields.push(Field::new(
                format!("videos/{key}/{suffix}"),
                DataType::Float64,
                true,
            ));
        }
    }
    for entry in &row.stats {
        let value_type = if entry.is_image {
            image_stat_type()
        } else {
            list_of(DataType::Float64)
        };
        for stat in ["min", "max", "mean", "std"] {
            fields.push(Field::new(
                format!("stats/{}/{stat}", entry.key),
                value_type.clone(),
                true,
            ));
        }
        fields.push(Field::new(
            format!("stats/{}/count", entry.key),
            list_of(DataType::Int64),
            true,
        ));
        for stat in QUANTILE_KEYS {
            fields.push(Field::new(
                format!("stats/{}/{stat}", entry.key),
                value_type.clone(),
                true,
            ));
        }
    }
    fields.push(Field::new(
        "meta/episodes/chunk_index",
        DataType::Int64,
        true,
    ));
    fields.push(Field::new(
        "meta/episodes/file_index",
        DataType::Int64,
        true,
    ));
    Schema::new(fields)
}

fn single_i64(value: i64) -> ArrayRef {
    Arc::new(Int64Array::from(vec![value]))
}

fn list_f64_row(values: &[f64]) -> ArrayRef {
    let mut builder =
        ListBuilder::new(Float64Builder::new()).with_field(element(DataType::Float64));
    builder.values().append_slice(values);
    builder.append(true);
    Arc::new(builder.finish())
}

fn list_i64_row(values: &[i64]) -> ArrayRef {
    let mut builder = ListBuilder::new(Int64Builder::new()).with_field(element(DataType::Int64));
    builder.values().append_slice(values);
    builder.append(true);
    Arc::new(builder.finish())
}

fn list_utf8_row(values: &[&str]) -> ArrayRef {
    let mut builder = ListBuilder::new(StringBuilder::new()).with_field(element(DataType::Utf8));
    for v in values {
        builder.values().append_value(v);
    }
    builder.append(true);
    Arc::new(builder.finish())
}

/// One row of shape (3,1,1): `[[[v0]], [[v1]], [[v2]]]`.
fn image_stat_row(values: &[f64]) -> ArrayRef {
    let inner = ListBuilder::new(Float64Builder::new()).with_field(element(DataType::Float64));
    let middle = ListBuilder::new(inner).with_field(element(list_of(DataType::Float64)));
    let mut outer =
        ListBuilder::new(middle).with_field(element(list_of(list_of(DataType::Float64))));
    for &v in values {
        outer.values().values().values().append_value(v);
        outer.values().values().append(true);
        outer.values().append(true);
    }
    outer.append(true);
    Arc::new(outer.finish())
}

fn stat_row(entry: &FeatureStatsEntry, values: &[f64]) -> ArrayRef {
    if entry.is_image {
        image_stat_row(values)
    } else {
        list_f64_row(values)
    }
}

pub fn episode_batch(row: &EpisodeRow) -> Result<RecordBatch, Error> {
    let schema = Arc::new(episodes_schema(row));
    let mut columns: Vec<ArrayRef> = vec![
        single_i64(row.episode_index),
        list_utf8_row(&[row.task.as_str()]),
        single_i64(row.length),
        single_i64(row.data_slot.chunk_index as i64),
        single_i64(row.data_slot.file_index as i64),
        single_i64(row.dataset_from_index),
        single_i64(row.dataset_to_index),
    ];
    for (_, location) in &row.videos {
        columns.push(single_i64(location.slot.chunk_index as i64));
        columns.push(single_i64(location.slot.file_index as i64));
        columns.push(Arc::new(arrow::array::Float64Array::from(vec![
            location.from_timestamp,
        ])));
        columns.push(Arc::new(arrow::array::Float64Array::from(vec![
            location.to_timestamp,
        ])));
    }
    for entry in &row.stats {
        let stats = &entry.stats;
        columns.push(stat_row(entry, &stats.min));
        columns.push(stat_row(entry, &stats.max));
        columns.push(stat_row(entry, &stats.mean));
        columns.push(stat_row(entry, &stats.std));
        columns.push(list_i64_row(&[stats.count as i64]));
        for quantile in &stats.quantiles {
            columns.push(stat_row(entry, quantile));
        }
    }
    columns.push(single_i64(row.episodes_slot.chunk_index as i64));
    columns.push(single_i64(row.episodes_slot.file_index as i64));

    let path = episodes_path(row.episodes_slot);
    RecordBatch::try_new(schema, columns).map_err(Error::arrow(path))
}

/// Appends the episode row to the episodes parquet at its slot.
pub fn append_episode_row(root: &Path, row: &EpisodeRow) -> Result<(), Error> {
    let batch = episode_batch(row)?;
    let schema = episodes_schema(row);
    crate::data::append_batch(&root.join(episodes_path(row.episodes_slot)), &schema, batch)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stats(dims: usize) -> FeatureStats {
        FeatureStats {
            min: vec![0.0; dims],
            max: vec![1.0; dims],
            mean: vec![0.5; dims],
            std: vec![0.1; dims],
            count: 40,
            quantiles: std::array::from_fn(|_| vec![0.5; dims]),
        }
    }

    fn row() -> EpisodeRow {
        EpisodeRow {
            episode_index: 0,
            task: "pick".into(),
            length: 40,
            data_slot: FileSlot::default(),
            dataset_from_index: 0,
            dataset_to_index: 40,
            videos: vec![(
                "observation.images.cam_a".into(),
                VideoLocation {
                    slot: FileSlot::default(),
                    from_timestamp: 0.0,
                    to_timestamp: 40.0 / 30.0,
                },
            )],
            stats: vec![
                FeatureStatsEntry {
                    key: "observation.state".into(),
                    is_image: false,
                    stats: stats(4),
                },
                FeatureStatsEntry {
                    key: "observation.images.cam_a".into(),
                    is_image: true,
                    stats: stats(3),
                },
            ],
            episodes_slot: FileSlot::default(),
        }
    }

    #[test]
    fn schema_and_batch_agree_and_round_trip() {
        let dir = tempfile::tempdir().unwrap();
        let row = row();
        append_episode_row(dir.path(), &row).unwrap();
        let mut second = self::row();
        second.episode_index = 1;
        append_episode_row(dir.path(), &second).unwrap();

        let file =
            std::fs::File::open(dir.path().join(episodes_path(FileSlot::default()))).unwrap();
        let reader = parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder::try_new(file)
            .unwrap()
            .build()
            .unwrap();
        let batches: Vec<_> = reader.collect::<Result<Vec<_>, _>>().unwrap();
        let rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(rows, 2);
        let schema = batches[0].schema();
        assert_eq!(
            schema.field(1).data_type(),
            &list_of(DataType::Utf8),
            "tasks column"
        );
        assert!(
            schema
                .field_with_name("stats/observation.images.cam_a/mean")
                .unwrap()
                .data_type()
                .equals_datatype(&image_stat_type())
        );
        assert_eq!(schema.fields().len(), 7 + 4 + 2 * 10 + 2);
    }
}
