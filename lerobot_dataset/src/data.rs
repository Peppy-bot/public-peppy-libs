//! The data pillar: episode frames as parquet, one file per [`FileSlot`],
//! episodes appended by read-concat-rewrite through an atomic rename (parquet
//! cannot be appended in place, and rewriting keeps every completed episode
//! durable on disk).

use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use arrow::array::{ArrayRef, FixedSizeListArray, Float32Array, Int64Array, RecordBatch};
use arrow::datatypes::{DataType, Field, Schema};
use parquet::arrow::ArrowWriter;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;

use crate::config::DatasetConfig;
use crate::error::Error;
use crate::layout::FileSlot;

pub use crate::layout::data_path;

/// Column values for one episode, in dataset order.
pub struct EpisodeData {
    /// Per vector feature (config order): frame-major flattened values.
    pub vectors: Vec<Vec<f32>>,
    pub timestamps: Vec<f32>,
    pub episode_index: i64,
    pub first_global_index: i64,
    pub task_index: i64,
}

/// The `element` child field arrow uses for list and fixed-size-list columns.
pub(crate) fn element(data_type: DataType) -> Field {
    Field::new("element", data_type, true)
}

pub fn data_schema(config: &DatasetConfig) -> Schema {
    // lerobot stores a 1-D feature as a scalar column (matching a `shape: [1]`
    // Value); only multi-element features become fixed-size lists.
    let mut fields: Vec<Field> = config
        .vectors
        .iter()
        .map(|feature| {
            let dim = feature.dim_names.len();
            let data_type = if dim == 1 {
                DataType::Float32
            } else {
                DataType::FixedSizeList(Arc::new(element(DataType::Float32)), dim as i32)
            };
            Field::new(&feature.key, data_type, true)
        })
        .collect();
    fields.push(Field::new("timestamp", DataType::Float32, true));
    for name in crate::layout::INT64_BOOKKEEPING_COLUMNS {
        fields.push(Field::new(name, DataType::Int64, true));
    }
    Schema::new(fields)
}

fn episode_batch(config: &DatasetConfig, episode: &EpisodeData) -> Result<RecordBatch, Error> {
    let frames = episode.timestamps.len();
    let mut columns: Vec<ArrayRef> = Vec::new();
    for (feature, values) in config.vectors.iter().zip(&episode.vectors) {
        let dim = feature.dim_names.len();
        assert_eq!(values.len(), frames * dim);
        if dim == 1 {
            columns.push(Arc::new(Float32Array::from(values.clone())));
        } else {
            columns.push(Arc::new(FixedSizeListArray::new(
                Arc::new(element(DataType::Float32)),
                dim as i32,
                Arc::new(Float32Array::from(values.clone())),
                None,
            )));
        }
    }
    columns.push(Arc::new(Float32Array::from(episode.timestamps.clone())));
    columns.push(Arc::new(Int64Array::from_iter_values(0..frames as i64)));
    columns.push(Arc::new(Int64Array::from(vec![
        episode.episode_index;
        frames
    ])));
    columns.push(Arc::new(Int64Array::from_iter_values(
        (0..frames as i64).map(|i| episode.first_global_index + i),
    )));
    columns.push(Arc::new(Int64Array::from(vec![episode.task_index; frames])));

    RecordBatch::try_new(Arc::new(data_schema(config)), columns)
        .map_err(Error::arrow(data_path(FileSlot::default())))
}

/// Appends `batch` to the parquet at `path` (created if absent) by rewriting
/// existing row groups plus the new batch into a temp file and renaming.
pub fn append_batch(path: &Path, schema: &Schema, batch: RecordBatch) -> Result<(), Error> {
    let existing: Vec<RecordBatch> = if path.exists() {
        let file = File::open(path).map_err(Error::io(path))?;
        let reader = ParquetRecordBatchReaderBuilder::try_new(file)
            .map_err(Error::parquet(path))?
            .build()
            .map_err(Error::parquet(path))?;
        reader
            .collect::<Result<Vec<_>, _>>()
            .map_err(Error::arrow(path))?
    } else {
        Vec::new()
    };

    crate::atomic::replace_via_temp(path, |file, temp_path| {
        let props = WriterProperties::builder()
            .set_compression(Compression::SNAPPY)
            .build();
        let mut writer = ArrowWriter::try_new(file, Arc::new(schema.clone()), Some(props))
            .map_err(Error::parquet(temp_path))?;
        for prior in &existing {
            writer.write(prior).map_err(Error::parquet(temp_path))?;
        }
        writer.write(&batch).map_err(Error::parquet(temp_path))?;
        writer.close().map_err(Error::parquet(temp_path))?;
        Ok(())
    })
}

/// Appends one episode to the data file at `slot`, returning
/// `(dataset_from_index, dataset_to_index)`.
pub fn append_episode(
    root: &Path,
    config: &DatasetConfig,
    slot: FileSlot,
    episode: &EpisodeData,
) -> Result<(i64, i64), Error> {
    let frames = episode.timestamps.len() as i64;
    let batch = episode_batch(config, episode)?;
    append_batch(&root.join(data_path(slot)), &data_schema(config), batch)?;
    Ok((
        episode.first_global_index,
        episode.first_global_index + frames,
    ))
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use super::*;
    use crate::config::DatasetConfig;

    fn config() -> DatasetConfig {
        DatasetConfig::builder("bot", NonZeroU32::new(30).unwrap())
            .state(vec!["a".into(), "b".into()])
            .action(vec!["x".into()])
            .build()
            .unwrap()
    }

    fn episode(index: i64, first_global: i64, frames: usize) -> EpisodeData {
        EpisodeData {
            vectors: vec![
                (0..frames * 2).map(|i| i as f32).collect(),
                (0..frames).map(|i| -(i as f32)).collect(),
            ],
            timestamps: (0..frames).map(|i| i as f32 / 30.0).collect(),
            episode_index: index,
            first_global_index: first_global,
            task_index: 0,
        }
    }

    #[test]
    fn appends_episodes_and_reads_back() {
        let dir = tempfile::tempdir().unwrap();
        let config = config();
        let slot = FileSlot::default();
        let (from, to) = append_episode(dir.path(), &config, slot, &episode(0, 0, 3)).unwrap();
        assert_eq!((from, to), (0, 3));
        let (from, to) = append_episode(dir.path(), &config, slot, &episode(1, 3, 2)).unwrap();
        assert_eq!((from, to), (3, 5));

        let file = File::open(dir.path().join(data_path(slot))).unwrap();
        let reader = ParquetRecordBatchReaderBuilder::try_new(file)
            .unwrap()
            .build()
            .unwrap();
        let batches: Vec<_> = reader.collect::<Result<Vec<_>, _>>().unwrap();
        let rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(rows, 5);
        assert_eq!(batches[0].schema().field(0).name(), "observation.state");
        let last = batches.last().unwrap();
        let index_col = last
            .column_by_name("index")
            .unwrap()
            .as_any()
            .downcast_ref::<Int64Array>()
            .unwrap();
        assert_eq!(index_col.value(index_col.len() - 1), 4);
    }
}
