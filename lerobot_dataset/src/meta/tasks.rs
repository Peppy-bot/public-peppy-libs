//! `meta/tasks.parquet`: task string to task_index, read by the loader with
//! `pd.read_parquet`. The pandas schema metadata marking `task` as the
//! DataFrame index is load-bearing: the loader resolves task strings via the
//! restored index (`tasks.iloc[task_index].name`), so rows must be ordered by
//! `task_index`, contiguous from 0, and the metadata blob must be present.

use std::path::Path;
use std::sync::Arc;

use arrow::array::{Int64Array, RecordBatch, StringArray};
use arrow::datatypes::{DataType, Field, Schema};
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;
use serde_json::json;

use crate::error::Error;
use crate::layout::TASKS_PATH;

#[derive(Debug, Default)]
pub struct TaskTable {
    tasks: Vec<String>,
}

impl TaskTable {
    /// Index for `task`, interning it if new. Returns `(index, was_new)`.
    pub fn intern(&mut self, task: &str) -> (i64, bool) {
        if let Some(index) = self.tasks.iter().position(|t| t == task) {
            return (index as i64, false);
        }
        self.tasks.push(task.to_string());
        (self.tasks.len() as i64 - 1, true)
    }

    pub fn len(&self) -> u64 {
        self.tasks.len() as u64
    }

    fn pandas_metadata() -> String {
        json!({
            "index_columns": ["task"],
            "column_indexes": [{
                "name": null,
                "field_name": null,
                "pandas_type": "unicode",
                "numpy_type": "object",
                "metadata": {"encoding": "UTF-8"},
            }],
            "columns": [
                {
                    "name": "task_index",
                    "field_name": "task_index",
                    "pandas_type": "int64",
                    "numpy_type": "int64",
                    "metadata": null,
                },
                {
                    "name": "task",
                    "field_name": "task",
                    "pandas_type": "unicode",
                    "numpy_type": "object",
                    "metadata": null,
                },
            ],
            "attributes": {},
            "creator": {"library": "lerobot_dataset", "version": env!("CARGO_PKG_VERSION")},
            "pandas_version": "2.0.0",
        })
        .to_string()
    }

    /// Atomically rewrites `meta/tasks.parquet` under `root`.
    pub fn write(&self, root: &Path) -> Result<(), Error> {
        let path = root.join(TASKS_PATH);
        let metadata = [("pandas".to_string(), Self::pandas_metadata())];
        let schema = Arc::new(
            Schema::new(vec![
                Field::new("task_index", DataType::Int64, true),
                Field::new("task", DataType::Utf8, true),
            ])
            .with_metadata(metadata.iter().cloned().collect()),
        );
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![
                Arc::new(Int64Array::from_iter_values(0..self.tasks.len() as i64)),
                Arc::new(StringArray::from(self.tasks.clone())),
            ],
        )
        .map_err(Error::arrow(&path))?;

        crate::atomic::replace_via_temp(&path, |file, temp_path| {
            let props = WriterProperties::builder()
                .set_compression(Compression::SNAPPY)
                .build();
            let mut writer = ArrowWriter::try_new(file, schema.clone(), Some(props))
                .map_err(Error::parquet(temp_path))?;
            writer.write(&batch).map_err(Error::parquet(temp_path))?;
            writer.close().map_err(Error::parquet(temp_path))?;
            Ok(())
        })
    }
}

#[cfg(test)]
mod tests {
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    use super::*;

    #[test]
    fn interns_contiguously_and_dedupes() {
        let mut table = TaskTable::default();
        assert_eq!(table.intern("pick"), (0, true));
        assert_eq!(table.intern("place"), (1, true));
        assert_eq!(table.intern("pick"), (0, false));
        assert_eq!(table.len(), 2);
    }

    #[test]
    fn writes_parquet_with_pandas_index_metadata() {
        let dir = tempfile::tempdir().unwrap();
        let mut table = TaskTable::default();
        table.intern("pick");
        table.intern("place");
        table.write(dir.path()).unwrap();

        let file = std::fs::File::open(dir.path().join(TASKS_PATH)).unwrap();
        let builder = ParquetRecordBatchReaderBuilder::try_new(file).unwrap();
        let metadata = builder.schema().metadata().clone();
        let pandas: serde_json::Value =
            serde_json::from_str(metadata.get("pandas").expect("pandas metadata")).unwrap();
        assert_eq!(pandas["index_columns"], serde_json::json!(["task"]));

        let batches: Vec<_> = builder
            .build()
            .unwrap()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(batches[0].num_rows(), 2);
        let tasks = batches[0]
            .column_by_name("task")
            .unwrap()
            .as_any()
            .downcast_ref::<StringArray>()
            .unwrap();
        assert_eq!(tasks.value(0), "pick");
        assert_eq!(tasks.value(1), "place");
    }
}
