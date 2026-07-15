//! Data-file rollover under a lowered size limit: episodes land in
//! consecutive files, offsets stay contiguous, and every row survives.

mod common;

use std::fs::File;

use common::{Lcg, one_mb, state_action_builder};
use lerobot_dataset::{DatasetWriter, Frame};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

const STATE_DIM: usize = 256;
const FRAMES_PER_EPISODE: usize = 400;
const EPISODES: usize = 5;

fn read_rows(path: &std::path::Path) -> usize {
    let reader = ParquetRecordBatchReaderBuilder::try_new(File::open(path).unwrap())
        .unwrap()
        .build()
        .unwrap();
    reader.map(|batch| batch.unwrap().num_rows()).sum()
}

#[test]
fn episodes_roll_into_new_data_files() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("ds");
    let config = state_action_builder(STATE_DIM)
        .file_size_limits(one_mb(), one_mb())
        .build()
        .unwrap();
    let mut writer = DatasetWriter::create(&root, config).unwrap();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();

    let mut rng = Lcg(7);
    let mut reported_finalized: Vec<std::path::PathBuf> = Vec::new();
    for _ in 0..EPISODES {
        let mut episode = writer.begin_episode("roll").unwrap();
        for _ in 0..FRAMES_PER_EPISODE {
            let state = rng.fill(STATE_DIM);
            let action = rng.fill(1);
            episode
                .add_frame(Frame {
                    vectors: &[(state_id, &state), (action_id, &action)],
                    images: &[],
                })
                .unwrap();
        }
        reported_finalized.extend(episode.end().unwrap().finalized_files);
    }
    let summary = writer.finalize().unwrap();
    assert_eq!(summary.total_frames as usize, EPISODES * FRAMES_PER_EPISODE);

    // Every data file except the last (still open at finalize) must have been
    // reported immutable exactly once, so a mirror can upload it on rollover.
    let last_data_file = (0..EPISODES)
        .map(|i| format!("data/chunk-000/file-{i:03}.parquet"))
        .rfind(|p| root.join(p).exists())
        .unwrap();
    for i in 0..EPISODES {
        let rel = std::path::PathBuf::from(format!("data/chunk-000/file-{i:03}.parquet"));
        if root.join(&rel).exists() && rel.to_str().unwrap() != last_data_file {
            assert!(
                reported_finalized.contains(&rel),
                "rolled-over {rel:?} must be reported as finalized"
            );
        }
    }
    assert!(
        !reported_finalized
            .iter()
            .any(|p| p.to_str() == Some(last_data_file.as_str())),
        "the still-open last data file must not be reported finalized"
    );

    // ~410 KB of incompressible floats per episode against a 1 MB limit:
    // the third episode must have rolled into a second file.
    assert!(
        root.join("data/chunk-000/file-001.parquet").exists(),
        "expected rollover into file-001"
    );
    let total_rows: usize = (0..EPISODES)
        .map(|i| root.join(format!("data/chunk-000/file-{i:03}.parquet")))
        .filter(|p| p.exists())
        .map(|p| read_rows(&p))
        .sum();
    assert_eq!(
        total_rows,
        EPISODES * FRAMES_PER_EPISODE,
        "all rows must survive across rolled files"
    );

    // Episode bookkeeping: contiguous global offsets and monotone file
    // indices that actually reach file 1.
    let episodes_file = root.join("meta/episodes/chunk-000/file-000.parquet");
    let reader = ParquetRecordBatchReaderBuilder::try_new(File::open(&episodes_file).unwrap())
        .unwrap()
        .build()
        .unwrap();
    let mut next_from = 0i64;
    let mut max_file_index = 0i64;
    for batch in reader {
        let batch = batch.unwrap();
        let from = batch
            .column_by_name("dataset_from_index")
            .unwrap()
            .as_any()
            .downcast_ref::<arrow::array::Int64Array>()
            .unwrap()
            .clone();
        let to = batch
            .column_by_name("dataset_to_index")
            .unwrap()
            .as_any()
            .downcast_ref::<arrow::array::Int64Array>()
            .unwrap()
            .clone();
        let file_index = batch
            .column_by_name("data/file_index")
            .unwrap()
            .as_any()
            .downcast_ref::<arrow::array::Int64Array>()
            .unwrap()
            .clone();
        for row in 0..batch.num_rows() {
            assert_eq!(from.value(row), next_from, "contiguous from_index");
            next_from = to.value(row);
            max_file_index = max_file_index.max(file_index.value(row));
        }
    }
    assert_eq!(next_from as usize, EPISODES * FRAMES_PER_EPISODE);
    assert!(max_file_index >= 1, "episode rows must reference file-001");
}
