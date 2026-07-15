//! Pure path and rollover math for the v3 on-disk layout.

use std::path::PathBuf;

pub const CODEBASE_VERSION: &str = "v3.0";
pub const CHUNKS_SIZE: u64 = 1000;
pub const DATA_FILES_SIZE_IN_MB: u64 = 100;
pub const VIDEO_FILES_SIZE_IN_MB: u64 = 200;

/// The int64 bookkeeping columns the writer appends after `timestamp`, in
/// canonical (info.json and parquet) order.
pub(crate) const INT64_BOOKKEEPING_COLUMNS: [&str; 4] =
    ["frame_index", "episode_index", "index", "task_index"];

/// All bookkeeping columns the writer appends after the declared features, in
/// canonical order: the float32 `timestamp` then [`INT64_BOOKKEEPING_COLUMNS`].
pub(crate) const BOOKKEEPING_COLUMNS: [&str; 5] = [
    "timestamp",
    INT64_BOOKKEEPING_COLUMNS[0],
    INT64_BOOKKEEPING_COLUMNS[1],
    INT64_BOOKKEEPING_COLUMNS[2],
    INT64_BOOKKEEPING_COLUMNS[3],
];

pub const INFO_PATH: &str = "meta/info.json";
pub const STATS_PATH: &str = "meta/stats.json";
pub const TASKS_PATH: &str = "meta/tasks.parquet";

pub const DATA_PATH_TEMPLATE: &str = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet";
pub const VIDEO_PATH_TEMPLATE: &str =
    "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4";

/// Position in the chunked file sequence shared by data, episodes, and video files.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FileSlot {
    pub chunk_index: u64,
    pub file_index: u64,
}

impl FileSlot {
    /// The slot after this one; `file_index` wraps into a new chunk at [`CHUNKS_SIZE`].
    pub fn next(self) -> FileSlot {
        if self.file_index + 1 == CHUNKS_SIZE {
            FileSlot {
                chunk_index: self.chunk_index + 1,
                file_index: 0,
            }
        } else {
            FileSlot {
                chunk_index: self.chunk_index,
                file_index: self.file_index + 1,
            }
        }
    }
}

pub fn data_path(slot: FileSlot) -> PathBuf {
    PathBuf::from(format!(
        "data/chunk-{:03}/file-{:03}.parquet",
        slot.chunk_index, slot.file_index
    ))
}

pub fn episodes_path(slot: FileSlot) -> PathBuf {
    PathBuf::from(format!(
        "meta/episodes/chunk-{:03}/file-{:03}.parquet",
        slot.chunk_index, slot.file_index
    ))
}

pub fn video_path(video_key: &str, slot: FileSlot) -> PathBuf {
    PathBuf::from(format!(
        "videos/{video_key}/chunk-{:03}/file-{:03}.mp4",
        slot.chunk_index, slot.file_index
    ))
}

pub fn mb_to_bytes(mb: u64) -> u64 {
    mb * 1024 * 1024
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_match_spec_templates() {
        let slot = FileSlot {
            chunk_index: 0,
            file_index: 0,
        };
        assert_eq!(
            data_path(slot).to_str().unwrap(),
            "data/chunk-000/file-000.parquet"
        );
        assert_eq!(
            episodes_path(slot).to_str().unwrap(),
            "meta/episodes/chunk-000/file-000.parquet"
        );
        assert_eq!(
            video_path("observation.images.cam_a", slot)
                .to_str()
                .unwrap(),
            "videos/observation.images.cam_a/chunk-000/file-000.mp4"
        );
        let wide = FileSlot {
            chunk_index: 12,
            file_index: 345,
        };
        assert_eq!(
            data_path(wide).to_str().unwrap(),
            "data/chunk-012/file-345.parquet"
        );
    }

    #[test]
    fn slot_wraps_at_chunk_size() {
        let last = FileSlot {
            chunk_index: 3,
            file_index: CHUNKS_SIZE - 1,
        };
        assert_eq!(
            last.next(),
            FileSlot {
                chunk_index: 4,
                file_index: 0
            }
        );
        let mid = FileSlot {
            chunk_index: 3,
            file_index: 7,
        };
        assert_eq!(
            mid.next(),
            FileSlot {
                chunk_index: 3,
                file_index: 8
            }
        );
    }
}
