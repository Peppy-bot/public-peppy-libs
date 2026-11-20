//! The updates that one scan gives the voxels of a map, gathered before the map takes
//! any of them: a byte for each voxel of the grid, at the voxel's index, that holds the
//! strongest update that the scan gives the voxel, and the list of the indices that hold
//! one. A voxel takes one update for each scan, the strongest: a voxel that a cutout
//! holds a point of takes the lowest log-odds, else a hit is a hit, else a crossed voxel
//! is a miss.
//!
//! An insert visits each voxel that its rays cross, often many times. The byte of a
//! voxel is at the voxel's place in the grid, so one visit is one read of memory with no
//! search, and the bytes of the neighbors of a voxel along a ray lie near it in memory,
//! where the CPU finds most of them in its caches. The list of the indices lets the map
//! take the updates and clear the bytes with no visit of a voxel that no ray reached.

use crate::sensor_model::SensorModel;

/// An update that a scan gives a voxel, from the weakest to the strongest: of the
/// updates that one scan gives a voxel, the voxel takes the strongest alone.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub(crate) enum ScanUpdate {
    /// A ray crosses the voxel, or the ray of a point beyond the range ends in it.
    Miss = 1,
    /// A point of the scan lies in it, which no cutout holds.
    Hit = 2,
    /// A point of the scan that a cutout holds lies in it.
    CutOut = 3,
}

/// The byte of a voxel that the scan gives no update.
const NO_UPDATE: u8 = 0;

impl ScanUpdate {
    /// The update whose byte is `byte`; none for `NO_UPDATE`.
    fn of_byte(byte: u8) -> Option<Self> {
        match byte {
            NO_UPDATE => None,
            1 => Some(Self::Miss),
            2 => Some(Self::Hit),
            3 => Some(Self::CutOut),
            other => unreachable!("byte {other} is no update: only `give` writes a byte"),
        }
    }

    /// The log-odds that the update adds to a voxel by `model`.
    pub fn log_odds(self, model: &SensorModel) -> f32 {
        match self {
            Self::Miss => model.miss,
            Self::Hit => model.hit,
            Self::CutOut => model.cut_out,
        }
    }
}

/// The updates of one scan, at the index of each voxel in the grid. Between two scans
/// every byte is `NO_UPDATE` and the list is empty, so a map keeps one `ScanUpdates`
/// from one insert to the next and an insert allocates nothing for each voxel of the
/// map.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct ScanUpdates {
    /// For each voxel of the grid, the byte of the strongest update that the scan gives
    /// it, `NO_UPDATE` where it gives none. Empty before the first scan.
    strongest: Vec<u8>,
    /// The index of each voxel whose byte holds an update, in the order of its first
    /// update.
    touched: Vec<u32>,
}

impl ScanUpdates {
    /// Makes the updates ready for a scan of a grid of `voxels` voxels: a byte for each,
    /// each `NO_UPDATE`. The first scan of a grid makes the bytes, and each later scan
    /// finds them ready.
    pub fn ready_for(&mut self, voxels: usize) {
        if self.strongest.len() != voxels {
            // Zeroed memory. For a large grid, the allocator of glibc takes it from
            // pages that the system zeroes when they are first written, so a byte that no
            // ray reaches costs no work.
            self.strongest = vec![NO_UPDATE; voxels];
        }
    }

    /// Gives the voxel at grid index `index` the update `update`. The voxel keeps the
    /// strongest update that it is given.
    pub fn give(&mut self, index: usize, update: ScanUpdate) {
        let held = &mut self.strongest[index];
        if *held == NO_UPDATE {
            self.touched.push(
                u32::try_from(index).expect("a map holds fewer voxels than 2^32 (MAX_VOXELS)"),
            );
        }
        *held = (*held).max(update as u8);
    }

    /// Gives `take` the grid index of each voxel that holds an update, with the strongest
    /// update that it holds, in the order of the first update of each, and leaves no
    /// update held.
    pub fn drain(&mut self, mut take: impl FnMut(usize, ScanUpdate)) {
        for &index in &self.touched {
            let index = index as usize;
            let byte = std::mem::replace(&mut self.strongest[index], NO_UPDATE);
            take(
                index,
                ScanUpdate::of_byte(byte).expect("a voxel of the list holds an update"),
            );
        }
        self.touched.clear();
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    const UPDATES: [ScanUpdate; 3] = [ScanUpdate::Miss, ScanUpdate::Hit, ScanUpdate::CutOut];

    /// The updates that `updates` gives, in the order of `drain`.
    fn drained(updates: &mut ScanUpdates) -> Vec<(usize, ScanUpdate)> {
        let mut taken = Vec::new();
        updates.drain(|index, update| taken.push((index, update)));
        taken
    }

    // Each update reads back from its byte, and the byte of no update reads as none.
    #[test]
    fn each_update_reads_back_from_its_byte() {
        for update in UPDATES {
            assert_eq!(ScanUpdate::of_byte(update as u8), Some(update));
        }
        assert_eq!(ScanUpdate::of_byte(NO_UPDATE), None);
    }

    // Of drawn updates of drawn voxels, given in any order, each voxel given one comes out
    // once, in the order of its first update, with the strongest that it was given; and
    // after the drain no voxel holds an update, so the next scan of the grid starts
    // from none.
    #[hegel::test]
    fn each_voxel_comes_out_once_with_its_strongest_update(tc: TestCase) {
        let voxels: usize = tc.draw(gs::integers().min_value(1).max_value(64));
        let given: Vec<(usize, ScanUpdate)> = tc.draw(gs::vecs(hegel::tuples!(
            gs::integers::<usize>().max_value(voxels - 1),
            gs::sampled_from(UPDATES.as_slice()),
        )));
        let mut updates = ScanUpdates::default();
        updates.ready_for(voxels);
        let mut strongest = BTreeMap::new();
        let mut first_order = Vec::new();
        for &(index, update) in &given {
            updates.give(index, update);
            let held = strongest.entry(index).or_insert_with(|| {
                first_order.push(index);
                update
            });
            *held = (*held).max(update);
        }
        let expected: Vec<(usize, ScanUpdate)> = first_order
            .iter()
            .map(|index| (*index, strongest[index]))
            .collect();
        assert_eq!(drained(&mut updates), expected);
        assert!(updates.strongest.iter().all(|byte| *byte == NO_UPDATE));
        assert!(drained(&mut updates).is_empty());
    }

    // A later scan of the same grid finds the bytes of the first ready, and makes no new
    // ones.
    #[test]
    fn a_later_scan_of_the_grid_keeps_the_bytes_of_the_first() {
        let mut updates = ScanUpdates::default();
        updates.ready_for(1000);
        updates.give(7, ScanUpdate::Hit);
        assert_eq!(drained(&mut updates), [(7, ScanUpdate::Hit)]);
        let bytes = updates.strongest.as_ptr();
        updates.ready_for(1000);
        assert_eq!(updates.strongest.as_ptr(), bytes);
        updates.give(7, ScanUpdate::Miss);
        assert_eq!(drained(&mut updates), [(7, ScanUpdate::Miss)]);
    }
}
