//! Scores the runs of every room dataset scenario that the reference program wrote, writes
//! the statistics of each seed as the scenario's reference output, and appends the kind's
//! sources to the list of inputs. `run.sh --datasets` runs it after the reference program:
//!
//!     statistics <results directory> <directory of the reference outputs> <inputs list>

use std::path::PathBuf;

fn main() {
    let arguments: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
    let [results, outputs, inputs_list] = arguments.as_slice() else {
        eprintln!(
            "usage: statistics <results directory> <directory of the reference outputs> \
             <inputs list>"
        );
        std::process::exit(2);
    };
    ros_reference::room_dataset::write_statistics(results, outputs, inputs_list);
}
