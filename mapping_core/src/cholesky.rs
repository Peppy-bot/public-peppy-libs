// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// This file translates into Rust Eigen 3.4.0's Cholesky factorization of a 3 x 3 matrix,
// `llt_inplace<Scalar, Lower>::unblocked` of Eigen/src/Cholesky/LLT.h (Copyright (C) 2008 Gael
// Guennebaud).
//
// Rust translation by Peppy, 2026-11-13.
//
// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// This file is subject to the terms of the Mozilla Public License, version 2.0. You should have
// received a copy of the Mozilla Public License along with mapping_core, in LICENSE-MPL-2.0.
// If not, see <https://mozilla.org/MPL/2.0/>.

//! The upper Cholesky factor of a 3 x 3 information matrix, as Eigen's `LLT<Matrix3d>`
//! computes it for slam_toolbox's `PoseGraph2dErrorTerm` (`sqrt_information_`).

/// Eigen's `LLT<Matrix3d>::matrixU` of `information`: the transpose of its lower Cholesky
/// factor, computed in place column by column. When a pivot is not positive, the columns
/// from it on keep what the factorization had left in them, as Eigen leaves them.
pub(crate) fn upper_cholesky_factor(information: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut m = information;
    'columns: for k in 0..3 {
        let x = m[k][k] - (0..k).map(|j| m[k][j] * m[k][j]).sum::<f64>();
        if x <= 0.0 {
            break 'columns;
        }
        let pivot = x.sqrt();
        m[k][k] = pivot;
        for i in (k + 1)..3 {
            let dot: f64 = (0..k).map(|j| m[i][j] * m[k][j]).sum();
            m[i][k] = (m[i][k] - dot) / pivot;
        }
    }
    [0, 1, 2].map(|row| [0, 1, 2].map(|column| if column >= row { m[column][row] } else { 0.0 }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_upper_factor_times_itself_gives_the_information() {
        let information = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]];
        let u = upper_cholesky_factor(information);
        for row in 0..3 {
            for column in 0..3 {
                let product: f64 = (0..3).map(|k| u[k][row] * u[k][column]).sum();
                assert!((product - information[row][column]).abs() < 1e-12);
            }
        }
    }
}
