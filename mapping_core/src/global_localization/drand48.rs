//  Player - One Hell of a Robot Server
//  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
//                      gerkey@usc.edu    kaspers@robotics.usc.edu
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Copyright (C) 1995-2026 Free Software Foundation, Inc.
// This file is part of the GNU C Library.
//
// The GNU C Library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// The GNU C Library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with the GNU C Library; if not, see
// <https://www.gnu.org/licenses/>.
//
// This file translates into Rust Nav2 1.5.2's Gaussian draw of AMCL, `pf_ran_gaussian` of
// nav2_amcl/src/pf/pf_pdf.c (Copyright (C) 2000 Brian Gerkey & Kasper Stoy); and the `drand48`
// and `srand48` of glibc 2.43, stdlib/drand48.c, drand48-iter.c, erand48_r.c, srand48.c and
// srand48_r.c (Copyright (C) 1995-2026 Free Software Foundation, Inc.).
//
// Rust translation by Peppy, 2026-11-13.
//
// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// mapping_core is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// mapping_core is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// mapping_core, in COPYING. If not, see <https://www.gnu.org/licenses/>.

//! The generator that every random draw of AMCL comes from: glibc's `drand48`, which
//! nav2_amcl calls with `HAVE_DRAND48` defined (src/pf/pf_pdf.c `pf_ran_gaussian`,
//! src/pf/pf.c `pf_update_resample`, src/amcl_node.cpp `uniformPoseGenerator`). It is a
//! linear congruential generator of 48 bits, x' = (0x5DEECE66D x + 0xB) mod 2^48, that
//! `srand48` seeds with the seed's low 32 bits above 0x330E, and each draw is the new state
//! over 2^48, as glibc's `erand48_r` builds the double. The Gaussian draw of
//! `pf_ran_gaussian` is here too, with its draws in its order.

/// The multiplier of the state.
const MULTIPLIER: u64 = 0x5_DEEC_E66D;
/// The increment of the state.
const INCREMENT: u64 = 0xB;
/// The bits of the state.
const STATE_BITS: u32 = 48;
const STATE_MASK: u64 = (1 << STATE_BITS) - 1;
/// The low 16 bits of the state that `srand48` sets.
const SEED_LOW_BITS: u64 = 0x330E;

/// The state of `drand48`: the stream every random draw of a search takes, which a caller
/// seeds once and hands to each search, as Nav2's AMCL node seeds its stream once.
#[derive(Debug, Clone, PartialEq)]
pub struct Drand48 {
    state: u64,
}

impl Drand48 {
    /// The generator after `srand48(seed)`, which keeps the low 32 bits of its seed, all of
    /// `seed`.
    pub fn seeded(seed: u32) -> Self {
        Self {
            state: (u64::from(seed) << 16) | SEED_LOW_BITS,
        }
    }

    /// `drand48()`: the next state, as the double in [0, 1) that holds its 48 bits.
    pub(crate) fn draw(&mut self) -> f64 {
        self.state = MULTIPLIER.wrapping_mul(self.state).wrapping_add(INCREMENT) & STATE_MASK;
        self.state as f64 / (1u64 << STATE_BITS) as f64
    }

    /// `pf_ran_gaussian(sigma)`: a draw of the Gaussian of mean 0 and standard deviation
    /// `sigma`, by the polar form of the Box-Muller transform: two draws other than 0 make
    /// a point of the square (-1, 1)^2, drawn again until it lies within the unit circle
    /// and off its center.
    pub(crate) fn gaussian(&mut self, sigma: f64) -> f64 {
        loop {
            let x1 = 2.0 * self.draw_other_than_zero() - 1.0;
            let x2 = 2.0 * self.draw_other_than_zero() - 1.0;
            let w = x1 * x1 + x2 * x2;
            if w > 1.0 || w == 0.0 {
                continue;
            }
            return sigma * x2 * (-2.0 * w.ln() / w).sqrt();
        }
    }

    fn draw_other_than_zero(&mut self) -> f64 {
        loop {
            let draw = self.draw();
            if draw != 0.0 {
                return draw;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The draws that glibc 2.43's drand48 gives after srand48(0) and srand48(1).
    #[test]
    fn a_seed_draws_the_stream_of_glibc() {
        let draws = |seed: u32| {
            let mut generator = Drand48::seeded(seed);
            [(); 3].map(|()| generator.draw().to_bits())
        };
        assert_eq!(
            draws(0),
            [0x3fc5ddb16e288080, 0x3fe7ff32702c6f00, 0x3fb8abd0152a2300]
        );
        assert_eq!(
            draws(1),
            [0x3fa5509292a20200, 0x3fdd16677a98de00, 0x3feab6d299454460]
        );
    }

    #[test]
    fn a_gaussian_of_no_deviation_draws_zero_and_moves_the_stream_on() {
        let mut generator = Drand48::seeded(7);
        assert_eq!(generator.gaussian(0.0), 0.0);
        assert_ne!(generator, Drand48::seeded(7));
    }
}
