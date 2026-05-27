//! CubieCube2L - Extended cube state with center tracking for two-layer solver.
//! Ported from CubieCube2L.java.

use crate::cubie_cube::CubieCube;
use crate::util;
use std::sync::OnceLock;

pub static MOVE2STR: [&str; 20] = [
    "(z1z0) U ",  // 0
    "(z2z0) U2",  // 1
    "(z3z0) U'",  // 2
    "(z0z1) R ",  // 3
    "(z0z2) R2",  // 4
    "(z0z3) R'",  // 5
    "(z1s0) y ",  // 6
    "(z2s0) y2",  // 7
    "(z3s0) y'",  // 8
    "(s0z1) x ",  // 9
    "(s0z2) x2",  // 10
    "(s0z3) x'",  // 11
    "(z0s1)   ",  // 12
    "(s1z0)   ",  // 13
    "(z1s1) y ",  // 14
    "(z2s1) y2",  // 15
    "(z3s1) y'",  // 16
    "(s1z1) x ",  // 17
    "(s1z2) x2",  // 18
    "(s1z3) x'",  // 19
];

/// CubieCube2L extends CubieCube with center tracking (ct field).
#[derive(Clone, Copy)]
pub struct CubieCube2L {
    pub cube: CubieCube,
    pub ct: i32,
}

impl Default for CubieCube2L {
    fn default() -> Self {
        CubieCube2L {
            cube: CubieCube::default(),
            ct: 0x543210,
        }
    }
}

impl CubieCube2L {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_cubie(c: &CubieCube) -> Self {
        CubieCube2L {
            cube: *c,
            ct: 0x543210,
        }
    }

    pub fn from_coords(cperm: i32, twist: i32, eperm: i32, flip: i32, ct: i32) -> Self {
        CubieCube2L {
            cube: CubieCube::from_coords(cperm, twist, eperm, flip),
            ct,
        }
    }

    pub fn copy_from_2l(&mut self, other: &CubieCube2L) {
        *self = *other;
    }

    pub fn copy_from_cube(&mut self, other: &CubieCube) {
        self.cube = *other;
        self.ct = 0x543210;
    }

    pub fn get_slice_f4(&self) -> i32 {
        let slice = self.cube.get_ud_slice();
        let mut flip4 = 0i32;
        for i in 0..12 {
            if (self.cube.ea[i] >> 1) >= 8 {
                flip4 = flip4 << 1 | (self.cube.ea[i] & 1) as i32;
            }
        }
        slice << 4 | flip4
    }

    pub fn set_slice_f4(&mut self, mut idx: i32) {
        self.cube.set_ud_slice(idx >> 4);
        for i in (0..12).rev() {
            if (self.cube.ea[i] >> 1) >= 8 {
                self.cube.ea[i] = (self.cube.ea[i] & 0xfe) | (idx & 1) as u8;
                idx >>= 1;
            }
        }
    }

    pub fn get_slice_f8(&self) -> i32 {
        let slice = self.cube.get_ud_slice();
        let mut flip8 = 0i32;
        for i in 0..12 {
            if (self.cube.ea[i] >> 1) < 8 {
                flip8 = flip8 << 1 | (self.cube.ea[i] & 1) as i32;
            }
        }
        slice << 8 | flip8
    }

    pub fn set_slice_f8(&mut self, mut idx: i32) {
        self.cube.set_ud_slice(idx >> 8);
        for i in (0..12).rev() {
            if (self.cube.ea[i] >> 1) < 8 {
                self.cube.ea[i] = (self.cube.ea[i] & 0xfe) | (idx & 1) as u8;
                idx >>= 1;
            }
        }
    }

    pub fn get_ct_idx(&self) -> i32 {
        let ct_vals = get_ct_idx2val();
        ct_vals
            .iter()
            .take(24)
            .position(|&v| v == self.ct)
            .map_or(-1, |i| i as i32)
    }

    pub fn set_ct_idx(&mut self, idx: i32) {
        let ct_vals = get_ct_idx2val();
        self.ct = ct_vals[idx as usize];
    }

    /// Perform a move on this cube, writing result to `cc`.
    pub fn do_move_to(&self, m: i32, cc: &mut CubieCube2L) {
        let axis = m / 3;
        let pow = m % 3;
        let ct_tables = crate::cubie_cube::get_tables();

        if m >= util::UX1 as i32 && m <= util::BX3 as i32 {
            // Cube move - remap through center
            let real_axis = (self.ct >> (axis << 2)) & 0xf;
            let real_m = real_axis * 3 + pow;
            CubieCube::edge_mult(&self.cube, &ct_tables.move_cube[real_m as usize], &mut cc.cube);
            CubieCube::corn_mult(&self.cube, &ct_tables.move_cube[real_m as usize], &mut cc.cube);
            cc.ct = self.ct;
        } else {
            // Rotation move - only changes center
            cc.cube = self.cube;
            let mut ct = self.ct;
            for _ in 0..=pow {
                match axis {
                    6 => { // y
                        ct = (ct & 0x00000f) | ((ct & 0x0000f0) << 4)
                            | ((ct & 0x000f00) << 8) | (ct & 0x00f000)
                            | ((ct & 0x0f0000) << 4) | ((ct & 0xf00000) >> 16);
                    }
                    7 => { // x
                        ct = ((ct & 0x00000f) << 20) | (ct & 0x0000f0)
                            | ((ct & 0x000f00) >> 8) | ((ct & 0x00f000) >> 4)
                            | (ct & 0x0f0000) | ((ct & 0xf00000) >> 8);
                    }
                    8 => { // z (unused in current code but kept for completeness)
                        ct = ((ct & 0x00000f) << 4) | ((ct & 0x0000f0) << 8)
                            | (ct & 0x000f00) | ((ct & 0x00f000) << 4)
                            | ((ct & 0x0f0000) >> 16) | (ct & 0xf00000);
                    }
                    _ => {}
                }
            }
            cc.ct = ct;
        }
    }
}

// ==================== Center index table ====================

static CT_IDX2VAL: OnceLock<[i32; 24]> = OnceLock::new();

pub fn get_ct_idx2val() -> &'static [i32; 24] {
    CT_IDX2VAL.get_or_init(init_center)
}

fn init_center() -> [i32; 24] {
    let mut table = [0i32; 24];
    let mut ct: i32 = 0x543210;
    for i in 0..24 {
        table[i] = ct;
        // URF rotation — `i % 1 == 0` is always true, kept verbatim from the
        // Java `CubieCube2L.initCenter` source so the three URF/U4/R2 stages
        // remain visually parallel. Do not "simplify".
        #[allow(clippy::modulo_one)]
        if i % 1 == 0 {
            ct = ((ct & 0x00000f) << 4) | ((ct & 0x0000f0) << 4)
                | ((ct & 0x000f00) >> 8) | ((ct & 0x00f000) << 4)
                | ((ct & 0x0f0000) << 4) | ((ct & 0xf00000) >> 8);
        }
        // U4
        if i % 3 == 2 {
            ct = (ct & 0x00000f) | ((ct & 0x0000f0) << 4)
                | ((ct & 0x000f00) << 8) | (ct & 0x00f000)
                | ((ct & 0x0f0000) << 4) | ((ct & 0xf00000) >> 16);
        }
        // R2
        if i % 12 == 11 {
            ct = ((ct & 0x00000f) << 12) | (ct & 0x0000f0)
                | ((ct & 0x000f00) << 12) | ((ct & 0x00f000) >> 12)
                | (ct & 0x0f0000) | ((ct & 0xf00000) >> 12);
        }
    }
    table
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ct_idx2val() {
        let table = get_ct_idx2val();
        assert_eq!(table[0], 0x543210);
        // All 24 values should be distinct
        let mut seen = std::collections::HashSet::new();
        for &v in table.iter() {
            assert!(seen.insert(v), "Duplicate ct value: {:x}", v);
        }
    }

    #[test]
    fn test_slice_f4_roundtrip() {
        let mut cc = CubieCube2L::new();
        cc.set_slice_f4(100);
        assert_eq!(cc.get_slice_f4(), 100);
    }
}
