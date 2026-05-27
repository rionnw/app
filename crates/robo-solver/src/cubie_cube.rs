//! CubieCube - Cube state representation and symmetry operations.
//! Ported from CubieCube.java.

use std::sync::OnceLock;
use crate::util;
use crate::coord_cube;

/// CubieCube represents a cube state at the cubie level.
#[derive(Clone, Copy)]
pub struct CubieCube {
    /// Corner array: ca[i] = ori << 3 | perm
    pub ca: [u8; 8],
    /// Edge array: ea[i] = perm << 1 | flip
    pub ea: [u8; 12],
}

impl Default for CubieCube {
    fn default() -> Self {
        CubieCube {
            ca: [0, 1, 2, 3, 4, 5, 6, 7],
            ea: [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22],
        }
    }
}

impl CubieCube {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_coords(cperm: i32, twist: i32, eperm: i32, flip: i32) -> Self {
        let mut c = Self::new();
        c.set_c_perm(cperm);
        c.set_twist(twist);
        util::set_n_perm(&mut c.ea, eperm, 12, true);
        c.set_flip(flip);
        c
    }

    pub fn copy_from(&mut self, other: &CubieCube) {
        *self = *other;
    }

    // ==================== Multiplication ====================

    /// prod = a * b, Corner Only.
    pub fn corn_mult(a: &CubieCube, b: &CubieCube, prod: &mut CubieCube) {
        for corn in 0..8 {
            let ori_a = (a.ca[(b.ca[corn] & 7) as usize] >> 3) as i32;
            let ori_b = (b.ca[corn] >> 3) as i32;
            let ori = ori_a + if ori_a < 3 { ori_b } else { 6 - ori_b };
            let ori = ori % 3 + if (ori_a < 3) == (ori_b < 3) { 0 } else { 3 };
            prod.ca[corn] = (a.ca[(b.ca[corn] & 7) as usize] & 7) | ((ori as u8) << 3);
        }
    }

    /// prod = a * b, Edge Only.
    pub fn edge_mult(a: &CubieCube, b: &CubieCube, prod: &mut CubieCube) {
        for ed in 0..12 {
            prod.ea[ed] = a.ea[(b.ea[ed] >> 1) as usize] ^ (b.ea[ed] & 1);
        }
    }

    /// b = S_idx^-1 * a * S_idx, Corner Only.
    pub fn corn_conjugate(a: &CubieCube, idx: usize, b: &mut CubieCube) {
        let tables = get_tables();
        let sinv = &tables.cube_sym[tables.sym_mult_inv[0][idx]];
        let s = &tables.cube_sym[idx];
        for corn in 0..8 {
            let ori_a = (sinv.ca[(a.ca[(s.ca[corn] & 7) as usize] & 7) as usize] >> 3) as i32;
            let ori_b = (a.ca[(s.ca[corn] & 7) as usize] >> 3) as i32;
            let ori = if ori_a < 3 { ori_b } else { (3 - ori_b) % 3 };
            b.ca[corn] = (sinv.ca[(a.ca[(s.ca[corn] & 7) as usize] & 7) as usize] & 7) | ((ori as u8) << 3);
        }
    }

    /// b = S_idx^-1 * a * S_idx, Edge Only.
    pub fn edge_conjugate(a: &CubieCube, idx: usize, b: &mut CubieCube) {
        let tables = get_tables();
        let sinv = &tables.cube_sym[tables.sym_mult_inv[0][idx]];
        let s = &tables.cube_sym[idx];
        for ed in 0..12 {
            b.ea[ed] = sinv.ea[(a.ea[(s.ea[ed] >> 1) as usize] >> 1) as usize]
                ^ (a.ea[(s.ea[ed] >> 1) as usize] & 1)
                ^ (s.ea[ed] & 1);
        }
    }

    /// Invert this cubie cube in place.
    pub fn inv_cubie_cube(&mut self) {
        let mut temp = CubieCube::new();
        for edge in 0..12u8 {
            temp.ea[(self.ea[edge as usize] >> 1) as usize] = edge << 1 | (self.ea[edge as usize] & 1);
        }
        for corn in 0..8u8 {
            temp.ca[(self.ca[corn as usize] & 0x7) as usize] =
                corn | ((0x20u8 >> (self.ca[corn as usize] >> 3)) & 0x18);
        }
        *self = temp;
    }

    /// this = S_urf^-1 * this * S_urf
    pub fn urf_conjugate(&mut self) {
        let urf1 = CubieCube::from_coords(2531, 1373, 67026819, 1367);
        let urf2 = CubieCube::from_coords(2089, 1906, 322752913, 2040);
        let mut temp = CubieCube::new();
        Self::corn_mult(&urf2, self, &mut temp);
        Self::corn_mult(&temp, &urf1, self);
        Self::edge_mult(&urf2, self, &mut temp);
        Self::edge_mult(&temp, &urf1, self);
    }

    // ==================== Coordinates ====================

    pub fn get_flip(&self) -> i32 {
        let mut idx = 0i32;
        for i in 0..11 {
            idx = idx << 1 | (self.ea[i] & 1) as i32;
        }
        idx
    }

    pub fn set_flip(&mut self, mut idx: i32) {
        let mut parity = 0;
        for i in (0..=10).rev() {
            let val = idx & 1;
            parity ^= val;
            self.ea[i] = (self.ea[i] & 0xfe) | val as u8;
            idx >>= 1;
        }
        self.ea[11] = (self.ea[11] & 0xfe) | parity as u8;
    }

    pub fn get_flip_sym(&self) -> i32 {
        flip_raw2sym(self.get_flip())
    }

    pub fn get_twist(&self) -> i32 {
        let mut idx = 0i32;
        for i in 0..7 {
            idx += (idx << 1) + (self.ca[i] >> 3) as i32;
        }
        idx
    }

    pub fn set_twist(&mut self, mut idx: i32) {
        let mut twst = 15i32;
        for i in (0..=6).rev() {
            let val = idx % 3;
            twst -= val;
            self.ca[i] = (self.ca[i] & 0x7) | ((val as u8) << 3);
            idx /= 3;
        }
        self.ca[7] = (self.ca[7] & 0x7) | (((twst % 3) as u8) << 3);
    }

    pub fn get_twist_sym(&self) -> i32 {
        let raw = self.get_twist();
        twist_raw2sym(raw)
    }

    pub fn get_ud_slice(&self) -> i32 {
        494 - util::get_comb(&self.ea, 8, true)
    }

    pub fn set_ud_slice(&mut self, idx: i32) {
        util::set_comb(&mut self.ea, 494 - idx, 8, true);
    }

    pub fn get_c_perm(&self) -> i32 {
        util::get_n_perm(&self.ca, 8, false)
    }

    pub fn set_c_perm(&mut self, idx: i32) {
        util::set_n_perm(&mut self.ca, idx, 8, false);
    }

    pub fn get_c_perm_sym(&self) -> i32 {
        let tables = get_tables();
        let raw = self.get_c_perm();
        let k_raw = coord_cube::get_pruning_from_byte(&tables.e_perm_r2s, raw as usize);
        let k = (e_sym2c_sym(k_raw as i32) & 0xf) as usize;
        let mut pc = CubieCube::new();
        Self::corn_conjugate(self, tables.sym_mult_inv[0][k], &mut pc);
        let cp = pc.get_c_perm();
        let idx = tables.e_perm_s2r.binary_search(&(cp as u16)).unwrap();
        (idx as i32) << 4 | k as i32
    }

    pub fn get_e_perm(&self) -> i32 {
        util::get_n_perm(&self.ea, 8, true)
    }

    pub fn set_e_perm(&mut self, idx: i32) {
        util::set_n_perm(&mut self.ea, idx, 8, true);
    }

    pub fn get_e_perm_sym(&self) -> i32 {
        let tables = get_tables();
        let raw = self.get_e_perm();
        let k = coord_cube::get_pruning_from_byte(&tables.e_perm_r2s, raw as usize) as usize;
        let mut pc = CubieCube::new();
        Self::edge_conjugate(self, tables.sym_mult_inv[0][k], &mut pc);
        let ep = pc.get_e_perm();
        let idx = tables.e_perm_s2r.binary_search(&(ep as u16)).unwrap();
        (idx as i32) << 4 | k as i32
    }

    pub fn get_m_perm(&self) -> i32 {
        util::get_n_perm(&self.ea, 12, true) % 24
    }

    pub fn set_m_perm(&mut self, idx: i32) {
        util::set_n_perm(&mut self.ea, idx, 12, true);
    }

    pub fn get_c_comb(&self) -> i32 {
        util::get_comb(&self.ca, 0, false)
    }

    pub fn set_c_comb(&mut self, idx: i32) {
        util::set_comb(&mut self.ca, idx, 0, false);
    }

    /// Check a cubiecube for solvability. Return error code (0 = ok).
    pub fn verify(&self) -> i32 {
        let mut sum = 0i32;
        let mut edge_mask = 0u32;
        for e in 0..12 {
            edge_mask |= 1 << (self.ea[e] >> 1);
            sum ^= (self.ea[e] & 1) as i32;
        }
        if edge_mask != 0xfff {
            return -2;
        }
        if sum != 0 {
            return -3;
        }
        let mut corn_mask = 0u32;
        sum = 0;
        for c in 0..8 {
            corn_mask |= 1 << (self.ca[c] & 7);
            sum += (self.ca[c] >> 3) as i32;
        }
        if corn_mask != 0xff {
            return -4;
        }
        if sum % 3 != 0 {
            return -5;
        }
        if (util::get_n_parity(util::get_n_perm(&self.ea, 12, true), 12)
            ^ util::get_n_parity(self.get_c_perm(), 8)) != 0
        {
            return -6;
        }
        0
    }

    pub fn self_symmetry(&self) -> i64 {
        let tables = get_tables();
        let mut c = *self;
        let mut d = CubieCube::new();
        let mut sym = 0i64;
        for i in 0..96 {
            Self::corn_conjugate(&c, tables.sym_mult_inv[0][i % 16], &mut d);
            if d.ca == self.ca {
                Self::edge_conjugate(&c, tables.sym_mult_inv[0][i % 16], &mut d);
                if d.ea == self.ea {
                    sym |= 1i64 << (i.min(48));
                }
            }
            if i % 16 == 15 {
                c.urf_conjugate();
            }
            if i % 48 == 47 {
                c.inv_cubie_cube();
            }
        }
        sym
    }
}

// ==================== Constants ====================

pub static URF1: CubieCube = CubieCube {
    ca: [0, 1, 2, 3, 4, 5, 6, 7], // Will be initialized properly
    ea: [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22],
};

pub static URF2: CubieCube = CubieCube {
    ca: [0, 1, 2, 3, 4, 5, 6, 7],
    ea: [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22],
};

// These are computed at init time
pub static URF_MOVE: [[u8; 18]; 6] = [
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],
    [6, 7, 8, 0, 1, 2, 3, 4, 5, 15, 16, 17, 9, 10, 11, 12, 13, 14],
    [3, 4, 5, 6, 7, 8, 0, 1, 2, 12, 13, 14, 15, 16, 17, 9, 10, 11],
    [2, 1, 0, 5, 4, 3, 8, 7, 6, 11, 10, 9, 14, 13, 12, 17, 16, 15],
    [8, 7, 6, 2, 1, 0, 5, 4, 3, 17, 16, 15, 11, 10, 9, 14, 13, 12],
    [5, 4, 3, 8, 7, 6, 2, 1, 0, 14, 13, 12, 17, 16, 15, 11, 10, 9],
];

pub const SYM_E2C_MAGIC: i32 = 0x00DDDD00u32 as i32;

pub fn e_sym2c_sym(idx: i32) -> i32 {
    idx ^ ((SYM_E2C_MAGIC >> ((idx & 0xf) << 1)) & 3)
}

pub fn flip_raw2sym(raw: i32) -> i32 {
    let tables = get_tables();
    let high = tables.flip_r2s[(raw as usize) + coord_cube::N_FLIP_HALF] as i32;
    let low = coord_cube::get_pruning_from_byte(&tables.flip_r2s, raw as usize) as i32;
    (0xfff & (high << 4)) | low
}

pub fn twist_raw2sym(raw: i32) -> i32 {
    let tables = get_tables();
    let high = tables.twist_r2s[(raw as usize) + coord_cube::N_TWIST_HALF] as i32;
    let low = coord_cube::get_pruning_from_byte(&tables.twist_r2s, raw as usize) as i32;
    (0xfff & (high << 4)) | low
}

pub fn get_perm_sym_inv(idx: i32, sym: usize, is_corner: bool) -> i32 {
    let tables = get_tables();
    let mut idxi = tables.perm_inv_edge_sym[idx as usize] as i32;
    if is_corner {
        idxi = e_sym2c_sym(idxi);
    }
    (idxi & 0xfff0) | tables.sym_mult[(idxi & 0xf) as usize][sym]
}

pub fn get_skip_moves(ssym: i64) -> i32 {
    let tables = get_tables();
    let mut ret = 0;
    let mut s = ssym;
    for i in 1..48 {
        s >>= 1;
        if s == 0 { break; }
        if (s & 1) == 1 {
            ret |= tables.first_move_sym[i];
        }
    }
    ret
}

// ==================== Tables ====================

/// All global tables for CubieCube, lazily initialized.
pub struct CubieCubeTables {
    pub cube_sym: Vec<CubieCube>,       // [16]
    pub move_cube: Vec<CubieCube>,      // [18]
    pub move_cube_sym: [i64; 18],
    pub first_move_sym: [i32; 48],
    pub sym_mult: [[i32; 16]; 16],
    pub sym_mult_inv: [[usize; 16]; 16],
    pub sym_move: [[i32; 18]; 16],
    pub sym8_move: [i32; 144],          // [8*18]
    pub sym_move_ud: [[i32; 18]; 16],

    pub flip_s2r: Vec<u16>,             // [N_FLIP_SYM]
    pub twist_s2r: Vec<u16>,            // [N_TWIST_SYM]
    pub e_perm_s2r: Vec<u16>,           // [N_PERM_SYM]
    pub perm2_comb_p: Vec<u8>,          // [N_PERM_SYM]
    pub perm_inv_edge_sym: Vec<u16>,    // [N_PERM_SYM]

    pub flip_r2s: Vec<u8>,              // [N_FLIP_HALF + N_FLIP]
    pub twist_r2s: Vec<u8>,             // [N_TWIST_HALF + N_TWIST]
    pub e_perm_r2s: Vec<u8>,            // [N_PERM_HALF]
    pub flip_s2rf: Vec<u16>,            // [N_FLIP_SYM * 8]

    pub sym_state_twist: Vec<u16>,      // [N_TWIST_SYM]
    pub sym_state_flip: Vec<u16>,       // [N_FLIP_SYM]
    pub sym_state_perm: Vec<u16>,       // [N_PERM_SYM]
}

static TABLES: OnceLock<CubieCubeTables> = OnceLock::new();

pub fn get_tables() -> &'static CubieCubeTables {
    TABLES.get_or_init(init_all_tables)
}

pub fn ensure_tables_initialized() {
    let _ = get_tables();
}

fn init_all_tables() -> CubieCubeTables {
    let mut tables = CubieCubeTables {
        cube_sym: vec![CubieCube::new(); 16],
        move_cube: vec![CubieCube::new(); 18],
        move_cube_sym: [0i64; 18],
        first_move_sym: [0i32; 48],
        sym_mult: [[0i32; 16]; 16],
        sym_mult_inv: [[0usize; 16]; 16],
        sym_move: [[0i32; 18]; 16],
        sym8_move: [0i32; 144],
        sym_move_ud: [[0i32; 18]; 16],
        flip_s2r: vec![0u16; coord_cube::N_FLIP_SYM],
        twist_s2r: vec![0u16; coord_cube::N_TWIST_SYM],
        e_perm_s2r: vec![0u16; coord_cube::N_PERM_SYM],
        perm2_comb_p: vec![0u8; coord_cube::N_PERM_SYM],
        perm_inv_edge_sym: vec![0u16; coord_cube::N_PERM_SYM],
        flip_r2s: vec![0u8; coord_cube::N_FLIP_HALF + coord_cube::N_FLIP],
        twist_r2s: vec![0u8; coord_cube::N_TWIST_HALF + coord_cube::N_TWIST],
        e_perm_r2s: vec![0u8; coord_cube::N_PERM_HALF],
        flip_s2rf: vec![0u16; coord_cube::N_FLIP_SYM * 8],
        sym_state_twist: vec![0u16; coord_cube::N_TWIST_SYM],
        sym_state_flip: vec![0u16; coord_cube::N_FLIP_SYM],
        sym_state_perm: vec![0u16; coord_cube::N_PERM_SYM],
    };

    init_move(&mut tables);
    init_sym(&mut tables);
    init_flip_sym2raw(&mut tables);
    init_twist_sym2raw(&mut tables);
    init_perm_sym2raw(&mut tables);

    tables
}

fn init_move(tables: &mut CubieCubeTables) {
    tables.move_cube[0] = CubieCube::from_coords(15120, 0, 119750400, 0);
    tables.move_cube[3] = CubieCube::from_coords(21021, 1494, 323403417, 0);
    tables.move_cube[6] = CubieCube::from_coords(8064, 1236, 29441808, 550);
    tables.move_cube[9] = CubieCube::from_coords(9, 0, 5880, 0);
    tables.move_cube[12] = CubieCube::from_coords(1230, 412, 2949660, 0);
    tables.move_cube[15] = CubieCube::from_coords(224, 137, 328552, 137);

    for a in (0..18).step_by(3) {
        for p in 0..2 {
            let mut prod = CubieCube::new();
            CubieCube::edge_mult(&tables.move_cube[a + p], &tables.move_cube[a], &mut prod);
            CubieCube::corn_mult(&tables.move_cube[a + p], &tables.move_cube[a], &mut prod);
            tables.move_cube[a + p + 1] = prod;
        }
    }
}

fn init_sym(tables: &mut CubieCubeTables) {
    let mut c = CubieCube::new();
    let _d = CubieCube::new();

    let f2 = CubieCube::from_coords(28783, 0, 259268407, 0);
    let u4 = CubieCube::from_coords(15138, 0, 119765538, 7);
    let mut lr2 = CubieCube::from_coords(5167, 0, 83473207, 0);
    for i in 0..8 {
        lr2.ca[i] |= 3 << 3;
    }

    for i in 0..16 {
        tables.cube_sym[i] = c;
        let mut tmp = CubieCube::new();
        CubieCube::corn_mult(&c, &u4, &mut tmp);
        CubieCube::edge_mult(&c, &u4, &mut tmp);
        std::mem::swap(&mut c, &mut tmp);
        if i % 4 == 3 {
            let mut tmp = CubieCube::new();
            CubieCube::corn_mult(&c, &lr2, &mut tmp);
            CubieCube::edge_mult(&c, &lr2, &mut tmp);
            std::mem::swap(&mut c, &mut tmp);
        }
        if i % 8 == 7 {
            let mut tmp = CubieCube::new();
            CubieCube::corn_mult(&c, &f2, &mut tmp);
            CubieCube::edge_mult(&c, &f2, &mut tmp);
            std::mem::swap(&mut c, &mut tmp);
        }
    }

    // SymMult, SymMultInv
    for i in 0..16 {
        for j in 0..16 {
            CubieCube::corn_mult(&tables.cube_sym[i], &tables.cube_sym[j], &mut c);
            for k in 0..16 {
                if tables.cube_sym[k].ca == c.ca {
                    tables.sym_mult[i][j] = k as i32;
                    tables.sym_mult_inv[k][j] = i;
                    break;
                }
            }
        }
    }

    // SymMove, SymMoveUD
    let std2ud = util::std2ud();
    for j in 0..18 {
        for s in 0..16 {
            corn_conjugate_raw(&tables.move_cube[j], tables.sym_mult_inv[0][s], &tables.cube_sym, &tables.sym_mult_inv, &mut c);
            for m in 0..18 {
                if tables.move_cube[m].ca == c.ca {
                    tables.sym_move[s][j] = m as i32;
                    tables.sym_move_ud[s][std2ud[j] as usize] = std2ud[m];
                    break;
                }
            }
            if s % 2 == 0 {
                tables.sym8_move[j << 3 | s >> 1] = tables.sym_move[s][j];
            }
        }
    }

    // moveCubeSym, firstMoveSym
    for i in 0..18 {
        tables.move_cube_sym[i] = self_symmetry_with_tables(&tables.move_cube[i], tables);
        let mut j = i;
        for s in 0..48 {
            if tables.sym_move[s % 16][j] < i as i32 {
                tables.first_move_sym[s] |= 1 << i;
            }
            if s % 16 == 15 {
                j = URF_MOVE[2][j] as usize;
            }
        }
    }
}

/// Compute self_symmetry using provided tables (during initialization).
fn self_symmetry_with_tables(cube: &CubieCube, tables: &CubieCubeTables) -> i64 {
    let mut c = *cube;
    let mut d = CubieCube::new();
    let mut sym = 0i64;

    for i in 0..96 {
        corn_conjugate_raw(&c, tables.sym_mult_inv[0][i % 16], &tables.cube_sym, &tables.sym_mult_inv, &mut d);
        if d.ca == cube.ca {
            edge_conjugate_raw(&c, tables.sym_mult_inv[0][i % 16], &tables.cube_sym, &tables.sym_mult_inv, &mut d);
            if d.ea == cube.ea {
                sym |= 1i64 << (i.min(48));
            }
        }
        if i % 16 == 15 {
            urf_conjugate_raw(&mut c);
        }
        if i % 48 == 47 {
            c.inv_cubie_cube();
        }
    }
    sym
}

/// Public wrapper for edge_conjugate_raw (used by coord_cube)
pub fn corn_conjugate_raw(a: &CubieCube, idx: usize, cube_sym: &[CubieCube], sym_mult_inv: &[[usize; 16]; 16], b: &mut CubieCube) {
    let sinv = &cube_sym[sym_mult_inv[0][idx]];
    let s = &cube_sym[idx];
    for corn in 0..8 {
        let ori_a = (sinv.ca[(a.ca[(s.ca[corn] & 7) as usize] & 7) as usize] >> 3) as i32;
        let ori_b = (a.ca[(s.ca[corn] & 7) as usize] >> 3) as i32;
        let ori = if ori_a < 3 { ori_b } else { (3 - ori_b) % 3 };
        b.ca[corn] = (sinv.ca[(a.ca[(s.ca[corn] & 7) as usize] & 7) as usize] & 7) | ((ori as u8) << 3);
    }
}

pub fn edge_conjugate_raw(a: &CubieCube, idx: usize, cube_sym: &[CubieCube], sym_mult_inv: &[[usize; 16]; 16], b: &mut CubieCube) {
    let sinv = &cube_sym[sym_mult_inv[0][idx]];
    let s = &cube_sym[idx];
    for ed in 0..12 {
        b.ea[ed] = sinv.ea[(a.ea[(s.ea[ed] >> 1) as usize] >> 1) as usize]
            ^ (a.ea[(s.ea[ed] >> 1) as usize] & 1)
            ^ (s.ea[ed] & 1);
    }
}

fn urf_conjugate_raw(cube: &mut CubieCube) {
    let urf1 = CubieCube::from_coords(2531, 1373, 67026819, 1367);
    let urf2 = CubieCube::from_coords(2089, 1906, 322752913, 2040);
    let mut temp = CubieCube::new();
    CubieCube::corn_mult(&urf2, cube, &mut temp);
    CubieCube::corn_mult(&temp, &urf1, cube);
    CubieCube::edge_mult(&urf2, cube, &mut temp);
    CubieCube::edge_mult(&temp, &urf1, cube);
}

fn init_sym2raw(
    n_raw: usize,
    sym2raw: &mut [u16],
    raw2sym: &mut [u8],
    sym_state: &mut [u16],
    flip_s2rf: Option<&mut [u16]>,
    coord: i32,
    tables: &CubieCubeTables,
) -> usize {
    let n_raw_half = n_raw.div_ceil(2);
    let sym_inc: usize = if coord >= 2 { 1 } else { 2 };
    let is_edge = coord != 1;

    let mut count = 0usize;
    for i in 0..n_raw {
        if coord_cube::get_pruning_from_byte(raw2sym, i) != 0 {
            continue;
        }
        let mut c = CubieCube::new();
        match coord {
            0 => c.set_flip(i as i32),
            1 => c.set_twist(i as i32),
            2 => c.set_e_perm(i as i32),
            _ => {}
        }
        for s in (0..16).step_by(sym_inc) {
            let mut d = CubieCube::new();
            if is_edge {
                edge_conjugate_raw(&c, s, &tables.cube_sym, &tables.sym_mult_inv, &mut d);
            } else {
                corn_conjugate_raw(&c, s, &tables.cube_sym, &tables.sym_mult_inv, &mut d);
            }
            let idx = match coord {
                0 => d.get_flip(),
                1 => d.get_twist(),
                2 => d.get_e_perm(),
                _ => 0,
            } as usize;

            if coord == 0 {
                if let Some(_f) = flip_s2rf.as_deref() {
                    // Can't use this pattern with Option<&mut [u16]>
                }
            }

            if idx == i {
                sym_state[count] |= 1 << (s / sym_inc);
            }
            let sym_idx = (count << 4 | s) / sym_inc;
            if coord_cube::get_pruning_from_byte(raw2sym, idx) == 0 {
                coord_cube::set_pruning_in_byte(raw2sym, idx, (sym_idx & 0xf) as u8);
                if coord != 2 {
                    raw2sym[idx + n_raw_half] = (sym_idx >> 4) as u8;
                }
            }
        }
        sym2raw[count] = i as u16;
        count += 1;
    }
    count
}

fn init_flip_sym2raw(tables: &mut CubieCubeTables) {
    // Need to handle flip_s2rf separately
    let n_raw = coord_cube::N_FLIP;
    let n_raw_half = n_raw.div_ceil(2);
    let sym_inc: usize = 2;

    let mut count = 0usize;
    for i in 0..n_raw {
        if coord_cube::get_pruning_from_byte(&tables.flip_r2s, i) != 0 {
            continue;
        }
        let mut c = CubieCube::new();
        c.set_flip(i as i32);
        for s in (0..16).step_by(sym_inc) {
            let mut d = CubieCube::new();
            edge_conjugate_raw(&c, s, &tables.cube_sym, &tables.sym_mult_inv, &mut d);
            let idx = d.get_flip() as usize;

            // FlipS2RF
            tables.flip_s2rf[count << 3 | s >> 1] = idx as u16;

            if idx == i {
                tables.sym_state_flip[count] |= 1 << (s / sym_inc);
            }
            let sym_idx = (count << 4 | s) / sym_inc;
            if coord_cube::get_pruning_from_byte(&tables.flip_r2s, idx) == 0 {
                coord_cube::set_pruning_in_byte(&mut tables.flip_r2s, idx, (sym_idx & 0xf) as u8);
                tables.flip_r2s[idx + n_raw_half] = (sym_idx >> 4) as u8;
            }
        }
        tables.flip_s2r[count] = i as u16;
        count += 1;
    }
}

fn init_twist_sym2raw(tables: &mut CubieCubeTables) {
    let n_raw = coord_cube::N_TWIST;
    let n_raw_half = n_raw.div_ceil(2);
    let sym_inc: usize = 2;

    let mut count = 0usize;
    for i in 0..n_raw {
        if coord_cube::get_pruning_from_byte(&tables.twist_r2s, i) != 0 {
            continue;
        }
        let mut c = CubieCube::new();
        c.set_twist(i as i32);
        for s in (0..16).step_by(sym_inc) {
            let mut d = CubieCube::new();
            corn_conjugate_raw(&c, s, &tables.cube_sym, &tables.sym_mult_inv, &mut d);
            let idx = d.get_twist() as usize;

            if idx == i {
                tables.sym_state_twist[count] |= 1 << (s / sym_inc);
            }
            let sym_idx = (count << 4 | s) / sym_inc;
            if coord_cube::get_pruning_from_byte(&tables.twist_r2s, idx) == 0 {
                coord_cube::set_pruning_in_byte(&mut tables.twist_r2s, idx, (sym_idx & 0xf) as u8);
                tables.twist_r2s[idx + n_raw_half] = (sym_idx >> 4) as u8;
            }
        }
        tables.twist_s2r[count] = i as u16;
        count += 1;
    }
}

fn init_perm_sym2raw(tables: &mut CubieCubeTables) {
    let n_raw = coord_cube::N_PERM;
    let _sym_inc: usize = 1;

    let mut count = 0usize;
    for i in 0..n_raw {
        if coord_cube::get_pruning_from_byte(&tables.e_perm_r2s, i) != 0 {
            continue;
        }
        let mut c = CubieCube::new();
        c.set_e_perm(i as i32);
        for s in 0..16 {
            let mut d = CubieCube::new();
            edge_conjugate_raw(&c, s, &tables.cube_sym, &tables.sym_mult_inv, &mut d);
            let idx = d.get_e_perm() as usize;

            if idx == i {
                tables.sym_state_perm[count] |= 1 << s;
            }
            let sym_idx = count << 4 | s;
            if coord_cube::get_pruning_from_byte(&tables.e_perm_r2s, idx) == 0 {
                coord_cube::set_pruning_in_byte(&mut tables.e_perm_r2s, idx, (sym_idx & 0xf) as u8);
                // For perm (coord==2), we don't store the high byte in raw2sym[idx + n_raw_half]
            }
        }
        tables.e_perm_s2r[count] = i as u16;
        count += 1;
    }

    // Perm2CombP and PermInvEdgeSym
    for i in 0..coord_cube::N_PERM_SYM {
        let mut cc = CubieCube::new();
        cc.set_e_perm(tables.e_perm_s2r[i] as i32);
        let comb = util::get_comb(&cc.ea, 0, true);
        let parity = util::get_n_parity(tables.e_perm_s2r[i] as i32, 8);
        // USE_COMBP_PRUN = true (matches Java USE_TWIST_FLIP_PRUN = true)
        tables.perm2_comb_p[i] = (comb + parity * 70) as u8;
        cc.inv_cubie_cube();
        // Need getEPermSym - but we can compute it inline here
        let raw = cc.get_e_perm();
        let k = coord_cube::get_pruning_from_byte(&tables.e_perm_r2s, raw as usize) as usize;
        let mut pc = CubieCube::new();
        edge_conjugate_raw(&cc, tables.sym_mult_inv[0][k], &tables.cube_sym, &tables.sym_mult_inv, &mut pc);
        let ep = pc.get_e_perm();
        let idx = tables.e_perm_s2r.binary_search(&(ep as u16)).unwrap();
        tables.perm_inv_edge_sym[i] = (idx << 4 | k) as u16;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cubie_cube_default() {
        let c = CubieCube::new();
        assert_eq!(c.ca, [0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(c.ea, [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22]);
    }

    #[test]
    fn test_identity_mult() {
        let a = CubieCube::new();
        let b = CubieCube::new();
        let mut prod = CubieCube::new();
        CubieCube::corn_mult(&a, &b, &mut prod);
        CubieCube::edge_mult(&a, &b, &mut prod);
        assert_eq!(prod.ca, a.ca);
        assert_eq!(prod.ea, a.ea);
    }

    #[test]
    fn test_verify_solved() {
        let c = CubieCube::new();
        assert_eq!(c.verify(), 0);
    }

    #[test]
    fn test_flip_roundtrip() {
        let mut c = CubieCube::new();
        c.set_flip(1234);
        assert_eq!(c.get_flip(), 1234);
    }

    #[test]
    fn test_twist_roundtrip() {
        let mut c = CubieCube::new();
        c.set_twist(1500);
        assert_eq!(c.get_twist(), 1500);
    }

    #[test]
    fn test_tables_init() {
        let tables = get_tables();
        // Check that move_cube[0] is not identity
        assert_ne!(tables.move_cube[0].ca, [0, 1, 2, 3, 4, 5, 6, 7]);
        // Check sym_mult identity: 0 * 0 = 0
        assert_eq!(tables.sym_mult[0][0], 0);
    }
}
