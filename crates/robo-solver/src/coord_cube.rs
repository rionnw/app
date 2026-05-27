//! CoordCube - Coordinate representation, move tables, and pruning tables.
//! Ported from CoordCube.java.

use std::sync::OnceLock;
use crate::cubie_cube::{self, CubieCube, CubieCubeTables};
use crate::util;

// ==================== Constants ====================

pub const N_MOVES: usize = 18;
pub const N_MOVES2: usize = 10;

pub const N_SLICE: usize = 495;
pub const N_TWIST: usize = 2187;
pub const N_TWIST_HALF: usize = N_TWIST.div_ceil(2);
pub const N_TWIST_SYM: usize = 324;
pub const N_FLIP: usize = 2048;
pub const N_FLIP_HALF: usize = N_FLIP.div_ceil(2);
pub const N_FLIP_SYM: usize = 336;
pub const N_PERM: usize = 40320;
pub const N_PERM_HALF: usize = N_PERM.div_ceil(2);
pub const N_PERM_SYM: usize = 2768;
pub const N_MPERM: usize = 24;
pub const N_COMB: usize = 140; // USE_COMBP_PRUN = true
pub const P2_PARITY_MOVE: i32 = 0xA5; // USE_COMBP_PRUN = true

// USE_TWIST_FLIP_PRUN = true
// USE_CONJ_PRUN = true
// USE_COMBP_PRUN = true

// ==================== Pruning helpers ====================

/// Get 4-bit pruning value from packed i32 array.
#[inline]
pub fn get_pruning(table: &[i32], index: usize) -> i32 {
    (table[index >> 3] >> ((index & 7) << 2)) & 0xf
}

/// Set 4-bit pruning value in packed i32 array (XOR-based).
#[inline]
pub fn set_pruning(table: &mut [i32], index: usize, value: i32) {
    table[index >> 3] ^= value << ((index & 7) << 2);
}

/// Get 4-bit pruning value from packed byte array.
#[inline]
pub fn get_pruning_from_byte(table: &[u8], index: usize) -> u8 {
    (table[index >> 1] >> ((index & 1) << 2)) & 0xf
}

/// Set 4-bit pruning value in packed byte array (XOR-based).
#[inline]
pub fn set_pruning_in_byte(table: &mut [u8], index: usize, value: u8) {
    table[index >> 1] ^= value << ((index & 1) << 2);
}

// ==================== Move and Pruning Tables ====================

pub struct CoordCubeTables {
    // Phase 1
    pub ud_slice_move: Vec<Vec<u16>>,      // [N_SLICE][N_MOVES]
    pub twist_move: Vec<Vec<u16>>,         // [N_TWIST_SYM][N_MOVES]
    pub flip_move: Vec<Vec<u16>>,          // [N_FLIP_SYM][N_MOVES]
    pub ud_slice_conj: Vec<Vec<u16>>,      // [N_SLICE][8]
    pub ud_slice_twist_prun: Vec<i32>,     // packed
    pub ud_slice_flip_prun: Vec<i32>,      // packed
    pub twist_flip_prun: Vec<i32>,         // packed (USE_TWIST_FLIP_PRUN)

    // Phase 2
    pub c_perm_move: Vec<Vec<u16>>,        // [N_PERM_SYM][N_MOVES2]
    pub e_perm_move: Vec<Vec<u16>>,        // [N_PERM_SYM][N_MOVES2]
    pub m_perm_move: Vec<Vec<u16>>,        // [N_MPERM][N_MOVES2]
    pub m_perm_conj: Vec<Vec<u16>>,        // [N_MPERM][16]
    pub c_comb_p_move: Vec<Vec<u16>>,      // [N_COMB][N_MOVES2]
    pub c_comb_p_conj: Vec<Vec<u16>>,      // [N_COMB][16]
    pub mc_perm_prun: Vec<i32>,            // packed
    pub e_perm_c_comb_p_prun: Vec<i32>,    // packed
}

static COORD_TABLES: OnceLock<CoordCubeTables> = OnceLock::new();

pub fn get_coord_tables() -> &'static CoordCubeTables {
    COORD_TABLES.get_or_init(init_coord_tables)
}

pub fn ensure_initialized() {
    let _ = get_coord_tables();
}

fn init_coord_tables() -> CoordCubeTables {
    // Ensure CubieCube tables are ready
    let ct = cubie_cube::get_tables();

    let mut tables = CoordCubeTables {
        ud_slice_move: vec![vec![0u16; N_MOVES]; N_SLICE],
        twist_move: vec![vec![0u16; N_MOVES]; N_TWIST_SYM],
        flip_move: vec![vec![0u16; N_MOVES]; N_FLIP_SYM],
        ud_slice_conj: vec![vec![0u16; 8]; N_SLICE],
        ud_slice_twist_prun: vec![0i32; N_SLICE * N_TWIST_SYM / 8 + 1],
        ud_slice_flip_prun: vec![0i32; N_SLICE * N_FLIP_SYM / 8 + 1],
        twist_flip_prun: vec![0i32; N_FLIP * N_TWIST_SYM / 8 + 1],
        c_perm_move: vec![vec![0u16; N_MOVES2]; N_PERM_SYM],
        e_perm_move: vec![vec![0u16; N_MOVES2]; N_PERM_SYM],
        m_perm_move: vec![vec![0u16; N_MOVES2]; N_MPERM],
        m_perm_conj: vec![vec![0u16; 16]; N_MPERM],
        c_comb_p_move: vec![vec![0u16; N_MOVES2]; N_COMB],
        c_comb_p_conj: vec![vec![0u16; 16]; N_COMB],
        mc_perm_prun: vec![0i32; N_MPERM * N_PERM_SYM / 8 + 1],
        e_perm_c_comb_p_prun: vec![0i32; N_COMB * N_PERM_SYM / 8 + 1],
    };

    // Init order matters (matches Java)
    init_c_perm_move(&mut tables, ct);
    init_e_perm_move(&mut tables, ct);
    init_m_perm_move_conj(&mut tables, ct);
    init_comb_p_move_conj(&mut tables, ct);
    init_flip_move(&mut tables, ct);
    init_twist_move(&mut tables, ct);
    init_ud_slice_move_conj(&mut tables, ct);

    // Pruning tables
    init_mc_perm_prun(&mut tables, ct);
    init_perm_comb_p_prun(&mut tables, ct);
    init_slice_twist_prun(&mut tables, ct);
    init_slice_flip_prun(&mut tables, ct);
    init_twist_flip_prun(&mut tables, ct);

    tables
}

fn init_ud_slice_move_conj(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_SLICE {
        let mut c = CubieCube::new();
        c.set_ud_slice(i as i32);
        for j in (0..N_MOVES).step_by(3) {
            let mut d = CubieCube::new();
            CubieCube::edge_mult(&c, &ct.move_cube[j], &mut d);
            tables.ud_slice_move[i][j] = d.get_ud_slice() as u16;
        }
        for j in (0..16).step_by(2) {
            let mut d = CubieCube::new();
            cubie_cube::edge_conjugate_raw(&c, ct.sym_mult_inv[0][j], &ct.cube_sym, &ct.sym_mult_inv, &mut d);
            tables.ud_slice_conj[i][j >> 1] = d.get_ud_slice() as u16;
        }
    }
    // Fill in x2, x3 moves
    for i in 0..N_SLICE {
        for j in (0..N_MOVES).step_by(3) {
            let mut udslice = tables.ud_slice_move[i][j] as usize;
            for k in 1..3 {
                udslice = tables.ud_slice_move[udslice][j] as usize;
                tables.ud_slice_move[i][j + k] = udslice as u16;
            }
        }
    }
}

fn init_flip_move(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_FLIP_SYM {
        let mut c = CubieCube::new();
        c.set_flip(ct.flip_s2r[i] as i32);
        for j in 0..N_MOVES {
            let mut d = CubieCube::new();
            CubieCube::edge_mult(&c, &ct.move_cube[j], &mut d);
            tables.flip_move[i][j] = d.get_flip_sym() as u16;
        }
    }
}

fn init_twist_move(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_TWIST_SYM {
        let mut c = CubieCube::new();
        c.set_twist(ct.twist_s2r[i] as i32);
        for j in 0..N_MOVES {
            let mut d = CubieCube::new();
            CubieCube::corn_mult(&c, &ct.move_cube[j], &mut d);
            tables.twist_move[i][j] = d.get_twist_sym() as u16;
        }
    }
}

fn init_c_perm_move(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_PERM_SYM {
        let mut c = CubieCube::new();
        c.set_c_perm(ct.e_perm_s2r[i] as i32);
        for j in 0..N_MOVES2 {
            let mut d = CubieCube::new();
            CubieCube::corn_mult(&c, &ct.move_cube[util::UD2STD[j] as usize], &mut d);
            tables.c_perm_move[i][j] = d.get_c_perm_sym() as u16;
        }
    }
}

fn init_e_perm_move(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_PERM_SYM {
        let mut c = CubieCube::new();
        c.set_e_perm(ct.e_perm_s2r[i] as i32);
        for j in 0..N_MOVES2 {
            let mut d = CubieCube::new();
            CubieCube::edge_mult(&c, &ct.move_cube[util::UD2STD[j] as usize], &mut d);
            tables.e_perm_move[i][j] = d.get_e_perm_sym() as u16;
        }
    }
}

fn init_m_perm_move_conj(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_MPERM {
        let mut c = CubieCube::new();
        c.set_m_perm(i as i32);
        for j in 0..N_MOVES2 {
            let mut d = CubieCube::new();
            CubieCube::edge_mult(&c, &ct.move_cube[util::UD2STD[j] as usize], &mut d);
            tables.m_perm_move[i][j] = d.get_m_perm() as u16;
        }
        for j in 0..16 {
            let mut d = CubieCube::new();
            cubie_cube::edge_conjugate_raw(&c, ct.sym_mult_inv[0][j], &ct.cube_sym, &ct.sym_mult_inv, &mut d);
            tables.m_perm_conj[i][j] = d.get_m_perm() as u16;
        }
    }
}

fn init_comb_p_move_conj(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    for i in 0..N_COMB {
        let mut c = CubieCube::new();
        c.set_c_comb((i % 70) as i32);
        for j in 0..N_MOVES2 {
            let mut d = CubieCube::new();
            CubieCube::corn_mult(&c, &ct.move_cube[util::UD2STD[j] as usize], &mut d);
            let comb = d.get_c_comb();
            tables.c_comb_p_move[i][j] = (comb + 70 * (((P2_PARITY_MOVE >> j) & 1) ^ (i as i32 / 70))) as u16;
        }
        for j in 0..16 {
            let mut d = CubieCube::new();
            cubie_cube::corn_conjugate_raw(&c, ct.sym_mult_inv[0][j], &ct.cube_sym, &ct.sym_mult_inv, &mut d);
            tables.c_comb_p_conj[i][j] = (d.get_c_comb() + 70 * (i as i32 / 70)) as u16;
        }
    }
}

// ==================== Pruning Table Initialization ====================

fn has_zero(val: i32) -> bool {
    ((val.wrapping_sub(0x11111111)) & !val & (0x88888888u32 as i32)) != 0
}

/// Generic pruning table initialization using BFS with symmetry reduction.
fn init_raw_sym_prun(
    prun_table: &mut [i32],
    raw_move: Option<&Vec<Vec<u16>>>,
    raw_conj: Option<&Vec<Vec<u16>>>,
    sym_move: &[Vec<u16>],
    sym_state: &[u16],
    flip_s2rf: Option<&[u16]>,
    flip_move: Option<&Vec<Vec<u16>>>,
    sym8_move: &[i32],
    prun_flag: i32,
) {
    let sym_shift = (prun_flag & 0xf) as u32;
    let sym_e2c_magic: i32 = if ((prun_flag >> 4) & 1) == 1 { cubie_cube::SYM_E2C_MAGIC } else { 0 };
    let is_phase2 = ((prun_flag >> 5) & 1) == 1;
    let inv_depth = (prun_flag >> 8) & 0xf;
    let max_depth = (prun_flag >> 12) & 0xf;
    let _min_depth = (prun_flag >> 16) & 0xf;

    let sym_mask = (1i32 << sym_shift) - 1;
    let is_tfp = raw_move.is_none();
    let n_raw = if is_tfp { N_FLIP } else { raw_move.unwrap().len() };
    let n_size = n_raw * sym_move.len();
    let n_moves_local = if is_phase2 { 10 } else { 18 };
    let next_axis_magic: i32 = if n_moves_local == 10 { 0x42 } else { 0x92492 };

    // Check if already partially initialized
    let depth_start = get_pruning(prun_table, n_size) - 1;

    if depth_start == -1 {
        // First time init
        prun_table[..n_size / 8 + 1].fill(0x11111111);
        set_pruning(prun_table, 0, 1);
    }

    let mut depth = if depth_start == -1 { 0 } else { depth_start };

    while depth < max_depth {
        let mask = ((depth + 1) as u32).wrapping_mul(0x11111111) ^ 0xffffffff;
        let mask = mask as i32;
        for slot in prun_table.iter_mut() {
            let val = *slot ^ mask;
            let v2 = val & (val >> 1);
            *slot = slot.wrapping_add(v2 & (v2 >> 2) & 0x11111111);
        }

        let inv = depth > inv_depth;
        let select = if inv { depth + 2 } else { depth };
        let sel_arr_mask = (select as u32).wrapping_mul(0x11111111) as i32;
        let check = if inv { depth } else { depth + 2 };
        depth += 1;
        let xor_val = depth ^ (depth + 1);
        let mut val = 0i32;

        let mut i = 0usize;
        while i < n_size {
            if (i & 7) == 0 {
                val = prun_table[i >> 3];
                if !has_zero(val ^ sel_arr_mask) {
                    i += 8;
                    continue;
                }
            }
            if (val & 0xf) != select {
                val >>= 4;
                i += 1;
                continue;
            }
            let raw = i % n_raw;
            let sym = i / n_raw;

            let mut flip = 0i32;
            let mut fsym = 0i32;
            if is_tfp {
                let fr = cubie_cube::flip_raw2sym(raw as i32);
                fsym = fr & 7;
                flip = fr >> 3;
            }

            let mut m: i32 = 0;
            while m < n_moves_local {
                let symx_full = sym_move[sym][m as usize] as i32;
                let rawx: i32 = if is_tfp {
                    let fm = flip_move.unwrap()[flip as usize][sym8_move[(m as usize) << 3 | fsym as usize] as usize] as i32;
                    let fidx = fm ^ fsym ^ (symx_full & sym_mask);
                    flip_s2rf.unwrap()[fidx as usize] as i32
                } else {
                    let rm = raw_move.unwrap()[raw][m as usize] as usize;
                    raw_conj.unwrap()[rm][(symx_full & sym_mask) as usize] as i32
                };

                let symx = symx_full >> sym_shift;
                let idx = symx as usize * n_raw + rawx as usize;
                let prun = get_pruning(prun_table, idx);
                if prun != check {
                    if prun < depth - 1 {
                        m += (next_axis_magic >> m) & 3;
                    }
                    m += 1;
                    continue;
                }
                if inv {
                    set_pruning(prun_table, i, xor_val);
                    break;
                }
                set_pruning(prun_table, idx, xor_val);

                // Propagate to symmetric positions
                let mut sym_state_val = sym_state[symx as usize] as i32;
                let mut j = 1;
                sym_state_val >>= 1;
                while sym_state_val != 0 {
                    if (sym_state_val & 1) == 1 {
                        let idxx: usize = if is_tfp {
                            let fr = cubie_cube::flip_raw2sym(rawx);
                            symx as usize * n_raw + flip_s2rf.unwrap()[(fr ^ j) as usize] as usize
                        } else {
                            let conj_idx = (j ^ ((sym_e2c_magic >> (j << 1)) & 3)) as usize;
                            symx as usize * n_raw + raw_conj.unwrap()[rawx as usize][conj_idx] as usize
                        };
                        if get_pruning(prun_table, idxx) == check {
                            set_pruning(prun_table, idxx, xor_val);
                        }
                    }
                    j += 1;
                    sym_state_val >>= 1;
                }
                m += 1;
            }
            val >>= 4;
            i += 1;
        }
    }
    set_pruning(prun_table, n_size, (depth + 1) ^ 1);
}

fn init_mc_perm_prun(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    init_raw_sym_prun(
        &mut tables.mc_perm_prun,
        Some(&tables.m_perm_move),
        Some(&tables.m_perm_conj),
        &tables.c_perm_move,
        &ct.sym_state_perm,
        None, None,
        &ct.sym8_move,
        0x8ea34,
    );
}

fn init_perm_comb_p_prun(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    init_raw_sym_prun(
        &mut tables.e_perm_c_comb_p_prun,
        Some(&tables.c_comb_p_move),
        Some(&tables.c_comb_p_conj),
        &tables.e_perm_move,
        &ct.sym_state_perm,
        None, None,
        &ct.sym8_move,
        0x7d824,
    );
}

fn init_slice_twist_prun(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    init_raw_sym_prun(
        &mut tables.ud_slice_twist_prun,
        Some(&tables.ud_slice_move),
        Some(&tables.ud_slice_conj),
        &tables.twist_move,
        &ct.sym_state_twist,
        None, None,
        &ct.sym8_move,
        0x69603,
    );
}

fn init_slice_flip_prun(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    init_raw_sym_prun(
        &mut tables.ud_slice_flip_prun,
        Some(&tables.ud_slice_move),
        Some(&tables.ud_slice_conj),
        &tables.flip_move,
        &ct.sym_state_flip,
        None, None,
        &ct.sym8_move,
        0x69603,
    );
}

fn init_twist_flip_prun(tables: &mut CoordCubeTables, ct: &CubieCubeTables) {
    init_raw_sym_prun(
        &mut tables.twist_flip_prun,
        None,
        None,
        &tables.twist_move,
        &ct.sym_state_twist,
        Some(&ct.flip_s2rf),
        Some(&tables.flip_move),
        &ct.sym8_move,
        0x19603,
    );
}

// ==================== CoordCube struct (search node) ====================

#[derive(Clone, Copy, Default)]
pub struct CoordCubeNode {
    pub twist: i32,
    pub tsym: i32,
    pub flip: i32,
    pub fsym: i32,
    pub slice: i32,
    pub prun: i32,
    pub twistc: i32,
    pub flipc: i32,
}

impl CoordCubeNode {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&mut self, other: &CoordCubeNode) {
        self.twist = other.twist;
        self.tsym = other.tsym;
        self.flip = other.flip;
        self.fsym = other.fsym;
        self.slice = other.slice;
        self.prun = other.prun;
        self.twistc = other.twistc;
        self.flipc = other.flipc;
    }

    pub fn calc_pruning(&mut self, _is_phase1: bool) {
        let coord = get_coord_tables();
        let ct = cubie_cube::get_tables();
        self.prun = get_pruning(&coord.ud_slice_twist_prun,
            (self.twist as usize) * N_SLICE + coord.ud_slice_conj[self.slice as usize][self.tsym as usize] as usize)
            .max(get_pruning(&coord.ud_slice_flip_prun,
                (self.flip as usize) * N_SLICE + coord.ud_slice_conj[self.slice as usize][self.fsym as usize] as usize))
            .max(get_pruning(&coord.twist_flip_prun,
                ((self.twistc >> 3) as usize) << 11 | ct.flip_s2rf[(self.flipc ^ (self.twistc & 7)) as usize] as usize))
            .max(get_pruning(&coord.twist_flip_prun,
                (self.twist as usize) << 11 | ct.flip_s2rf[((self.flip << 3) | (self.fsym ^ self.tsym)) as usize] as usize));
    }

    pub fn set_with_prun(&mut self, cc: &CubieCube, depth: i32) -> bool {
        let coord = get_coord_tables();
        let ct = cubie_cube::get_tables();

        let twist_sym = cc.get_twist_sym();
        self.flip = cc.get_flip_sym();
        self.tsym = twist_sym & 7;
        self.twist = twist_sym >> 3;

        self.prun = get_pruning(&coord.twist_flip_prun,
            (self.twist as usize) << 11 | ct.flip_s2rf[(self.flip ^ self.tsym) as usize] as usize);
        if self.prun > depth {
            return false;
        }

        self.fsym = self.flip & 7;
        self.flip >>= 3;
        self.slice = cc.get_ud_slice();

        self.prun = self.prun
            .max(get_pruning(&coord.ud_slice_twist_prun,
                (self.twist as usize) * N_SLICE + coord.ud_slice_conj[self.slice as usize][self.tsym as usize] as usize))
            .max(get_pruning(&coord.ud_slice_flip_prun,
                (self.flip as usize) * N_SLICE + coord.ud_slice_conj[self.slice as usize][self.fsym as usize] as usize));
        if self.prun > depth {
            return false;
        }

        // Conjugate pruning (USE_CONJ_PRUN = true)
        let mut pc = CubieCube::new();
        CubieCube::corn_conjugate(cc, 1, &mut pc);
        CubieCube::edge_conjugate(cc, 1, &mut pc);
        self.twistc = pc.get_twist_sym();
        self.flipc = pc.get_flip_sym();
        self.prun = self.prun.max(get_pruning(&coord.twist_flip_prun,
            ((self.twistc >> 3) as usize) << 11 | ct.flip_s2rf[(self.flipc ^ (self.twistc & 7)) as usize] as usize));

        self.prun <= depth
    }

    /// Perform a move and compute pruning value.
    pub fn do_move_prun(&mut self, cc: &CoordCubeNode, m: usize, _is_phase1: bool) -> i32 {
        let coord = get_coord_tables();
        let ct = cubie_cube::get_tables();

        self.slice = coord.ud_slice_move[cc.slice as usize][m] as i32;

        let flip_val = coord.flip_move[cc.flip as usize][ct.sym8_move[m << 3 | cc.fsym as usize] as usize] as i32;
        self.fsym = (flip_val & 7) ^ cc.fsym;
        self.flip = flip_val >> 3;

        let twist_val = coord.twist_move[cc.twist as usize][ct.sym8_move[m << 3 | cc.tsym as usize] as usize] as i32;
        self.tsym = (twist_val & 7) ^ cc.tsym;
        self.twist = twist_val >> 3;

        self.prun = get_pruning(&coord.ud_slice_twist_prun,
            (self.twist as usize) * N_SLICE + coord.ud_slice_conj[self.slice as usize][self.tsym as usize] as usize)
            .max(get_pruning(&coord.ud_slice_flip_prun,
                (self.flip as usize) * N_SLICE + coord.ud_slice_conj[self.slice as usize][self.fsym as usize] as usize))
            .max(get_pruning(&coord.twist_flip_prun,
                (self.twist as usize) << 11 | ct.flip_s2rf[((self.flip << 3) | (self.fsym ^ self.tsym)) as usize] as usize));

        self.prun
    }

    /// Perform a move on conjugate coordinates and compute pruning.
    pub fn do_move_prun_conj(&mut self, cc: &CoordCubeNode, m: usize) -> i32 {
        let coord = get_coord_tables();
        let ct = cubie_cube::get_tables();

        let m2 = ct.sym_move[3][m] as usize;
        let fc = coord.flip_move[(cc.flipc >> 3) as usize][ct.sym8_move[m2 << 3 | (cc.flipc & 7) as usize] as usize] as i32;
        self.flipc = fc ^ (cc.flipc & 7);
        let tc = coord.twist_move[(cc.twistc >> 3) as usize][ct.sym8_move[m2 << 3 | (cc.twistc & 7) as usize] as usize] as i32;
        self.twistc = tc ^ (cc.twistc & 7);

        get_pruning(&coord.twist_flip_prun,
            ((self.twistc >> 3) as usize) << 11 | ct.flip_s2rf[(self.flipc ^ (self.twistc & 7)) as usize] as usize)
    }
}
