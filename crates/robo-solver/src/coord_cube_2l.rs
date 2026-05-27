//! CoordCube2L - 2L coordinate tables, move tables, and pruning tables.
//! Ported from CoordCube2L.java.

use std::sync::{LazyLock, Mutex, OnceLock};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::io::{BufReader, BufWriter, Read, Write};
use std::fs::File;
use std::path::{Path, PathBuf};

use crate::cubie_cube::{self, CubieCube};
use crate::cubie_cube_2l::CubieCube2L;
use crate::util;

/// Data directory for 2L pruning tables.
/// Priority: `MIN2PHASE_2L_DATA_DIR` env var > `<exe_dir>/2l_data` > `./2l_data`.
/// Re-settable at runtime via [`set_data_dir`].
static DATA_DIR: LazyLock<Mutex<PathBuf>> = LazyLock::new(|| {
    let dir = std::env::var("MIN2PHASE_2L_DATA_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            // Try next to the executable first
            if let Ok(exe) = std::env::current_exe() {
                let candidate = exe.parent().unwrap_or(Path::new(".")).join("2l_data");
                if candidate.is_dir() {
                    return candidate;
                }
            }
            PathBuf::from("2l_data")
        });
    Mutex::new(dir)
});

/// Whether to generate / load the giant TwistSliceF8Prun (~2.5 GB).
/// On by default to match Java's `CoordCube2L.calcPruning`, which always reads
/// `TwistSliceF8Prun`. Set `MIN2PHASE_2L_USE_F8=0` (or call
/// [`set_use_slice_f8(false)`]) to skip loading the (~2.5GB) F8 table, at the
/// cost of weaker pruning and significantly worse solver cost on hard cubes.
///
/// Internally tracked via an `AtomicBool` + an "explicitly set" flag so the
/// env-var default still applies when no explicit `set_use_slice_f8` call was made.
static USE_SLICE_F8: AtomicBool = AtomicBool::new(true);
static USE_SLICE_F8_SET: AtomicBool = AtomicBool::new(false);

pub fn set_data_dir<P: Into<PathBuf>>(path: P) {
    *DATA_DIR.lock().unwrap() = path.into();
}

pub fn set_use_slice_f8(v: bool) {
    USE_SLICE_F8.store(v, Ordering::Relaxed);
    USE_SLICE_F8_SET.store(true, Ordering::Relaxed);
}

fn data_dir() -> PathBuf {
    DATA_DIR.lock().unwrap().clone()
}

fn use_slice_f8() -> bool {
    if USE_SLICE_F8_SET.load(Ordering::Relaxed) {
        return USE_SLICE_F8.load(Ordering::Relaxed);
    }
    // Default: ON, matching Java which always uses TwistSliceF8Prun.
    // Override via env var: MIN2PHASE_2L_USE_F8=0 disables it.
    std::env::var("MIN2PHASE_2L_USE_F8").map(|v| v != "0").unwrap_or(true)
}

pub const N_CUBE_MOVE: usize = 18;
pub const N_CUBE_MOVE2: usize = 10;
pub const N_EFF_MOVE: usize = 6;
pub const N_LEG_MOVES: usize = 20;

// Leg move to cube move mapping (-1 = no cube move)
pub static M_ON_CUBE: [i32; 20] = [0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1];
// Leg move to center move mapping (-1 = no center move)
pub static M_ON_CT: [i32; 20] = [-1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5, -1, -1, 0, 1, 2, 3, 4, 5];

// Leg state machine: NextState[leg][move] -> next_leg (-1 = invalid)
pub static NEXT_STATE: [[i32; 20]; 3] = [
    [ 1,  0,  1,  2,  0,  2,  1,  0,  1,  2,  0,  2,  2,  1, -1,  2, -1, -1,  1, -1],  // pp
    [ 0,  1,  0, -1, -1, -1,  0,  1,  0, -1,  1, -1, -1,  0,  2, -1,  2,  2,  0,  2],  // vp
    [-1, -1, -1,  0,  2,  0, -1,  2, -1,  0,  2,  0,  0, -1,  1,  0,  1,  1, -1,  1],  // pv
];

// Move cost per leg state
pub static M_COST: [[i32; 20]; 3] = [
    [1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4],
    [1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4],
    [1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4],
];

pub static MOVE_COST: [[i32; 20]; 3] = [
    [51, 87, 51, 51, 87, 51, 211, 284, 211, 211, 284, 211, 138, 138, 249, 322, 249, 249, 322, 249],
    [51, 87, 51, 51, 87, 51, 211, 284, 211, 211, 360, 211, 138, 138, 249, 322, 249, 249, 322, 249],
    [51, 87, 51, 51, 87, 51, 211, 360, 211, 211, 284, 211, 138, 138, 249, 322, 249, 249, 322, 249],
];

pub static RELEASED_LEGS: [i32; 21] = [
    0x0004, 0x0004, 0x0004, 0x0008, 0x0008, 0x0008,
    0x0001, 0x00c1, 0x0301, 0x0002, 0x0112, 0x0062,
    0x0001, 0x0002, 0x0011, 0x0021, 0x0001, 0x0002, 0x0202, 0x0082,
    0x0000,
];

pub static PARALLEL_MOVES: [i32; 21] = [
    0x010000, 0x010000, 0x010000, 0x020000, 0x020000, 0x020000,
    0x010000, 0x310000, 0x010000, 0x020000, 0x320000, 0x020000,
    0x020000, 0x010000, 0x000000, 0x100000, 0x000000, 0x000000, 0x200000, 0x000000,
    0x000000,
];

pub static CT_STD_CONJ: [i32; 24] = [
    0, 0, 0, 3, 3, 3, 2, 2, 2, 1, 1, 1, 10, 10, 10, 11, 11, 11, 8, 8, 8, 9, 9, 9,
];

/// 2L Coordinate tables (lazily initialized).
pub struct CoordCube2LTables {
    pub move_conj: Vec<Vec<i32>>,           // [24][N_EFF_MOVE]
    pub ct_move: Vec<Vec<i32>>,             // [24][N_EFF_MOVE]

    // Phase 1 raw move tables
    pub flip_move: Vec<Vec<i32>>,           // [2048][18]
    pub twist_move: Vec<Vec<i32>>,          // [2187][18]
    pub ud_slice_move: Vec<Vec<i32>>,       // [495][18]
    pub flip_conj: Vec<Vec<i32>>,           // [2048][16]
    pub twist_conj: Vec<Vec<i32>>,          // [2187][16]
    pub ud_slice_conj: Vec<Vec<i32>>,       // [495][16]
    pub flip_uds_move: Vec<Vec<i32>>,       // [495*2048][18]
    pub flip_uds_conj: Vec<Vec<i32>>,       // [495*2048][16]
    pub flip_uds2_slice_f4: Vec<i32>,       // [495*2048]
    pub flip_uds2_slice_f8: Vec<i32>,       // [495*2048]
    pub flip_conj_xor: Vec<i32>,            // [495]
    pub slice_f4_move: Vec<Vec<i32>>,       // [7920][18]
    pub slice_f4_conj: Vec<Vec<i32>>,       // [7920][16]
    pub slice_f8_move: Vec<Vec<i32>>,       // [126720][18]
    pub slice_f8_conj: Vec<Vec<i32>>,       // [126720][16]

    // Phase 2 raw move tables
    pub e_perm_move: Vec<Vec<i32>>,         // [40320][10]
    pub c_perm_move: Vec<Vec<i32>>,         // [40320][10]
    pub m_perm_move: Vec<Vec<i32>>,         // [24][10]
    pub c_comb_move: Vec<Vec<i32>>,         // [70][10]
    pub mpc_cb_move: Vec<Vec<i32>>,         // [1680][10]
    pub c_perm2_c_comb: Vec<i32>,           // [40320]
    pub e_perm_conj: Vec<Vec<i32>>,         // [40320][16]
    pub c_perm_conj: Vec<Vec<i32>>,         // [40320][16]
    pub m_perm_conj: Vec<Vec<i32>>,         // [24][16]
    pub c_comb_conj: Vec<Vec<i32>>,         // [70][16]
    pub mpc_cb_conj: Vec<Vec<i32>>,         // [1680][16]

    // Pruning tables (loaded from file or generated)
    pub twist_slice_f4_prun: Option<Vec<Vec<u8>>>,    // [2187][7920*9]
    pub twist_slice_f8_prun: Option<Vec<Vec<u8>>>,    // [2187][126720*9]
    pub flip_uds_prun: Option<Vec<Vec<u8>>>,          // [1][495*2048*9]
    pub e_perm_mpc_cb_prun: Option<Vec<Vec<u8>>>,     // [40320][1680*9]
    pub c_perm_m_perm_prun: Option<Vec<Vec<u8>>>,     // [40320][24*9]
}

static COORD_2L_TABLES: OnceLock<CoordCube2LTables> = OnceLock::new();

pub fn get_tables() -> &'static CoordCube2LTables {
    COORD_2L_TABLES.get_or_init(init_coord_2l_tables)
}

pub fn ensure_initialized() {
    let _ = get_tables();
}

fn init_coord_2l_tables() -> CoordCube2LTables {
    // Ensure base tables are ready
    cubie_cube::ensure_tables_initialized();
    let ct = cubie_cube::get_tables();
    let ct_idx2val = crate::cubie_cube_2l::get_ct_idx2val();

    let mut tables = CoordCube2LTables {
        move_conj: vec![vec![0; N_EFF_MOVE]; 24],
        ct_move: vec![vec![0; N_EFF_MOVE]; 24],
        flip_move: vec![vec![0; N_CUBE_MOVE]; 2048],
        twist_move: vec![vec![0; N_CUBE_MOVE]; 2187],
        ud_slice_move: vec![vec![0; N_CUBE_MOVE]; 495],
        flip_conj: vec![vec![0; 16]; 2048],
        twist_conj: vec![vec![0; 16]; 2187],
        ud_slice_conj: vec![vec![0; 16]; 495],
        flip_uds_move: vec![vec![0; N_CUBE_MOVE]; 495 * 2048],
        flip_uds_conj: vec![vec![0; 16]; 495 * 2048],
        flip_uds2_slice_f4: vec![0; 495 * 2048],
        flip_uds2_slice_f8: vec![0; 495 * 2048],
        flip_conj_xor: vec![0; 495],
        slice_f4_move: vec![vec![0; N_CUBE_MOVE]; 7920],
        slice_f4_conj: vec![vec![0; 16]; 7920],
        slice_f8_move: vec![vec![0; N_CUBE_MOVE]; 126720],
        slice_f8_conj: vec![vec![0; 16]; 126720],
        e_perm_move: vec![vec![0; N_CUBE_MOVE2]; 40320],
        c_perm_move: vec![vec![0; N_CUBE_MOVE2]; 40320],
        m_perm_move: vec![vec![0; N_CUBE_MOVE2]; 24],
        c_comb_move: vec![vec![0; N_CUBE_MOVE2]; 70],
        mpc_cb_move: vec![vec![0; N_CUBE_MOVE2]; 1680],
        c_perm2_c_comb: vec![0; 40320],
        e_perm_conj: vec![vec![0; 16]; 40320],
        c_perm_conj: vec![vec![0; 16]; 40320],
        m_perm_conj: vec![vec![0; 16]; 24],
        c_comb_conj: vec![vec![0; 16]; 70],
        mpc_cb_conj: vec![vec![0; 16]; 1680],
        twist_slice_f4_prun: None,
        twist_slice_f8_prun: None,
        flip_uds_prun: None,
        e_perm_mpc_cb_prun: None,
        c_perm_m_perm_prun: None,
    };

    init_center_move(&mut tables, ct_idx2val);
    init_phase1_move(&mut tables, ct);
    init_phase2_move(&mut tables, ct);

    // Ensure data dir exists
    let dir = data_dir();
    if !dir.exists() {
        let _ = std::fs::create_dir_all(&dir);
    }

    // Load or generate pruning tables (stored separately in `2l_data/`).
    eprintln!("[2L] data dir: {}", dir.display());

    // Fast path: if every required prun file is already on disk with the
    // expected size, load them all in parallel. Each load is bound by
    // memcpy throughput from page cache (~10 GiB/s), so running them on
    // independent threads roughly halves wall-clock time on multi-core
    // machines (the 2.5 GiB F8 table sets the lower bound, ~210 ms).
    let want_f8 = use_slice_f8();
    let specs: Vec<PrunSpec> = build_prun_specs(&tables, want_f8);
    if specs.iter().all(|s| prun_file_ok(&dir.join(s.filename), s.cord1_size, s.cord2_size)) {
        let dir_ref = &dir;
        let loaded: Vec<(String, Option<Vec<Vec<u8>>>)> = std::thread::scope(|scope| {
            let handles: Vec<_> = specs
                .iter()
                .map(|spec| {
                    let name = spec.filename.to_string();
                    let cord1 = spec.cord1_size;
                    let cord2 = spec.cord2_size;
                    scope.spawn(move || {
                        let path = dir_ref.join(&name);
                        let t = std::time::Instant::now();
                        let data = load_prun_from_file(&path, cord1, cord2);
                        if data.is_some() {
                            let dt = t.elapsed();
                            let bytes = (cord1 as u64) * (cord2 as u64) * 9;
                            let mibs = (bytes as f64 / (1024.0 * 1024.0)) / dt.as_secs_f64().max(1e-9);
                            eprintln!(
                                "[2L] loaded {} ({}x{}) in {:.3}s ({:.0} MiB/s) [par]",
                                name, cord1, cord2 * 9, dt.as_secs_f64(), mibs
                            );
                        }
                        (name, data)
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });
        for (name, data) in loaded {
            match name.as_str() {
                "TwistSliceF4Prun.data" => tables.twist_slice_f4_prun = data,
                "FlipUDSPrun.data"      => tables.flip_uds_prun = data,
                "EPermMPCCbPrun.data"   => tables.e_perm_mpc_cb_prun = data,
                "CPermMPermPrun.data"   => tables.c_perm_m_perm_prun = data,
                "TwistSliceF8Prun.data" => tables.twist_slice_f8_prun = data,
                _ => {}
            }
        }
        return tables;
    }

    // Slow path: at least one table is missing or stale. Fall back to
    // sequential load-or-generate, which the generators depend on (they
    // borrow `&tables` mutably across calls).
    tables.twist_slice_f4_prun = load_or_generate_prun(
        &dir, "TwistSliceF4Prun.data",
        &tables.twist_move, &tables.slice_f4_move,
        &tables.twist_conj, &tables.slice_f4_conj,
        &tables, false,
    );

    tables.flip_uds_prun = load_or_generate_prun_single_empty(
        &dir, "FlipUDSPrun.data",
        &tables.flip_uds_move, &tables.flip_uds_conj,
        &tables, false,
    );

    tables.e_perm_mpc_cb_prun = load_or_generate_prun(
        &dir, "EPermMPCCbPrun.data",
        &tables.e_perm_move, &tables.mpc_cb_move,
        &tables.e_perm_conj, &tables.mpc_cb_conj,
        &tables, true,
    );

    tables.c_perm_m_perm_prun = load_or_generate_prun(
        &dir, "CPermMPermPrun.data",
        &tables.c_perm_move, &tables.m_perm_move,
        &tables.c_perm_conj, &tables.m_perm_conj,
        &tables, true,
    );

    if want_f8 {
        tables.twist_slice_f8_prun = load_or_generate_prun(
            &dir, "TwistSliceF8Prun.data",
            &tables.twist_move, &tables.slice_f8_move,
            &tables.twist_conj, &tables.slice_f8_conj,
            &tables, false,
        );
    }

    tables
}

/// Metadata describing one pruning table on disk: just the geometry we need
/// to (a) validate the file's size and (b) parse it into rows. The actual
/// move/conj tables are still required for generation but the parallel
/// fast-path only consumes shape info.
struct PrunSpec {
    filename: &'static str,
    cord1_size: usize,
    cord2_size: usize,
}

fn build_prun_specs(t: &CoordCube2LTables, want_f8: bool) -> Vec<PrunSpec> {
    let mut v = vec![
        PrunSpec { filename: "TwistSliceF4Prun.data", cord1_size: t.twist_move.len(),  cord2_size: t.slice_f4_move.len() },
        PrunSpec { filename: "FlipUDSPrun.data",      cord1_size: 1,                   cord2_size: t.flip_uds_move.len() },
        PrunSpec { filename: "EPermMPCCbPrun.data",   cord1_size: t.e_perm_move.len(), cord2_size: t.mpc_cb_move.len()   },
        PrunSpec { filename: "CPermMPermPrun.data",   cord1_size: t.c_perm_move.len(), cord2_size: t.m_perm_move.len()   },
    ];
    if want_f8 {
        v.push(PrunSpec { filename: "TwistSliceF8Prun.data", cord1_size: t.twist_move.len(), cord2_size: t.slice_f8_move.len() });
    }
    v
}

fn prun_file_ok(path: &Path, cord1_size: usize, cord2_size: usize) -> bool {
    let expected = match (cord1_size as u64).checked_mul(cord2_size as u64).and_then(|x| x.checked_mul(9)) {
        Some(v) => v,
        None => return false,
    };
    match std::fs::metadata(path) {
        Ok(m) => m.len() == expected,
        Err(_) => false,
    }
}

fn init_center_move(tables: &mut CoordCube2LTables, ct_idx2val: &[i32; 24]) {
    // CtMove
    for i in 0..24 {
        let mut cc = CubieCube2L::new();
        cc.set_ct_idx(i as i32);
        for m in 0..N_EFF_MOVE {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to((m as i32) + 18, &mut cc2);
            tables.ct_move[i][m] = cc2.get_ct_idx();
        }
    }

    // MoveConj
    for idx in 0..24 {
        let ct = ct_idx2val[idx];
        for m in 0..6 {
            let axis = m / 3;
            let pow = m % 3;
            let real_axis = (ct >> (axis << 2)) & 0xf;
            tables.move_conj[idx][m] = real_axis * 3 + pow as i32;
        }
    }
}

fn init_phase1_move(tables: &mut CoordCube2LTables, ct: &cubie_cube::CubieCubeTables) {
    // FlipMove, TwistMove, UDSliceMove (raw, non-symmetry-reduced)
    for i in 0..2048 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_flip(i as i32);
        for m in 0..N_CUBE_MOVE {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(m as i32, &mut cc2);
            tables.flip_move[i][m] = cc2.cube.get_flip();
        }
    }

    for i in 0..2187 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_twist(i as i32);
        for m in 0..N_CUBE_MOVE {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(m as i32, &mut cc2);
            tables.twist_move[i][m] = cc2.cube.get_twist();
        }
    }

    for i in 0..495 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_ud_slice(i as i32);
        for m in 0..N_CUBE_MOVE {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(m as i32, &mut cc2);
            tables.ud_slice_move[i][m] = cc2.cube.get_ud_slice();
        }
    }

    // Conjugation tables
    for i in 0..2048 {
        let mut cc = CubieCube::new();
        cc.set_ud_slice(0);
        cc.set_flip(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::edge_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.flip_conj[i][j] = d.get_flip();
        }
    }

    for i in 0..2187 {
        let mut cc = CubieCube::new();
        cc.set_twist(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::corn_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.twist_conj[i][j] = d.get_twist();
        }
    }

    for i in 0..495 {
        let mut cc = CubieCube::new();
        cc.set_ud_slice(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::edge_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.ud_slice_conj[i][j] = d.get_ud_slice();
        }
    }

    // FlipConjXor
    for i in 0..495 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_ud_slice(i as i32);
        cc.cube.set_flip(0);
        for j in 0..12 {
            if (cc.cube.ea[j] >> 1) >= 8 {
                cc.cube.ea[j] |= 1;
            }
        }
        tables.flip_conj_xor[i] = cc.cube.get_flip() ^ 7;
    }

    // FlipUDSMove, FlipUDSConj
    for i in 0..495 {
        for j in 0..2048 {
            let idx = i << 11 | j;
            for m in 0..N_CUBE_MOVE {
                tables.flip_uds_move[idx][m] =
                    tables.ud_slice_move[i][m] << 11 | tables.flip_move[j][m];
            }
            for k in 0..16 {
                let sc = tables.ud_slice_conj[i][k];
                let xor_val = if k % 2 == 1 { tables.flip_conj_xor[sc as usize] } else { 0 };
                tables.flip_uds_conj[idx][k] = sc << 11 | (tables.flip_conj[j][k] ^ xor_val);
            }
        }
    }

    // FlipUDS2SliceF4 and FlipUDS2SliceF8
    for i in 0..495 {
        for j in 0..2048 {
            let mut cc = CubieCube2L::new();
            cc.cube.set_ud_slice(i as i32);
            cc.cube.set_flip(j as i32);
            tables.flip_uds2_slice_f4[i << 11 | j] = cc.get_slice_f4();
            tables.flip_uds2_slice_f8[i << 11 | j] = cc.get_slice_f8();
        }
    }

    // SliceF4Move, SliceF4Conj
    for i in 0..7920 {
        let mut cc = CubieCube2L::new();
        cc.set_slice_f4(i as i32);
        for m in 0..N_CUBE_MOVE {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(m as i32, &mut cc2);
            tables.slice_f4_move[i][m] = cc2.get_slice_f4();
        }
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::edge_conjugate(&cc.cube, ct.sym_mult_inv[0][j], &mut d);
            let cc2 = CubieCube2L { cube: d, ct: cc.ct };
            tables.slice_f4_conj[i][j] = cc2.get_slice_f4();
        }
    }

    // SliceF8Move, SliceF8Conj (very large: 126720 entries)
    for i in 0..126720 {
        let mut cc = CubieCube2L::new();
        cc.set_slice_f8(i as i32);
        for m in 0..N_CUBE_MOVE {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(m as i32, &mut cc2);
            tables.slice_f8_move[i][m] = cc2.get_slice_f8();
        }
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::edge_conjugate(&cc.cube, ct.sym_mult_inv[0][j], &mut d);
            let cc2 = CubieCube2L { cube: d, ct: cc.ct };
            tables.slice_f8_conj[i][j] = cc2.get_slice_f8();
        }
    }
}

fn init_phase2_move(tables: &mut CoordCube2LTables, ct: &cubie_cube::CubieCubeTables) {
    let _std2ud = util::std2ud();

    for i in 0..40320 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_e_perm(i as i32);
        for m in 0..N_CUBE_MOVE2 {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(util::UD2STD[m], &mut cc2);
            tables.e_perm_move[i][m] = cc2.cube.get_e_perm();
        }
    }

    for i in 0..40320 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_c_perm(i as i32);
        tables.c_perm2_c_comb[i] = cc.cube.get_c_comb();
        for m in 0..N_CUBE_MOVE2 {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(util::UD2STD[m], &mut cc2);
            tables.c_perm_move[i][m] = cc2.cube.get_c_perm();
        }
    }

    for i in 0..24 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_m_perm(i as i32);
        for m in 0..N_CUBE_MOVE2 {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(util::UD2STD[m], &mut cc2);
            tables.m_perm_move[i][m] = cc2.cube.get_m_perm();
        }
    }

    for i in 0..70 {
        let mut cc = CubieCube2L::new();
        cc.cube.set_c_comb(i as i32);
        for m in 0..N_CUBE_MOVE2 {
            let mut cc2 = CubieCube2L::new();
            cc.do_move_to(util::UD2STD[m], &mut cc2);
            tables.c_comb_move[i][m] = cc2.cube.get_c_comb();
        }
    }

    // Conjugation tables
    for i in 0..40320 {
        let mut cc = CubieCube::new();
        cc.set_e_perm(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::edge_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.e_perm_conj[i][j] = d.get_e_perm();
        }
    }

    for i in 0..40320 {
        let mut cc = CubieCube::new();
        cc.set_c_perm(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::corn_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.c_perm_conj[i][j] = d.get_c_perm();
        }
    }

    for i in 0..24 {
        let mut cc = CubieCube::new();
        cc.set_m_perm(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::edge_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.m_perm_conj[i][j] = d.get_m_perm();
        }
    }

    for i in 0..70 {
        let mut cc = CubieCube::new();
        cc.set_c_comb(i as i32);
        for j in 0..16 {
            let mut d = CubieCube::new();
            CubieCube::corn_conjugate(&cc, ct.sym_mult_inv[0][j], &mut d);
            tables.c_comb_conj[i][j] = d.get_c_comb();
        }
    }

    // MPCCb combined tables
    for i in 0..1680 {
        for j in 0..N_CUBE_MOVE2 {
            tables.mpc_cb_move[i][j] = tables.c_comb_move[i / 24][j] * 24 + tables.m_perm_move[i % 24][j];
        }
        for j in 0..16 {
            tables.mpc_cb_conj[i][j] = tables.c_comb_conj[i / 24][j] * 24 + tables.m_perm_conj[i % 24][j];
        }
    }
}

// ==================== Pruning table generation ====================

fn load_or_generate_prun(
    dir: &Path,
    filename: &str,
    cord1_move: &[Vec<i32>],
    cord2_move: &[Vec<i32>],
    cord1_conj: &[Vec<i32>],
    cord2_conj: &[Vec<i32>],
    tables: &CoordCube2LTables,
    is_phase2: bool,
) -> Option<Vec<Vec<u8>>> {
    let path = dir.join(filename);
    let cord1_size = cord1_move.len();
    let cord2_size = cord2_move.len();
    let t_load = std::time::Instant::now();
    if let Some(data) = load_prun_from_file(&path, cord1_size, cord2_size) {
        let dt = t_load.elapsed();
        let bytes = (cord1_size as u64) * (cord2_size as u64) * 9;
        let mibs = (bytes as f64 / (1024.0 * 1024.0)) / dt.as_secs_f64().max(1e-9);
        eprintln!(
            "[2L] loaded {} ({}x{}) in {:.3}s ({:.0} MiB/s)",
            filename, cord1_size, cord2_size * 9, dt.as_secs_f64(), mibs
        );
        return Some(data);
    }
    eprintln!("[2L] generating {} ({}x{}) ...", filename, cord1_size, cord2_size * 9);
    let t0 = std::time::Instant::now();
    let table = init_prun_table(
        cord1_move, cord2_move, cord1_conj, cord2_conj, tables, is_phase2,
    );
    eprintln!("[2L] generated {} in {:.1}s", filename, t0.elapsed().as_secs_f64());
    if let Err(e) = save_prun_to_file(&path, &table) {
        eprintln!("[2L] warning: failed to save {}: {}", filename, e);
    }
    Some(table)
}

fn load_or_generate_prun_single_empty(
    dir: &Path,
    filename: &str,
    cord2_move: &[Vec<i32>],
    cord2_conj: &[Vec<i32>],
    tables: &CoordCube2LTables,
    is_phase2: bool,
) -> Option<Vec<Vec<u8>>> {
    let path = dir.join(filename);
    let cord1_size = 1usize;
    let cord2_size = cord2_move.len();
    let t_load = std::time::Instant::now();
    if let Some(data) = load_prun_from_file(&path, cord1_size, cord2_size) {
        let dt = t_load.elapsed();
        let bytes = (cord1_size as u64) * (cord2_size as u64) * 9;
        let mibs = (bytes as f64 / (1024.0 * 1024.0)) / dt.as_secs_f64().max(1e-9);
        eprintln!(
            "[2L] loaded {} ({}x{}) in {:.3}s ({:.0} MiB/s)",
            filename, cord1_size, cord2_size * 9, dt.as_secs_f64(), mibs
        );
        return Some(data);
    }
    eprintln!("[2L] generating {} ({}x{}) ...", filename, cord1_size, cord2_size * 9);
    let t0 = std::time::Instant::now();
    // EmptyMove = [[0;20]], EmptyConj = [[0;16]]
    let empty_move: Vec<Vec<i32>> = vec![vec![0i32; 20]];
    let empty_conj: Vec<Vec<i32>> = vec![vec![0i32; 16]];
    let table = init_prun_table(
        &empty_move, cord2_move, &empty_conj, cord2_conj, tables, is_phase2,
    );
    eprintln!("[2L] generated {} in {:.1}s", filename, t0.elapsed().as_secs_f64());
    if let Err(e) = save_prun_to_file(&path, &table) {
        eprintln!("[2L] warning: failed to save {}: {}", filename, e);
    }
    Some(table)
}

fn load_prun_from_file(path: &Path, cord1_size: usize, cord2_size: usize) -> Option<Vec<Vec<u8>>> {
    if !path.exists() {
        return None;
    }
    let row_size = cord2_size * 9;
    let total = (cord1_size as u64).checked_mul(row_size as u64)?;

    let file = File::open(path).ok()?;
    let meta = file.metadata().ok()?;
    if meta.len() != total {
        eprintln!(
            "[2L] {} has unexpected size {} (want {}), regenerating",
            path.display(), meta.len(), total
        );
        return None;
    }

    // Two strategies, picked per-table:
    //   * Small rows (e.g. CPermMPermPrun = 216 B / row): one `read_exact`
    //     per row means 40 320 syscalls. Wrap the file in an 8 MiB
    //     `BufReader` so each syscall amortizes over hundreds of rows.
    //     Measured: 12 ms -> 1 ms (~12x speedup).
    //   * Large rows (e.g. TwistSliceF8Prun = ~1.1 MiB / row): a single
    //     `read_exact` already saturates memcpy throughput. `BufReader`
    //     would add a useless copy through the 8 MiB buffer, so we read
    //     straight into the row Vec.
    //
    // Threshold chosen to keep large-row tables on the zero-copy path
    // while still batching tiny rows.
    const ROW_BUF_THRESHOLD: usize = 64 * 1024;

    let mut table = Vec::with_capacity(cord1_size);
    if row_size < ROW_BUF_THRESHOLD {
        let mut reader = BufReader::with_capacity(8 * 1024 * 1024, file);
        for _ in 0..cord1_size {
            let mut row = vec![0u8; row_size];
            reader.read_exact(&mut row).ok()?;
            table.push(row);
        }
    } else {
        let mut file = file;
        for _ in 0..cord1_size {
            let mut row = vec![0u8; row_size];
            file.read_exact(&mut row).ok()?;
            table.push(row);
        }
    }
    Some(table)
}

pub fn save_prun_to_file(path: &Path, table: &[Vec<u8>]) -> std::io::Result<()> {
    // 8 MiB BufWriter collapses per-row `write_all` syscalls into batched
    // writes for tables whose rows are small. For large-row tables the
    // buffer just passes through without harm.
    let file = File::create(path)?;
    let mut writer = BufWriter::with_capacity(8 * 1024 * 1024, file);
    for row in table {
        writer.write_all(row)?;
    }
    writer.flush()?;
    Ok(())
}

/// Generate a pruning table using cost-bucketed BFS.
///
/// Dispatches to either a single-threaded reference implementation or a
/// multi-threaded one based on `MIN2PHASE_2L_GEN_SEQ`. Both must produce
/// bitwise-identical tables; the parallel version is the default and is
/// the only one ever exercised by the bench, but the sequential one is
/// kept as a fallback for parity debugging.
pub fn init_prun_table(
    cord1_move: &[Vec<i32>],
    cord2_move: &[Vec<i32>],
    cord1_conj: &[Vec<i32>],
    cord2_conj: &[Vec<i32>],
    tables: &CoordCube2LTables,
    is_phase2: bool,
) -> Vec<Vec<u8>> {
    let seq = std::env::var("MIN2PHASE_2L_GEN_SEQ")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        .unwrap_or(false);
    if seq {
        init_prun_table_seq(cord1_move, cord2_move, cord1_conj, cord2_conj, tables, is_phase2)
    } else {
        init_prun_table_par(cord1_move, cord2_move, cord1_conj, cord2_conj, tables, is_phase2)
    }
}

/// Single-threaded reference implementation. Kept verbatim from the
/// original 1:1 port so that the parallel version can be diffed against
/// it whenever the parity tests detect a discrepancy.
pub fn init_prun_table_seq(
    cord1_move: &[Vec<i32>],
    cord2_move: &[Vec<i32>],
    cord1_conj: &[Vec<i32>],
    cord2_conj: &[Vec<i32>],
    tables: &CoordCube2LTables,
    is_phase2: bool,
) -> Vec<Vec<u8>> {
    let cord1_size = cord1_move.len();
    let cord2_size = cord2_move.len();
    let row_size = cord2_size * 9;
    let std2ud = util::std2ud();

    let mut prun_table = vec![vec![127u8; row_size]; cord1_size];
    // Seed
    prun_table[0][..9].fill(0);

    let mut done_total: u64 = 0;
    for depth in 0..0xffffu32 {
        let mut done: u64 = 0;
        for cord1 in 0..cord1_size {
            for i in 0..row_size {
                if prun_table[cord1][i] != depth as u8 {
                    continue;
                }
                done += 1;
                let cord2 = i / 9;
                let ct_val = (i / 3) % 3;
                let leg = i % 3;
                let next_state = &NEXT_STATE[leg];

                for m in 0..N_LEG_MOVES {
                    let leg_ = next_state[m];
                    if leg_ == -1 {
                        continue;
                    }
                    let mut cord1_ = cord1 as i32;
                    let mut cord2_ = cord2 as i32;
                    let mut ct_ = ct_val as i32;

                    let cube_move = M_ON_CUBE[m];
                    if cube_move != -1 {
                        let mut cm = tables.move_conj[ct_val][cube_move as usize];
                        if is_phase2 {
                            cm = std2ud[cm as usize];
                            if cm >= 10 {
                                continue;
                            }
                        }
                        cord1_ = cord1_move[cord1][cm as usize];
                        cord2_ = cord2_move[cord2][cm as usize];
                    }
                    let ct_move = M_ON_CT[m];
                    if ct_move != -1 {
                        ct_ = tables.ct_move[ct_val][ct_move as usize];
                    }
                    if ct_ >= 3 {
                        let conj = CT_STD_CONJ[ct_ as usize];
                        cord1_ = cord1_conj[cord1_ as usize][conj as usize];
                        cord2_ = cord2_conj[cord2_ as usize][conj as usize];
                        ct_ %= 3;
                    }
                    let idx = (cord2_ as usize * 3 + ct_ as usize) * 3 + leg_ as usize;
                    let min_cost = (depth as i32 + M_COST[leg][m]) as u8;
                    if min_cost < prun_table[cord1_ as usize][idx] {
                        prun_table[cord1_ as usize][idx] = min_cost;
                    }
                }
            }
        }
        done_total += done;
        if done_total / cord1_size as u64 == row_size as u64 {
            break;
        }
    }
    prun_table
}

/// Multi-threaded version of [`init_prun_table_seq`]. The parallelism is
/// per BFS layer: within a single `depth` value, the work is split along
/// the outer `cord1` axis between N worker threads using `std::thread::scope`.
///
/// Correctness rests on two facts:
///
/// 1. `M_COST` is strictly positive, so every write within layer `depth`
///    stores a value `>= depth + 1`. The current layer's scan is therefore
///    immune to its own writes — no thread can observe a freshly-written
///    cell as belonging to the current layer.
///
/// 2. Writes to `prun_table[c1'][idx]` are min-reductions, performed with
///    `AtomicU8` CAS, so concurrent writers always converge to the same
///    minimum regardless of interleaving.
///
/// The output therefore equals the sequential output bitwise.
fn init_prun_table_par(
    cord1_move: &[Vec<i32>],
    cord2_move: &[Vec<i32>],
    cord1_conj: &[Vec<i32>],
    cord2_conj: &[Vec<i32>],
    tables: &CoordCube2LTables,
    is_phase2: bool,
) -> Vec<Vec<u8>> {
    let cord1_size = cord1_move.len();
    let cord2_size = cord2_move.len();
    let row_size = cord2_size * 9;
    let std2ud = util::std2ud();

    // Allocate as a flat Vec<AtomicU8> per row so each row can be borrowed
    // independently across threads (Vec<Vec<AtomicU8>> is fine since each
    // AtomicU8 already provides interior mutability through &).
    let mut prun_table: Vec<Vec<AtomicU8>> = Vec::with_capacity(cord1_size);
    for _ in 0..cord1_size {
        let mut row = Vec::with_capacity(row_size);
        for _ in 0..row_size {
            row.push(AtomicU8::new(127));
        }
        prun_table.push(row);
    }
    // Seed
    for cell in prun_table[0].iter().take(9) {
        cell.store(0, Ordering::Relaxed);
    }

    // Pick a reasonable worker count: tables with very small `cord1_size`
    // (e.g. 1 for the FlipUDSPrun degenerate table) don't benefit, so cap
    // worker count by the number of rows.
    let threads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
        .min(cord1_size.max(1));

    let done_total = AtomicU64::new(0);
    for depth in 0..0xffffu32 {
        let done_layer = AtomicU64::new(0);

        // Slice `cord1` range across workers. Each worker only *writes* to
        // rows it doesn't own (via the AtomicU8 min CAS), and only *reads*
        // the row it currently scans plus the destination cell during CAS.
        // No row is exclusively owned, so we pass &prun_table to all.
        let pt = &prun_table;
        let done_layer_ref = &done_layer;
        let std2ud_ref = &std2ud;

        let chunk = cord1_size.div_ceil(threads);
        std::thread::scope(|s| {
            for t in 0..threads {
                let start = t * chunk;
                let end = ((t + 1) * chunk).min(cord1_size);
                if start >= end {
                    continue;
                }
                s.spawn(move || {
                    let mut local_done: u64 = 0;
                    for cord1 in start..end {
                        let row = &pt[cord1];
                        for i in 0..row_size {
                            // Relaxed: we only need to recognise cells stored
                            // by *previous* layers; their writes are
                            // synchronised by the join at the end of the
                            // previous layer's scope.
                            if row[i].load(Ordering::Relaxed) != depth as u8 {
                                continue;
                            }
                            local_done += 1;
                            let cord2 = i / 9;
                            let ct_val = (i / 3) % 3;
                            let leg = i % 3;
                            let next_state = &NEXT_STATE[leg];

                            for m in 0..N_LEG_MOVES {
                                let leg_ = next_state[m];
                                if leg_ == -1 {
                                    continue;
                                }
                                let mut cord1_ = cord1 as i32;
                                let mut cord2_ = cord2 as i32;
                                let mut ct_ = ct_val as i32;

                                let cube_move = M_ON_CUBE[m];
                                if cube_move != -1 {
                                    let mut cm = tables.move_conj[ct_val][cube_move as usize];
                                    if is_phase2 {
                                        cm = std2ud_ref[cm as usize];
                                        if cm >= 10 {
                                            continue;
                                        }
                                    }
                                    cord1_ = cord1_move[cord1][cm as usize];
                                    cord2_ = cord2_move[cord2][cm as usize];
                                }
                                let ct_move = M_ON_CT[m];
                                if ct_move != -1 {
                                    ct_ = tables.ct_move[ct_val][ct_move as usize];
                                }
                                if ct_ >= 3 {
                                    let conj = CT_STD_CONJ[ct_ as usize];
                                    cord1_ = cord1_conj[cord1_ as usize][conj as usize];
                                    cord2_ = cord2_conj[cord2_ as usize][conj as usize];
                                    ct_ %= 3;
                                }
                                let idx = (cord2_ as usize * 3 + ct_ as usize) * 3 + leg_ as usize;
                                let min_cost = (depth as i32 + M_COST[leg][m]) as u8;
                                // Atomic min via CAS loop. `fetch_min` is
                                // unstable for u8 on stable Rust, so we open-code it.
                                let cell = &pt[cord1_ as usize][idx];
                                let mut cur = cell.load(Ordering::Relaxed);
                                while min_cost < cur {
                                    match cell.compare_exchange_weak(
                                        cur,
                                        min_cost,
                                        Ordering::Relaxed,
                                        Ordering::Relaxed,
                                    ) {
                                        Ok(_) => break,
                                        Err(observed) => cur = observed,
                                    }
                                }
                            }
                        }
                    }
                    done_layer_ref.fetch_add(local_done, Ordering::Relaxed);
                });
            }
        });

        let done = done_layer.load(Ordering::Relaxed);
        let total = done_total.fetch_add(done, Ordering::Relaxed) + done;
        if total / cord1_size as u64 == row_size as u64 {
            break;
        }
    }

    // Convert back to Vec<Vec<u8>>. Atomic loads with Relaxed are fine
    // because the per-layer `scope` join already synchronises memory.
    prun_table
        .into_iter()
        .map(|row| row.into_iter().map(|c| c.into_inner()).collect())
        .collect()
}
