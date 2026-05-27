//! Search2L - Two-layer ("leg") solver variant.
//! 1:1 port of Search2L.java.
//!
//! Unlike `Search`, this solver performs its own IDA* using leg-moves (0..20)
//! and produces step strings like `(z1z0) U ` instead of plain `R U F'`.
//!
//! Pruning tables are stored under `<crate>/2l_data/` and are generated
//! separately from the Java implementation's data files.

use crate::cubie_cube::{self, CubieCube};
use crate::cubie_cube_2l::{CubieCube2L, MOVE2STR};
use crate::coord_cube_2l as cc2l;
use crate::coord_cube::CoordCubeNode;
use crate::util;

use std::collections::{HashSet, HashMap};
use std::sync::OnceLock;

// ===== Constants matching Java =====
pub const MAX_LENGTH2: i32 = 36;
const N_FILTER_MOVES: i32 = 5;
const FILTER_MASK: i64 = (1i64 << ((N_FILTER_MOVES - 1) * 5)) - 1;
const FILTER_MASKL: i64 = (1i64 << (N_FILTER_MOVES * 5)) - 1;

const MAX_DEPTH_TOTAL: usize = 80;

// ===== AllowedMoves initialisation =====

struct AllowedMovesData {
    /// key = ((lm >> 5) - 1) * 3 + prev_leg ; or -1 for the initial state.
    /// value = bitmask of allowed leg-moves (lower 20 bits).
    map: HashMap<i64, i32>,
}

static ALLOWED_MOVES: OnceLock<AllowedMovesData> = OnceLock::new();

fn allowed_moves() -> &'static AllowedMovesData {
    ALLOWED_MOVES.get_or_init(build_allowed_moves)
}

/// Mirror Java `initAllowedMoves` recursion; produces the allowed-move filter.
fn build_allowed_moves() -> AllowedMovesData {
    // Make sure tables are ready (we read NEXT_STATE / mOnCt etc. which are
    // pure consts, but we also use ctStdConj-aware moves later).
    cc2l::ensure_initialized();

    let mut set: std::collections::BTreeMap<String, i64> = std::collections::BTreeMap::new();
    let mut node_cnt: i32 = 0;
    let mut allowed: HashMap<i64, i32> = HashMap::new();

    for maxl in 0..=N_FILTER_MOVES {
        let cc0 = CubieCube2L::new();
        for leg_start in 0..3 {
            init_allowed_recursive(
                &cc0,
                leg_start,
                leg_start,
                0,
                -1,
                0,
                maxl,
                &mut set,
                &mut node_cnt,
                &mut allowed,
            );
        }
    }

    log::info!(
        "[2L] allowedMoves init: set={} nodes={} allowed_keys={}",
        set.len(),
        node_cnt,
        allowed.len()
    );

    AllowedMovesData { map: allowed }
}

#[allow(clippy::too_many_arguments)]
fn init_allowed_recursive(
    cc: &CubieCube2L,
    leg: i32,
    leg0: i32,
    prev_leg: i32,
    lm: i64,
    _depth: i32,
    maxl: i32,
    set: &mut std::collections::BTreeMap<String, i64>,
    node_cnt: &mut i32,
    allowed: &mut HashMap<i64, i32>,
) {
    if maxl == 0 {
        // Canonical key: serialise (ca + ea + ct + leg0 + leg + ct_raw).
        // Java uses cc.toString(); we mimic with a stable fingerprint.
        let mut key = String::with_capacity(96);
        for &v in &cc.cube.ca {
            key.push_str(&format!("{:02x}", v));
        }
        key.push('|');
        for &v in &cc.cube.ea {
            key.push_str(&format!("{:02x}", v));
        }
        key.push('|');
        key.push_str(&format!("{}|{}|{:06x}|", leg0, leg, cc.ct));

        if let std::collections::btree_map::Entry::Vacant(e) = set.entry(key) {
            e.insert(lm * 3 + leg as i64);
            let key2: i64 = if lm == -1 { -1 } else { ((lm >> 5) - 1) * 3 + prev_leg as i64 };
            let bitmap = *allowed.get(&key2).unwrap_or(&0);
            let bit = 1i32 << (lm & 0x1f);
            allowed.insert(key2, bitmap | bit);
        }
        *node_cnt += 1;
        return;
    }

    let next_state = &cc2l::NEXT_STATE[leg as usize];
    let lm_idx = if lm == -1 { 20 } else { (lm & 0x1f) as usize };
    let lm_mask = cc2l::RELEASED_LEGS[lm_idx] | cc2l::PARALLEL_MOVES[lm_idx];

    for m in 0..cc2l::N_LEG_MOVES {
        if next_state[m] == -1
            || (cc2l::RELEASED_LEGS[m] & lm_mask) != 0
            || ((cc2l::PARALLEL_MOVES[m] & lm_mask) != 0 && m > lm_idx)
        {
            continue;
        }
        let mut cc2 = CubieCube2L::new();
        // Java applies mOnCt then mOnCube sequentially (only one is != -1 in practice,
        // but stay faithful to source).
        if cc2l::M_ON_CT[m] != -1 {
            cc.do_move_to(cc2l::M_ON_CT[m] + 18, &mut cc2);
        }
        if cc2l::M_ON_CUBE[m] != -1 {
            cc.do_move_to(cc2l::M_ON_CUBE[m], &mut cc2);
        }
        let lm_next = ((lm + 1) << 5 | m as i64) & FILTER_MASKL;
        init_allowed_recursive(
            &cc2,
            next_state[m],
            leg0,
            leg,
            lm_next,
            _depth + 1,
            maxl - 1,
            set,
            node_cnt,
            allowed,
        );
    }
}

// ===== Search2L =====

pub struct Search2L {
    // ---- Internal cube / cube history ----
    cc: CubieCube,
    urf_cubie_cube: [CubieCube; 6],

    /// 2L cubie state for each phase-1 depth (0..=80).
    phase1_cubie_2l: Vec<CubieCube2L>,
    /// Phase-1 leg moves recorded by depth.
    mov: Vec<i32>,
    /// Coordinate nodes used by phase1 (leg-search).
    node_ud: Vec<CoordCubeNode>,

    // ---- Search control ----
    urf_idx: usize,
    length1: i32,
    depth1: i32,
    sol: i32,
    solution: Option<String>,
    probe: i64,
    probe_max: i64,
    probe_min: i64,
    verbose: i32,
    valid1: i32,
    is_rec: bool,

    // ---- Phase-2 state ----
    depth2_maxl: Vec<i32>,
    depth2: i32,
    max_len2: i32,
    solved_phase2_states: HashSet<i64>,

    // ---- Stats (parity with Java for reporting) ----
    pub phase2_duration_ns: u128,
    pub phase2s_duration_ns: u128,
    pub estimated: i64,

    /// When true, log every length1/urf transition and every solution found
    /// (cost / phase1 length / phase2 length / probe). Off by default — only
    /// the diagnostic `diag_2l` example flips this on.
    pub trace: bool,
}

impl Default for Search2L {
    fn default() -> Self {
        Search2L {
            cc: CubieCube::default(),
            urf_cubie_cube: [CubieCube::default(); 6],
            phase1_cubie_2l: vec![CubieCube2L::default(); MAX_DEPTH_TOTAL],
            mov: vec![0; MAX_DEPTH_TOTAL],
            node_ud: vec![CoordCubeNode::default(); MAX_DEPTH_TOTAL],

            urf_idx: 0,
            length1: 0,
            depth1: 0,
            sol: 0,
            solution: None,
            probe: 0,
            probe_max: 0,
            probe_min: 0,
            verbose: 0,
            valid1: 0,
            is_rec: false,

            depth2_maxl: vec![0; 128],
            depth2: 0,
            max_len2: MAX_LENGTH2,
            solved_phase2_states: HashSet::new(),

            phase2_duration_ns: 0,
            phase2s_duration_ns: 0,
            estimated: 0,
            trace: false,
        }
    }
}

impl Search2L {
    pub fn new() -> Self {
        Self::default()
    }

    /// Initialise all tables required for Search2L. Idempotent.
    pub fn init() {
        cubie_cube::ensure_tables_initialized();
        cc2l::ensure_initialized();
        let _ = allowed_moves();
    }

    /// Allow callers to override the data directory before [`init`] runs.
    pub fn set_data_dir<P: Into<std::path::PathBuf>>(p: P) {
        cc2l::set_data_dir(p);
    }

    /// Solve the given facelets string. Mirrors `Search2L.solution`.
    /// `verbose` follows the same flags as `Search` (`INVERSE_SOLUTION` / `OPTIMAL_SOLUTION`
    /// are *not* supported here — Java's Search2L only ever calls the non-optimal path).
    pub fn solution(
        &mut self,
        facelets: &str,
        max_depth: i32,
        probe_max: i64,
        probe_min: i64,
        verbose: i32,
    ) -> String {
        // ---- verify ----
        let check = self.verify_facelets(facelets);
        if check != 0 {
            return format!("Error {}", check.abs());
        }

        // Init globally first
        Self::init();

        // Make sure the prun tables are actually present.
        let tab = cc2l::get_tables();
        if tab.twist_slice_f4_prun.is_none()
            || tab.flip_uds_prun.is_none()
            || tab.e_perm_mpc_cb_prun.is_none()
            || tab.c_perm_m_perm_prun.is_none()
        {
            return "Error 9: 2L pruning tables not loaded".to_string();
        }

        self.sol = max_depth + 1;
        self.probe = 0;
        self.probe_max = probe_max;
        self.probe_min = probe_min.min(probe_max);
        self.verbose = verbose;
        self.solution = None;
        self.is_rec = false;
        self.solved_phase2_states.clear();
        self.max_len2 = MAX_LENGTH2;
        self.phase2_duration_ns = 0;
        self.phase2s_duration_ns = 0;
        self.estimated = 0;

        // Prepare 6 URF-conjugated cubie copies (only 3 are ever used since
        // Java sets TRY_INVERSE=false / TRY_THREE_AXES=true / MAX_PRE_MOVES=0).
        let mut cur = self.cc;
        for i in 0..6 {
            self.urf_cubie_cube[i] = cur;
            cur.urf_conjugate();
            if i % 3 == 2 {
                cur.inv_cubie_cube();
            }
        }

        self.search()
    }

    // ===== Facelets → cubie =====
    fn verify_facelets(&mut self, facelets: &str) -> i32 {
        if facelets.len() != 54 {
            return -1;
        }
        let chars: Vec<char> = facelets.chars().collect();
        let center = [
            chars[util::U5 as usize],
            chars[util::R5 as usize],
            chars[util::F5 as usize],
            chars[util::D5 as usize],
            chars[util::L5 as usize],
            chars[util::B5 as usize],
        ];
        let mut f = [0u8; 54];
        let mut count = 0u32;
        for i in 0..54 {
            let idx = center.iter().position(|&c| c == chars[i]);
            match idx {
                Some(v) => {
                    f[i] = v as u8;
                    count += 1 << (v << 2);
                }
                None => return -1,
            }
        }
        if count != 0x999999 {
            return -1;
        }
        util::to_cubie_cube(&f, &mut self.cc.ca, &mut self.cc.ea);
        self.cc.verify()
    }

    // ===== Outer driver =====

    fn search(&mut self) -> String {
        // Java: TRY_INVERSE=false, TRY_THREE_AXES=true => conj_mask blocks
        // urf_idx in {3,4,5} (inverse) but allows {0,1,2}. We just iterate 0..3.
        // `sol` here is the *total leg-move count* of the best solution found
        // so far (`length1 + length2`). It starts at `maxDepth + 1` and is
        // reduced every time `initPhase2Pre` discovers a better total.
        //
        // The loop bound must therefore be re-evaluated on every iteration
        // (mirror Java's `for (length1 = ...; length1 < sol; length1++)`).
        // The naive `for length1 in 0..self.sol` snapshots `self.sol` once
        // at loop entry, which caused us to keep searching past the best
        // total and overwrite the optimal solution with longer-but-still-
        // legal ones (bug repro: cubes 5166/6845 in the 10000-cube baseline).
        let mut length1: i32 = 0;
        while length1 < self.sol {
            self.length1 = length1;
            for urf_idx in 0..3usize {
                self.urf_idx = urf_idx;
                let cc = self.urf_cubie_cube[urf_idx];
                self.valid1 = 0;

                if length1 + 1 >= self.node_ud.len() as i32 {
                    continue;
                }
                // Seed root node at slot `length1 + 1`.
                //
                // IMPORTANT (parity with Java `Search2L.setWithPrun`):
                // Java sets `fsym = 0` on the root *only for the prun lookup*,
                // and then re-assigns `node.fsym = (4>>urfIdx)&3` at the top of
                // `phase1` when `depth==0`. Using the wrong fsym for the root
                // prun lookup gives a *lower* prun estimate for urf={1,2}, which
                // causes Rust to enter phase1 at smaller `length1` than Java
                // does, wasting probe budget on sub-optimal early hits — and
                // for some cubes the probe budget exhausts before Java's final
                // refinement window at `length1=30`, leaving Rust with a
                // strictly worse cost than Java's baseline.
                //
                // Mirror Java exactly: compute root prun with fsym=0, then
                // re-set fsym to `(4>>urf)&3` *after* the prun check so phase1
                // sees the correct value at depth=0.
                let root_idx = (length1 + 1) as usize;
                self.node_ud[root_idx] = CoordCubeNode::new();
                self.node_ud[root_idx].twist = cc.get_twist();
                self.node_ud[root_idx].flip = cc.get_flip();
                self.node_ud[root_idx].slice = cc.get_ud_slice();
                self.node_ud[root_idx].fsym = 0; // Java setWithPrun: fsym=0
                self.node_ud[root_idx].tsym = 0; // leg start = pp
                self.calc_pruning_2l(root_idx);
                let prun = self.node_ud[root_idx].prun;
                // Now re-set fsym to the URF-dependent value for phase1.
                self.node_ud[root_idx].fsym = (4 >> urf_idx) & 3;

                self.phase1_cubie_2l[0].copy_from_cube(&cc);

                if prun > length1 {
                    if self.trace {
                        log::debug!(
                            "[trace] length1={} urf={} prun={} > length1 -> skip",
                            length1, urf_idx, prun
                        );
                    }
                    continue;
                }

                if self.trace {
                    log::debug!(
                        "[trace] enter phase1: length1={} urf={} prun={} probe={} sol={}",
                        length1, urf_idx, prun, self.probe, self.sol
                    );
                }

                let node = self.node_ud[root_idx];
                if self.phase1(&node, 0, length1, -1) == 0 {
                    if self.trace {
                        log::debug!(
                            "[trace] search RETURN 0 at length1={} urf={} probe={} sol={}",
                            length1, urf_idx, self.probe, self.sol
                        );
                    }
                    return self
                        .solution
                        .clone()
                        .unwrap_or_else(|| "Error 8".to_string());
                }
            }
            length1 += 1;
        }
        self.solution.clone().unwrap_or_else(|| "Error 7".to_string())
    }

    // ===== Phase-1 (leg search) =====
    // Mirrors Search2L.phase1(node, depth, maxl, lm).
    // - `depth` is the move count (recursion depth, 0-based).
    // - `maxl`  is the remaining cost budget.
    fn phase1(&mut self, node: &CoordCubeNode, depth: i32, maxl: i32, lm: i32) -> i32 {
        if node.twist == 0 && node.flip == 0 && node.slice == 0 && maxl < 5 {
            self.depth1 = depth;
            // Java: `return maxl == 0 ? 1 : initPhase2Pre();`
            // i.e. when the budget is exactly exhausted (maxl == 0), do NOT enter phase2
            // (the cube already solved purely in phase1); when there is still slack budget
            // (maxl ∈ {1..4}) we *do* enter phase2 to refine.
            return if maxl == 0 { 1 } else { self.init_phase2_pre() };
        }

        let next_state = &cc2l::NEXT_STATE[node.tsym as usize];
        let lm_idx = if lm == -1 { 20usize } else { (lm & 0x1f) as usize };
        let lm_mask = cc2l::RELEASED_LEGS[lm_idx] | cc2l::PARALLEL_MOVES[lm_idx];

        // Filter mask via allowedMoves
        // Java: `Long val = allowedMoves.get(Long.valueOf(((lm & FILTER_MASK) * 3 + node.tsym)));`
        // Note: when lm == -1, (lm & FILTER_MASK) = FILTER_MASK (Java -1 == 0xFFFFFFFF, masked by 0xFFFFF),
        // so the key is FILTER_MASK*3 + tsym, which will NOT match the init key (-1) — so val == null —
        // and we fall back to 0xfffff (all leg-moves allowed) for the very first step.
        let masked_lm: i64 = if lm == -1 { FILTER_MASK } else { (lm as i64) & FILTER_MASK };
        let key2: i64 = masked_lm * 3 + node.tsym as i64;
        let lm_mask2: i32 = match allowed_moves().map.get(&key2) {
            Some(v) => *v,
            None => if lm == -1 { 0xfffff } else { 0 },
        };

        for m in 0..cc2l::N_LEG_MOVES {
            if next_state[m] == -1
                || (lm_mask2 >> m) & 1 == 0
                || (cc2l::RELEASED_LEGS[m] & lm_mask) != 0
                || ((cc2l::PARALLEL_MOVES[m] & lm_mask) != 0 && m > lm_idx)
            {
                continue;
            }
            if self.is_rec && (m as i32) != self.mov[depth as usize] {
                continue;
            }
            // Borrow node values before doing the move (which mutates node_ud).
            // Cost depends on the *current* node's leg state (node.tsym).
            let cost = cc2l::M_COST[node.tsym as usize][m];
            let maxl_next = maxl - cost;
            if maxl_next < 0 {
                continue;
            }
            // Compute prun via doMovePrun on a scratch slot (node_ud[maxl]).
            let slot = maxl as usize;
            if slot >= self.node_ud.len() {
                continue;
            }
            self.do_move_prun_2l(node, m, slot);
            let prun = self.node_ud[slot].prun;
            if prun > maxl_next {
                continue;
            }
            self.mov[depth as usize] = m as i32;
            self.depth2_maxl[(depth + 1) as usize] = maxl;
            self.valid1 = self.valid1.min(depth);
            let lm_next = (((lm + 1) << 5) | m as i32) & FILTER_MASK as i32;
            let child = self.node_ud[slot];
            let ret = self.phase1(&child, depth + 1, maxl_next, lm_next);
            if ret == 0 {
                return 0;
            }
        }
        1
    }

    /// 2L pruning calculation: max(TwistSliceF4Prun, FlipUDSPrun [, TwistSliceF8Prun]).
    /// Mirrors `CoordCube2L.calcPruning(true)`, which mutates the node's
    /// flip/twist/slice/fsym to its symmetry-reduced form.
    fn calc_pruning_2l(&mut self, slot: usize) {
        let tab = cc2l::get_tables();
        let n = &mut self.node_ud[slot];

        // If fsym >= 3, conjugate down via ctStdConj (mirrors Java calcPruning).
        if n.fsym >= 3 {
            let conj = cc2l::CT_STD_CONJ[n.fsym as usize] as usize;
            let new_slice = tab.ud_slice_conj[n.slice as usize][conj];
            let mut new_flip = tab.flip_conj[n.flip as usize][conj];
            if conj % 2 == 1 {
                new_flip ^= tab.flip_conj_xor[new_slice as usize];
            }
            n.slice = new_slice;
            n.flip = new_flip;
            n.twist = tab.twist_conj[n.twist as usize][conj];
            n.fsym %= 3;
        }

        let slice = n.slice as usize;
        let flip = n.flip as usize;
        let twist = n.twist as usize;
        let fsym = n.fsym as usize;
        let tsym = n.tsym as usize;

        let f4_idx = tab.flip_uds2_slice_f4[(slice << 11) | flip] as usize;
        let mut prun = tab.twist_slice_f4_prun.as_ref().unwrap()[twist]
            [f4_idx * 9 + fsym * 3 + tsym] as i32;

        let uds_idx = (slice << 11) | flip;
        let p2 = tab.flip_uds_prun.as_ref().unwrap()[0]
            [uds_idx * 9 + fsym * 3 + tsym] as i32;
        if p2 > prun {
            prun = p2;
        }

        if let Some(f8_tab) = tab.twist_slice_f8_prun.as_ref() {
            let f8_idx = tab.flip_uds2_slice_f8[uds_idx] as usize;
            let p3 = f8_tab[twist][f8_idx * 9 + fsym * 3 + tsym] as i32;
            if p3 > prun {
                prun = p3;
            }
        }
        n.prun = prun;
    }

    /// Apply leg-move `m` on `cc` storing result in `node_ud[slot]` (with prun).
    /// Mirrors `CoordCube2L.doMovePrun(cc, m, true)`.
    fn do_move_prun_2l(&mut self, cc: &CoordCubeNode, m: usize, slot: usize) {
        let tab = cc2l::get_tables();
        let tsym_next = cc2l::NEXT_STATE[cc.tsym as usize][m];

        let mut flip = cc.flip;
        let mut twist = cc.twist;
        let mut slice = cc.slice;
        let mut fsym = cc.fsym;

        let cube_move = cc2l::M_ON_CUBE[m];
        if cube_move != -1 {
            // MoveConj indexed by cc.fsym (which is 0..2 from previous calcPruning).
            let cm = tab.move_conj[fsym as usize][cube_move as usize] as usize;
            flip = tab.flip_move[flip as usize][cm];
            twist = tab.twist_move[twist as usize][cm];
            slice = tab.ud_slice_move[slice as usize][cm];
        }
        let ct_move = cc2l::M_ON_CT[m];
        if ct_move != -1 {
            fsym = tab.ct_move[cc.fsym as usize][ct_move as usize];
        }

        let n = &mut self.node_ud[slot];
        n.flip = flip;
        n.twist = twist;
        n.slice = slice;
        n.fsym = fsym;
        n.tsym = tsym_next;
        // calc_pruning_2l will conjugate-down if fsym >= 3 and write n.prun.
        self.calc_pruning_2l(slot);
    }

    // ===== initPhase2Pre =====

    fn init_phase2_pre(&mut self) -> i32 {
        self.is_rec = false;
        let probe_limit = if self.solution.is_none() { self.probe_max } else { self.probe_min };
        if self.probe >= probe_limit {
            return 0;
        }
        self.probe += 1;

        let start = std::time::Instant::now();

        // depth1 here equals the *true* phase1 length consumed (in cost units).
        // We need to derive it. In Java, depth1 was set by the recursion: at the
        // terminal of phase1 it does `depth1 = depth;` and returns. In our flow,
        // we just track `self.depth1` (= length1) externally and `mov[]` contains
        // moves 0..self.depth1. But `depth1` in Java is in *move count*, not cost.
        // Re-reading: Java's phase1 uses `depth` as recursion depth, i.e. *move count*,
        // and stores `depth1 = depth`. So depth1 = number of moves so far.
        // We use a local "move count" tracked via `phase1_cubie_2l` filling.
        // For initPhase2Pre, we need the move count = number of valid moves in self.mov[].
        // That is captured by `self.depth1` (set at terminal in phase1).
        let depth1 = self.depth1 as usize;

        // Java: if (valid1 == 0) phase1Cubie2L[0].copy(phase1Cubie[0]); phase1Cubie2L[0].setCtIdx(4>>urfIdx & 3);
        // We already copied urf_cubie_cube[urf_idx] into phase1_cubie_2l[0] in search().
        // But we must also set the ct index based on URF idx.
        if self.valid1 == 0 {
            // copy was done; just set ct idx
            let ct_idx = (4 >> self.urf_idx) & 3;
            self.phase1_cubie_2l[0].set_ct_idx(ct_idx);
            self.depth2_maxl[0] = self.length1 + 1;
        }

        // Java: `for (int i = valid1; i <= depth1; i++)` — note the inclusive bound,
        // which is different from the standard Search parent class.
        for i in (self.valid1 as usize)..=depth1 {
            if i >= self.mov.len() {
                break;
            }
            let m = self.mov[i];
            let cube_move = cc2l::M_ON_CUBE[m as usize];
            let ct_move = cc2l::M_ON_CT[m as usize];
            // Borrow check workaround: copy the source slot.
            let src = self.phase1_cubie_2l[i];
            if cube_move != -1 {
                src.do_move_to(cube_move, &mut self.phase1_cubie_2l[i + 1]);
            } else if ct_move != -1 {
                src.do_move_to(ct_move + 18, &mut self.phase1_cubie_2l[i + 1]);
            } else {
                self.phase1_cubie_2l[i + 1] = src;
            }
        }
        self.valid1 = depth1 as i32;

        let end = &self.phase1_cubie_2l[depth1];
        let mut eperm = end.cube.get_e_perm();
        let mut cperm = end.cube.get_c_perm();
        let mut mperm = end.cube.get_m_perm();
        let mut ct = end.get_ct_idx();
        let leg_slot = self.depth2_maxl[depth1] as usize;
        let leg = self.node_ud[leg_slot].tsym;

        if self.trace {
            // dump the 12-leg path that produced this state
            let mut path = String::new();
            for i in 0..depth1 {
                use std::fmt::Write;
                let _ = write!(path, "{},", self.mov[i]);
            }
            log::info!(
                "[trace2]   p1-end length1={} urf={} depth1={} eperm={} cperm={} mperm={} ct={} leg={} (legSlot={}) path=[{}]",
                self.length1, self.urf_idx, depth1, eperm, cperm, mperm, ct, leg, leg_slot, path
            );
        }

        let tab = cc2l::get_tables();
        if ct >= 3 {
            let conj = cc2l::CT_STD_CONJ[ct as usize] as usize;
            eperm = tab.e_perm_conj[eperm as usize][conj];
            cperm = tab.c_perm_conj[cperm as usize][conj];
            mperm = tab.m_perm_conj[mperm as usize][conj];
            ct %= 3;
        }

        let key: i64 = ((((eperm as i64) * 40320 + cperm as i64) * 24
            + mperm as i64) * 3 + ct as i64) * 3 + leg as i64;
        if self.solved_phase2_states.contains(&key) {
            self.phase2_duration_ns += start.elapsed().as_nanos();
            return 1;
        }
        self.solved_phase2_states.insert(key);

        // pruning lookup for initial p2 state
        let prun_eperm_mpccb = tab.e_perm_mpc_cb_prun.as_ref().unwrap()[eperm as usize]
            [(tab.c_perm2_c_comb[cperm as usize] as usize * 24 + mperm as usize) * 9
                + ct as usize * 3 + leg as usize] as i32;
        let prun_cperm_mperm = tab.c_perm_m_perm_prun.as_ref().unwrap()[cperm as usize]
            [mperm as usize * 9 + ct as usize * 3 + leg as usize] as i32;
        let prun = prun_eperm_mpccb.max(prun_cperm_mperm);

        let start_s = std::time::Instant::now();
        let mut length2 = self.max_len2 - 1;
        let mut found_any = false;
        if self.trace {
            log::debug!(
                "[trace2]   init_phase2_pre: length1={} urf={} depth1={} max_len2={} prun={} (start length2={}) key_seen_before=false",
                self.length1, self.urf_idx, depth1, self.max_len2, prun, length2
            );
        }
        while length2 >= prun {
            let ret = self.phase2(eperm, cperm, mperm, ct, leg, length2, depth1 as i32, 20);
            if ret < 0 {
                if self.trace {
                    log::debug!("[trace2]     phase2(length2={}) -> -1 (break)", length2);
                }
                break;
            }
            if self.trace {
                log::debug!("[trace2]     phase2(length2={}) -> ret={} (sol_cand={}+{}={})", length2, ret, self.length1, length2 - ret, self.length1 + length2 - ret);
            }
            length2 -= ret;
            found_any = true;
            self.sol = self.length1 + length2;
            // Build the solution string
            self.estimated = 0;
            let mut sb = String::new();
            let mut leg2: i32 = 0;
            for i in 0..(self.depth2 as usize) {
                let m = self.mov[i] as usize;
                sb.push_str(MOVE2STR[m]);
                sb.push(' ');
                self.estimated += cc2l::MOVE_COST[leg2 as usize][m] as i64;
                leg2 = cc2l::NEXT_STATE[leg2 as usize][m];
            }
            self.solution = Some(sb.trim_end().to_string());

            if self.trace {
                log::debug!(
                    "[trace]   FOUND sol at length1={} urf={} depth1={} length2={} sol={} probe={} estimated_cost={}",
                    self.length1, self.urf_idx, depth1, length2, self.sol, self.probe, self.estimated
                );
            }

            // Try shorter
            length2 -= 1;
        }
        self.phase2_duration_ns += start.elapsed().as_nanos();
        self.phase2s_duration_ns += start_s.elapsed().as_nanos();

        if found_any {
            self.max_len2 = MAX_LENGTH2.min(self.sol - self.length1);
            if self.probe >= self.probe_min {
                if self.trace {
                    log::debug!(
                        "[trace]   init_phase2_pre RETURN 0 (probe={} >= probe_min={})",
                        self.probe, self.probe_min
                    );
                }
                return 0;
            }
        }
        1
    }

    // ===== Phase-2 =====
    #[allow(clippy::too_many_arguments)]
    fn phase2(
        &mut self,
        eperm: i32,
        cperm: i32,
        mperm: i32,
        ct: i32,
        leg: i32,
        maxl: i32,
        depth: i32,
        lm: i32,
    ) -> i32 {
        if eperm == 0 && cperm == 0 && mperm == 0 {
            self.depth2 = depth;
            return maxl;
        }
        let next_state = &cc2l::NEXT_STATE[leg as usize];
        let lm_idx = if lm == -1 { 20usize } else { lm as usize };
        let lm_mask = cc2l::RELEASED_LEGS[lm_idx] | cc2l::PARALLEL_MOVES[lm_idx];

        let tab = cc2l::get_tables();
        let std2ud = util::get_std2ud();

        for m in 0..cc2l::N_LEG_MOVES {
            if next_state[m] == -1
                || (cc2l::RELEASED_LEGS[m] & lm_mask) != 0
                || ((cc2l::PARALLEL_MOVES[m] & lm_mask) != 0 && m > lm_idx)
            {
                continue;
            }
            if self.is_rec && (m as i32) != self.mov[depth as usize] {
                // Java: `if (isRec && m != move[depth1 - maxl]) continue;`
                // In phase2, `depth1 - maxl` evaluates to the current `depth` argument
                // (see Java init_phase2_pre and recursion).
                continue;
            }
            let leg_x = next_state[m];
            let mut eperm_x = eperm;
            let mut cperm_x = cperm;
            let mut mperm_x = mperm;
            let mut ct_x = ct;

            let cube_move = cc2l::M_ON_CUBE[m];
            if cube_move != -1 {
                let cm0 = tab.move_conj[ct as usize][cube_move as usize] as usize;
                let cm = std2ud[cm0];
                if cm >= 10 {
                    continue;
                }
                eperm_x = tab.e_perm_move[eperm_x as usize][cm as usize];
                cperm_x = tab.c_perm_move[cperm_x as usize][cm as usize];
                mperm_x = tab.m_perm_move[mperm_x as usize][cm as usize];
            }
            let ct_move = cc2l::M_ON_CT[m];
            if ct_move != -1 {
                ct_x = tab.ct_move[ct as usize][ct_move as usize];
            }
            if ct_x >= 3 {
                let conj = cc2l::CT_STD_CONJ[ct_x as usize] as usize;
                eperm_x = tab.e_perm_conj[eperm_x as usize][conj];
                cperm_x = tab.c_perm_conj[cperm_x as usize][conj];
                mperm_x = tab.m_perm_conj[mperm_x as usize][conj];
                ct_x %= 3;
            }

            let p1 = tab.e_perm_mpc_cb_prun.as_ref().unwrap()[eperm_x as usize]
                [(tab.c_perm2_c_comb[cperm_x as usize] as usize * 24 + mperm_x as usize) * 9
                    + ct_x as usize * 3 + leg_x as usize] as i32;
            let p2 = tab.c_perm_m_perm_prun.as_ref().unwrap()[cperm_x as usize]
                [mperm_x as usize * 9 + ct_x as usize * 3 + leg_x as usize] as i32;
            let prun = p1.max(p2);
            let maxl_next = maxl - cc2l::M_COST[leg as usize][m];
            if prun > maxl_next {
                continue;
            }
            let ret = self.phase2(eperm_x, cperm_x, mperm_x, ct_x, leg_x, maxl_next, depth + 1, m as i32);
            if ret >= 0 {
                self.mov[depth as usize] = m as i32;
                return ret;
            }
        }
        -1
    }
}

// ===== Tests =====
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_allowed_moves_builds() {
        Search2L::init();
        let am = allowed_moves();
        // Java prints set.size() / allowedMoves.size(); just check non-empty.
        assert!(!am.map.is_empty());
    }
}
