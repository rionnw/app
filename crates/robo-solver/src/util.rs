//! Utility constants and helper functions for the min2phase solver.
//! Ported from Util.java.

use std::sync::OnceLock;

static STD2UD_TABLE: OnceLock<[i32; 18]> = OnceLock::new();
static CKMV2BIT_TABLE: OnceLock<[i32; 11]> = OnceLock::new();

// Moves
pub const UX1: u8 = 0;
pub const UX2: u8 = 1;
pub const UX3: u8 = 2;
pub const RX1: u8 = 3;
pub const RX2: u8 = 4;
pub const RX3: u8 = 5;
pub const FX1: u8 = 6;
pub const FX2: u8 = 7;
pub const FX3: u8 = 8;
pub const DX1: u8 = 9;
pub const DX2: u8 = 10;
pub const DX3: u8 = 11;
pub const LX1: u8 = 12;
pub const LX2: u8 = 13;
pub const LX3: u8 = 14;
pub const BX1: u8 = 15;
pub const BX2: u8 = 16;
pub const BX3: u8 = 17;

// Facelets
pub const U1: u8 = 0;
pub const U2: u8 = 1;
pub const U3: u8 = 2;
pub const U4: u8 = 3;
pub const U5: u8 = 4;
pub const U6: u8 = 5;
pub const U7: u8 = 6;
pub const U8: u8 = 7;
pub const U9: u8 = 8;
pub const R1: u8 = 9;
pub const R2: u8 = 10;
pub const R3: u8 = 11;
pub const R4: u8 = 12;
pub const R5: u8 = 13;
pub const R6: u8 = 14;
pub const R7: u8 = 15;
pub const R8: u8 = 16;
pub const R9: u8 = 17;
pub const F1: u8 = 18;
pub const F2: u8 = 19;
pub const F3: u8 = 20;
pub const F4: u8 = 21;
pub const F5: u8 = 22;
pub const F6: u8 = 23;
pub const F7: u8 = 24;
pub const F8: u8 = 25;
pub const F9: u8 = 26;
pub const D1: u8 = 27;
pub const D2: u8 = 28;
pub const D3: u8 = 29;
pub const D4: u8 = 30;
pub const D5: u8 = 31;
pub const D6: u8 = 32;
pub const D7: u8 = 33;
pub const D8: u8 = 34;
pub const D9: u8 = 35;
pub const L1: u8 = 36;
pub const L2: u8 = 37;
pub const L3: u8 = 38;
pub const L4: u8 = 39;
pub const L5: u8 = 40;
pub const L6: u8 = 41;
pub const L7: u8 = 42;
pub const L8: u8 = 43;
pub const L9: u8 = 44;
pub const B1: u8 = 45;
pub const B2: u8 = 46;
pub const B3: u8 = 47;
pub const B4: u8 = 48;
pub const B5: u8 = 49;
pub const B6: u8 = 50;
pub const B7: u8 = 51;
pub const B8: u8 = 52;
pub const B9: u8 = 53;

// Colors
pub const COLOR_U: u8 = 0;
pub const COLOR_R: u8 = 1;
pub const COLOR_F: u8 = 2;
pub const COLOR_D: u8 = 3;
pub const COLOR_L: u8 = 4;
pub const COLOR_B: u8 = 5;

/// Corner facelet positions. cornerFacelet[i] = [f1, f2, f3] for corner i.
pub static CORNER_FACELET: [[u8; 3]; 8] = [
    [U9, R1, F3], [U7, F1, L3], [U1, L1, B3], [U3, B1, R3],
    [D3, F9, R7], [D1, L9, F7], [D7, B9, L7], [D9, R9, B7],
];

/// Edge facelet positions. edgeFacelet[i] = [f1, f2] for edge i.
pub static EDGE_FACELET: [[u8; 2]; 12] = [
    [U6, R2], [U8, F2], [U4, L2], [U2, B2], [D6, R8], [D2, F8],
    [D4, L8], [D8, B8], [F6, R4], [F4, L6], [B6, L4], [B4, R6],
];

/// Binomial coefficients C(n, k) for n <= 12, k <= 12
pub static CNK: [[i32; 13]; 13] = {
    let mut table = [[0i32; 13]; 13];
    let mut i = 0;
    while i < 13 {
        table[i][0] = 1;
        table[i][i] = 1;
        let mut j = 1;
        while j < i {
            table[i][j] = table[i - 1][j - 1] + table[i - 1][j];
            j += 1;
        }
        i += 1;
    }
    table
};

/// Move names for string conversion.
pub static MOVE2STR: [&str; 18] = [
    "U ", "U2", "U'", "R ", "R2", "R'", "F ", "F2", "F'",
    "D ", "D2", "D'", "L ", "L2", "L'", "B ", "B2", "B'",
];

/// ud2std move mapping (phase 2 uses only 10 moves; this maps them to standard 18-move indices).
pub static UD2STD: [i32; 18] = [
    UX1 as i32, UX2 as i32, UX3 as i32, RX2 as i32, FX2 as i32,
    DX1 as i32, DX2 as i32, DX3 as i32, LX2 as i32, BX2 as i32,
    RX1 as i32, RX3 as i32, FX1 as i32, FX3 as i32, LX1 as i32, LX3 as i32, BX1 as i32, BX3 as i32,
];

/// Inverse mapping from standard move to ud-phase2 index.
pub fn std2ud() -> [i32; 18] {
    let mut s2u = [0i32; 18];
    for i in 0..18 {
        s2u[UD2STD[i] as usize] = i as i32;
    }
    s2u
}

/// Bitmask for checking if consecutive moves can be pruned.
pub fn ckmv2bit() -> [i32; 11] {
    let mut bits = [0i32; 11];
    for i in 0..10 {
        let ix = UD2STD[i] as usize / 3;
        bits[i] = 0;
        for j in 0..10 {
            let jx = UD2STD[j] as usize / 3;
            if ix == jx || (ix % 3 == jx % 3 && ix >= jx) {
                bits[i] |= 1 << j;
            }
        }
    }
    bits[10] = 0;
    bits
}

/// Get the std2ud lookup table (lazily initialized).
pub fn get_std2ud() -> &'static [i32; 18] {
    STD2UD_TABLE.get_or_init(std2ud)
}

/// Get the ckmv2bit lookup table (lazily initialized).
pub fn get_ckmv2bit() -> &'static [i32; 11] {
    CKMV2BIT_TABLE.get_or_init(ckmv2bit)
}

/// Convert facelet string to CubieCube representation.
/// `f` is a 54-byte array where each value is a color index (0-5).
/// Returns (ca, ea) arrays.
pub fn to_cubie_cube(f: &[u8; 54], ca: &mut [u8; 8], ea: &mut [u8; 12]) {
    *ca = [0; 8];
    *ea = [0; 12];

    // Corners
    for i in 0..8 {
        let mut ori: usize = 0;
        while ori < 3 {
            let col = f[CORNER_FACELET[i][ori] as usize];
            if col == COLOR_U || col == COLOR_D {
                break;
            }
            ori += 1;
        }
        let col1 = f[CORNER_FACELET[i][(ori + 1) % 3] as usize];
        let col2 = f[CORNER_FACELET[i][(ori + 2) % 3] as usize];
        for j in 0..8 {
            if col1 == CORNER_FACELET[j][1] / 9 && col2 == CORNER_FACELET[j][2] / 9 {
                ca[i] = ((ori as u8) % 3) << 3 | j as u8;
                break;
            }
        }
    }

    // Edges
    for i in 0..12 {
        for j in 0..12 {
            let f0 = f[EDGE_FACELET[i][0] as usize];
            let f1 = f[EDGE_FACELET[i][1] as usize];
            let e0 = EDGE_FACELET[j][0] / 9;
            let e1 = EDGE_FACELET[j][1] / 9;
            if f0 == e0 && f1 == e1 {
                ea[i] = (j as u8) << 1;
                break;
            }
            if f0 == e1 && f1 == e0 {
                ea[i] = (j as u8) << 1 | 1;
                break;
            }
        }
    }
}

/// Convert CubieCube to facelet string (54 characters).
pub fn to_face_cube(ca: &[u8; 8], ea: &[u8; 12]) -> String {
    const TS: [char; 6] = ['U', 'R', 'F', 'D', 'L', 'B'];
    // Start with the center colour for each face (1 char per facelet).
    let mut f: [char; 54] = std::array::from_fn(|i| TS[i / 9]);

    for c in 0..8 {
        let j = (ca[c] & 0x7) as usize;
        let ori = (ca[c] >> 3) as usize;
        for n in 0..3 {
            f[CORNER_FACELET[c][(n + ori) % 3] as usize] = TS[(CORNER_FACELET[j][n] / 9) as usize];
        }
    }

    for e in 0..12 {
        let j = (ea[e] >> 1) as usize;
        let ori = (ea[e] & 1) as usize;
        for n in 0..2 {
            f[EDGE_FACELET[e][(n + ori) % 2] as usize] = TS[(EDGE_FACELET[j][n] / 9) as usize];
        }
    }

    f.iter().collect()
}

/// Get the parity of a permutation index.
pub fn get_n_parity(mut idx: i32, n: i32) -> i32 {
    let mut p = 0;
    for i in (0..=(n - 2)).rev() {
        p ^= idx % (n - i);
        idx /= n - i;
    }
    p & 1
}

/// Helper: set value into corner/edge byte depending on type.
#[inline]
pub fn set_val(val0: u8, val: u8, is_edge: bool) -> u8 {
    if is_edge {
        val << 1 | (val0 & 1)
    } else {
        val | (val0 & 0xf8)
    }
}

/// Helper: get value from corner/edge byte depending on type.
#[inline]
pub fn get_val(val0: u8, is_edge: bool) -> u8 {
    if is_edge {
        val0 >> 1
    } else {
        val0 & 7
    }
}

/// Set the N-permutation into array arr at index idx.
/// This encodes a permutation of n elements into the array.
pub fn set_n_perm(arr: &mut [u8], mut idx: i32, n: i32, is_edge: bool) {
    let mut val: u64 = 0xFEDCBA9876543210u64;
    let mut extract: u64 = 0;
    for p in 2..=n {
        extract = extract << 4 | (idx % p) as u64;
        idx /= p;
    }
    for i in 0..(n - 1) as usize {
        let v = ((extract as u32) & 0xf) << 2;
        extract >>= 4;
        arr[i] = set_val(arr[i], ((val >> v) & 0xf) as u8, is_edge);
        let m = (1u64 << v).wrapping_sub(1);
        val = (val & m) | ((val >> 4) & !m);
    }
    arr[(n - 1) as usize] = set_val(arr[(n - 1) as usize], (val & 0xf) as u8, is_edge);
}

/// Get the N-permutation index from array arr.
pub fn get_n_perm(arr: &[u8], n: i32, is_edge: bool) -> i32 {
    let mut idx: i32 = 0;
    let mut val: u64 = 0xFEDCBA9876543210u64;
    for i in 0..(n - 1) as usize {
        let v = (get_val(arr[i], is_edge) as u32) << 2;
        idx = (n - i as i32) * idx + ((val >> v) & 0xf) as i32;
        val = val.wrapping_sub(0x1111111111111110u64 << v);
    }
    idx
}

/// Get the combination index (position of 4 elements matching mask).
pub fn get_comb(arr: &[u8], mask: i32, is_edge: bool) -> i32 {
    let end = arr.len() as i32 - 1;
    let mut idx_c: i32 = 0;
    let mut r: i32 = 4;
    for i in (0..=end).rev() {
        let perm = get_val(arr[i as usize], is_edge) as i32;
        if (perm & 0xc) == mask {
            idx_c += CNK[i as usize][r as usize];
            r -= 1;
        }
    }
    idx_c
}

/// Set the combination (place 4 elements matching mask at positions determined by idxC).
pub fn set_comb(arr: &mut [u8], mut idx_c: i32, mask: i32, is_edge: bool) {
    let end = arr.len() as i32 - 1;
    let mut r: i32 = 4;
    let mut fill = end;
    for i in (0..=end).rev() {
        if idx_c >= CNK[i as usize][r as usize] {
            idx_c -= CNK[i as usize][r as usize];
            r -= 1;
            arr[i as usize] = set_val(arr[i as usize], (r | mask) as u8, is_edge);
        } else {
            if (fill & 0xc) == mask {
                fill -= 4;
            }
            arr[i as usize] = set_val(arr[i as usize], fill as u8, is_edge);
            fill -= 1;
        }
    }
}

/// Parse a facelet string (54 chars of "URFDLB") into a color index array.
pub fn parse_facelets(s: &str) -> Option<[u8; 54]> {
    if s.len() != 54 {
        return None;
    }
    let mut f = [0u8; 54];
    for (i, &b) in s.as_bytes().iter().enumerate() {
        f[i] = match b {
            b'U' => COLOR_U,
            b'R' => COLOR_R,
            b'F' => COLOR_F,
            b'D' => COLOR_D,
            b'L' => COLOR_L,
            b'B' => COLOR_B,
            _ => return None,
        };
    }
    Some(f)
}

// ==================== High-level tools ====================

use crate::cubie_cube::{self, CubieCube};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::sync::{LazyLock, Mutex};

/// Process-wide RNG. Defaults to a fresh entropy-seeded `StdRng`; callers can
/// override it via [`set_random_source`] for deterministic test runs.
static RNG: LazyLock<Mutex<StdRng>> = LazyLock::new(|| Mutex::new(StdRng::from_entropy()));

/// Set the random source with a specific seed (for deterministic results).
pub fn set_random_source(seed: u64) {
    *RNG.lock().unwrap() = StdRng::seed_from_u64(seed);
}

/// Generate a random cube facelet string.
pub fn random_cube() -> String {
    let mut rng = RNG.lock().unwrap();
    random_state_internal(&mut *rng)
}

fn random_state_internal<R: Rng>(rng: &mut R) -> String {
    let mut cc = CubieCube::new();
    cc.set_c_perm(rng.gen_range(0..40320));
    cc.set_e_perm(rng.gen_range(0..40320));

    if get_n_parity(cc.get_c_perm(), 8) != get_n_parity(cc.get_e_perm(), 8) {
        cc.ea.swap(0, 1);
    }

    cc.set_twist(rng.gen_range(0..2187));
    cc.set_flip(rng.gen_range(0..2048));

    to_face_cube(&cc.ca, &cc.ea)
}

/// Convert a scramble string (e.g., "R U2 F' L2 D B2") to a facelet string.
pub fn from_scramble(s: &str) -> String {
    from_scramble_moves(&parse_moves(s))
}

/// Convert a sequence of move indices to a facelet string.
pub fn from_scramble_moves(moves: &[i32]) -> String {
    let ct = cubie_cube::get_tables();
    let mut cc = CubieCube::new();

    for &m in moves {
        if !(0..18).contains(&m) {
            continue;
        }
        let mut prod = CubieCube::new();
        CubieCube::corn_mult(&cc, &ct.move_cube[m as usize], &mut prod);
        CubieCube::edge_mult(&cc, &ct.move_cube[m as usize], &mut prod);
        cc = prod;
    }

    to_face_cube(&cc.ca, &cc.ea)
}

fn parse_moves(s: &str) -> Vec<i32> {
    s.split_whitespace()
        .map(parse_single_move)
        .filter(|&m| m >= 0)
        .collect()
}

fn parse_single_move(s: &str) -> i32 {
    let bytes = s.as_bytes();
    let face = match bytes.first() {
        Some(b'U') => 0,
        Some(b'R') => 3,
        Some(b'F') => 6,
        Some(b'D') => 9,
        Some(b'L') => 12,
        Some(b'B') => 15,
        _ => return -1,
    };
    let modifier = match bytes.get(1) {
        Some(b'2') => 1,
        Some(b'\'') | Some(b'3') => 2,
        _ => 0,
    };
    face + modifier
}

/// Verify a facelet string for solvability.
/// Returns 0 if valid, negative error code otherwise.
pub fn verify(facelets: &str) -> i32 {
    let Some(f) = parse_facelets(facelets) else {
        return -1;
    };
    let mut ca = [0u8; 8];
    let mut ea = [0u8; 12];
    to_cubie_cube(&f, &mut ca, &mut ea);
    CubieCube { ca, ea }.verify()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cnk() {
        assert_eq!(CNK[0][0], 1);
        assert_eq!(CNK[4][2], 6);
        assert_eq!(CNK[12][4], 495);
        assert_eq!(CNK[12][6], 924);
    }

    #[test]
    fn test_std2ud_roundtrip() {
        let s2u = get_std2ud();
        for i in 0..18 {
            assert_eq!(s2u[UD2STD[i] as usize], i as i32);
        }
    }

    #[test]
    fn test_parse_facelets() {
        let solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
        let f = parse_facelets(solved).unwrap();
        assert_eq!(f[0], COLOR_U);
        assert_eq!(f[9], COLOR_R);
        assert_eq!(f[18], COLOR_F);
        assert_eq!(f[27], COLOR_D);
        assert_eq!(f[36], COLOR_L);
        assert_eq!(f[45], COLOR_B);
    }

    #[test]
    fn test_solved_cube_roundtrip() {
        let solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
        let f = parse_facelets(solved).unwrap();
        let mut ca = [0u8; 8];
        let mut ea = [0u8; 12];
        to_cubie_cube(&f, &mut ca, &mut ea);
        let result = to_face_cube(&ca, &ea);
        assert_eq!(result, solved);
    }

    #[test]
    fn test_n_perm_roundtrip() {
        // Test with edge permutation (12 elements)
        let mut arr = [0u8; 12];
        set_n_perm(&mut arr, 0, 12, true);
        assert_eq!(get_n_perm(&arr, 12, true), 0);

        set_n_perm(&mut arr, 100, 12, true);
        assert_eq!(get_n_perm(&arr, 12, true), 100);

        // Test with corner permutation (8 elements)
        let mut arr2 = [0u8; 8];
        set_n_perm(&mut arr2, 5039, 8, false); // 8! - 1 = max index
        assert_eq!(get_n_perm(&arr2, 8, false), 5039);
    }

    #[test]
    fn test_get_n_parity() {
        assert_eq!(get_n_parity(0, 8), 0);
        assert_eq!(get_n_parity(1, 8), 1);
    }
}
