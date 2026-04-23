//! 8×8 coefficient scan tables.
//!
//! MS-MPEG4 (like MPEG-4 Part 2 / H.263) reorders the 64 DCT
//! coefficients into 1-D using one of three scan orders depending on
//! the AC prediction direction chosen by the intra-block predictor.
//!
//! These orders are identical across MPEG-4 Part 2, H.263 Annex I, and
//! the MS-MPEG4 family — they describe the natural-to-zigzag mapping
//! of an 8×8 block and are not codec-specific. Reproduction does not
//! require attribution.

/// Natural to zigzag order: `ZIGZAG[i]` is the natural-order index
/// produced for scan position `i`.
#[rustfmt::skip]
pub const ZIGZAG: [usize; 64] = [
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
];

/// Alternate horizontal scan — used when AC prediction picks the top
/// neighbour (horizontal smoothness, so values run along rows).
#[rustfmt::skip]
pub const ALTERNATE_HORIZONTAL: [usize; 64] = [
     0,  1,  2,  3,  8,  9, 16, 17,
    10, 11,  4,  5,  6,  7, 15, 14,
    13, 12, 19, 18, 24, 25, 32, 33,
    26, 27, 20, 21, 22, 23, 28, 29,
    30, 31, 34, 35, 40, 41, 48, 49,
    42, 43, 36, 37, 38, 39, 44, 45,
    46, 47, 50, 51, 56, 57, 58, 59,
    52, 53, 54, 55, 60, 61, 62, 63,
];

/// Alternate vertical scan — used when AC prediction picks the left
/// neighbour (vertical smoothness, so values run down columns).
#[rustfmt::skip]
pub const ALTERNATE_VERTICAL: [usize; 64] = [
     0,  8, 16, 24,  1,  9,  2, 10,
    17, 25, 32, 40, 48, 56, 57, 49,
    41, 33, 26, 18,  3, 11,  4, 12,
    19, 27, 34, 42, 50, 58, 35, 43,
    51, 59, 20, 28,  5, 13,  6, 14,
    21, 29, 36, 44, 52, 60, 37, 45,
    53, 61, 22, 30,  7, 15, 23, 31,
    38, 46, 54, 62, 39, 47, 55, 63,
];

#[cfg(test)]
mod tests {
    use super::*;

    fn is_permutation_of_0_63(table: &[usize; 64]) -> bool {
        let mut seen = [false; 64];
        for &v in table {
            if v >= 64 || seen[v] {
                return false;
            }
            seen[v] = true;
        }
        seen.iter().all(|&s| s)
    }

    #[test]
    fn zigzag_is_permutation() {
        assert!(is_permutation_of_0_63(&ZIGZAG));
    }

    #[test]
    fn alternate_horizontal_is_permutation() {
        assert!(is_permutation_of_0_63(&ALTERNATE_HORIZONTAL));
    }

    #[test]
    fn alternate_vertical_is_permutation() {
        assert!(is_permutation_of_0_63(&ALTERNATE_VERTICAL));
    }

    #[test]
    fn zigzag_starts_at_dc() {
        assert_eq!(ZIGZAG[0], 0);
        assert_eq!(ALTERNATE_HORIZONTAL[0], 0);
        assert_eq!(ALTERNATE_VERTICAL[0], 0);
    }

    #[test]
    fn zigzag_classic_early_positions() {
        // 0 -> (0,0) DC
        // 1 -> (0,1)
        // 8 -> (1,0)
        // The classic zigzag after DC goes right, then diag-down-left.
        assert_eq!(ZIGZAG[1], 1); // (0,1)
        assert_eq!(ZIGZAG[2], 8); // (1,0)
        assert_eq!(ZIGZAG[3], 16); // (2,0)
        assert_eq!(ZIGZAG[4], 9); // (1,1)
        assert_eq!(ZIGZAG[5], 2); // (0,2)
    }
}
