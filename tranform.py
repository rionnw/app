"""
Transform 2L solver output into hardware motor commands for a two-arm robot.

Hardware model:
  - L arm: controls U-face rotation
  - R arm: controls R-face rotation
  - Each arm has: 1 rotation motor + 1 claw (open/close)
  - Total: 4 actuators (L_rotate, L_claw, R_rotate, R_claw)

Leg states represent ARM POSITION (not claw state!):
  - leg=0 (pp): both arms at home position
  - leg=1 (vp): L arm displaced (rotated but not returned), R arm at home
  - leg=2 (pv): L arm at home, R arm displaced

In ALL leg states, both claws are CLOSED (holding the cube).
Claws only open temporarily DURING a move (e.g., for whole-cube rotation).

Encoding: (L_action R_action)
  z0 = arm doesn't rotate
  z1 = arm rotates +90°
  z2 = arm rotates 180°
  z3 = arm rotates -90° (270°)
  s0 = that side's claw OPENS (releases cube so other arm can spin whole cube)
  s1 = that side's claw TOGGLES (used for regrab/transfer operations)
"""

# MOVE2STR patterns for parsing
MOVE2STR = [
    "(z1z0) U ",  "(z2z0) U2", "(z3z0) U'",
    "(z0z1) R ",  "(z0z2) R2", "(z0z3) R'",
    "(z1s0) y ",  "(z2s0) y2", "(z3s0) y'",
    "(s0z1) x ",  "(s0z2) x2", "(s0z3) x'",
    "(z0s1)   ",  "(s1z0)   ",
    "(z1s1) y ",  "(z2s1) y2", "(z3s1) y'",
    "(s1z1) x ",  "(s1z2) x2", "(s1z3) x'",
]

# Leg state transitions
NEXT_STATE = [
    [ 1,  0,  1,  2,  0,  2,  1,  0,  1,  2,  0,  2,  2,  1, -1,  2, -1, -1,  1, -1],  # pp
    [ 0,  1,  0, -1, -1, -1,  0,  1,  0, -1,  1, -1, -1,  0,  2, -1,  2,  2,  0,  2],  # vp
    [-1, -1, -1,  0,  2,  0, -1,  2, -1,  0,  2,  0,  0, -1,  1,  0,  1,  1, -1,  1],  # pv
]

# For each move, define the fixed motor command sequence.
# Each move produces a list of elementary commands:
#   "CL0" = L claw open,  "CL1" = L claw close
#   "CR0" = R claw open,  "CR1" = R claw close
#   "L+1" = L rotate +90, "L+2" = L rotate 180, "L-1" = L rotate -90
#   "R+1" = R rotate +90, "R+2" = R rotate 180, "R-1" = R rotate -90

def _make_move_sequences():
    """
    Build the motor sequence for each of the 20 moves.
    
    Rules:
      - z1/z2/z3 on L side: L arm rotates +90/180/-90
      - z1/z2/z3 on R side: R arm rotates +90/180/-90
      - s0 on R side: R claw opens before L rotates, then R claw closes after
      - s0 on L side: L claw opens before R rotates, then L claw closes after
      - s1 on R side: R claw toggles (open→close or close→open); for moves with
        rotation this means: open R → rotate L → close R (if coming from closed)
        or: close R → rotate L → open R (if coming from open)
      - s1 on L side: L claw toggles similarly
    
    The tricky part: s1 (toggle) depends on the CURRENT state of the claw,
    which is always CLOSED at the start/end of a move (between moves, both
    claws are closed). So s1 from a "between-moves" perspective always means:
      - From pp: claw is closed → toggle = open (same as s0 but may stay open)
      - But NEXT_STATE shows that after the move, we're in a new leg state
        where both claws end up closed again.
    
    Actually examining NEXT_STATE more carefully:
      move[12] (z0s1): pp→pv (cost=3)  
      move[13] (s1z0): pp→vp (cost=3)
    
    These "grip" moves with cost=3 suggest 3 steps:
      move[12]: L claw open → (nothing, maybe R adjusts grip) → L claw close
      But that makes pp→pv which is "R displaced"... 
    
    Let me re-examine: perhaps pp/vp/pv means which arm was the LAST to move
    (and thus is displaced). The state just tracks "who moved last" for the
    purpose of determining which subsequent moves are legal.
    
    For hardware translation, what matters is:
      - Moves 0-5: pure single-layer rotation, just one motor step
      - Moves 6-11 (s0): open one claw, rotate other arm (whole cube), close claw
      - Moves 12-13 (pure s1): "regrab" — open one, adjust, close
      - Moves 14-19 (s1+rotation): toggle claw + rotate
    """
    
    L_ANGLES = {0: None, 1: "+90", 2: "+180", 3: "-90"}
    R_ANGLES = {0: None, 1: "+90", 2: "+180", 3: "-90"}
    
    sequences = {}
    
    # Move 0-2: (z1z0)U, (z2z0)U2, (z3z0)U'
    # Both claws closed, L rotates → single layer U
    for i, lz in enumerate([1, 2, 3]):
        sequences[i] = [f"ROTATE_L({L_ANGLES[lz]});"]
    
    # Move 3-5: (z0z1)R, (z0z2)R2, (z0z3)R'
    # Both claws closed, R rotates → single layer R
    for i, rz in enumerate([1, 2, 3]):
        sequences[3 + i] = [f"ROTATE_R({R_ANGLES[rz]});"]
    
    # Move 6-8: (z1s0)y, (z2s0)y2, (z3s0)y'
    # R claw opens, L rotates whole cube, R claw closes
    for i, lz in enumerate([1, 2, 3]):
        sequences[6 + i] = [
            "CLAW_R(0);",
            f"ROTATE_L({L_ANGLES[lz]});",
            "CLAW_R(1);",
        ]
    
    # Move 9-11: (s0z1)x, (s0z2)x2, (s0z3)x'
    # L claw opens, R rotates whole cube, L claw closes
    for i, rz in enumerate([1, 2, 3]):
        sequences[9 + i] = [
            "CLAW_L(0);",
            f"ROTATE_R({R_ANGLES[rz]});",
            "CLAW_L(1);",
        ]
    
    # Move 12: (z0s1) — R side toggles (R claw open then close = regrab)
    sequences[12] = [
        "CLAW_R(0);",
        "CLAW_R(1);",
    ]
    
    # Move 13: (s1z0) — L side toggles (L claw open then close = regrab)
    sequences[13] = [
        "CLAW_L(0);",
        "CLAW_L(1);",
    ]
    
    # Move 14-16: (z1s1)y, (z2s1)y2, (z3s1)y'
    # R claw toggles + L rotates. From examining costs (4 steps):
    # If R is currently closed (between-moves default): open R → rotate L → close R → ?
    # Actually cost=4 suggests: close_other + open + rotate + close
    # Most likely: same as s0 (open R, L rotates whole, close R) but with an
    # extra step because it's a "toggle" from a different starting leg state.
    # From pp: move 14 is INVALID (NEXT_STATE[pp][14]=-1)
    # From vp: move 14 → leg=2. From pv: move 14 → leg=1.
    # So these only execute from vp or pv states.
    # From vp (L displaced): R opens, L rotates whole (also re-centering), R closes
    # The extra cost compared to s0 moves (3→4) likely comes from needing an
    # additional grip adjustment step.
    for i, lz in enumerate([1, 2, 3]):
        sequences[14 + i] = [
            "CLAW_R(0);",
            f"ROTATE_L({L_ANGLES[lz]});",
            "CLAW_R(1);",
        ]
    
    # Move 17-19: (s1z1)x, (s1z2)x2, (s1z3)x'
    # L claw toggles + R rotates
    for i, rz in enumerate([1, 2, 3]):
        sequences[17 + i] = [
            "CLAW_L(0);",
            f"ROTATE_R({R_ANGLES[rz]});",
            "CLAW_L(1);",
        ]
    
    return sequences

MOVE_SEQUENCES = _make_move_sequences()


def parse_solution(solution_str: str) -> list[int]:
    """Parse solver output string into move indices."""
    moves = []
    s = solution_str.strip()
    while s:
        s = s.lstrip()
        if not s:
            break
        found = False
        for idx, pattern in enumerate(MOVE2STR):
            pat = pattern.rstrip()
            if s.startswith(pat):
                moves.append(idx)
                s = s[len(pat):]
                found = True
                break
        if not found:
            s = s[1:]
    return moves


def moves_to_hardware(move_indices: list[int]) -> list[str]:
    """Convert move indices to hardware command sequence."""
    commands = []
    leg = 0

    commands.append("CLAW_L(1); CLAW_R(1);  // init: both claws closed")

    for step, m in enumerate(move_indices):
        next_leg = NEXT_STATE[leg][m]
        assert next_leg != -1, f"Invalid move {m} in leg state {leg} at step {step+1}"

        desc = MOVE2STR[m].strip()
        commands.append(f"// step {step+1}: [{m}] {desc}  (leg {leg}→{next_leg})")
        commands.extend(MOVE_SEQUENCES[m])

        leg = next_leg

    return commands


if __name__ == "__main__":
    raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U' (z0z3) R' (z2s1) y2 (z0z2) R2 (z1z0) U  (z1s1) y  (z0z1) R  (z3z0) U' (s1z3) x' (z0z1) R  (z2z0) U2 (z0z1) R  (z2s0) y2 (z0z3) R' (z3z0) U' (s1z3) x' (z0z2) R2 (z2s0) y2 (z0z2) R2 (z1s1) y  (z1z0) U  (z0z2) R2 (s1z2) x2 (z1z0) U  (z0z2) R2 (s1z0)    (z1s0) y  (z0z2) R2"

    print(f"Input:\n  {raw}\n")
    moves = parse_solution(raw)
    print(f"Parsed {len(moves)} moves: {moves}\n")

    cmds = moves_to_hardware(moves)
    print("Hardware commands:")
    for c in cmds:
        print(f"  {c}")

    # Count total motor operations
    ops = sum(1 for c in cmds if not c.startswith("//") and c.strip())
    print(f"\nTotal motor operations: {ops}")
