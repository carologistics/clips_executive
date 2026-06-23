;; Multi-level wooden-blocks domain (Kulshrestha CoRL-2023 — redesigned 2026-05).
;;
;; Two-table workspace:
;;
;;   * ``parts`` — 2D :class:`GridRegion` behind the robot.  Blocks rest on
;;     this surface; no stacking here.
;;   * ``stack_L0`` … ``stack_L4`` — five 2D regions on the stack table in
;;     front of the robot, one per vertical level.  The 3D stack grid lives
;;     here.  Cells in ``stack_L0`` are at table-top z; ``stack_L<k>`` cells
;;     sit one cube-size higher than ``stack_L<k-1>``.
;;
;; Block shapes:
;;
;;   * 1×1 cube (``(cube ?b)``): occupies one cell.
;;   * 2×1 oblong (``(oblong ?b)``): occupies two cells.  Three valid
;;     orientations:
;;       - flat-x: two cells along +x in the same level (``east-of c1 c2``)
;;       - flat-y: two cells along +y in the same level (``north-of c1 c2``)
;;       - upright: two cells at the same (ix,iy) one level apart
;;         (``above c_low c_high``).  Upright only valid on the stack table.
;;
;; Grasps:
;;
;;   * pick-cube: top-down, centred.
;;   * pick-flat-x / pick-flat-y: top-down on the long sides of the oblong.
;;   * pick-upright: front-facing horizontal grasp on the vertical sides.
;;
;; In-hand transforms (no MP collision risk — performed in free air):
;;
;;   * make-upright-from-{x,y}: tilt the held block from horizontal to vertical.
;;   * make-flat-{x,y}-from-upright: reverse tilt.
;;   * turn-x-to-y / turn-y-to-x: yaw 90 deg in the horizontal plane.

(define (domain multilevel-blocks)
  (:requirements
    :strips :typing :negative-preconditions
    :disjunctive-preconditions :existential-preconditions
  )

  (:types
    cell block
  )

  (:predicates
    ;; --- static (set in initial state of every problem) ---
    (cube ?b - block)
    (oblong ?b - block)
    (long ?b - block)                         ; 3×1 long block — same
                                              ; shape family as oblong but
                                              ; 3 cells along the long axis;
                                              ; support rule is (centre
                                              ; alone) OR (both ends), so
                                              ; only single-end-cantilever
                                              ; layouts are rejected (vs
                                              ; oblong's "every cell must
                                              ; be supported").
    (in-parts ?c - cell)
    (in-stack ?c - cell)
    ;; Static: ``puttable`` is True for every cell that can be a put-* target.
    ;; In Phase 1 of the multilevel_blocks redesign this is set to be True
    ;; for every ``stack_L*`` cell and False (i.e., not asserted) for every
    ;; ``parts__*`` cell — the parts table is now strictly a pick source.
    ;; Eliminates the action-space bloat of ``put-* ?b parts__X_Y`` actions
    ;; from rgnet's GNN expansion at every state.
    (puttable ?c - cell)
    ;; directional adjacency — asymmetric.  3 predicates cover all 6
    ;; geometric directions: the "reverse" direction is the same predicate
    ;; with parameters swapped.
    (above ?c-low - cell ?c-up - cell)        ; +z
    (east-of ?c1 - cell ?c2 - cell)           ; +x; c2 is east of c1
    (north-of ?c1 - cell ?c2 - cell)          ; +y; c2 is north of c1

    ;; --- fluent ---
    (in ?b - block ?c - cell)                 ; block occupies cell (multi-
                                              ; cell blocks have multiple facts)
    (empty ?c - cell)
    (gripper-empty)
    (held-cube ?b - block)
    (held-flat-x ?b - block)
    (held-flat-y ?b - block)
    (held-upright ?b - block)
  )

  ;; ====================================================================
  ;; PICK actions
  ;; ====================================================================

  ;; Pick a cube.  The cell directly above (if any) must be empty so the
  ;; cube being picked isn't supporting another block.
  (:action pick-cube
    :parameters (?b - block ?c - cell)
    :precondition (and
      (cube ?b) (in ?b ?c) (gripper-empty)
      (not (exists (?c-above - cell)
              (and (above ?c ?c-above) (not (empty ?c-above)))))
    )
    :effect (and
      (not (in ?b ?c)) (empty ?c)
      (held-cube ?b) (not (gripper-empty))
    )
  )

  ;; Pick a flat-x oblong (block spans c1, c2 with c2 east of c1).
  (:action pick-flat-x
    :parameters (?b - block ?c1 - cell ?c2 - cell)
    :precondition (and
      (oblong ?b) (in ?b ?c1) (in ?b ?c2) (east-of ?c1 ?c2)
      (gripper-empty)
      (not (exists (?ca - cell)
              (and (above ?c1 ?ca) (not (empty ?ca)))))
      (not (exists (?ca - cell)
              (and (above ?c2 ?ca) (not (empty ?ca)))))
    )
    :effect (and
      (not (in ?b ?c1)) (not (in ?b ?c2))
      (empty ?c1) (empty ?c2)
      (held-flat-x ?b) (not (gripper-empty))
    )
  )

  ;; Pick a flat-y oblong (block spans c1, c2 with c2 north of c1).
  (:action pick-flat-y
    :parameters (?b - block ?c1 - cell ?c2 - cell)
    :precondition (and
      (oblong ?b) (in ?b ?c1) (in ?b ?c2) (north-of ?c1 ?c2)
      (gripper-empty)
      (not (exists (?ca - cell)
              (and (above ?c1 ?ca) (not (empty ?ca)))))
      (not (exists (?ca - cell)
              (and (above ?c2 ?ca) (not (empty ?ca)))))
    )
    :effect (and
      (not (in ?b ?c1)) (not (in ?b ?c2))
      (empty ?c1) (empty ?c2)
      (held-flat-y ?b) (not (gripper-empty))
    )
  )

  ;; Pick an upright oblong (block spans c-low, c-high with c-high above c-low).
  ;; Only valid on the stack table.  The cell directly above c-high (if any)
  ;; must be empty.
  (:action pick-upright
    :parameters (?b - block ?c-low - cell ?c-high - cell)
    :precondition (and
      (oblong ?b) (in ?b ?c-low) (in ?b ?c-high) (above ?c-low ?c-high)
      (gripper-empty)
      (not (exists (?ca - cell)
              (and (above ?c-high ?ca) (not (empty ?ca)))))
    )
    :effect (and
      (not (in ?b ?c-low)) (not (in ?b ?c-high))
      (empty ?c-low) (empty ?c-high)
      (held-upright ?b) (not (gripper-empty))
    )
  )

  ;; ====================================================================
  ;; PUT actions
  ;; ====================================================================

  ;; Put a cube.  Cell c must be empty and EITHER at table-level (no cell
  ;; below) OR directly above an occupied cell.
  (:action put-cube
    :parameters (?b - block ?c - cell)
    :precondition (and
      (cube ?b) (held-cube ?b) (empty ?c)
      (puttable ?c)
      (or
        (not (exists (?cb - cell) (above ?cb ?c)))
        (exists (?cb - cell)
           (and (above ?cb ?c) (not (empty ?cb)))))
    )
    :effect (and
      (in ?b ?c) (not (empty ?c))
      (not (held-cube ?b)) (gripper-empty)
    )
  )

  ;; Put a held-flat-x oblong at c1, c2 (c2 east of c1, same level).
  ;; Both cells must be empty and supported (table-level OR cell below
  ;; occupied).
  (:action put-flat-x
    :parameters (?b - block ?c1 - cell ?c2 - cell)
    :precondition (and
      (oblong ?b) (held-flat-x ?b) (east-of ?c1 ?c2)
      (empty ?c1) (empty ?c2)
      (puttable ?c1) (puttable ?c2)
      (or
        (not (exists (?cb - cell) (above ?cb ?c1)))
        (exists (?cb - cell)
           (and (above ?cb ?c1) (not (empty ?cb)))))
      (or
        (not (exists (?cb - cell) (above ?cb ?c2)))
        (exists (?cb - cell)
           (and (above ?cb ?c2) (not (empty ?cb)))))
    )
    :effect (and
      (in ?b ?c1) (in ?b ?c2)
      (not (empty ?c1)) (not (empty ?c2))
      (not (held-flat-x ?b)) (gripper-empty)
    )
  )

  ;; Put a held-flat-y oblong at c1, c2 (c2 north of c1, same level).
  (:action put-flat-y
    :parameters (?b - block ?c1 - cell ?c2 - cell)
    :precondition (and
      (oblong ?b) (held-flat-y ?b) (north-of ?c1 ?c2)
      (empty ?c1) (empty ?c2)
      (puttable ?c1) (puttable ?c2)
      (or
        (not (exists (?cb - cell) (above ?cb ?c1)))
        (exists (?cb - cell)
           (and (above ?cb ?c1) (not (empty ?cb)))))
      (or
        (not (exists (?cb - cell) (above ?cb ?c2)))
        (exists (?cb - cell)
           (and (above ?cb ?c2) (not (empty ?cb)))))
    )
    :effect (and
      (in ?b ?c1) (in ?b ?c2)
      (not (empty ?c1)) (not (empty ?c2))
      (not (held-flat-y ?b)) (gripper-empty)
    )
  )

  ;; Put a held-upright oblong at c-low, c-high (c-high above c-low).
  ;; Only c-low needs the support check; c-high inherits support from
  ;; being directly above c-low (which the block itself will occupy).
  (:action put-upright
    :parameters (?b - block ?c-low - cell ?c-high - cell)
    :precondition (and
      (oblong ?b) (held-upright ?b) (above ?c-low ?c-high)
      (empty ?c-low) (empty ?c-high)
      (puttable ?c-low) (puttable ?c-high)
      (or
        (not (exists (?cb - cell) (above ?cb ?c-low)))
        (exists (?cb - cell)
           (and (above ?cb ?c-low) (not (empty ?cb)))))
    )
    :effect (and
      (in ?b ?c-low) (in ?b ?c-high)
      (not (empty ?c-low)) (not (empty ?c-high))
      (not (held-upright ?b)) (gripper-empty)
    )
  )

  ;; ====================================================================
  ;; In-hand TRANSFORM actions (free-air gripper rotation)
  ;; ====================================================================
  ;; No MP collision check needed — gripper rotates while held above the
  ;; workspace.  Bridge feasibility verifies the rotation actually clears
  ;; the air space (e.g., the held block doesn't sweep through anything).

  (:action make-upright-from-x
    :parameters (?b - block)
    :precondition (and (oblong ?b) (held-flat-x ?b))
    :effect (and (not (held-flat-x ?b)) (held-upright ?b))
  )

  (:action make-upright-from-y
    :parameters (?b - block)
    :precondition (and (oblong ?b) (held-flat-y ?b))
    :effect (and (not (held-flat-y ?b)) (held-upright ?b))
  )

  (:action make-flat-x-from-upright
    :parameters (?b - block)
    :precondition (and (oblong ?b) (held-upright ?b))
    :effect (and (not (held-upright ?b)) (held-flat-x ?b))
  )

  (:action make-flat-y-from-upright
    :parameters (?b - block)
    :precondition (and (oblong ?b) (held-upright ?b))
    :effect (and (not (held-upright ?b)) (held-flat-y ?b))
  )

  (:action turn-x-to-y
    :parameters (?b - block)
    :precondition (and (oblong ?b) (held-flat-x ?b))
    :effect (and (not (held-flat-x ?b)) (held-flat-y ?b))
  )

  (:action turn-y-to-x
    :parameters (?b - block)
    :precondition (and (oblong ?b) (held-flat-y ?b))
    :effect (and (not (held-flat-y ?b)) (held-flat-x ?b))
  )

  ;; ====================================================================
  ;; 3×1 LONG block actions
  ;; ====================================================================
  ;; The long block reuses the held-flat-{x,y} / held-upright fluents;
  ;; the (long ?b) static predicate selects between the oblong and long
  ;; action variants.  Pick/put preconditions span 3 cells.  Put-long-{x,y}
  ;; uses a permissive support rule: the CENTRE alone suffices (the long
  ;; balances on its midpoint) OR both ENDS together suffice (the long
  ;; bridges across an empty centre).  Only single-end-cantilever layouts
  ;; are rejected.

  (:action pick-long-x
    :parameters (?b - block ?c1 - cell ?c2 - cell ?c3 - cell)
    :precondition (and
      (long ?b) (in ?b ?c1) (in ?b ?c2) (in ?b ?c3)
      (east-of ?c1 ?c2) (east-of ?c2 ?c3)
      (gripper-empty)
      (not (exists (?ca - cell)
              (and (above ?c1 ?ca) (not (empty ?ca)))))
      (not (exists (?ca - cell)
              (and (above ?c2 ?ca) (not (empty ?ca)))))
      (not (exists (?ca - cell)
              (and (above ?c3 ?ca) (not (empty ?ca)))))
    )
    :effect (and
      (not (in ?b ?c1)) (not (in ?b ?c2)) (not (in ?b ?c3))
      (empty ?c1) (empty ?c2) (empty ?c3)
      (held-flat-x ?b) (not (gripper-empty))
    )
  )

  (:action pick-long-y
    :parameters (?b - block ?c1 - cell ?c2 - cell ?c3 - cell)
    :precondition (and
      (long ?b) (in ?b ?c1) (in ?b ?c2) (in ?b ?c3)
      (north-of ?c1 ?c2) (north-of ?c2 ?c3)
      (gripper-empty)
      (not (exists (?ca - cell)
              (and (above ?c1 ?ca) (not (empty ?ca)))))
      (not (exists (?ca - cell)
              (and (above ?c2 ?ca) (not (empty ?ca)))))
      (not (exists (?ca - cell)
              (and (above ?c3 ?ca) (not (empty ?ca)))))
    )
    :effect (and
      (not (in ?b ?c1)) (not (in ?b ?c2)) (not (in ?b ?c3))
      (empty ?c1) (empty ?c2) (empty ?c3)
      (held-flat-y ?b) (not (gripper-empty))
    )
  )

  (:action pick-long-upright
    :parameters (?b - block ?c-low - cell ?c-mid - cell ?c-high - cell)
    :precondition (and
      (long ?b) (in ?b ?c-low) (in ?b ?c-mid) (in ?b ?c-high)
      (above ?c-low ?c-mid) (above ?c-mid ?c-high)
      (gripper-empty)
      (not (exists (?ca - cell)
              (and (above ?c-high ?ca) (not (empty ?ca)))))
    )
    :effect (and
      (not (in ?b ?c-low)) (not (in ?b ?c-mid)) (not (in ?b ?c-high))
      (empty ?c-low) (empty ?c-mid) (empty ?c-high)
      (held-upright ?b) (not (gripper-empty))
    )
  )

  ;; Put a held flat-x long block at c1, c2, c3 (east-of chain).  Each
  ;; of c1, c2, c3 must be empty.  Support rule: the CENTRE cell c2
  ;; supported alone suffices (the long balances on its midpoint), OR
  ;; both ENDS c1 and c3 supported (the long bridges across an empty
  ;; centre).  Any single-end-only configuration (just c1, just c3) is
  ;; rejected because the block would cantilever and topple.  Note:
  ;; centre-only covers (c2), (c1+c2), (c2+c3), (c1+c2+c3); ends-only
  ;; covers (c1+c3); the only excluded configurations are (c1 only),
  ;; (c3 only), and (none).
  (:action put-long-x
    :parameters (?b - block ?c1 - cell ?c2 - cell ?c3 - cell)
    :precondition (and
      (long ?b) (held-flat-x ?b)
      (east-of ?c1 ?c2) (east-of ?c2 ?c3)
      (empty ?c1) (empty ?c2) (empty ?c3)
      (puttable ?c1) (puttable ?c2) (puttable ?c3)
      (or
        ;; Centre c2 supported (covers centre-only, 2/3 with centre,
        ;; and full 3/3).
        (or (not (exists (?cb - cell) (above ?cb ?c2)))
            (exists (?cb - cell)
               (and (above ?cb ?c2) (not (empty ?cb)))))
        ;; Both ends c1 AND c3 supported (bridge / span, c2 empty
        ;; below is OK).
        (and
          (or (not (exists (?cb - cell) (above ?cb ?c1)))
              (exists (?cb - cell)
                 (and (above ?cb ?c1) (not (empty ?cb)))))
          (or (not (exists (?cb - cell) (above ?cb ?c3)))
              (exists (?cb - cell)
                 (and (above ?cb ?c3) (not (empty ?cb)))))))
    )
    :effect (and
      (in ?b ?c1) (in ?b ?c2) (in ?b ?c3)
      (not (empty ?c1)) (not (empty ?c2)) (not (empty ?c3))
      (not (held-flat-x ?b)) (gripper-empty)
    )
  )

  ;; Put a held flat-y long block at c1, c2, c3 (north-of chain).
  ;; Same (centre-alone) OR (both-ends) support rule as put-long-x.
  (:action put-long-y
    :parameters (?b - block ?c1 - cell ?c2 - cell ?c3 - cell)
    :precondition (and
      (long ?b) (held-flat-y ?b)
      (north-of ?c1 ?c2) (north-of ?c2 ?c3)
      (empty ?c1) (empty ?c2) (empty ?c3)
      (puttable ?c1) (puttable ?c2) (puttable ?c3)
      (or
        ;; Centre c2 supported.
        (or (not (exists (?cb - cell) (above ?cb ?c2)))
            (exists (?cb - cell)
               (and (above ?cb ?c2) (not (empty ?cb)))))
        ;; Both ends c1 AND c3 supported.
        (and
          (or (not (exists (?cb - cell) (above ?cb ?c1)))
              (exists (?cb - cell)
                 (and (above ?cb ?c1) (not (empty ?cb)))))
          (or (not (exists (?cb - cell) (above ?cb ?c3)))
              (exists (?cb - cell)
                 (and (above ?cb ?c3) (not (empty ?cb)))))))
    )
    :effect (and
      (in ?b ?c1) (in ?b ?c2) (in ?b ?c3)
      (not (empty ?c1)) (not (empty ?c2)) (not (empty ?c3))
      (not (held-flat-y ?b)) (gripper-empty)
    )
  )

  ;; Put a held upright long block at c-low, c-mid, c-high (above chain).
  ;; Only c-low needs the support check — c-mid and c-high are supported
  ;; by the block itself once c-low is anchored.  Same single-cell
  ;; support rule as oblong put-upright.
  (:action put-long-upright
    :parameters (?b - block ?c-low - cell ?c-mid - cell ?c-high - cell)
    :precondition (and
      (long ?b) (held-upright ?b)
      (above ?c-low ?c-mid) (above ?c-mid ?c-high)
      (empty ?c-low) (empty ?c-mid) (empty ?c-high)
      (puttable ?c-low) (puttable ?c-mid) (puttable ?c-high)
      (or
        (not (exists (?cb - cell) (above ?cb ?c-low)))
        (exists (?cb - cell)
           (and (above ?cb ?c-low) (not (empty ?cb)))))
    )
    :effect (and
      (in ?b ?c-low) (in ?b ?c-mid) (in ?b ?c-high)
      (not (empty ?c-low)) (not (empty ?c-mid)) (not (empty ?c-high))
      (not (held-upright ?b)) (gripper-empty)
    )
  )

  ;; The 6 long-block in-hand transforms are identical to the oblong
  ;; ones at the executor level (the transform rotates whatever is held);
  ;; we expose them as separate PDDL actions only so the planner can
  ;; gate them on (long ?b) and so the action trace is unambiguous.

  (:action make-long-upright-from-x
    :parameters (?b - block)
    :precondition (and (long ?b) (held-flat-x ?b))
    :effect (and (not (held-flat-x ?b)) (held-upright ?b))
  )

  (:action make-long-upright-from-y
    :parameters (?b - block)
    :precondition (and (long ?b) (held-flat-y ?b))
    :effect (and (not (held-flat-y ?b)) (held-upright ?b))
  )

  (:action make-long-flat-x-from-upright
    :parameters (?b - block)
    :precondition (and (long ?b) (held-upright ?b))
    :effect (and (not (held-upright ?b)) (held-flat-x ?b))
  )

  (:action make-long-flat-y-from-upright
    :parameters (?b - block)
    :precondition (and (long ?b) (held-upright ?b))
    :effect (and (not (held-upright ?b)) (held-flat-y ?b))
  )

  (:action turn-long-x-to-y
    :parameters (?b - block)
    :precondition (and (long ?b) (held-flat-x ?b))
    :effect (and (not (held-flat-x ?b)) (held-flat-y ?b))
  )

  (:action turn-long-y-to-x
    :parameters (?b - block)
    :precondition (and (long ?b) (held-flat-y ?b))
    :effect (and (not (held-flat-y ?b)) (held-flat-x ?b))
  )
)
