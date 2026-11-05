# Servo Output Safety Implementation Plan

**Goal:** Bound and expose servo output without changing geometry or adding motion control.

**Architecture:** Joint owns per-servo mapping and validation; Leg prevalidates all
joint outputs; the existing serial tools report and exercise those APIs.

**Spec:** User's attached servo safety requirements, 2026-09-27.

## Tasks

1. Add behavioral tests for pulse mapping, invalid configurations, neutral attach,
   and rejection of a whole foot target without partial commands.
2. Add required per-joint pulse endpoints, conservative front-right joint limits,
   a diagnostic output structure with rejection reasons, and bounded neutral attach.
3. Update Leg and all callers. Prevalidate all neutrals before multi-joint attach;
   roll back attachments on failure. Keep IK source untouched.
4. Update both serial tools to use Joint's mapping. Add isolated mathematical
   joint commands to the IK tool and export pulse endpoints from calibration.
5. Run all host tests and all three firmware builds; document calibration changes,
   rejected old stance, and remaining hardware verification.

## Decisions

- Preserve current neutral/direction values: hip 97/-1, thigh 86/-1, knee 95/+1.
- Use 1000..2000 us per joint initially. This changes physical angle scaling;
  it is not merely clipping the old mapping. Physical recalibration is required.
- Preserve the existing 20 cm stance constant; report its rejection under the
  new thigh limit rather than silently selecting a different pose.
- Retain library-supported 500..2500 us as an absolute configuration envelope,
  not as the command mapping. Reject endpoints outside it rather than let the
  library clamp them.
- Attachment writes validated neutral immediately after library attachment;
  software cannot guarantee the absence of an electrical transient before that.
- Existing single-leg source has a missing block; restore its helpers while
  implementing diagnostics. Preserve the user's calibration edits.

## Verification record

- All six host suites passed using `test/run_native.ps1`: calibration safety,
  joint/leg, kinematics, motion, serial/PWM boundaries, and servo safety.
- PlatformIO builds passed for `esp32_wroom`, `leg_calibration`, and
  `single_leg_ik` after the startup stance guard was added.
- IK source and header are unchanged. Diff whitespace checks passed for the
  implementation files; the unrelated existing `docs/CONTEXT.md` edit was excluded.
- No firmware was uploaded and no physical movement or electrical waveform was
  measured. Recheck physical neutrals with the new pulse mapping before IK testing.
