<!-- Linked issue (optional but appreciated): Closes #__ -->

## Summary

<!-- 1-3 sentences on the *why*, not just the what. The diff already shows the what. -->

## Checklist

- [ ] `powershell -ExecutionPolicy Bypass -File build.ps1` succeeds end-to-end on this branch.
- [ ] If behaviour changed, the relevant wiki page in `wiki/` was updated.
- [ ] If the IPC protocol changed, `protocol::Version` was bumped and `wiki/Driver-Protocol.md` reflects it.
- [ ] Commit subjects pass `.githooks/commit-msg` — no duplicate `(YYYY.M.D.N-XXXX)` build-version stamps.

## Notes for calibration / driver / IPC changes

<!-- Delete this section if irrelevant. Otherwise: -->
<!-- - Did you change the calibration math? Mention which validation tests you ran. -->
<!-- - Did you add a new IPC message? Bump protocol::Version and update Driver-Protocol.md. -->
<!-- - Did you touch driver pose application? Verify the no-stuck-offset checks in Continuous-Calibration.md still hold. -->
