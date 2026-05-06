# Release workflow

How a release ships on this repo. Read this before publishing one. The TL;DR
is at the bottom.

## What runs when

There are three workflow files under `.github/workflows/` that touch the
release lifecycle:

- `changelog-append.yml` -- triggers on every push to `main`. Walks the
  push range, parses commit subjects, and appends bullets to the
  `## Unreleased` heading in `CHANGELOG.md` and `wiki/Changelog.md`.
- `release.yml` -- triggers on a `v*` tag push. Promotes the `Unreleased`
  section to a versioned section, builds the distribution, composes the
  release body from the slice, validates it, publishes the release, and
  pushes the promoted `CHANGELOG.md` back to `main`.
- `wiki-sync.yml` -- mirrors `wiki/` into the GitHub Wiki repo. Triggers
  on `wiki/**` pushes.

The shared script that drives the bullet parsing, section promotion, and
body extraction is `.github/scripts/Update-Changelog.ps1`. Four modes:
`Append`, `Promote`, `Notes`, `Validate`.

## Conventional-commit policy

Commit subjects MUST follow `type(scope): description (YYYY.M.D.N-XXXX)`
where the trailing `(...)` is the build stamp the `prepare-commit-msg`
hook appends.

Recognised types and their changelog buckets:

| Type | Bucket | Notes |
|---|---|---|
| `feat`     | Added    | User-visible new functionality |
| `fix`      | Fixed    | User-visible bug fix |
| `perf`     | Changed  | Performance change |
| `refactor` | Changed  | Internal refactor that's still worth surfacing |
| `revert`   | Changed  | Revert of an earlier commit |
| `diag`     | Changed  | Pure logging additions; user-visible because they show up in attached debug logs |
| `chore(deps*)` | Changed | Dependency bumps. Other `chore:` are skipped |
| `docs`     | (skipped) | Not user-visible release notes |
| `build`    | (skipped) | Not user-visible |
| `ci`       | (skipped) | Not user-visible |
| `test`     | (skipped) | Not user-visible |
| `chore` (non-deps) | (skipped) | Not user-visible |

Trailing `!` after the type marks a breaking change and routes the entry
to a `### Breaking` section that renders above all others.

Commit subjects that include `[skip changelog]` are ignored entirely
(useful for commits that fix typos, edit comments, or tweak workflows
themselves).

Non-conventional commits don't fail the appender; they fall through to a
`### Changed` bucket without a `**scope:**` prefix. If more than 20% of
commits in a push fall through, the appender emits a workflow warning so
the maintainer can tighten discipline before the next release.

## Cutting a release

1. Land all the work for the release on `main`. Each push appends bullets
   to `## Unreleased` automatically.
2. Audit at write time. Before each commit, the subject should:
   - Use a recognised conventional-commit type.
   - Avoid marketing puffery from the standard list (see `Update-Changelog.ps1`'s
     Validate mode for the live list).
   - Avoid internal-only vocabulary (`investigator`, `tier 2/3`, `scope plan`,
     `Pattern A-Z`, etc.).
   - Be ASCII (auto-normalised, but cleaner if it's clean to begin with).
3. Tag and push:

   ```bash
   git tag -a v2026.M.D.N origin/main -m "Release v2026.M.D.N"
   git push origin v2026.M.D.N
   ```

4. The `release.yml` workflow runs end-to-end:
   - Promotes `Unreleased` to `## [v2026.M.D.N] -- YYYY-MM-DD`.
   - Stashes the promoted files for the post-release replay.
   - Builds the distribution + NSIS installer.
   - Composes the release body: title + download options + integrity
     table + slice body + (optional) Additional notes section.
   - Validates the composed body for marketing puffery / internal-only vocab
     / non-ASCII / stray `@WhyKnot`. Soft mode -- warnings only, doesn't
     block the publish.
   - Publishes the release with the composed body.
   - Verifies the published body byte-matches what was composed (warns on
     divergence).
   - Pushes the promoted `CHANGELOG.md` and `wiki/Changelog.md` back to
     `main` so the next push starts with a fresh `## Unreleased`.

5. Spot-check the published release page after the workflow finishes.
   Address any `::warning::` annotations from the workflow run by amending
   the relevant commit subject before the next release (do NOT history-
   rewrite the published commit).

## Optional Additional notes

For the rare case where the slice doesn't carry enough context (a maintenance
release, a user-facing migration step, a deployment caveat), drop a markdown
file at:

```
release/<tag>/extra-details.md
```

If present, the workflow appends it to the body below the slice with a
`---` separator and a `## Additional notes` heading. The same Validate
pass runs against its contents; voice issues in the extras file get
flagged the same as voice issues in commit subjects.

The extras file is staged on `main` BEFORE you tag the release (it lives
in the source tree, so the tag includes it). Commit it with `[skip
changelog]` so the appender doesn't pick it up.

```bash
mkdir -p release/v2026.5.5.1
$EDITOR release/v2026.5.5.1/extra-details.md
git add release/v2026.5.5.1/extra-details.md
git commit -m "docs(release): add v2026.5.5.1 extras [skip changelog]"
git push origin main
git tag -a v2026.5.5.1 origin/main -m "Release v2026.5.5.1"
git push origin v2026.5.5.1
```

Absence of the file is the default and a no-op; the body is just the slice.

## Hand-writing the body is forbidden

Per the project rule landed 2026-05-06: future release bodies are
mechanically derived from commit subjects in the tag range. The escape
hatch is the optional extras file above. There is no `gh release edit`
step in the normal flow.

This rule exists because hand-writing release bodies has produced
hand-rewritten marketing copy and false-date claims (the v2026.5.5.0
first pass opened with "Five weeks of work" when the actual range was
six days).
The auto-changelog body is mechanical; commit-subject discipline is the
new gate.

If you absolutely must edit a published release body after the fact (a
typo in the integrity table, a missing CVE reference, etc.), fix the
underlying source (the commit subject or the extras file), re-tag at a
new version, and re-publish. Edit-in-place is reserved for true
emergencies and should be flagged in `project_release_notes_audit_*`
memory entries.

## Validate mode (manual run)

Outside of the release workflow, you can run the validation pass against
any text:

```pwsh
# Inline string
./.github/scripts/Update-Changelog.ps1 -Mode Validate -Body @'
## Some draft text
Comprehensive new features...
'@

# File
./.github/scripts/Update-Changelog.ps1 -Mode Validate -Path some-draft.md

# Strict mode (non-zero exit on any finding) for pre-commit hooks
./.github/scripts/Update-Changelog.ps1 -Mode Validate -Path some-draft.md -Strict
```

Findings come out as `Write-Warning` lines on stderr plus a tab-separated
summary on stdout (one finding per line, columns: category / line /
match / excerpt).

## TL;DR

- Land work on main with conventional-commit subjects -> changelog
  bullets are appended automatically.
- Tag with `git tag -a v<X> origin/main && git push origin v<X>` ->
  `release.yml` does everything else.
- Body is sliced from the auto-changelog. Hand-writing is forbidden.
- Optional context goes in `release/<tag>/extra-details.md`.
- Voice issues get flagged at workflow run-time. Fix at the source for
  the next release.
