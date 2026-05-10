<#
.SYNOPSIS
  Maintains CHANGELOG.md (root, embedded into the in-app viewer) and
  wiki/Changelog.md (synced to the GitHub Wiki) so the changelog stays
  current without manual editing on every release.

.DESCRIPTION
  Four modes:

    Append    Parses commit subjects in -Range, buckets them by conventional-
              commit type, and inserts bullets under the "## Unreleased" heading
              at the top of both files. Skips merge commits, bot commits, and
              commits explicitly tagged "[skip changelog]" in the subject. Skips
              types that aren't user-visible (docs/build/ci/chore/test) so the
              changelog stays focused on behavior changes. Auto-remaps stray
              `@WhyKnot` to `@RealWhyKnot`, normalises Unicode
              punctuation to ASCII. Logs coverage stats and warns when a high
              fraction of commits fall through to the non-conventional path.

    Promote   Renames the "## Unreleased" heading to "## [vTAG](...) -- DATE",
              links to the GitHub release page, and inserts a fresh empty
              "## Unreleased" section above. Used by release.yml on tag push so
              the embedded CHANGELOG.md inside the shipped exe carries the
              versioned notes for that release.

    Notes     Reads the current "## Unreleased" section content (or, with
              -ForVersion, reads the section for that version) and writes it to
              stdout. Used to inject the section into the GitHub release body.
              Warns on empty / placeholder-only slices.

    Validate  Pre-publish voice check. Scans an arbitrary text body
              (-Body or -Path) for marketing puffery, internal-only vocabulary,
              false-date claims, residual non-ASCII bytes, and stray
              `@WhyKnot` mentions. Default is soft (warnings only, exit 0).
              -Strict turns warnings into a non-zero exit.

  The script is invoked from .github/workflows/changelog-append.yml and
  .github/workflows/release.yml. PowerShell Core (pwsh) is used because both
  Windows and Ubuntu runners ship it, and the existing build tooling is
  PowerShell-native.

.PARAMETER Mode
  Append | Promote | Notes | Validate

.PARAMETER Range
  git-log range, e.g. "abc123..def456". Required for Append.

.PARAMETER Version
  Tag, e.g. "v2026.4.27.3". Required for Promote and Notes (with -ForVersion).

.PARAMETER ForVersion
  When set on Notes mode, returns the section for that already-promoted
  version instead of the live "## Unreleased" section.

.PARAMETER Repo
  owner/repo for release-link generation, defaults to $env:GITHUB_REPOSITORY.

.PARAMETER RepoRoot
  Repo root, defaults to the script's parent's parent's parent (.github/scripts/.. -> .github/.. -> repo root).
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [ValidateSet('Append', 'Promote', 'Notes', 'Validate')]
    [string]$Mode,

    [string]$Range,
    [string]$Version,
    [switch]$ForVersion,
    [string]$Repo = $env:GITHUB_REPOSITORY,
    [string]$RepoRoot,

    # Validate mode: text to scan. Either pass via -Body (one-shot string)
    # or via -Path (read from a file). Other modes ignore both.
    [string]$Body,
    [string]$Path,

    # Validate mode: when -Strict, exit non-zero on any warning. Default
    # is soft: emit warnings to stderr + a summary to stdout, exit 0.
    [switch]$Strict
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

if (-not $RepoRoot) {
    $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
}

# Both files are kept in lock-step. The root copy is the canonical CHANGELOG;
# the wiki/ copy is mirrored to the GitHub Wiki by .github/workflows/wiki-sync.yml.
$RootChangelog = Join-Path $RepoRoot 'CHANGELOG.md'
$WikiChangelog = Join-Path $RepoRoot 'wiki/Changelog.md'

# Validate mode operates on an arbitrary string (or file via -Path) and does
# NOT require the canonical changelog files to exist. The other three modes
# (Append/Promote/Notes) all touch CHANGELOG.md / wiki/Changelog.md and need
# them present.
if ($Mode -ne 'Validate') {
    if (-not (Test-Path $RootChangelog)) { throw "CHANGELOG.md not found at $RootChangelog" }
    if (-not (Test-Path $WikiChangelog)) { throw "wiki/Changelog.md not found at $WikiChangelog" }
}

# --- Warnings collector ----------------------------------------------------
#
# Each mode accumulates warnings (auto-handle remaps, voice findings, etc.) into
# this list and emits them via Write-Warning at the end. GitHub Actions
# surfaces Write-Warning as a workflow annotation. The Validate mode also
# returns a summary on stdout.

$script:Warnings = New-Object System.Collections.Generic.List[string]

function Add-ChangelogWarning {
    param([string]$Message)
    $script:Warnings.Add($Message) | Out-Null
}

# --- Helpers ---------------------------------------------------------------

# Read a file as UTF-8 text. We deliberately avoid BOM round-trips: the prepare-
# commit-msg hook already had to defend against PowerShell-written BOMs in
# version.txt; let's not seed any new ones in changelog files.
function Read-TextUtf8 {
    param([string]$Path)
    return [System.IO.File]::ReadAllText($Path, [System.Text.UTF8Encoding]::new($false))
}

function Write-TextUtf8 {
    param([string]$Path, [string]$Content)
    [System.IO.File]::WriteAllText($Path, $Content, [System.Text.UTF8Encoding]::new($false))
}

# Strip the (YYYY.M.D.N-XXXX) or (YYYY.M.D.N) build stamp the prepare-commit-msg
# hook appends to every subject in this repo. Without this every changelog bullet
# would carry a noisy "(2026.4.27.14-A130)" tail.
function Strip-BuildStamp {
    param([string]$Subject)
    return ($Subject -replace ' \(\d{4}\.\d+\.\d+\.\d+(-[A-Fa-f0-9]{4,8})?\)$', '').Trim()
}

# Remap public-facing author handles in commit subjects. The user owns
# both `WhyKnot` (personal account) and `RealWhyKnot` (org/fork). The
# public surface should consistently use `@RealWhyKnot`. A commit subject
# that says `@WhyKnot` is technically not wrong but inconsistent with the
# rest of the public artifact stream. Auto-remap with a logged warning so
# the maintainer fixes the source commit at the next iteration.
function Remap-AuthorHandles {
    param([string]$Text, [string]$Sha)
    if ([string]::IsNullOrEmpty($Text)) { return $Text }
    # Word-boundary `@WhyKnot` not preceded by `Real`. (?<![A-Za-z]) ensures
    # we don't accidentally chop `@RealWhyKnot` to `@RealRealWhyKnot`.
    $remapped = $Text -replace '(?<![A-Za-z])@WhyKnot\b', '@RealWhyKnot'
    if ($remapped -ne $Text) {
        $shortSha = if ($Sha) { ' (' + $Sha.Substring(0, [math]::Min(7, $Sha.Length)) + ')' } else { '' }
        Add-ChangelogWarning "Remapped @WhyKnot -> @RealWhyKnot in subject$shortSha. Consider amending the source commit."
    }
    return $remapped
}

# Replace the common Unicode punctuation an older commit subject might
# carry (em-dash, en-dash, ellipsis, curly quotes) with ASCII equivalents.
# CHANGELOG entries are GENERATED public-facing output -- the project rule is
# ASCII-only -- and a stray Unicode byte from one historical commit should not
# pollute the entire file.
function Normalise-ToAscii {
    param([string]$Text)
    if ([string]::IsNullOrEmpty($Text)) { return $Text }
    $t = $Text
    $t = $t -replace [char]0x2014, '--'   # em-dash
    $t = $t -replace [char]0x2013, '-'    # en-dash
    $t = $t -replace [char]0x2026, '...'  # ellipsis
    $t = $t -replace [char]0x201C, '"'    # left double quote
    $t = $t -replace [char]0x201D, '"'    # right double quote
    $t = $t -replace [char]0x2018, "'"    # left single quote
    $t = $t -replace [char]0x2019, "'"    # right single quote
    $t = $t -replace [char]0x00A0, ' '    # non-breaking space
    $t = $t -replace [char]0x2022, '-'    # bullet
    return $t
}

# Conventional-commit bucketing. Returns $null for types we deliberately skip
# (docs/build/ci/test/non-deps chore) -- those are real work but not user-visible
# release notes. Returns @{Bucket=...; Bullet=...} otherwise.
function Parse-CommitSubject {
    param([string]$Sha, [string]$Subject)

    $stripped = Strip-BuildStamp -Subject $Subject
    $stripped = Normalise-ToAscii -Text $stripped
    $stripped = Remap-AuthorHandles -Text $stripped -Sha $Sha
    if ([string]::IsNullOrWhiteSpace($stripped)) { return $null }

    # Skip explicit opt-out token. Lets a user push a no-op-from-changelog-
    # perspective commit (typo fix, doc reword) without it showing up.
    if ($stripped -match '\[skip changelog\]') { return $null }

    # Skip merge subjects. `git log --no-merges` already excludes them in
    # Append mode, but be defensive.
    if ($stripped -match '^Merge ') { return $null }

    # `diag` is SC-specific: pure logging additions (new annotation lines, new
    # CSV columns) that aren't a feature change but are user-visible because
    # they show up in debug logs users attach to bug reports. Bucketed below
    # under Changed so they appear in the changelog instead of being dropped
    # via the non-conventional fallback.
    $pattern = '^(?<type>feat|fix|perf|refactor|docs|build|ci|chore|test|revert|diag)(?:\((?<scope>[^)]+)\))?(?<bang>!)?:\s+(?<desc>.+)$'
    $m = [regex]::Match($stripped, $pattern)
    if (-not $m.Success) {
        # Non-conventional commit -- surface it under "Changed" rather than
        # silently dropping. Engineering-Standards mandates the format, but
        # one slipped through is better in the changelog than missing.
        return @{
            Bucket = 'Changed'
            Bullet = "- $stripped (" + $Sha.Substring(0, 7) + ')'
        }
    }

    $type = $m.Groups['type'].Value
    $scope = $m.Groups['scope'].Value
    $isBreaking = $m.Groups['bang'].Success
    $desc = $m.Groups['desc'].Value

    # Capitalise the description to match the existing changelog's "Added"
    # / "Fixed" prose style.
    if ($desc.Length -gt 0) {
        $desc = $desc.Substring(0, 1).ToUpper() + $desc.Substring(1)
    }

    $bucket = switch ($type) {
        'feat'     { 'Added' }
        'fix'      { 'Fixed' }
        'perf'     { 'Changed' }
        'refactor' { 'Changed' }
        'revert'   { 'Changed' }
        'diag'     { 'Changed' }
        'chore'    {
            # Surface dependency bumps (Dependabot et al.) -- those are user-
            # facing in the security/footprint sense -- but skip everything
            # else. `chore(deps)` and `chore(deps-dev)` both qualify.
            if ($scope -and $scope -match '^deps') { 'Changed' } else { $null }
        }
        default    { $null }  # docs / build / ci / test
    }

    if (-not $bucket) { return $null }
    if ($isBreaking)  { $bucket = 'Breaking' }

    $scopePrefix = if ($scope) { "**${scope}:** " } else { '' }
    $shortSha = $Sha.Substring(0, 7)
    $bullet = "- $scopePrefix$desc ($shortSha)"

    return @{ Bucket = $bucket; Bullet = $bullet }
}

# Order matters for rendering -- Breaking comes first, then Added, Changed, Fixed.
$BucketOrder = @('Breaking', 'Added', 'Changed', 'Fixed')

# Find the "## Unreleased" section in $content. Returns @{Start=int; End=int;
# Body=string} where Body is everything between the heading and the next "---"
# separator (exclusive). Returns $null if the section isn't present.
function Find-UnreleasedSection {
    param([string]$Content)

    $lines = $Content -split "`n"
    $startIdx = -1
    for ($i = 0; $i -lt $lines.Length; $i++) {
        if ($lines[$i] -match '^##\s+Unreleased\s*$') {
            $startIdx = $i
            break
        }
    }
    if ($startIdx -lt 0) { return $null }

    # Section body runs from startIdx+1 up to (but not including) the next
    # "---" separator. The convention in CHANGELOG.md is "---" between sections.
    $endIdx = $lines.Length
    for ($j = $startIdx + 1; $j -lt $lines.Length; $j++) {
        if ($lines[$j] -match '^---\s*$') {
            $endIdx = $j
            break
        }
        # Defensive: another ## heading without a separator means we ran past.
        if ($lines[$j] -match '^##\s+') {
            $endIdx = $j
            break
        }
    }

    return @{
        StartIdx = $startIdx
        EndIdx   = $endIdx
        Lines    = $lines
    }
}

# Parse an existing Unreleased body into bucket -> bullets[] map.
# Body lines look like:
#   ### Added
#   - foo (abc1234)
#   - bar (def5678)
#
#   ### Fixed
#   - baz (...)
function Parse-UnreleasedBody {
    param([string[]]$BodyLines)

    $buckets = [ordered]@{}
    $current = $null
    foreach ($line in $BodyLines) {
        if ($line -match '^###\s+(?<name>.+?)\s*$') {
            $current = $matches['name']
            if (-not $buckets.Contains($current)) { $buckets[$current] = @() }
            continue
        }
        if ($line -match '^\s*- ' -and $current) {
            $buckets[$current] += $line
        }
    }
    return $buckets
}

# Render bucket map to body lines.
function Render-UnreleasedBody {
    param([hashtable]$Buckets)

    if ($Buckets.Count -eq 0) {
        # Empty body -- match the seed placeholder so the file stays consistent
        # and the seed comment in the file makes sense.
        return @('', '_No notable changes since the last release._', '')
    }

    $out = @('')
    $emitted = @{}
    foreach ($name in $BucketOrder) {
        if ($Buckets.Contains($name) -and $Buckets[$name].Count -gt 0) {
            $out += "### $name"
            $out += $Buckets[$name]
            $out += ''
            $emitted[$name] = $true
        }
    }
    # Anything not in the canonical order (custom bucket someone hand-added)
    # -- preserve it after the canonical ones.
    foreach ($name in $Buckets.Keys) {
        if (-not $emitted.ContainsKey($name) -and $Buckets[$name].Count -gt 0) {
            $out += "### $name"
            $out += $Buckets[$name]
            $out += ''
        }
    }
    return $out
}

function Update-OneFile {
    param(
        [string]$Path,
        [hashtable]$NewBullets   # bucket -> string[] of bullets
    )

    $content = Read-TextUtf8 -Path $Path
    $section = Find-UnreleasedSection -Content $content
    if (-not $section) {
        throw "$Path is missing the '## Unreleased' section. Add a stub heading at the top before running the appender."
    }

    $bodyStart = $section.StartIdx + 1
    $bodyEnd   = $section.EndIdx - 1
    $bodyLines = if ($bodyEnd -ge $bodyStart) { $section.Lines[$bodyStart..$bodyEnd] } else { @() }

    $existing = Parse-UnreleasedBody -BodyLines $bodyLines

    foreach ($bucket in $NewBullets.Keys) {
        if (-not $existing.Contains($bucket)) { $existing[$bucket] = @() }
        foreach ($bullet in $NewBullets[$bucket]) {
            # De-dupe: skip if the same short-sha is already present (rerun-
            # safety for cases where the workflow re-fires for some reason).
            $sha = if ($bullet -match '\(([a-f0-9]{7})\)\s*$') { $matches[1] } else { $null }
            if ($sha) {
                $alreadyHas = $false
                foreach ($line in $existing[$bucket]) {
                    if ($line -match "\($sha\)") { $alreadyHas = $true; break }
                }
                if ($alreadyHas) { continue }
            }
            $existing[$bucket] += $bullet
        }
    }

    $rendered = Render-UnreleasedBody -Buckets $existing
    $before = if ($section.StartIdx -gt 0) { $section.Lines[0..($section.StartIdx)] } else { @($section.Lines[0]) }
    $after  = if ($section.EndIdx -lt $section.Lines.Length) { $section.Lines[$section.EndIdx..($section.Lines.Length - 1)] } else { @() }

    $newLines = @()
    $newLines += $before
    $newLines += $rendered
    $newLines += $after

    $newContent = ($newLines -join "`n")
    Write-TextUtf8 -Path $Path -Content $newContent
}

# --- Mode: Append ----------------------------------------------------------

if ($Mode -eq 'Append') {
    if (-not $Range) { throw "Append mode requires -Range (e.g. abc..def)." }

    Push-Location $RepoRoot
    try {
        # %H = full sha, %s = subject, %ae = author email, %P = parents (for
        # merge filtering -- we also pass --no-merges as a belt-and-braces).
        $log = & git log --no-merges --format='%H%x09%s%x09%ae' $Range 2>$null
        if ($LASTEXITCODE -ne 0) {
            Write-Host "git log returned non-zero for range '$Range' -- treating as no commits."
            return
        }
    } finally {
        Pop-Location
    }

    if (-not $log) {
        Write-Host "No commits in range $Range -- nothing to append."
        return
    }

    $newBullets = @{}
    $considered = 0
    $included = 0
    $skippedBot = 0
    $skippedSkipTag = 0
    $skippedSilent = 0     # docs / build / ci / test / non-deps chore
    $nonConventional = New-Object System.Collections.Generic.List[string]
    foreach ($line in ($log -split "`r?`n")) {
        if ([string]::IsNullOrWhiteSpace($line)) { continue }
        $parts = $line -split "`t"
        if ($parts.Length -lt 3) { continue }
        $sha = $parts[0]; $subject = $parts[1]; $email = $parts[2]
        $considered++

        # Skip bot commits. The appender itself pushes as github-actions[bot]
        # via GITHUB_TOKEN, so any future re-runs across that boundary
        # would otherwise self-cite.
        if ($email -match 'github-actions\[bot\]' -or $email -match 'noreply@github.com') {
            $skippedBot++
            continue
        }

        # Track explicit-skip vs silent-skip vs included. Easy to miss
        # commit-message discipline regressions if the workflow doesn't
        # tell us how the buckets shake out.
        $stripped = Strip-BuildStamp -Subject $subject
        if ($stripped -match '\[skip changelog\]') { $skippedSkipTag++; continue }

        $parsed = Parse-CommitSubject -Sha $sha -Subject $subject
        if (-not $parsed) { $skippedSilent++; continue }

        # Detect non-conventional fallthrough so we can warn the maintainer
        # about commit-message-discipline drift. Parse-CommitSubject's regex
        # tags non-matching commits with bucket=Changed and a bullet that
        # has no `**scope:**` prefix -- detect both conditions to be sure.
        $bucket = $parsed.Bucket
        if ($bucket -eq 'Changed' -and $parsed.Bullet -notmatch '^\- \*\*[^*]+:\*\* ') {
            $nonConventional.Add("$($sha.Substring(0,7)): $stripped") | Out-Null
        }

        if (-not $newBullets.ContainsKey($bucket)) { $newBullets[$bucket] = @() }
        $newBullets[$bucket] += $parsed.Bullet
        $included++
    }

    Write-Host "Considered $considered commit(s):"
    Write-Host "  included           = $included"
    Write-Host "  skipped (bot)      = $skippedBot"
    Write-Host "  skipped (skip-tag) = $skippedSkipTag"
    Write-Host "  skipped (silent)   = $skippedSilent  (docs/build/ci/test/non-deps chore)"
    Write-Host "  non-conventional   = $($nonConventional.Count)"

    # If a non-trivial fraction of commits in the range fell through to the
    # non-conventional fallback, the maintainer's commit-message discipline
    # is slipping. Surface those specifically -- the bullets WILL appear in
    # the changelog, but the lack of bucket scope means the rendered output
    # is less informative than it should be.
    if ($considered -gt 0) {
        $nonConvPct = [math]::Round(100.0 * $nonConventional.Count / $considered, 1)
        if ($nonConvPct -ge 20.0) {
            Add-ChangelogWarning "$($nonConventional.Count) of $considered commits ($nonConvPct%) fell through to the non-conventional fallback. Consider adopting `<type>(<scope>): ...` consistently."
            foreach ($nc in $nonConventional) {
                Write-Host "  non-conventional: $nc"
            }
        }
    }

    if ($included -eq 0) {
        Write-Host "Nothing user-visible in this push; CHANGELOG unchanged."
        # Emit accumulated warnings before exiting.
        foreach ($w in $script:Warnings) { Write-Warning $w }
        return
    }

    Update-OneFile -Path $RootChangelog -NewBullets $newBullets
    Update-OneFile -Path $WikiChangelog -NewBullets $newBullets

    Write-Host "Appended $included entr(ies) to both CHANGELOG.md and wiki/Changelog.md."
    foreach ($w in $script:Warnings) { Write-Warning $w }
    return
}

# --- Mode: Promote ---------------------------------------------------------

if ($Mode -eq 'Promote') {
    if (-not $Version) { throw "Promote mode requires -Version (e.g. v2026.4.27.3)." }
    if (-not $Repo)    { throw "Promote mode requires -Repo or `$env:GITHUB_REPOSITORY (e.g. owner/repo)." }

    $today = (Get-Date -Format 'yyyy-MM-dd')
    # ASCII separator. The project rule is ASCII-only across all generated
    # public-facing output -- em-dashes are a voice issue + a Unicode encoding
    # hazard on PS 5.1.
    $heading = "## [$Version](https://github.com/$Repo/releases/tag/$Version) -- $today"

    foreach ($path in @($RootChangelog, $WikiChangelog)) {
        $content = Read-TextUtf8 -Path $path
        $section = Find-UnreleasedSection -Content $content
        if (-not $section) {
            throw "$path is missing the '## Unreleased' section. Cannot promote."
        }

        $lines = $section.Lines
        $bodyStart = $section.StartIdx + 1
        $bodyEnd   = $section.EndIdx - 1
        $bodyLines = if ($bodyEnd -ge $bodyStart) { $lines[$bodyStart..$bodyEnd] } else { @() }

        # Drop the placeholder if it's the only thing in the body.
        $hasReal = $false
        foreach ($l in $bodyLines) {
            if ($l -match '^\s*- ' -or $l -match '^###\s+') { $hasReal = $true; break }
        }
        if (-not $hasReal) {
            # Empty section: the released version still gets an entry, just
            # with a stub note. This keeps the heading -> release-page link
            # alive so users browsing the changelog can click through.
            $bodyLines = @('', '_Maintenance release; see commit log for details._', '')
        }

        # Build the new file:
        #   <preamble through line StartIdx-1>
        #   ## Unreleased
        #
        #   _No notable changes since the last release._
        #
        #   ---
        #
        #   ## [vX] - DATE
        #   <bodyLines>
        #   ---
        #   <rest>
        $before = if ($section.StartIdx -gt 0) { $lines[0..($section.StartIdx - 1)] } else { @() }
        $after  = if ($section.EndIdx -lt $lines.Length) { $lines[$section.EndIdx..($lines.Length - 1)] } else { @() }

        $newLines = @()
        $newLines += $before
        $newLines += '## Unreleased'
        $newLines += ''
        $newLines += '_No notable changes since the last release._'
        $newLines += ''
        $newLines += '---'
        $newLines += ''
        $newLines += $heading
        $newLines += $bodyLines
        # $after starts with the existing "---" separator (or next ## heading).
        $newLines += $after

        Write-TextUtf8 -Path $path -Content (($newLines -join "`n"))
    }

    Write-Host "Promoted Unreleased -> $heading in both files."
    return
}

# --- Mode: Notes -----------------------------------------------------------

# Helper for Notes mode: detect the "empty slice" case where a section
# contains only the placeholder text (no real `### Added` / `### Changed`
# / `### Fixed` blocks). Returns true when the body is empty-or-placeholder.
function Test-EmptySliceBody {
    param([string]$Body)
    if ([string]::IsNullOrWhiteSpace($Body)) { return $true }
    # Check whether ANY canonical bucket heading is present. The placeholder
    # body is a single italics line ("_Maintenance release..._" or
    # "_No notable changes since the last release._") -- those do not match
    # any `### Added/Changed/Fixed/Breaking` pattern.
    return -not ($Body -match '(?m)^###\s+(Added|Changed|Fixed|Breaking)\s*$')
}

if ($Mode -eq 'Notes') {
    # Read from the root copy -- same content as wiki copy by construction.
    $content = Read-TextUtf8 -Path $RootChangelog

    if ($ForVersion) {
        if (-not $Version) { throw "Notes -ForVersion requires -Version." }
        # Find "## [vTag](...) -- DATE" -- the heading pattern Promote writes.
        $escaped = [regex]::Escape($Version)
        $pattern = "(?ms)^##\s+\[" + $escaped + "\][^\n]*\n(.*?)(?=^---\s*$|^##\s+|\z)"
        $m = [regex]::Match($content, $pattern)
        if (-not $m.Success) {
            Write-Error "No section found for $Version in CHANGELOG.md."
            exit 1
        }
        $body = $m.Groups[1].Value.Trim()
        if (Test-EmptySliceBody -Body $body) {
            Add-ChangelogWarning "Notes for ${Version}: section is empty / placeholder-only. Release will ship with the placeholder body."
        }
        Write-Output $body
        foreach ($w in $script:Warnings) { Write-Warning $w }
        return
    }

    $section = Find-UnreleasedSection -Content $content
    if (-not $section) { Write-Error "No '## Unreleased' section found."; exit 1 }
    $bodyStart = $section.StartIdx + 1
    $bodyEnd   = $section.EndIdx - 1
    $bodyLines = if ($bodyEnd -ge $bodyStart) { $section.Lines[$bodyStart..$bodyEnd] } else { @() }
    $body = (($bodyLines -join "`n").Trim())
    if (Test-EmptySliceBody -Body $body) {
        Add-ChangelogWarning "Unreleased section is empty / placeholder-only."
    }
    Write-Output $body
    foreach ($w in $script:Warnings) { Write-Warning $w }
    return
}

# --- Mode: Validate --------------------------------------------------------
#
# Pre-publish voice check. Scans an arbitrary text body for marketing puffery,
# internal-only vocabulary, residual non-ASCII, false-date claims, and
# stray @WhyKnot mentions that the auto-remap missed (e.g. inside
# code-fenced text, where the remap is more conservative). Default mode
# is soft: warnings only, exits 0. -Strict turns warnings into a non-zero
# exit so the workflow step can fail-fast.
#
# Input: either -Body '<inline text>' or -Path '<file path>'. Caller
# composes the final release-body string and feeds it here before
# `gh release create / edit`.
#
# Output:
#   - stderr: per-finding `Write-Warning` (renders as ::warning:: in GHA)
#   - stdout: a tab-separated summary `category<TAB>line<TAB>match`
#             so the workflow can grep / count if it wants to.

if ($Mode -eq 'Validate') {
    $text = $null
    if ($Body) { $text = $Body }
    elseif ($Path) {
        if (-not (Test-Path $Path)) { throw "Validate: -Path '$Path' does not exist." }
        $text = Read-TextUtf8 -Path $Path
    }
    else { throw "Validate mode requires -Body or -Path." }

    # Grep tables. Each entry: regex (case-insensitive unless explicitly
    # cased), human-readable category, advice fragment.
    $voicePatterns = @(
        @{ Pattern = '\bcomprehensive\b';      Category = 'voice';            Advice = 'use a concrete description instead of "comprehensive"' },
        @{ Pattern = '\brobust(ly)?\b';        Category = 'voice';            Advice = 'OK only when used as a math term (Tukey biweight robust kernel); otherwise drop' },
        @{ Pattern = '\bleveraging\b';         Category = 'voice';            Advice = 'replace with "uses" or "with"' },
        @{ Pattern = '\bempowers?\b';          Category = 'voice';            Advice = 'replace with "lets" or remove' },
        @{ Pattern = '\bstreamlin(es?|ed|ing)\b'; Category = 'voice';         Advice = 'replace with "simplifies" or just describe what changed' },
        @{ Pattern = "it'?s important to note"; Category = 'voice';            Advice = 'just state the thing' },
        @{ Pattern = '\b(furthermore|moreover|additionally)\b'; Category = 'voice'; Advice = 'these connectives almost never carry meaning; drop' },
        @{ Pattern = '\bwhether you\b';        Category = 'voice';            Advice = 'no audience-bracketing; just describe what shipped' },
        @{ Pattern = '\bcutting-edge\b';       Category = 'voice';            Advice = 'drop' },
        @{ Pattern = '\bindustry-leading\b';   Category = 'voice';            Advice = 'drop' },
        @{ Pattern = '\bseamless(ly)?\b';      Category = 'voice';            Advice = 'drop or describe the actual integration' },
        @{ Pattern = 'investigator triage';    Category = 'voice';            Advice = 'public release notes should not reference internal triage vocabulary' }
    )
    $internalVocab = @(
        @{ Pattern = '\bagent\b';              Category = 'internal-vocab';     Advice = 'public release notes should not reference internal-tooling concepts' },
        @{ Pattern = '\binvestigator\b';       Category = 'internal-vocab';     Advice = 'use "developer" / "anyone reading the log" or just describe what the user does' },
        @{ Pattern = '\bdiagnostic batch\b';   Category = 'internal-vocab';     Advice = 'internal grouping; describe the changes individually' },
        @{ Pattern = '\btier (1|2|3|4)\b';     Category = 'internal-vocab';     Advice = 'internal-priority bucketing; not user-facing' },
        @{ Pattern = '\bscope plan\b';         Category = 'internal-vocab';     Advice = 'internal-planning vocabulary' },
        @{ Pattern = '\bPattern [A-Z]\b';      Category = 'internal-vocab';     Advice = 'internal letter-coded patterns are not user-facing' }
    )
    $dateClaims = @(
        @{ Pattern = '\b\d+\s+(weeks?|days?|months?|years?)\s+of\s+work\b'; Category = 'date-claim';     Advice = 'verify against `git log` actual timestamps; the v2026.5.5.0 release notes had a hand-claimed "Five weeks of work" that was actually six days' },
        @{ Pattern = '\bsince \d+\s+(weeks?|days?|months?|years?) ago\b';   Category = 'date-claim';     Advice = 'replace with "since v<prior-tag>" or actual computed days' }
    )
    $stray = @(
        @{ Pattern = '(?<![A-Za-z])@WhyKnot\b'; Category = 'wrong-handle';      Advice = 'remap to @RealWhyKnot; the auto-remap should already do this for commit subjects, but inline-text remap is conservative' }
    )

    function Test-Patterns {
        param([string]$Text, [array]$Patterns)
        $hits = New-Object System.Collections.Generic.List[hashtable]
        $lines = $Text -split "`r?`n"
        for ($i = 0; $i -lt $lines.Count; $i++) {
            $line = $lines[$i]
            foreach ($p in $Patterns) {
                # Use Matches (plural) so a line with multiple instances of
                # the same voice pattern gets one finding per instance.
                $matchSet = [regex]::Matches($line, $p.Pattern, [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
                foreach ($m in $matchSet) {
                    $hits.Add(@{
                        Line     = $i + 1
                        Pattern  = $p.Pattern
                        Match    = $m.Value
                        Category = $p.Category
                        Advice   = $p.Advice
                        Excerpt  = if ($line.Length -le 100) { $line } else { $line.Substring(0, 100) + '...' }
                    }) | Out-Null
                }
            }
        }
        return $hits
    }

    $allFindings = New-Object System.Collections.Generic.List[hashtable]
    foreach ($g in (Test-Patterns -Text $text -Patterns $voicePatterns)) { $allFindings.Add($g) | Out-Null }
    foreach ($g in (Test-Patterns -Text $text -Patterns $internalVocab)) { $allFindings.Add($g) | Out-Null }
    foreach ($g in (Test-Patterns -Text $text -Patterns $dateClaims))    { $allFindings.Add($g) | Out-Null }
    foreach ($g in (Test-Patterns -Text $text -Patterns $stray))         { $allFindings.Add($g) | Out-Null }

    # Non-ASCII byte scan. ASCII normalisation is supposed to handle this
    # upstream, but a body composed by the workflow can include text from
    # the extras file or from auto-generated wrapper sections that did
    # NOT pass through Normalise-ToAscii. Fail loud here.
    $bytes = [System.Text.Encoding]::UTF8.GetBytes($text)
    $nonAsciiCount = 0
    foreach ($b in $bytes) { if ($b -gt 127) { $nonAsciiCount++ } }
    if ($nonAsciiCount -gt 0) {
        $allFindings.Add(@{
            Line     = 0
            Pattern  = '(?:non-ASCII byte)'
            Match    = "$nonAsciiCount byte(s)"
            Category = 'non-ascii'
            Advice   = 'use Normalise-ToAscii or replace at the source; project rule is ASCII-only across public-facing output'
            Excerpt  = '(see body for context; bytes above 0x7F)'
        }) | Out-Null
    }

    foreach ($f in $allFindings) {
        Write-Warning ("[{0}] line {1}: '{2}' -- {3}" -f $f.Category, $f.Line, $f.Match, $f.Advice)
    }

    Write-Host "Validate: $($allFindings.Count) finding(s) across $((($text -split "`r?`n").Count)) line(s)."

    # Tab-separated summary on stdout for downstream tooling. One line per
    # finding. Empty if no findings.
    foreach ($f in $allFindings) {
        $excerpt = $f.Excerpt -replace "`t", '    '
        Write-Output ("{0}`t{1}`t{2}`t{3}" -f $f.Category, $f.Line, $f.Match, $excerpt)
    }

    if ($Strict -and $allFindings.Count -gt 0) { exit 2 }
    return
}
