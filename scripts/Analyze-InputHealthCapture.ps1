param(
    [string]$Path
)

$ErrorActionPreference = "Stop"

function Get-DefaultCapturePath {
    $dir = Join-Path $env:USERPROFILE "AppData\LocalLow\SpaceCalibrator\InputHealth"
    if (!(Test-Path $dir)) {
        throw "InputHealth capture directory not found: $dir"
    }

    $latest = Get-ChildItem -LiteralPath $dir -Filter "inputhealth_capture.*.csv" |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -eq $latest) {
        throw "No InputHealth capture CSV files found in $dir"
    }
    return $latest.FullName
}

function To-Double($value, [double]$default = 0.0) {
    if ([string]::IsNullOrWhiteSpace($value)) { return $default }
    return [double]::Parse($value, [Globalization.CultureInfo]::InvariantCulture)
}

function To-UInt64($value, [UInt64]$default = 0) {
    if ([string]::IsNullOrWhiteSpace($value)) { return $default }
    return [UInt64]::Parse($value, [Globalization.CultureInfo]::InvariantCulture)
}

function Get-Hint($row) {
    $pathLower = $row.path.ToLowerInvariant()
    $n = To-UInt64 $row.welford_count
    $obsMin = To-Double $row.observed_min
    $obsMax = To-Double $row.observed_max
    $coverage = To-Double $row.polar_coverage
    $entropy = To-Double $row.polar_entropy
    $weakBins = [int](To-Double $row.polar_weak_bins)
    $globalMax = To-Double $row.polar_global_max_r

    if ($row.kind -eq "bool") {
        $presses = To-UInt64 $row.press_count
        if ($presses -eq 0) { return "no presses observed" }
        return "presses observed"
    }

    if ($row.range_initialized -ne "1") {
        return "no scalar range yet"
    }

    if ($row.ph_triggered -eq "1") {
        if ($row.ph_triggered_positive -eq "1") { return "drift detector tripped +" }
        return "drift detector tripped -"
    }

    if ($pathLower.Contains("trigger") -and ($pathLower.Contains("value") -or $pathLower.StartsWith("/input/trigger"))) {
        if ($n -ge 20 -and $obsMin -gt 0.08) { return "trigger rest high?" }
        if ($n -ge 20 -and $obsMax -lt 0.85) { return "trigger max low?" }
        return "trigger range ok/provisional"
    }

    if ($row.axis_role -eq "stick.x") {
        if ($coverage -lt 0.70 -or $entropy -lt 0.75 -or $globalMax -lt 0.60) {
            return ("stick sweep insufficient: coverage={0:P0}, entropy={1:N2}, max={2:N2}" -f $coverage, $entropy, $globalMax)
        }
        if ($weakBins -gt 0) { return "possible weak stick arc: $weakBins bins" }
        return "stick sweep coverage ok"
    }

    return "range recorded"
}

if ([string]::IsNullOrWhiteSpace($Path)) {
    $Path = Get-DefaultCapturePath
}
if (!(Test-Path -LiteralPath $Path)) {
    throw "Capture file not found: $Path"
}

$rows = Import-Csv -LiteralPath $Path
if ($rows.Count -eq 0) {
    throw "Capture file is empty: $Path"
}

$latestByComponent = @{}
foreach ($row in $rows) {
    $key = "$($row.device_serial_hash)|$($row.path)"
    $old = $latestByComponent[$key]
    if ($null -eq $old -or (To-Double $row.time_s) -ge (To-Double $old.time_s)) {
        $latestByComponent[$key] = $row
    }
}

$components = $latestByComponent.Values | Sort-Object device_serial_hash, path

Write-Host "InputHealth capture: $Path"
Write-Host ("Rows: {0}; components: {1}" -f $rows.Count, $components.Count)
Write-Host ""

$components | ForEach-Object {
    $range = "-"
    if ($_.range_initialized -eq "1") {
        $range = "{0:N3}..{1:N3}" -f (To-Double $_.observed_min), (To-Double $_.observed_max)
    }

    [PSCustomObject]@{
        Device = $_.device_serial_hash
        Path = $_.path
        Kind = $_.kind
        Role = $_.axis_role
        Samples = To-UInt64 $_.welford_count
        Range = $range
        Mean = "{0:N4}" -f (To-Double $_.welford_mean)
        StdDev = "{0:N4}" -f (To-Double $_.stddev)
        Presses = To-UInt64 $_.press_count
        Hint = Get-Hint $_
    }
} | Format-Table -AutoSize
