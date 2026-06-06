param(
    [int]$Count = 3000,
    [int]$FeedPps = 600,
    [int]$ControlHz = 10000,
    [int]$MinEffectivePps = 16,
    [int]$MaxPps = 10000,
    [double]$MaxReverseEntryPps = 1.0,
    [double]$MaxSyncErrorSec = 0.0,
    [string]$SegmentCsvPath = ""
)

$ErrorActionPreference = "Stop"

function Get-TestPulsePoint {
    param([int]$Index, [int]$Total)
    $u = [double]$Index / [Math]::Max(1, $Total - 1)
    $p1 = [int][Math]::Round(800.0 * [Math]::Sin(2.0 * [Math]::PI * $u))
    $p2 = [int][Math]::Round(800.0 * [Math]::Cos(2.0 * [Math]::PI * $u))
    return [pscustomobject]@{ P1 = $p1; P2 = $p2 }
}

function Abs-I32 {
    param([int]$Value)
    if ($Value -lt 0) { return -$Value }
    return $Value
}

function Sign-Of {
    param([int]$Value)
    if ($Value -gt 0) { return 1 }
    if ($Value -lt 0) { return -1 }
    return 0
}

if ($Count -lt 2) {
    throw "Count must be at least 2"
}
if ($FeedPps -le 0 -or $FeedPps -gt $MaxPps) {
    throw "FeedPps must be in 1..MaxPps"
}

$points = New-Object System.Collections.Generic.List[object]
for ($i = 0; $i -lt $Count; $i++) {
    $points.Add((Get-TestPulsePoint -Index $i -Total $Count))
}

$cur1 = 0
$cur2 = 0
$entrySpeed1 = 0
$entrySpeed2 = 0
$totalDom = 0
$maxTargetJump = 0
$maxReverseEntry = 0.0
$maxSyncError = 0.0
$zeroLengthSegments = 0
$segments = New-Object System.Collections.Generic.List[object]

for ($i = 0; $i -lt $points.Count; $i++) {
    $point = $points[$i]
    $d1 = [int]($point.P1 - $cur1)
    $d2 = [int]($point.P2 - $cur2)
    $ad1 = Abs-I32 -Value $d1
    $ad2 = Abs-I32 -Value $d2
    $dom = [Math]::Max($ad1, $ad2)
    $v1 = 0
    $v2 = 0
    $exit1 = 0
    $exit2 = 0

    if ($dom -gt 0) {
        $v1 = [int](($ad1 * $FeedPps) / $dom)
        $v2 = [int](($ad2 * $FeedPps) / $dom)
        if ($d1 -ne 0 -and $v1 -lt $MinEffectivePps) { $v1 = $MinEffectivePps }
        if ($d2 -ne 0 -and $v2 -lt $MinEffectivePps) { $v2 = $MinEffectivePps }
    } else {
        $zeroLengthSegments++
    }

    if ($i + 1 -lt $points.Count) {
        $next = $points[$i + 1]
        $nd1 = [int]($next.P1 - $point.P1)
        $nd2 = [int]($next.P2 - $point.P2)
        if (($d1 -gt 0 -and $nd1 -gt 0) -or ($d1 -lt 0 -and $nd1 -lt 0)) { $exit1 = [int]($v1 / 2) }
        if (($d2 -gt 0 -and $nd2 -gt 0) -or ($d2 -lt 0 -and $nd2 -lt 0)) { $exit2 = [int]($v2 / 2) }
    }

    $signedTarget1 = (Sign-Of -Value $d1) * $v1
    $signedTarget2 = (Sign-Of -Value $d2) * $v2
    $signedExit1 = (Sign-Of -Value $d1) * $exit1
    $signedExit2 = (Sign-Of -Value $d2) * $exit2

    if ($d1 -ne 0 -and $entrySpeed1 -ne 0 -and (Sign-Of -Value $d1) -ne (Sign-Of -Value $entrySpeed1)) {
        $maxReverseEntry = [Math]::Max($maxReverseEntry, [Math]::Abs($entrySpeed1))
    }
    if ($d2 -ne 0 -and $entrySpeed2 -ne 0 -and (Sign-Of -Value $d2) -ne (Sign-Of -Value $entrySpeed2)) {
        $maxReverseEntry = [Math]::Max($maxReverseEntry, [Math]::Abs($entrySpeed2))
    }

    $maxTargetJump = [Math]::Max($maxTargetJump, [Math]::Abs($signedTarget1 - $entrySpeed1))
    $maxTargetJump = [Math]::Max($maxTargetJump, [Math]::Abs($signedTarget2 - $entrySpeed2))

    if ($ad1 -gt 0 -and $ad2 -gt 0 -and $v1 -gt 0 -and $v2 -gt 0) {
        $t1 = [double]$ad1 / [double]$v1
        $t2 = [double]$ad2 / [double]$v2
        $maxSyncError = [Math]::Max($maxSyncError, [Math]::Abs($t1 - $t2))
    }

    $nominalTicks = if ($dom -gt 0) { [int][Math]::Ceiling(([double]$dom * [double]$ControlHz) / [double]$FeedPps) } else { 0 }
    $totalDom += $dom
    $segments.Add([pscustomobject]@{
        Segment = $i
        StartP1 = $cur1
        StartP2 = $cur2
        TargetP1 = $point.P1
        TargetP2 = $point.P2
        D1 = $d1
        D2 = $d2
        Dom = $dom
        V1 = $v1
        V2 = $v2
        Entry1 = $entrySpeed1
        Entry2 = $entrySpeed2
        Exit1 = $signedExit1
        Exit2 = $signedExit2
        NominalTicks = $nominalTicks
    })

    $cur1 = $point.P1
    $cur2 = $point.P2
    $entrySpeed1 = $signedExit1
    $entrySpeed2 = $signedExit2
}

if (-not [string]::IsNullOrWhiteSpace($SegmentCsvPath)) {
    $dir = Split-Path -Parent $SegmentCsvPath
    if (-not [string]::IsNullOrWhiteSpace($dir) -and -not (Test-Path -LiteralPath $dir)) {
        New-Item -ItemType Directory -Path $dir | Out-Null
    }
    $segments | Export-Csv -LiteralPath $SegmentCsvPath -NoTypeInformation -Encoding UTF8
    Write-Host "SEGMENT_CSV $SegmentCsvPath"
}

$durationSec = if ($FeedPps -gt 0) { [double]$totalDom / [double]$FeedPps } else { 0.0 }
$syncLimitSec = $MaxSyncErrorSec
if ($syncLimitSec -le 0.0) {
    $syncLimitSec = [Math]::Max((1.0 / [double]$ControlHz), (1.0 / [double]$FeedPps))
}
$last = $points[$points.Count - 1]
$posErr1 = [Math]::Abs($cur1 - $last.P1)
$posErr2 = [Math]::Abs($cur2 - $last.P2)

Write-Host ("BINARY_INTERP_SIM segments={0} feed_pps={1} dom_pulses={2} nominal_s={3:F3} final_p=({4},{5}) err_p=({6},{7}) max_reverse_entry_pps={8:F3} max_target_jump_pps={9} max_sync_error_s={10:F6} sync_limit_s={11:F6} zero_len={12}" -f $Count, $FeedPps, $totalDom, $durationSec, $cur1, $cur2, $posErr1, $posErr2, $maxReverseEntry, $maxTargetJump, $maxSyncError, $syncLimitSec, $zeroLengthSegments)

if ($posErr1 -ne 0 -or $posErr2 -ne 0) {
    throw "Simulation final pulse position mismatch"
}
if ($maxReverseEntry -gt $MaxReverseEntryPps) {
    throw ("Reverse segment entered with nonzero opposite speed: {0:F3}pps limit={1:F3}pps" -f $maxReverseEntry, $MaxReverseEntryPps)
}
if ($maxSyncError -gt $syncLimitSec) {
    throw ("Axis sync time error exceeded limit: {0:F6}s limit={1:F6}s" -f $maxSyncError, $syncLimitSec)
}

Write-Host "BINARY_INTERP_SIM PASS"
exit 0
