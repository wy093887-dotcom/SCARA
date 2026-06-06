param(
    [string]$Port = "COM13",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 1500,
    [int]$Count = 3000,
    [int[]]$FeedPpsList = @(300, 600, 900),
    [int[]]$ChunkPointsList = @(10, 20, 40),
    [double]$MaxErrorMm = 1.0,
    [double]$MaxRmsMm = 0.5,
    [string]$OutDir = "",
    [switch]$EnableMotion,
    [switch]$KeepEnabled,
    [switch]$AnalyzeOnly,
    [switch]$ContinueOnFail,
    [switch]$NoFail
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutDir = Join-Path (Join-Path $PSScriptRoot "..") "logs\binary_sweep_$stamp"
}

if (-not [System.IO.Path]::IsPathRooted($OutDir)) {
    $OutDir = Join-Path (Get-Location) $OutDir
}
$OutDir = [System.IO.Path]::GetFullPath($OutDir)
$summaryPath = Join-Path $OutDir "binary_sweep_summary.csv"
$stressScript = Join-Path $PSScriptRoot "binary_joint_traj_stress.ps1"
$analysisScript = Join-Path $PSScriptRoot "analyze_feedback_error_csv.ps1"

function Ensure-Directory {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        New-Item -ItemType Directory -Path $Path | Out-Null
    }
}

function Safe-Name {
    param([int]$FeedPps, [int]$ChunkPoints)
    return "feed{0:D5}_chunk{1:D3}" -f $FeedPps, $ChunkPoints
}

function Invoke-FeedbackAnalysis {
    param(
        [string]$CsvPath,
        [string]$RunSummaryPath
    )
    $args = @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", $analysisScript,
        "-CsvPath", $CsvPath,
        "-MaxErrorMm", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxErrorMm)),
        "-MaxRmsMm", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxRmsMm)),
        "-WorstCount", "5",
        "-SummaryCsvPath", $RunSummaryPath,
        "-NoFail"
    )
    $analysisOutput = & powershell @args 2>&1
    foreach ($line in $analysisOutput) { Write-Host $line }
    if ($LASTEXITCODE -ne 0) {
        throw "Analysis failed for $CsvPath"
    }
    return @(Import-Csv -LiteralPath $RunSummaryPath)[0]
}

Ensure-Directory -Path $OutDir

if (-not $AnalyzeOnly -and -not $EnableMotion) {
    throw "Use -EnableMotion to run motor sweep. Use -AnalyzeOnly to summarize existing CSV files without motion."
}

$results = New-Object System.Collections.Generic.List[object]
$stopSweep = $false
Write-Host ("BINARY_SWEEP start out={0} count={1} feeds={2} chunks={3} analyze_only={4}" -f $OutDir, $Count, ($FeedPpsList -join ","), ($ChunkPointsList -join ","), $AnalyzeOnly.IsPresent)

foreach ($feed in $FeedPpsList) {
    if ($stopSweep) { break }
    foreach ($chunk in $ChunkPointsList) {
        if ($stopSweep) { break }
        $runName = Safe-Name -FeedPps $feed -ChunkPoints $chunk
        $csvPath = Join-Path $OutDir "$runName.csv"
        $runSummaryPath = Join-Path $OutDir "$runName.summary.csv"
        $status = "PASS"
        $message = ""
        $elapsedSec = 0.0

        try {
            if (-not $AnalyzeOnly) {
                $stressArgs = @(
                    "-NoProfile",
                    "-ExecutionPolicy", "Bypass",
                    "-File", $stressScript,
                    "-Port", $Port,
                    "-Baud", ([string]$Baud),
                    "-TimeoutMs", ([string]$TimeoutMs),
                    "-Count", ([string]$Count),
                    "-ChunkPoints", ([string]$chunk),
                    "-FeedPps", ([string]$feed),
                    "-MaxErrorMm", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxErrorMm)),
                    "-CsvPath", $csvPath,
                    "-EnableMotion"
                )
                if ($KeepEnabled) {
                    $stressArgs += "-KeepEnabled"
                }

                Write-Host ("BINARY_SWEEP_RUN feed_pps={0} chunk={1} csv={2}" -f $feed, $chunk, $csvPath)
                $started = Get-Date
                $stressOutput = & powershell @stressArgs 2>&1
                $elapsedSec = ((Get-Date) - $started).TotalSeconds
                foreach ($line in $stressOutput) { Write-Host $line }
                if ($LASTEXITCODE -ne 0) {
                    $status = "STRESS_FAIL"
                    $message = "stress exit $LASTEXITCODE"
                    if (-not (Test-Path -LiteralPath $csvPath -PathType Leaf)) {
                        throw $message
                    }
                }
            } elseif (-not (Test-Path -LiteralPath $csvPath -PathType Leaf)) {
                throw "Missing CSV for analyze-only run: $csvPath"
            }

            $summary = Invoke-FeedbackAnalysis -CsvPath $csvPath -RunSummaryPath $runSummaryPath
            $maxNorm = [double]::Parse($summary.MaxNorm, [System.Globalization.CultureInfo]::InvariantCulture)
            $rmsNorm = [double]::Parse($summary.RmsNorm, [System.Globalization.CultureInfo]::InvariantCulture)
            if ($maxNorm -gt $MaxErrorMm -or $rmsNorm -gt $MaxRmsMm) {
                $status = if ($status -eq "PASS") { "LIMIT_FAIL" } else { $status }
                $message = ("max={0:F4} rms={1:F4}" -f $maxNorm, $rmsNorm)
            }

            $results.Add([pscustomobject]@{
                FeedPps = $feed
                ChunkPoints = $chunk
                Status = $status
                Count = $summary.Count
                MaxX = $summary.MaxX
                MaxY = $summary.MaxY
                MeanX = $summary.MeanX
                MeanY = $summary.MeanY
                RmsX = $summary.RmsX
                RmsY = $summary.RmsY
                MaxNorm = $summary.MaxNorm
                RmsNorm = $summary.RmsNorm
                ElapsedSec = [Math]::Round($elapsedSec, 3)
                CsvPath = $csvPath
                Message = $message
            })

            Write-Host ("BINARY_SWEEP_RESULT feed_pps={0} chunk={1} status={2} max={3} rms={4}" -f $feed, $chunk, $status, $summary.MaxNorm, $summary.RmsNorm)
            if ($status -ne "PASS" -and -not $ContinueOnFail) {
                throw "Sweep limit failed for feed=$feed chunk=$chunk"
            }
        } catch {
            $err = $_.Exception.Message
            $results.Add([pscustomobject]@{
                FeedPps = $feed
                ChunkPoints = $chunk
                Status = "ERROR"
                Count = 0
                MaxX = 0
                MaxY = 0
                MeanX = 0
                MeanY = 0
                RmsX = 0
                RmsY = 0
                MaxNorm = 0
                RmsNorm = 0
                ElapsedSec = [Math]::Round($elapsedSec, 3)
                CsvPath = $csvPath
                Message = $err
            })
            Write-Host ("BINARY_SWEEP_ERROR feed_pps={0} chunk={1} {2}" -f $feed, $chunk, $err)
            if (-not $ContinueOnFail) {
                $stopSweep = $true
            }
        }
    }
}

$results | Export-Csv -LiteralPath $summaryPath -NoTypeInformation -Encoding UTF8
Write-Host "BINARY_SWEEP_SUMMARY $summaryPath"

$passRows = @($results | Where-Object { $_.Status -eq "PASS" } | Sort-Object @{ Expression = { [double]::Parse([string]$_.RmsNorm, [System.Globalization.CultureInfo]::InvariantCulture) } }, @{ Expression = { [double]::Parse([string]$_.MaxNorm, [System.Globalization.CultureInfo]::InvariantCulture) } })
if ($passRows.Count -gt 0) {
    $best = $passRows[0]
    Write-Host ("BINARY_SWEEP_BEST feed_pps={0} chunk={1} rms={2} max={3}" -f $best.FeedPps, $best.ChunkPoints, $best.RmsNorm, $best.MaxNorm)
} else {
    Write-Host "BINARY_SWEEP_BEST none"
}

$failed = @($results | Where-Object { $_.Status -ne "PASS" })
if ($failed.Count -gt 0 -and -not $NoFail) {
    throw "Binary feedback sweep failed runs=$($failed.Count)"
}

Write-Host "BINARY_SWEEP PASS"
exit 0
