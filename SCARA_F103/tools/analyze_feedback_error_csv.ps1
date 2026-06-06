param(
    [Parameter(Mandatory = $true)]
    [string]$CsvPath,
    [double]$MaxErrorMm = 1.0,
    [double]$MaxRmsMm = 0.5,
    [int]$WorstCount = 10,
    [string]$SummaryCsvPath = "",
    [switch]$NoFail
)

$ErrorActionPreference = "Stop"

function Get-Number {
    param([object]$Value, [string]$Name)
    if ($null -eq $Value -or [string]::IsNullOrWhiteSpace([string]$Value)) {
        throw "Missing numeric column $Name"
    }
    return [double]::Parse([string]$Value, [System.Globalization.CultureInfo]::InvariantCulture)
}

function Get-Column {
    param([object]$Row, [string]$Name)
    if (-not ($Row.PSObject.Properties.Name -contains $Name)) {
        throw "CSV missing column $Name"
    }
    return $Row.$Name
}

if (-not (Test-Path -LiteralPath $CsvPath -PathType Leaf)) {
    throw "CSV not found: $CsvPath"
}

$rows = @(Import-Csv -LiteralPath $CsvPath)
if ($rows.Count -eq 0) {
    throw "CSV has no samples: $CsvPath"
}

$samples = New-Object System.Collections.Generic.List[object]
$sumDx = 0.0
$sumDy = 0.0
$sumX2 = 0.0
$sumY2 = 0.0
$sumE2 = 0.0
$maxX = 0.0
$maxY = 0.0
$maxE = 0.0

for ($i = 0; $i -lt $rows.Count; $i++) {
    $row = $rows[$i]
    $dx = Get-Number -Value (Get-Column -Row $row -Name "Dx") -Name "Dx"
    $dy = Get-Number -Value (Get-Column -Row $row -Name "Dy") -Name "Dy"
    $errValue = if ($row.PSObject.Properties.Name -contains "Error") {
        Get-Number -Value $row.Error -Name "Error"
    } else {
        [Math]::Sqrt($dx * $dx + $dy * $dy)
    }
    $index = if ($row.PSObject.Properties.Name -contains "Index") {
        [int](Get-Number -Value $row.Index -Name "Index")
    } else {
        $i + 1
    }

    $sample = [pscustomobject]@{
        Index = $index
        Dx = $dx
        Dy = $dy
        Error = $errValue
    }
    $samples.Add($sample)

    $sumDx += $dx
    $sumDy += $dy
    $sumX2 += $dx * $dx
    $sumY2 += $dy * $dy
    $sumE2 += $errValue * $errValue
    $maxX = [Math]::Max($maxX, [Math]::Abs($dx))
    $maxY = [Math]::Max($maxY, [Math]::Abs($dy))
    $maxE = [Math]::Max($maxE, $errValue)
}

$count = [Math]::Max(1, $samples.Count)
$meanDx = $sumDx / $count
$meanDy = $sumDy / $count
$rmsX = [Math]::Sqrt($sumX2 / $count)
$rmsY = [Math]::Sqrt($sumY2 / $count)
$rmsE = [Math]::Sqrt($sumE2 / $count)
$worst = @($samples | Sort-Object -Property Error -Descending | Select-Object -First ([Math]::Max(1, $WorstCount)))

$summary = [pscustomobject]@{
    CsvPath = (Resolve-Path -LiteralPath $CsvPath).Path
    Count = $samples.Count
    MaxX = $maxX
    MaxY = $maxY
    MeanX = $meanDx
    MeanY = $meanDy
    RmsX = $rmsX
    RmsY = $rmsY
    MaxNorm = $maxE
    RmsNorm = $rmsE
    MaxErrorLimit = $MaxErrorMm
    RmsLimit = $MaxRmsMm
}

Write-Host ("FEEDBACK_CSV_ANALYSIS samples={0} max_x={1:F4} max_y={2:F4} mean_x={3:F4} mean_y={4:F4} rms_x={5:F4} rms_y={6:F4} max_norm={7:F4} rms_norm={8:F4}" -f $summary.Count, $summary.MaxX, $summary.MaxY, $summary.MeanX, $summary.MeanY, $summary.RmsX, $summary.RmsY, $summary.MaxNorm, $summary.RmsNorm)
Write-Host "WORST_SAMPLES"
foreach ($item in $worst) {
    Write-Host ("  index={0} dx={1:F4} dy={2:F4} err={3:F4}" -f $item.Index, $item.Dx, $item.Dy, $item.Error)
}

if (-not [string]::IsNullOrWhiteSpace($SummaryCsvPath)) {
    $dir = Split-Path -Parent $SummaryCsvPath
    if (-not [string]::IsNullOrWhiteSpace($dir) -and -not (Test-Path -LiteralPath $dir)) {
        New-Item -ItemType Directory -Path $dir | Out-Null
    }
    $summary | Export-Csv -LiteralPath $SummaryCsvPath -NoTypeInformation -Encoding UTF8
    Write-Host "SUMMARY_CSV $SummaryCsvPath"
}

if (-not $NoFail) {
    if ($summary.MaxNorm -gt $MaxErrorMm) {
        throw ("Max trajectory error exceeded limit: max={0:F4}mm limit={1:F4}mm" -f $summary.MaxNorm, $MaxErrorMm)
    }
    if ($summary.RmsNorm -gt $MaxRmsMm) {
        throw ("RMS trajectory error exceeded limit: rms={0:F4}mm limit={1:F4}mm" -f $summary.RmsNorm, $MaxRmsMm)
    }
}

Write-Host "FEEDBACK_CSV_ANALYSIS PASS"
exit 0
