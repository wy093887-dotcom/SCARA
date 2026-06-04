param(
    [Parameter(Mandatory = $true)]
    [string]$Port,

    [int]$Baud = 115200,
    [int]$TimeoutMs = 1000
)

# 回零/限位输入检查脚本：读取 HOME_SENSOR，观察 PB0/PB1 是否被触发。

$ErrorActionPreference = "Stop"

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

try {
    $serial.Open()
    foreach ($cmd in @("HOME_SENSOR", "QSTAT")) {
        $serial.DiscardInBuffer()
        Write-Host "TX $cmd"
        $serial.WriteLine($cmd)
        try {
            $rx = $serial.ReadLine().Trim()
        } catch [TimeoutException] {
            $rx = "<timeout>"
        }
        Write-Host "RX $rx"
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
