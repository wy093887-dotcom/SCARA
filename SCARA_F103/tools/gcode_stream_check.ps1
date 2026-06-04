param(
    [string]$Port = "COM9",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 1500,
    [switch]$Motion
)

# 安全串口检查脚本：不做大轨迹，只确认 VERSION/HOSTCAP/G-code ACK/状态查询正常。

$ErrorActionPreference = "Stop"

function Read-Line-Safe {
    param([System.IO.Ports.SerialPort]$Serial)
    try {
        return $Serial.ReadLine().Trim()
    } catch [TimeoutException] {
        return "<timeout>"
    }
}

function Send-Expect {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Text,
        [string[]]$Prefixes,
        [string]$Name
    )

    # 发送一条调试命令，等待指定前缀响应；自动跳过异步状态行。
    $Serial.DiscardInBuffer()
    Write-Host "TX $Text"
    $Serial.WriteLine($Text)
    while ($true) {
        $rx = Read-Line-Safe -Serial $Serial
        Write-Host "RX $rx"
        foreach ($prefix in $Prefixes) {
            if ($rx.StartsWith($prefix)) {
                Write-Host "PASS $Name"
                return $rx
            }
        }
        if ($rx -eq "<timeout>") {
            throw "Unexpected response for $Name"
        }
    }
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

try {
    Write-Host "Opening $Port at $Baud 8N1 ..."
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    Send-Expect -Serial $serial -Text "VERSION" -Prefixes @("OK VERSION") -Name "VERSION" | Out-Null
    Send-Expect -Serial $serial -Text "HOSTCAP" -Prefixes @("OK HOSTCAP") -Name "HOSTCAP" | Out-Null
    Send-Expect -Serial $serial -Text '$G' -Prefixes @("ok") -Name "modal report" | Out-Null
    Send-Expect -Serial $serial -Text "G21" -Prefixes @("ok") -Name "G21 mm mode" | Out-Null
    Send-Expect -Serial $serial -Text "G90" -Prefixes @("ok") -Name "G90 absolute" | Out-Null
    Send-Expect -Serial $serial -Text "HEARTBEAT 42" -Prefixes @("OK HEARTBEAT seq=42") -Name "heartbeat" | Out-Null

    if ($Motion) {
        Write-Host "INFO Motion enabled. This sends real G-code moves."
        Send-Expect -Serial $serial -Text "ENABLE 1" -Prefixes @("OK ENABLE 1") -Name "ENABLE" | Out-Null
        Send-Expect -Serial $serial -Text "G0 X0 Y120" -Prefixes @("ok") -Name "G0 move" | Out-Null
        Send-Expect -Serial $serial -Text "G1 X1 Y120 F60" -Prefixes @("ok") -Name "G1 move" | Out-Null
    } else {
        Write-Host "INFO Motion disabled; only modal G-code commands were sent."
    }

    $serial.DiscardInBuffer()
    Write-Host "TX ?"
    $serial.Write("?")
    while ($true) {
        $rx = Read-Line-Safe -Serial $serial
        Write-Host "RX $rx"
        if ($rx.StartsWith("<")) {
            Write-Host "PASS status query"
            break
        }
        if ($rx -eq "<timeout>") {
            throw "No status response"
        }
    }

    Write-Host "GCODE STREAM CHECK PASS"
    exit 0
} catch {
    Write-Host "GCODE STREAM CHECK FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
