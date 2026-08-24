<#
.SYNOPSIS
    Vivado 2025.2 全流程自动化构建 — 李萨如图形显示控制装置
.DESCRIPTION
    synth → opt → place → phys_opt → route → reports
    目标: xc7z020clg400-2 (Mizar Z7)
.EXAMPLE
    .\vivado_build.ps1              # 全流程 (综合到布线+报告)
    .\vivado_build.ps1 -Bitstream   # 包含比特流生成 (需要 .xdc 引脚约束)
    .\vivado_build.ps1 -Gui         # 在GUI中打开合成后的工程
.PARAMETER Bitstream
    生成比特流 (需要预先添加 .xdc 引脚约束文件)
.PARAMETER Gui
    启动 Vivado GUI 并打开合成后的检查点
.PARAMETER VivadoHome
    Vivado 安装目录，例如 C:\Xilinx\Vivado\2019.2；也可设置环境变量 VIVADO_HOME。
#>
param(
    [switch]$Bitstream,
    [switch]$Gui,
    [string]$VivadoHome = $env:VIVADO_HOME
)

$ErrorActionPreference = "Stop"
$ScriptDir  = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectDir = $ScriptDir
$VivadoCommand = Get-Command vivado.bat -ErrorAction SilentlyContinue
if ($VivadoHome) {
    $VivadoExe = Join-Path $VivadoHome "bin\vivado.bat"
} elseif ($VivadoCommand) {
    $VivadoExe = $VivadoCommand.Source
} else {
    $VivadoExe = ""
}
$FlowScript = Join-Path $ScriptDir "scripts\lissajous_full_flow.tcl"

function _step($m) { Write-Host "`n$('='*55)`n  $m`n$('='*55)" -F Cyan }
function _ok($m)   { Write-Host "  [OK] $m" -F Green }
function _fail($m) { Write-Host "  [FAIL] $m" -F Red; exit 1 }

# ---- env check ----
_step "Vivado 2025.2 Full Flow: lissajous_top -> xc7z020clg400-2"
if (!$VivadoExe -or !(Test-Path $VivadoExe)) {
    _fail "未找到 Vivado。请使用 -VivadoHome、VIVADO_HOME 环境变量或将 vivado.bat 加入 PATH。"
}
if (!(Test-Path $FlowScript)) { _fail "Flow script not found at $FlowScript" }
_ok "environment check passed"

# ---- GUI mode ----
if ($Gui) {
    _step "Opening Vivado GUI"
    $outDir = Join-Path $ProjectDir "vivado_output"
    $dcp = Join-Path $outDir "post_route.dcp"
    if (Test-Path $dcp) {
        & $VivadoExe $dcp
    } else {
        & $VivadoExe -source $FlowScript
    }
    exit 0
}

# ---- modify for bitstream if requested ----
if ($Bitstream) {
    Write-Host "  Bitstream mode enabled (requires .xdc constraints)" -F Yellow
    (Get-Content $FlowScript -Raw) `
        -replace '# write_bitstream.*', 'write_bitstream -force [file join $outDir "lissajous_top.bit"]' `
        -replace '# write_debug_probes.*', 'write_debug_probes -force [file join $outDir "lissajous_top.ltx"]' |
        Set-Content (Join-Path $ScriptDir "scripts\_temp_flow_bitstream.tcl") -NoNewline
    $FlowScript = Join-Path $ScriptDir "scripts\_temp_flow_bitstream.tcl"
}

# ---- run Vivado batch ----
_step "Running Vivado 2025.2 Full Flow"
Push-Location $ProjectDir
try {
    $proc = Start-Process -FilePath $VivadoExe `
        -ArgumentList "-mode", "batch", "-source", $FlowScript, "-notrace", "-nojournal", "-nolog" `
        -NoNewWindow -Wait -PassThru

    if ($proc.ExitCode -ne 0) {
        _fail "Vivado exited with code $($proc.ExitCode)"
    }
    _ok "Full flow completed"
} finally {
    Pop-Location
    if ($Bitstream) {
        Remove-Item (Join-Path $ScriptDir "scripts\_temp_flow_bitstream.tcl") -ErrorAction SilentlyContinue
    }
}

# ---- results ----
_step "Results"
$outDir = Join-Path $ProjectDir "vivado_output"
Write-Host ""
Get-ChildItem $outDir -Filter "*.rpt" | Sort-Object Name | % {
    Write-Host "  $('{0,8:N1}' -f ($_.Length/1KB)) KB  $($_.Name)" -F Green
}
Get-ChildItem $outDir -Filter "*.dcp" | Sort-Object Name | % {
    Write-Host "  $('{0,8:N1}' -f ($_.Length/1KB)) KB  $($_.Name)" -F DarkGray
}
if ($Bitstream) {
    Get-ChildItem $outDir -Filter "*.bit" | Sort-Object Name | % {
        Write-Host "  $('{0,8:N1}' -f ($_.Length/1KB)) KB  $($_.Name)" -F Green
    }
}
Write-Host ""
Write-Host "  查看结果: .\vivado_build.ps1 -Gui" -F Yellow
