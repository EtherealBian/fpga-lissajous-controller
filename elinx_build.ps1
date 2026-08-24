<#
.SYNOPSIS
    eLinx 3.0 全流程自动化构建脚本 —— 李萨如图形显示控制装置

.DESCRIPTION
    将 SystemVerilog RTL 源码自动完成: 综合 → 布局 → 布线 → 比特流生成。
    目标器件: 亿海微 eHiChip6 系列 (EQ6HL130 / EQ6HL45 / EQ6HL9)

.PARAMETER Device
    目标器件型号, 默认为 EQ6HL130。
    可选值: EQ6HL130, EQ6HL45, EQ6HL9

.PARAMETER SkipSynth
    跳过综合步骤, 仅执行布局布线 (适用于已经生成 .vqm 的场景)

.PARAMETER SkipPnr
    仅执行综合, 不运行布局布线

.PARAMETER SynthOnly
    仅综合, 不执行 P&R (快捷方式: -SkipPnr)

.PARAMETER NoBitgen
    执行综合+布局布线, 但跳过比特流生成

.PARAMETER Clean
    构建前清理上次的输出文件

.PARAMETER Program
    构建成功后自动烧录到 FPGA (需要连接编程器)

.PARAMETER ELinxHome
    eLinx 3.0 安装目录；也可设置环境变量 ELINX_HOME。

.EXAMPLE
    .\elinx_build.ps1
    使用默认器件 EQ6HL130 执行全流程: 综合 + 布局布线 + 比特流

.EXAMPLE
    .\elinx_build.ps1 -Device EQ6HL45
    使用 EQ6HL45 器件执行全流程

.EXAMPLE
    .\elinx_build.ps1 -SkipSynth
    跳过综合, 仅执行 P&R 和比特流生成

.EXAMPLE
    .\elinx_build.ps1 -SynthOnly
    仅执行综合, 生成 .vqm 网表后停止

.EXAMPLE
    .\elinx_build.ps1 -Clean -Program
    清理后重新构建, 成功后自动烧录
#>

param(
    [ValidateSet("EQ6HL130", "EQ6HL45", "EQ6HL9")]
    [string] $Device = "EQ6HL130",

    [switch] $SkipSynth,
    [switch] $SkipPnr,
    [switch] $SynthOnly,
    [switch] $NoBitgen,
    [switch] $Clean,
    [switch] $Program,
    [string] $ELinxHome = $env:ELINX_HOME
)

$ErrorActionPreference = "Stop"

# ==========================================================================
#  路径配置
# ==========================================================================

$ScriptDir    = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot  = $ScriptDir                         # 工程根目录
$RtlDir       = Join-Path $ProjectRoot "rtl"
$ScriptsDir   = Join-Path $ProjectRoot "scripts"
$OutputDir    = Join-Path $ProjectRoot "elinx_output"

$TopModule    = "lissajous_top"
$ProjectName  = "lissajous_top"
$SeriesName   = "eHiChip6"

# eLinx 3.0 安装路径通过参数或环境变量提供，仓库不保存个人计算机绝对路径。
$eLinxHome    = $ELinxHome
$eLinxCommand = Get-Command eLinx_synthesis.exe -ErrorAction SilentlyContinue
if (-not $eLinxHome -and $eLinxCommand) {
    $eLinxHome = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $eLinxCommand.Source))
}
if ($eLinxHome) {
    $eLinxBin = Join-Path $eLinxHome "bin\shell\bin"
    $SynthExe = Join-Path $eLinxBin "eLinx_synthesis.exe"
    $InfraExe = Join-Path $eLinxBin "Infrastructure.exe"
    $ProgExe  = Join-Path $eLinxHome "Programmer.exe"
} else {
    $eLinxBin = ""
    $SynthExe = ""
    $InfraExe = ""
    $ProgExe  = ""
}
$SynthScript  = Join-Path $ScriptsDir "elinx_synth.tcl"
$FlowScript   = Join-Path $ScriptsDir "elinx_flow.tcl"

# ==========================================================================
#  工具函数
# ==========================================================================

function Write-Step {
    param([string] $Message)
    Write-Host ""
    Write-Host ("=" * 72) -ForegroundColor Cyan
    Write-Host "  $Message" -ForegroundColor Cyan
    Write-Host ("=" * 72) -ForegroundColor Cyan
}

function Write-OK {
    param([string] $Message)
    Write-Host "  [OK] $Message" -ForegroundColor Green
}

function Write-Warn {
    param([string] $Message)
    Write-Host "  [WARN] $Message" -ForegroundColor Yellow
}

function Write-Fail {
    param([string] $Message)
    Write-Host "  [FAIL] $Message" -ForegroundColor Red
}

# ==========================================================================
#  环境检查
# ==========================================================================

Write-Step "eLinx 3.0 全流程自动化构建"

Write-Host ""
Write-Host "  顶层模块:   $TopModule"
Write-Host "  器件系列:   $SeriesName"
Write-Host "  器件型号:   $Device"
Write-Host "  工程目录:   $ProjectRoot"
Write-Host "  eLinx目录:  $eLinxHome"
Write-Host ""

# 检查 eLinx 安装
if (-not $eLinxHome -or -not (Test-Path $SynthExe)) {
    Write-Fail "找不到 eLinx_synthesis.exe。请使用 -ELinxHome、ELINX_HOME 环境变量或加入 PATH。"
    exit 1
}
if (-not (Test-Path $InfraExe)) {
    Write-Fail "找不到 Infrastructure.exe: $InfraExe"
    exit 1
}
Write-OK "eLinx 3.0 安装路径验证通过"

# 检查 RTL 源文件
$RtlFiles = Get-ChildItem -Path $RtlDir -Filter "*.sv" | Sort-Object Name
if ($RtlFiles.Count -eq 0) {
    Write-Fail "未找到 SystemVerilog 源文件: $RtlDir"
    exit 1
}
Write-OK "找到 $($RtlFiles.Count) 个 SystemVerilog 源文件"

# ==========================================================================
#  清理
# ==========================================================================

if ($Clean) {
    Write-Step "清理上次构建输出"
    $CleanPatterns = @(
        "*.vqm", "*.place", "*.route", "*.psk", "*.psf",
        "*.udm", "*.fit.result", "psk_fit.log", "log4cpp.property"
    )
    foreach ($Pattern in $CleanPatterns) {
        Get-ChildItem -Path $ProjectRoot -Filter $Pattern -ErrorAction SilentlyContinue |
            Remove-Item -Force -ErrorAction SilentlyContinue
    }
    if (Test-Path $OutputDir) {
        Remove-Item -Recurse -Force $OutputDir -ErrorAction SilentlyContinue
    }
    Write-OK "清理完成"
}

# ==========================================================================
#  步骤 1: 综合 (Synthesis)
# ==========================================================================

if (-not $SkipSynth -and -not $SkipPnr) {
    Write-Step "步骤 1/3: SystemVerilog 综合 (eLinx_synthesis / Yosys)"

    # 切换到项目根目录, 使 Tcl 脚本中的相对路径正确
    Push-Location $ProjectRoot

    try {
        $SynthArgs = @(
            "-Q",                           # 不打印 banner
            "-c", $SynthScript              # 执行 Tcl 脚本
        )

        Write-Host "  命令: $SynthExe $($SynthArgs -join ' ')"
        Write-Host ""

        $SynthResult = & $SynthExe @SynthArgs 2>&1

        if ($LASTEXITCODE -ne 0) {
            Write-Fail "综合失败 (exit code: $LASTEXITCODE)"
            Write-Host $SynthResult
            Pop-Location
            exit 1
        }

        Write-Host $SynthResult
        Write-OK "综合完成"

        # 检查 VQM 输出
        $VqmFile = Join-Path $ProjectRoot "$ProjectName.vqm"
        if (-not (Test-Path $VqmFile)) {
            Write-Fail "综合未生成 VQM 文件: $VqmFile"
            Pop-Location
            exit 1
        }
        $VqmSize = (Get-Item $VqmFile).Length
        Write-OK "VQM 网表: $VqmFile ($('{0:N0}' -f $VqmSize) bytes)"
    }
    finally {
        Pop-Location
    }
}

if ($SynthOnly) {
    Write-OK "仅综合模式, 跳过 P&R"
    Write-Host ""
    Write-Host "  下一步:" -ForegroundColor Yellow
    Write-Host "    1. 在 eLinx GUI 中打开工程并分配引脚"
    Write-Host "    2. 导出 .psf 引脚约束文件"
    Write-Host "    3. 运行 .\elinx_build.ps1 -SkipSynth 继续 P&R"
    Write-Host ""
    exit 0
}

# ==========================================================================
#  步骤 2: 布局布线 (P&R + Bitgen)
# ==========================================================================

if (-not $SkipPnr) {
    Write-Step "步骤 2/3: 布局布线 + 比特流生成 (Infrastructure)"

    # Infrastructure.exe 需要在 bin/shell/ 或 bin/shell/bin/ 目录下运行
    # 因为 Tcl 脚本中使用相对路径 ../Infrastructure/...
    $ShellBinDir = Join-Path $eLinxHome "bin\shell\bin"

    # 将 flow Tcl 复制到 shell/bin 目录 (因为内部路径是相对于此目录的)
    $FlowScriptLocal = Join-Path $ShellBinDir "elinx_flow_lissajous.tcl"

    # 修改 Tcl 脚本中的 projectDir 为绝对路径
    $FlowContent = Get-Content $FlowScript -Raw -Encoding UTF8
    $FlowContent = $FlowContent -replace `
        'set projectDir\s+\[pwd\]',
        "set projectDir `"$($ProjectRoot -replace '\\','/')`""
    $FlowContent = $FlowContent -replace `
        'set deviceName\s+"EQ6HL130"',
        "set deviceName `"$Device`""
    Set-Content -Path $FlowScriptLocal -Value $FlowContent -Encoding UTF8 -NoNewline

    Push-Location $ShellBinDir

    try {
        $FlowArgs = @(
            "-f", "elinx_flow_lissajous.tcl"   # 执行 P&R Tcl 脚本
        )

        Write-Host "  工作目录: $ShellBinDir"
        Write-Host "  命令: $InfraExe $($FlowArgs -join ' ')"
        Write-Host ""

        $FlowResult = & $InfraExe @FlowArgs 2>&1

        if ($LASTEXITCODE -ne 0) {
            Write-Fail "布局布线失败 (exit code: $LASTEXITCODE)"
            Write-Host $FlowResult
            Pop-Location
            exit 1
        }

        Write-Host $FlowResult
        Write-OK "布局布线 + 比特流生成完成"
    }
    finally {
        Pop-Location
        # 清理临时脚本
        Remove-Item $FlowScriptLocal -ErrorAction SilentlyContinue
    }
}

# ==========================================================================
#  步骤 3: 结果汇总
# ==========================================================================

Write-Step "步骤 3/3: 构建结果"

$GeneratedFiles = @()

$VqmPath    = Join-Path $ProjectRoot "$ProjectName.vqm"
$PlacePath  = Join-Path $ProjectRoot "$ProjectName.place"
$RoutePath  = Join-Path $ProjectRoot "$ProjectName.route"
$PskPath    = Join-Path $ProjectRoot "$ProjectName.psk"
$FitRptPath = Join-Path $ProjectRoot "$ProjectName.fit.result"
$LogPath    = Join-Path $ProjectRoot "psk_fit.log"

if (Test-Path $VqmPath)    { $GeneratedFiles += @{Name="综合网表";   Path=$VqmPath;    Size=(Get-Item $VqmPath).Length} }
if (Test-Path $PlacePath)  { $GeneratedFiles += @{Name="布局结果";   Path=$PlacePath;  Size=(Get-Item $PlacePath).Length} }
if (Test-Path $RoutePath)  { $GeneratedFiles += @{Name="布线结果";   Path=$RoutePath;  Size=(Get-Item $RoutePath).Length} }
if (Test-Path $PskPath)    { $GeneratedFiles += @{Name="比特流文件"; Path=$PskPath;    Size=(Get-Item $PskPath).Length} }
if (Test-Path $FitRptPath) { $GeneratedFiles += @{Name="适配报告";   Path=$FitRptPath; Size=(Get-Item $FitRptPath).Length} }
if (Test-Path $LogPath)    { $GeneratedFiles += @{Name="构建日志";   Path=$LogPath;    Size=(Get-Item $LogPath).Length} }

Write-Host ""
Write-Host "  $("-" * 60)"
Write-Host "  {'Name',-16} {'Size',-10} Path"
Write-Host "  $("-" * 60)"
foreach ($File in $GeneratedFiles) {
    $SizeStr = if ($File.Size -gt 1MB) {
        "{0:N1} MB" -f ($File.Size / 1MB)
    } elseif ($File.Size -gt 1KB) {
        "{0:N1} KB" -f ($File.Size / 1KB)
    } else {
        "$($File.Size) B"
    }
    Write-Host ("  {0,-16} {1,-10} {2}" -f $File.Name, $SizeStr, $File.Path)
}
Write-Host "  $("-" * 60)"
Write-Host ""

# ---- 解析适配报告 ----
if (Test-Path $FitRptPath) {
    Write-Host "  --- 适配报告摘要 ---" -ForegroundColor Cyan
    $FitReport = Get-Content $FitRptPath -Raw -ErrorAction SilentlyContinue
    # 尝试提取关键行: LUT/REG/DSP/M4K 使用量和 Fmax
    $FitLines = $FitReport -split "`n" | Where-Object {
        $_ -match "LUT|Register|DSP|M4K|Fmax|Frequency|Utilization|利用率"
    }
    if ($FitLines) {
        foreach ($Line in $FitLines) {
            Write-Host "  $($Line.Trim())"
        }
    } else {
        Write-Host "  (请查看完整报告: $FitRptPath)"
    }
    Write-Host ""
}

# ==========================================================================
#  烧录 (可选)
# ==========================================================================

if ($Program) {
    Write-Step "烧录比特流到 FPGA"

    if (-not (Test-Path $PskPath)) {
        Write-Fail "比特流文件不存在: $PskPath"
        exit 1
    }

    $ProgExePath = Join-Path $eLinxHome "Programmer.exe"
    if (-not (Test-Path $ProgExePath)) {
        Write-Warn "找不到 Programmer.exe, 请手动烧录"
        Write-Host "  手动烧录: .\Programmer.exe --mode=jtag --file=$PskPath"
    }
    else {
        Write-Host "  命令: $ProgExePath --mode=jtag --file=$PskPath"
        Push-Location $ProjectRoot
        try {
            & $ProgExePath --mode=jtag --file=$PskPath
            if ($LASTEXITCODE -eq 0) {
                Write-OK "烧录成功"
            } else {
                Write-Fail "烧录失败 (exit code: $LASTEXITCODE)"
            }
        }
        finally {
            Pop-Location
        }
    }
}

# ==========================================================================
#  完成
# ==========================================================================

Write-Host ""
Write-Host ("=" * 72) -ForegroundColor Green
Write-Host "  构建流程全部完成!" -ForegroundColor Green
Write-Host ("=" * 72) -ForegroundColor Green
Write-Host ""
Write-Host "  快速参考:" -ForegroundColor Yellow
Write-Host "    仅综合:     .\elinx_build.ps1 -SynthOnly"
Write-Host "    仅P&R:      .\elinx_build.ps1 -SkipSynth"
Write-Host "    切换器件:   .\elinx_build.ps1 -Device EQ6HL45"
Write-Host "    清理重建:   .\elinx_build.ps1 -Clean"
Write-Host "    构建+烧录:  .\elinx_build.ps1 -Program"
Write-Host ""

exit 0
