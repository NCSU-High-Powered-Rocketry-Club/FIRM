# Script to pre-compile CMSIS-DSP functions needed for tests
# This script compiles specified CMSIS-DSP source files into object files
# Usage: .\compile_cmsis_dsp.ps1 [function1] [function2] ...

# Default functions to compile if none provided
if ($args.Count -eq 0) {
    $Functions = @("arm_mat_trans_f32", "arm_mat_add_f32", "arm_mat_scale_f32")
} else {
    $Functions = $args
}

# Get the script's directory
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Push-Location $ScriptDir

# Path configuration - resolve upward to FIRM root
$FirmRoot = Resolve-Path (Join-Path $ScriptDir "..")
$CMSISInclude = Join-Path (Join-Path (Join-Path $FirmRoot "STM32") "Drivers") "CMSIS-DSP"
$CMSISInclude = Join-Path $CMSISInclude "Include"
$CMSISARMInclude = Join-Path (Join-Path (Join-Path $FirmRoot "STM32") "Drivers") "CMSIS"
$CMSISARMInclude = Join-Path $CMSISARMInclude "Include"
$SourceBase = Join-Path (Join-Path (Join-Path (Join-Path $FirmRoot "STM32") "Drivers") "CMSIS-DSP") "Source"
$SourceBase = Join-Path $SourceBase "MatrixFunctions"

$OutputDir = "."

# Color codes for output
$Green = "`e[32m"
$Yellow = "`e[33m"
$Red = "`e[31m"
$Reset = "`e[0m"

Write-Host "${Yellow}Compiling CMSIS-DSP functions...${Reset}"

$compiled = 0
$failed = 0

foreach ($func in $Functions) {
    $sourceFile = Join-Path $SourceBase "$func.c"
    $outputFile = Join-Path $OutputDir "$func.o"
    
    # Check if source file exists
    if (-not (Test-Path $sourceFile)) {
        Write-Host "${Red}ERROR: Source file not found: $sourceFile${Reset}"
        $failed++
        continue
    }
    
    # Check if object file is already up-to-date
    if ((Test-Path $outputFile) -and ((Get-Item $sourceFile).LastWriteTime -le (Get-Item $outputFile).LastWriteTime)) {
        Write-Host "${Green}[OK]${Reset} $func.o (up-to-date)"
        $compiled++
        continue
    }
    
    # Compile the file
    Write-Host "  Compiling $func.c..."
    $output = & gcc -I"$CMSISInclude" -I"$CMSISARMInclude" -c "$sourceFile" -o "$outputFile" 2>&1
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "${Green}[OK]${Reset} $func.o"
        $compiled++
    } else {
        Write-Host "${Red}[FAIL]${Reset} $func.o failed:"
        Write-Host $output
        $failed++
    }
}

Pop-Location

Write-Host ""
Write-Host "Compilation summary: ${Green}$compiled compiled${Reset}, ${Red}$failed failed${Reset}"

if ($failed -gt 0) {
    exit 1
}
exit 0
Write-Host "Compilation summary: ${Green}$compiled compiled${Reset}, ${Red}$failed failed${Reset}"

if ($failed -gt 0) {
    exit 1
}
exit 0
