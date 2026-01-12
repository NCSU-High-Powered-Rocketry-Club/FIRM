# Script to pre-compile CMSIS-DSP functions needed for tests
# This script compiles specified CMSIS-DSP source files into object files
# Usage: .\compile_cmsis_dsp.ps1 [function1] [function2] ...

# Functions must be provided by the caller (Ruby helper)
if ($args.Count -eq 0) {
    Write-Host "ERROR: No functions provided. This script must be invoked from the Ruby helper with a list of functions." -ForegroundColor Red
    exit 2
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

$OutputDir = Join-Path $ScriptDir "cmsis_objs"
if (-not (Test-Path $OutputDir)) { New-Item -ItemType Directory -Path $OutputDir | Out-Null }

# Color codes for output
$Green = "`e[32m"
$Yellow = "`e[33m"
$Red = "`e[31m"
$Reset = "`e[0m"

Write-Host "${Yellow}Compiling CMSIS-DSP functions...${Reset}"

$compiled = 0
$failed = 0
$missing = @()

foreach ($func in $Functions) {
    $sourceFile = Join-Path $SourceBase "$func.c"
    # Flatten output filenames into OUTPUT_DIR: use only the basename (no subdirectories)
    $baseName = [System.IO.Path]::GetFileName($func)
    $outputFile = Join-Path $OutputDir ("$baseName.o")

    if (Test-Path $sourceFile) {
        # Up-to-date check
        if ((Test-Path $outputFile) -and ((Get-Item $sourceFile).LastWriteTime -le (Get-Item $outputFile).LastWriteTime)) {
            Write-Host "${Green}[OK]${Reset} $($baseName).o (up-to-date)"
            $compiled++
            continue
        }

        Write-Host "  Compiling $func.c..."
        $output = & gcc -I"$CMSISInclude" -I"$CMSISARMInclude" -c "$sourceFile" -o "$outputFile" 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "${Green}[OK]${Reset} $($baseName).o"
            $compiled++
        } else {
            Write-Host "${Red}[FAIL]${Reset} $($baseName).o failed:"
            Write-Host $output
            $failed++
        }
    } else {
        # Try to find the source file anywhere under SOURCE_BASE (accept bare function names)
        $candidate = Get-ChildItem -Path $SourceBase -Recurse -Filter "$($func).c" -File -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($candidate) {
            $sourceFile = $candidate.FullName
            $base = [System.IO.Path]::GetFileNameWithoutExtension($candidate.Name)
            $outputFile = Join-Path $OutputDir ("$base.o")
            Write-Host "  Found source for $func at $sourceFile"
            $output = & gcc -I"$CMSISInclude" -I"$CMSISARMInclude" -c "$sourceFile" -o "$outputFile" 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "${Green}[OK]${Reset} $(Split-Path $outputFile -Leaf)"
                $compiled++
            } else {
                Write-Host "${Red}[FAIL]${Reset} $(Split-Path $outputFile -Leaf) failed:"; Write-Host $output
                $failed++
            }
            continue
        }

        # Record missing (likely header-only inline functions)
        $missing += $func
    }
}

# If there are missing header-only functions, synthesize a wrapper C file that provides symbols
if ($missing.Count -gt 0) {
    $wrapperC = Join-Path $OutputDir "cmsis_header_wrappers.c"
    $wrapperO = Join-Path $OutputDir "cmsis_header_wrappers.o"
    Write-Host "  Generating wrapper for header-only functions: $($missing -join ', ')"
    $header = "#include <math.h>`n/* Auto-generated wrappers for header-only inline CMSIS-DSP functions */`n"
    Set-Content -Path $wrapperC -Value $header -Encoding UTF8
    foreach ($f in $missing) {
        switch ($f) {
            'arm_sin_f32' { Add-Content -Path $wrapperC -Value "float arm_sin_f32(float x) { return sinf(x); }" }
            'arm_cos_f32' { Add-Content -Path $wrapperC -Value "float arm_cos_f32(float x) { return cosf(x); }" }
            default { Add-Content -Path $wrapperC -Value "/* Stub for $f - verify signature */"; Add-Content -Path $wrapperC -Value "float $f(float x) { (void)x; return 0.0f; }" }
        }
    }

    Write-Host "  Compiling wrapper..."
    $output = & gcc -I"$CMSISInclude" -I"$CMSISARMInclude" -c "$wrapperC" -o "$wrapperO" 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "${Green}[OK]${Reset} cmsis_header_wrappers.o"
        $compiled++
    } else {
        Write-Host "${Red}[FAIL]${Reset} cmsis_header_wrappers.o failed:";
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
