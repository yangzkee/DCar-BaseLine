param(
    [Parameter(Mandatory=$true)][string]$Compiler
)
$ErrorActionPreference = 'Stop'
$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$buildRoot = Join-Path $tempRoot ('dcar-check-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot | Out-Null
$warnings = @('-std=c99','-Wall','-Werror')
if ([System.IO.Path]::GetFileName($Compiler) -notmatch '^tcc') {
    $warnings += @('-Wextra','-Wno-pointer-to-int-cast')
}
function Compile-C([string[]]$CompilerArgs) {
    & $Compiler @warnings @CompilerArgs
    if ($LASTEXITCODE) { throw ('Compile failed: ' + ($CompilerArgs -join ' ')) }
}
function Run-Test([string]$Executable, [string[]]$TestArgs = @()) {
    $result = & $Executable @TestArgs
    if ($LASTEXITCODE) { throw ('Test failed: ' + ($result -join "`n")) }
    $result | Select-Object -Last 1 | Write-Output
}
Push-Location $repoRoot
try {
    foreach ($variant in @('DFCom_Example','DFCom_PatchOnly')) {
        $inc = @('-Itests/host/stubs',"-I$variant/HARDWARE")
        Compile-C ($inc + @('-Dmain=demo_main','-c',"$variant/USER/main.c",'-o',"$buildRoot/main.o"))
        Compile-C ($inc + @('tests/host/test_demo.c',"$buildRoot/main.o",'-o',"$buildRoot/demo.exe"))
        foreach ($scenario in @('happy','with-odom','odom-only','no-state','no-telemetry','uncalibrated','begin-failure','submit-failure','wait-failure')) {
            Run-Test "$buildRoot/demo.exe" @($scenario)
        }
        Compile-C (@('-Itests/host/telemetry_stubs') + $inc + @('-Dprintf=test_printf','-c',"$variant/HARDWARE/DFCom_Print.c",'-o',"$buildRoot/print.o"))
        Compile-C ($inc + @('tests/host/test_telemetry_print.c',"$buildRoot/print.o",'-o',"$buildRoot/telemetry.exe"))
        Run-Test "$buildRoot/telemetry.exe"
        foreach ($test in @('test_queue','test_move_slots')) {
            Compile-C ($inc + @("tests/host/$test.c","$variant/HARDWARE/DFCom_Tx.c","$variant/HARDWARE/DFCom_Rx.c",'-o',"$buildRoot/$test.exe"))
            Run-Test "$buildRoot/$test.exe"
        }
        $driverInc = @('-Itests/host/driver_stubs',"-I$variant/SYSTEM/usart","-I$variant/HARDWARE",'-Itests/host/stubs')
        Compile-C ($driverInc + @('-Dfputc=driver_fputc','-c',"$variant/SYSTEM/usart/usart.c",'-o',"$buildRoot/usart.o"))
        Compile-C ($driverInc + @('-DDFCom_RxParse=Actual_DFCom_RxParse','-c',"$variant/HARDWARE/DFCom_Rx.c",'-o',"$buildRoot/rx.o"))
        Compile-C ($driverInc + @('-c',"$variant/HARDWARE/DFCom_Tx.c",'-o',"$buildRoot/tx.o"))
        Compile-C ($driverInc + @('tests/host/test_usart_dma.c',"$buildRoot/usart.o","$buildRoot/rx.o","$buildRoot/tx.o",'-o',"$buildRoot/driver.exe"))
        Run-Test "$buildRoot/driver.exe"
        Write-Output "${variant}: all host checks passed"
    }
    Write-Output 'Scope: host driver/logic checks; Keil and hardware validation are separate.'
} finally {
    Pop-Location
    $resolvedBuild = [System.IO.Path]::GetFullPath($buildRoot)
    if ($resolvedBuild.StartsWith($tempRoot,[System.StringComparison]::OrdinalIgnoreCase) -and
        [System.IO.Path]::GetFileName($resolvedBuild) -match '^dcar-check-[a-f0-9]{32}$') {
        Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
    }
}

