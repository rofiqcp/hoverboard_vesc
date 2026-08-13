@echo off
setlocal
cd /d "%~dp0"
echo ==========================================================
echo Hoverboard VESC Dual V17 - Full One-Shot Hardware Test
echo WARNING: full test moves both motors.
echo Lift both wheels and use a current-limited supply first.
echo ==========================================================
set /p PORT=Masukkan COM port (contoh COM5): 
set /p CONFIRM=Ketik RUN untuk lanjut: 
if /I not "%CONFIRM%"=="RUN" exit /b 1
python -c "import serial" >nul 2>&1
if errorlevel 1 (
  echo pyserial belum ada. Mencoba install...
  python -m pip install -r requirements-test.txt
  if errorlevel 1 goto :error
)
python tools\vesc_full_test.py --port %PORT% --full --yes
set RC=%ERRORLEVEL%
echo.
echo Exit code: %RC%
echo Kirim ZIP terbaru di vesc_test_logs untuk troubleshooting.
pause
exit /b %RC%
:error
echo Gagal menyiapkan Python/pyserial.
pause
exit /b 2
