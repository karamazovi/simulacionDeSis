@echo off
setlocal

set "MOSQUITTO_EXE=C:\Program Files\Mosquitto\mosquitto.exe"
set "CONF=%~dp0mosquitto_local.conf"

if not exist "%MOSQUITTO_EXE%" (
  echo [ERROR] No se encontro mosquitto.exe en:
  echo         %MOSQUITTO_EXE%
  pause
  exit /b 1
)

:: Detener instancia anterior si existe
taskkill /F /IM mosquitto.exe >nul 2>&1
timeout /t 1 /nobreak >nul

echo [INFO] Iniciando Mosquitto con configuracion local...
echo        Config: %CONF%
echo        Escuchando en 127.0.0.1:1883

start "Mosquitto Broker" "%MOSQUITTO_EXE%" -c "%CONF%" -v

timeout /t 2 /nobreak >nul

:: Verificar que este corriendo
powershell -NoProfile -Command ^
  "if (Get-NetTCPConnection -LocalPort 1883 -State Listen -ErrorAction SilentlyContinue) { Write-Host '[OK] Mosquitto escuchando en puerto 1883' } else { Write-Host '[WARN] Puerto 1883 no detectado aun, espera unos segundos' }"

echo.
echo Puedes cerrar esta ventana. Mosquitto seguira corriendo en segundo plano.
pause
