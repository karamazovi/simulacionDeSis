@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "PYTHON_EXE=%SCRIPT_DIR%..\.venv\Scripts\python.exe"

if not exist "%PYTHON_EXE%" (
  echo [ERROR] No se encontro Python del entorno virtual en:
  echo         %PYTHON_EXE%
  echo.
  echo Verifica que exista la carpeta .venv en la raiz de Scripts.
  pause
  exit /b 1
)

pushd "%SCRIPT_DIR%"
"%PYTHON_EXE%" -m streamlit run app.py --server.address 127.0.0.1 --server.port 8501
set "EXIT_CODE=%ERRORLEVEL%"
popd

if not "%EXIT_CODE%"=="0" (
  echo.
  echo [ERROR] Streamlit termino con codigo %EXIT_CODE%.
  pause
)

exit /b %EXIT_CODE%
