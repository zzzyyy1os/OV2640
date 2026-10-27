@echo off
set MSYSTEM=
set IDF_PATH=D:\esp\v5.5.4\esp-idf
set IDF_PYTHON_ENV_PATH=C:\Users\35063\.espressif\python_env\idf5.5_py3.11_env
set ESP_ROM_ELF_DIR=C:\Espressif\tools\esp-rom-elfs\20241011\
set PATH=C:\Users\35063\.espressif\python_env\idf5.5_py3.11_env\Scripts;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\ccache\4.12.1;C:\Espressif\tools\python;%PATH%
cd /d E:\esp\OV2640
python D:\esp\v5.5.4\esp-idf\tools\idf.py %*
echo BUILD_EXIT_CODE=%ERRORLEVEL%
