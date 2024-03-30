@echo off
setlocal enabledelayedexpansion

REM Set the directory containing the files you want to rename
set "rootFolder=./Raspberry Pi"

REM Loop through all files in the folder and its subfolders
for /r "%rootFolder%" %%f in (*) do (
    set "fullPath=%%f"
    set "fileName=%%~nxf"
    set "newName=!fileName!.txt"
    ren "%%f" "!newName!"
)

endlocal
