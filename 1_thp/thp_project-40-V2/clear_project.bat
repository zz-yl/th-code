@ECHO Deleting Debug files

@DEL /S /Q .\project\mdk_project\Output\*.*
@DEL /S /Q .\project\mdk_project\*.lst
@DEL /S /Q .\project\mdk_project\*.map
@DEL /S /Q .\project\mdk_project\*.bat
@DEL /S /Q .\project\mdk_project\*.scvd
@DEL /S /Q .\project\mdk_project\JLink*
@DEL /S /Q .\project\mdk_project\*.bak
@DEL /S /Q .\project\mdk_project\*.log
@DEL /S /Q .\project\mdk_project\*.uvguix.*

@RMDIR /S /Q .\project\mdk_project\DebugConfig
@RMDIR /S /Q .\project\mdk_project\Output

PAUSE