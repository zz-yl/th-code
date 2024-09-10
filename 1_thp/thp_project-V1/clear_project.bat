@ECHO Deleting Debug files

@DEL /S /Q .\project\Output\*.*
@DEL /S /Q .\project\*.lst
@DEL /S /Q .\project\*.map
@DEL /S /Q .\project\*.bat
@DEL /S /Q .\project\*.scvd
@DEL /S /Q .\project\JLink*
@DEL /S /Q .\project\*.bat
@DEL /S /Q .\project\*.log
@DEL /S /Q .\project\*.uvguix.*

@RMDIR /S /Q .\project\DebugConfig
@RMDIR /S /Q .\project\Output

PAUSE