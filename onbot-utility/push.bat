cd /D "%~dp0"

if [%1]==[-w] goto wipe
goto push
	
:wipe
rem ftc_http.exe --host http://ericgarland.com:8080 -w 
ftc_http.exe -w 

:push

rem ftc_http.exe --host http://ericgarland.com:8080 -u "..\TeamCode\src\main\java\org\firstinspires\ftc\teamcode" -b
ftc_http.exe -u "..\TeamCode\src\main\java\org\firstinspires\ftc\teamcode" -b
