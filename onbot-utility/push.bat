cd /D "%~dp0"

if [%1]==[-w] goto wipe
goto push
	
:wipe
ftc_http.exe --host http://ericgarland.com:8080 -w 

:push

ftc_http.exe --host http://10.0.0.6:8180 -u "..\teamcode" -b
