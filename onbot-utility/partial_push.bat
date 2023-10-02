cd /D "%~dp0"

if [%1]==[-w] goto wipe
goto push
	
:wipe
ftc_http.exe --host http://ericgarland.com:8080 -w 

:push
ftc_http.exe  -u "..\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\HPMC.java" 
ftc_http.exe  -u "..\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\EncoderTest.java" -b
