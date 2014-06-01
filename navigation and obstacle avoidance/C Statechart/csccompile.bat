:: This batch script compiles a C Statechart for the iRobot Navigation Simulator.
:: 
:: Dependencies:
::	Microsoft Visual Studio Express 2013
::	C Statechart source in current directory .
::	C Statechart project in ".\libstatechart Visual Studio Solution\libstatechart.vxproj"
:: 
:: Usage:
:: 	csccompile.bat <path to C statechart> <path to output compiled library>
:: 
:: How it works:
:: 	1. Locates Visual Studio path using the environment variable %VS120COMNTOOLS%
:: 	2. Calls a Visual Studio script to load environment variables
:: 	3. Copies the C Statechart into the C Statechart project source directory
:: 		(the original irobotNavigationStatechart.c is backed up and restored)
:: 	4. Builds the Visual Studio project with MSBuild
:: 	5. Writes logfile with the same name of the output library and the ".log" extension
:: 	6. Copies build output and log to the desired location of the output library

@echo off

:: path to visual studio compiler
set VISUALSTUDIOBINDIR=%VS120COMNTOOLS%..\..\vc\bin
:: path to C Statechart (CSC) source and solution folders
set STATECHARTSRCFILE="irobotNavigationStatechart.c"
set STATECHARTSRCFILEBAK=%STATECHARTSRCFILE%.bak
set STATECHARTSLNDIR=libstatechart Visual Studio Solution
set STATECHARTPROJECTPATH="%STATECHARTSLNDIR%\libstatechart.vcxproj"
set STATECHARTPROJECTOUTDIR=%STATECHARTSLNDIR%\Release
set STATECHARTPROJECTOUTPATH="%STATECHARTPROJECTOUTDIR%\libstatechart.dll"

:: Verify path to build tools and source.
set USAGEMSG="Usage: csccompile.bat (path to C statechart source) (path to save compiled library)"
IF [%1]==[] (echo Not enough arguments. && echo %USAGEMSG% && goto :RestoreBackup)
IF [%2]==[] (echo Not enough arguments. && echo %USAGEMSG% && goto :RestoreBackup)
IF NOT EXIST "%1" (echo File not found: %1 && goto :RestoreBackup)
IF NOT EXIST %STATECHARTPROJECTPATH% (echo Cannot find statechart project. && goto :RestoreBackup)
echo Compiling %1 into %2:

:: set up the visual studio environment variables if they have not been already
:: only call this once per environment, as it appends content to environment variables
IF "%VCINSTALLDIR%"=="" call "%VISUALSTUDIOBINDIR%\vcvars32.bat"

:: If not compiling the default statechart source file, back it up and replace.
:: If the statechart backup file already exists, assume it is left over from
:: a previous failed compilation. Do not overwrite and it will be restored.
IF NOT EXIST %STATECHARTSRCFILEBAK% (copy /y %STATECHARTSRCFILE% %STATECHARTSRCFILEBAK% > NUL)
IF ERRORLEVEL 1 (echo Could not create %STATECHARTSRCFILEBAK% && goto :RestoreBackup)
:: copy source file (%1) to source directory
IF /i NOT [%1]==[%STATECHARTSRCFILE%] (IF /i NOT ["%1"]==[%STATECHARTSRCFILE%] (copy /y "%1" %STATECHARTSRCFILE% > NUL ) )
IF ERRORLEVEL 1 (echo Could not copy %1 to %STATECHARTSRCFILE% && goto :RestoreBackup)

:: compile
MSBuild %STATECHARTPROJECTPATH% /p:Configuration=Release;OutDir=Release\ /clp:Summary;NoItemAndPropertyList;ShowTimestamp /nologo > "%2.log"
type "%2.log" | find "Build succeeded."
IF ERRORLEVEL 1 (echo Build failed. && goto :RestoreBackup)

:: Copy output
copy /y %STATECHARTPROJECTOUTPATH% "%2" > NUL
IF ERRORLEVEL 1 (echo Could not copy %STATECHARTPROJECTOUTPATH% to %2 && goto :RestoreBackup)

:: regardless of whether or not build failed, restore original statechart
:RestoreBackup
move /y %STATECHARTSRCFILEBAK% %STATECHARTSRCFILE% 1>NUL 2>NUL