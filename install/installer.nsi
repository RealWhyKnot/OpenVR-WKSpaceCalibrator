; OpenVR-WKSpaceCalibrator module installer.
; Enables the Space Calibrator module in an existing OpenVR-Pair install.

!include "MUI2.nsh"

!ifndef VERSION
	!define VERSION "0.1.0.0"
!endif

Name "OpenVR-WKSpaceCalibrator"
OutFile "..\build\artifacts\Release\OpenVR-WKSpaceCalibrator-Installer.exe"
InstallDir "$PROGRAMFILES64\OpenVR-Pair\features\OpenVR-WKSpaceCalibrator"
RequestExecutionLevel admin
ShowInstDetails show

VIProductVersion "${VERSION}"
VIAddVersionKey /LANG=1033 "ProductName" "OpenVR-WKSpaceCalibrator"
VIAddVersionKey /LANG=1033 "FileDescription" "OpenVR-WKSpaceCalibrator Module Installer"
VIAddVersionKey /LANG=1033 "LegalCopyright" "Open source at https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator"
VIAddVersionKey /LANG=1033 "FileVersion" "${VERSION}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${VERSION}"

Var vrRuntimePath

!define MUI_ABORTWARNING
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

Function RequirePair
	IfFileExists "$PROGRAMFILES64\OpenVR-Pair\OpenVR-Pair.exe" done
		MessageBox MB_YESNO|MB_ICONEXCLAMATION "OpenVR-Pair must be installed before OpenVR-WKSpaceCalibrator. Open the OpenVR-Pair releases page now?" IDNO abort
		ExecShell "open" "https://github.com/RealWhyKnot/OpenVR-WKPairDriver/releases"
	abort:
		Abort
	done:
FunctionEnd

!macro ResolveRuntimePath
	ReadRegStr $vrRuntimePath HKLM "Software\OpenVR-Pair\Driver" ""
	StrCmp $vrRuntimePath "" 0 pathOk
	nsExec::ExecToStack 'powershell -NoProfile -Command "try { ((Get-Content -Raw \"$env:LOCALAPPDATA\openvr\openvrpaths.vrpath\" | ConvertFrom-Json).runtime)[0] } catch { exit 1 }"'
	Pop $0
	Pop $vrRuntimePath
	StrCmp $0 "0" +3
		MessageBox MB_OK|MB_ICONEXCLAMATION "Could not locate the SteamVR runtime path. Install OpenVR-Pair first, then run this installer again."
		Abort
	Push $vrRuntimePath
	Call TrimNewlines
	Pop $vrRuntimePath
	pathOk:
!macroend

Function TrimNewlines
	Exch $R0
	Push $R1
	Push $R2
	StrCpy $R1 0
	loop:
		IntOp $R1 $R1 - 1
		StrCpy $R2 $R0 1 $R1
		StrCmp $R2 "$\r" loop
		StrCmp $R2 "$\n" loop
		IntOp $R1 $R1 + 1
		IntCmp $R1 0 noTrim
		StrCpy $R0 $R0 $R1
	noTrim:
	Pop $R2
	Pop $R1
	Exch $R0
FunctionEnd

Section "Install"
	Call RequirePair
	!insertmacro ResolveRuntimePath
	SetOutPath "$vrRuntimePath\drivers\01openvrpair\resources"
	FileOpen $0 "$vrRuntimePath\drivers\01openvrpair\resources\enable_calibration.flag" w
	FileWrite $0 "enabled"
	FileClose $0

	SetOutPath "$INSTDIR"
	WriteUninstaller "$INSTDIR\Uninstall.exe"
	WriteRegStr HKLM "Software\OpenVR-WKSpaceCalibrator\Main" "" "$INSTDIR"
	WriteRegStr HKLM "Software\OpenVR-WKSpaceCalibrator\Driver" "" "$vrRuntimePath"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVR-WKSpaceCalibrator" "DisplayName" "OpenVR-WKSpaceCalibrator"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVR-WKSpaceCalibrator" "DisplayVersion" "${VERSION}"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVR-WKSpaceCalibrator" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""
SectionEnd

Section "Uninstall"
	ReadRegStr $vrRuntimePath HKLM "Software\OpenVR-WKSpaceCalibrator\Driver" ""
	StrCmp $vrRuntimePath "" skipFlag
	Delete "$vrRuntimePath\drivers\01openvrpair\resources\enable_calibration.flag"
	skipFlag:
	Delete "$INSTDIR\Uninstall.exe"
	RMDir "$INSTDIR"
	DeleteRegKey HKLM "Software\OpenVR-WKSpaceCalibrator\Driver"
	DeleteRegKey HKLM "Software\OpenVR-WKSpaceCalibrator\Main"
	DeleteRegKey /ifempty HKLM "Software\OpenVR-WKSpaceCalibrator"
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVR-WKSpaceCalibrator"
SectionEnd
