;--------------------------------
;Include Modern UI

	!include "MUI2.nsh"

;--------------------------------
;General

	!define OVERLAY_BASEDIR "..\client_overlay\bin\win64"
	!define ARTIFACTS_BASEDIR "..\bin\artifacts\Release"
	!define DRIVER_RESDIR "..\bin\driver_01spacecalibrator"

	Name "Space Calibrator"
	OutFile "SpaceCalibrator.exe"
	InstallDir "$PROGRAMFILES64\SpaceCalibrator"
	InstallDirRegKey HKLM "Software\SpaceCalibrator\Main" ""
	RequestExecutionLevel admin
	ShowInstDetails show

	; Define the version number. Wrapped in !ifndef so the release CI workflow can
	; override it via `makensis /DVERSION=...` to match the git tag's version.
	!ifndef VERSION
		!define VERSION "1.5.1.0"
	!endif
	; Set version information
	VIProductVersion "${VERSION}"
	VIAddVersionKey /LANG=1033 "ProductName" "Space Calibrator"
	VIAddVersionKey /LANG=1033 "FileDescription" "Space Calibrator Installer"
	VIAddVersionKey /LANG=1033 "LegalCopyright" "Open source at https://github.com/hyblocker/OpenVR-SpaceCalibrator"
	VIAddVersionKey /LANG=1033 "FileVersion" "${VERSION}"
	VIAddVersionKey /LANG=1033 "ProductVersion" "${VERSION}"
	
;--------------------------------
;Variables

VAR upgradeInstallation

;--------------------------------
;Interface Settings

	!define MUI_ABORTWARNING

;--------------------------------
;Pages

	!insertmacro MUI_PAGE_LICENSE "..\LICENSE"
	!define MUI_PAGE_CUSTOMFUNCTION_PRE dirPre
	!insertmacro MUI_PAGE_DIRECTORY
	!insertmacro MUI_PAGE_INSTFILES
  
	!insertmacro MUI_UNPAGE_CONFIRM
	!insertmacro MUI_UNPAGE_INSTFILES
  
;--------------------------------
;Languages
 
	!insertmacro MUI_LANGUAGE "English"

;--------------------------------
;Macros

;--------------------------------
;Functions

Function dirPre
	StrCmp $upgradeInstallation "true" 0 +2 
		Abort
FunctionEnd

Function .onInit
	StrCpy $upgradeInstallation "false"
 
	ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "UninstallString"
	StrCmp $R0 "" done
	
	
	; If SteamVR is already running, display a warning message and exit
	FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
	StrCmp $0 0 +3
		MessageBox MB_OK|MB_ICONEXCLAMATION \
			"SteamVR is still running. Cannot install this software.$\nPlease close SteamVR and try again."
		Abort
 
	
	MessageBox MB_OKCANCEL|MB_ICONEXCLAMATION \
		"Space Calibrator is already installed. $\n$\nClick `OK` to upgrade the \
		existing installation or `Cancel` to cancel this upgrade." \
		IDOK upgrade
	Abort
 
	upgrade:
		StrCpy $upgradeInstallation "true"
	done:
FunctionEnd

;--------------------------------
;Installer Sections

Section "Install" SecInstall
	
	StrCmp $upgradeInstallation "true" 0 noupgrade 
		DetailPrint "Uninstall previous version..."
		ExecWait '"$INSTDIR\Uninstall.exe" /S _?=$INSTDIR'
		Delete $INSTDIR\Uninstall.exe
		Goto afterupgrade
		
	noupgrade:

	afterupgrade:

	SetOutPath "$INSTDIR"
	
	File "${ARTIFACTS_BASEDIR}\LICENSE"
	File "${ARTIFACTS_BASEDIR}\README"
	File "${ARTIFACTS_BASEDIR}\SpaceCalibrator.exe"
	File "${ARTIFACTS_BASEDIR}\openvr_api.dll"
	File "${ARTIFACTS_BASEDIR}\manifest.vrmanifest"
	File "${ARTIFACTS_BASEDIR}\icon.png"
	File "${ARTIFACTS_BASEDIR}\taskbar_icon.png"

	ExecWait '"$INSTDIR\vcredist_x64.exe" /install /quiet'
	
	Var /GLOBAL vrRuntimePath
	nsExec::ExecToStack '"$INSTDIR\SpaceCalibrator.exe" -openvrpath'
	Pop $0
	Pop $vrRuntimePath
	DetailPrint "VR runtime path: $vrRuntimePath"
	
	; Old beta driver
	StrCmp $upgradeInstallation "true" 0 nocleanupbeta 
		Delete "$vrRuntimePath\drivers\000spacecalibrator\driver.vrdrivermanifest"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\resources\driver.vrresources"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\resources\settings\default.vrsettings"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\bin\win64\driver_000spacecalibrator.dll"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\bin\win64\space_calibrator_driver.log"
		RMdir "$vrRuntimePath\drivers\000spacecalibrator\resources\settings"
		RMdir "$vrRuntimePath\drivers\000spacecalibrator\resources\"
		RMdir "$vrRuntimePath\drivers\000spacecalibrator\bin\win64\"
		RMdir "$vrRuntimePath\drivers\000spacecalibrator\bin\"
		RMdir "$vrRuntimePath\drivers\000spacecalibrator\"
	nocleanupbeta:

	SetOutPath "$vrRuntimePath\drivers\01spacecalibrator"
	File "${DRIVER_RESDIR}\driver.vrdrivermanifest"
	SetOutPath "$vrRuntimePath\drivers\01spacecalibrator\resources"
	File "${DRIVER_RESDIR}\resources\driver.vrresources"
	SetOutPath "$vrRuntimePath\drivers\01spacecalibrator\resources\settings"
	File "${DRIVER_RESDIR}\resources\settings\default.vrsettings"
	SetOutPath "$vrRuntimePath\drivers\01spacecalibrator\bin\win64"
	File "${DRIVER_RESDIR}\bin\win64\driver_01spacecalibrator.dll"
	
	WriteRegStr HKLM "Software\SpaceCalibrator\Main" "" $INSTDIR
	WriteRegStr HKLM "Software\SpaceCalibrator\Driver" "" $vrRuntimePath
  
	WriteUninstaller "$INSTDIR\Uninstall.exe"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "DisplayName" "Space Calibrator"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""

	CreateShortCut "$SMPROGRAMS\Space Calibrator.lnk" "$INSTDIR\SpaceCalibrator.exe"
	
	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\SpaceCalibrator.exe" -installmanifest'
	nsExec::ExecToLog '"$INSTDIR\SpaceCalibrator.exe" -activatemultipledrivers'

SectionEnd

;--------------------------------
;Uninstaller Section

Section "Uninstall"
	; If SteamVR is already running, display a warning message and exit
	FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
	StrCmp $0 0 +3
		MessageBox MB_OK|MB_ICONEXCLAMATION \
			"SteamVR is still running. Cannot uninstall this software.$\nPlease close SteamVR and try again."
		Abort
	
	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\SpaceCalibrator.exe" -removemanifest'

	Var /GLOBAL vrRuntimePath2
	ReadRegStr $vrRuntimePath2 HKLM "Software\SpaceCalibrator\Driver" ""
	DetailPrint "VR runtime path: $vrRuntimePath2"
	Delete "$vrRuntimePath2\drivers\01spacecalibrator\driver.vrdrivermanifest"
	Delete "$vrRuntimePath2\drivers\01spacecalibrator\resources\driver.vrresources"
	Delete "$vrRuntimePath2\drivers\01spacecalibrator\resources\settings\default.vrsettings"
	Delete "$vrRuntimePath2\drivers\01spacecalibrator\bin\win64\driver_01spacecalibrator.dll"
	Delete "$vrRuntimePath2\drivers\01spacecalibrator\bin\win64\space_calibrator_driver.log"
	RMdir "$vrRuntimePath2\drivers\01spacecalibrator\resources\settings"
	RMdir "$vrRuntimePath2\drivers\01spacecalibrator\resources\"
	RMdir "$vrRuntimePath2\drivers\01spacecalibrator\bin\win64\"
	RMdir "$vrRuntimePath2\drivers\01spacecalibrator\bin\"
	RMdir "$vrRuntimePath2\drivers\01spacecalibrator\"
	
	Delete "$INSTDIR\LICENSE"
	Delete "$INSTDIR\README"
	Delete "$INSTDIR\SpaceCalibrator.exe"
	Delete "$INSTDIR\openvr_api.dll"
	Delete "$INSTDIR\manifest.vrmanifest"
	Delete "$INSTDIR\icon.png"
	Delete "$INSTDIR\taskbar_icon.png"
	
	DeleteRegKey HKLM "Software\SpaceCalibrator\Main"
	DeleteRegKey HKLM "Software\SpaceCalibrator\Driver"
	DeleteRegKey HKLM "Software\SpaceCalibrator"
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator"

	Delete "$SMPROGRAMS\Space Calibrator.lnk"
SectionEnd

