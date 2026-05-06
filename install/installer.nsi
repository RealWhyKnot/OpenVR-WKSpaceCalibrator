;--------------------------------
; OpenVR-SpaceCalibrator installer.
;
; Installs the SpaceCalibrator.exe overlay and bundles the shared
; OpenVR-PairDriver tree -- the driver is auto-installed if missing or
; refreshed in place. enable_calibration.flag is dropped into the driver
; tree's resources/ folder so the driver wires up the calibration hooks at
; SteamVR startup.
;
; Migration: the pre-modular SC release shipped its own driver folder at
; <SteamVR>\drivers\01spacecalibrator\. On first install of the modular
; architecture we rename that folder's manifest to .disabled-by-pair-
; migration so SteamVR ignores it. The folder itself is kept (rollback =
; rename-the-manifest-back).
;
; Smoothing flag handling: we never touch enable_smoothing.flag. If
; OpenVR-Smoothing is also installed, the user can have both subsystems
; on at once (the shared driver opens both pipes and installs both hook
; sets when both flags are present).

;--------------------------------
;Includes

	!include "MUI2.nsh"
	!include "FileFunc.nsh"

;--------------------------------
;General

	!ifndef ARTIFACTS_BASEDIR
		!define ARTIFACTS_BASEDIR "..\bin\artifacts\Release"
	!endif
	!ifndef DRIVER_BASEDIR
		!define DRIVER_BASEDIR "..\lib\OpenVR-PairDriver\build\driver_openvrpair"
	!endif

	Name "Space Calibrator"
	OutFile "SpaceCalibrator-Setup.exe"
	InstallDir "$PROGRAMFILES64\SpaceCalibrator"
	InstallDirRegKey HKLM "Software\SpaceCalibrator\Main" ""
	RequestExecutionLevel admin
	ShowInstDetails show

	!ifndef VERSION
		!define VERSION "0.0.0.0"
	!endif
	VIProductVersion "${VERSION}"
	VIAddVersionKey /LANG=1033 "ProductName"     "Space Calibrator"
	VIAddVersionKey /LANG=1033 "FileDescription" "Space Calibrator Installer"
	VIAddVersionKey /LANG=1033 "LegalCopyright"  "Open source at https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator"
	VIAddVersionKey /LANG=1033 "FileVersion"     "${VERSION}"
	VIAddVersionKey /LANG=1033 "ProductVersion"  "${VERSION}"

;--------------------------------
;Variables

VAR vrRuntimePath
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
;Functions

Function dirPre
	StrCmp $upgradeInstallation "true" 0 +2
		Abort
FunctionEnd

Function .onInit
	StrCpy $upgradeInstallation "false"

	ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "UninstallString"
	StrCmp $R0 "" done

	FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
	StrCmp $0 0 +3
		MessageBox MB_OK|MB_ICONEXCLAMATION \
			"SteamVR is still running. Cannot install while SteamVR holds the shared driver open.$\nPlease close SteamVR and try again."
		Abort

	MessageBox MB_OKCANCEL|MB_ICONEXCLAMATION \
		"Space Calibrator is already installed.$\n$\nClick OK to upgrade or Cancel to abort." \
		IDOK upgrade
	Abort

	upgrade:
		StrCpy $upgradeInstallation "true"
	done:
FunctionEnd

;--------------------------------
;Helpers

!macro ResolveRuntimePath
	; Prefer asking the SpaceCalibrator overlay -- it links against OpenVR
	; and the -openvrpath flag returns the live runtime path. Falls back
	; to parsing %LOCALAPPDATA%\openvr\openvrpaths.vrpath via PowerShell
	; if the overlay flag fails.
	nsExec::ExecToStack '"$INSTDIR\SpaceCalibrator.exe" -openvrpath'
	Pop $0
	Pop $vrRuntimePath
	StrCmp $0 "0" pathOk
		nsExec::ExecToStack 'powershell -NoProfile -Command "try { ((Get-Content -Raw \"$env:LOCALAPPDATA\openvr\openvrpaths.vrpath\" | ConvertFrom-Json).runtime)[0] } catch { exit 1 }"'
		Pop $0
		Pop $vrRuntimePath
		StrCmp $0 "0" pathOk
			MessageBox MB_OK|MB_ICONEXCLAMATION "Could not locate the SteamVR runtime path. Make sure SteamVR has been launched at least once on this machine."
			Abort
	pathOk:
	Push $vrRuntimePath
	Call TrimNewlines
	Pop $vrRuntimePath
	DetailPrint "SteamVR runtime path: $vrRuntimePath"
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

;--------------------------------
;Installer

Section "Install" SecInstall

	StrCmp $upgradeInstallation "true" 0 noupgrade
		DetailPrint "Removing previous installation..."
		ExecWait '"$INSTDIR\Uninstall.exe" /S _?=$INSTDIR'
		Delete $INSTDIR\Uninstall.exe
	noupgrade:

	SetOutPath "$INSTDIR"
	File "..\LICENSE"
	File /oname=README.md "..\README.md"
	File "${ARTIFACTS_BASEDIR}\SpaceCalibrator.exe"
	File "..\lib\openvr\bin\win64\openvr_api.dll"
	File "..\src\overlay\manifest.vrmanifest"
	File "..\src\overlay\icon.png"
	File "..\src\overlay\taskbar_icon.png"

	; VC++ Redistributable. Bundled only when the release workflow has
	; fetched vcredist_x64.exe into install/ and passed /DBUNDLE_VCREDIST
	; to makensis.
	!ifdef BUNDLE_VCREDIST
		File "vcredist_x64.exe"
		ExecWait '"$INSTDIR\vcredist_x64.exe" /install /quiet'
		Delete "$INSTDIR\vcredist_x64.exe"
	!endif

	!insertmacro ResolveRuntimePath

	; Legacy migration: pre-modular SC's driver folder. Rename the manifest
	; so SteamVR ignores it; folder kept in place for rollback.
	IfFileExists "$vrRuntimePath\drivers\01spacecalibrator\driver.vrdrivermanifest" 0 nolegacy
		Rename "$vrRuntimePath\drivers\01spacecalibrator\driver.vrdrivermanifest" \
		       "$vrRuntimePath\drivers\01spacecalibrator\driver.vrdrivermanifest.disabled-by-pair-migration"
		DetailPrint "Disabled legacy 01spacecalibrator driver."
	nolegacy:

	; Older beta layout (pre-fork). Same migration treatment, but this one
	; we know is dead so we drop it entirely.
	StrCmp $upgradeInstallation "true" 0 nocleanupbeta
		Delete "$vrRuntimePath\drivers\000spacecalibrator\driver.vrdrivermanifest"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\resources\driver.vrresources"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\resources\settings\default.vrsettings"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\bin\win64\driver_000spacecalibrator.dll"
		Delete "$vrRuntimePath\drivers\000spacecalibrator\bin\win64\space_calibrator_driver.log"
		RMDir "$vrRuntimePath\drivers\000spacecalibrator\resources\settings"
		RMDir "$vrRuntimePath\drivers\000spacecalibrator\resources"
		RMDir "$vrRuntimePath\drivers\000spacecalibrator\bin\win64"
		RMDir "$vrRuntimePath\drivers\000spacecalibrator\bin"
		RMDir "$vrRuntimePath\drivers\000spacecalibrator"
	nocleanupbeta:

	; Lay the shared driver tree down. Auto-install / refresh -- if Smoothing's
	; installer ran first, this overwrites the same files with the bundled
	; copy from this installer. Driver-DLL version-skew is intentionally
	; last-write-wins for v1 of the installer; a future build-time gate can
	; refuse-to-downgrade.
	SetOutPath "$vrRuntimePath\drivers\01openvrpair"
	File "${DRIVER_BASEDIR}\driver.vrdrivermanifest"
	SetOutPath "$vrRuntimePath\drivers\01openvrpair\resources"
	File "${DRIVER_BASEDIR}\resources\driver.vrresources"
	SetOutPath "$vrRuntimePath\drivers\01openvrpair\resources\settings"
	File "${DRIVER_BASEDIR}\resources\settings\default.vrsettings"
	SetOutPath "$vrRuntimePath\drivers\01openvrpair\bin\win64"
	File "${DRIVER_BASEDIR}\bin\win64\driver_openvrpair.dll"

	FileOpen $0 "$vrRuntimePath\drivers\01openvrpair\resources\enable_calibration.flag" w
	FileWrite $0 "enabled"
	FileClose $0
	DetailPrint "Dropped enable_calibration.flag in $vrRuntimePath\drivers\01openvrpair\resources\"

	WriteRegStr HKLM "Software\SpaceCalibrator\Main"   "" $INSTDIR
	WriteRegStr HKLM "Software\SpaceCalibrator\Driver" "" $vrRuntimePath
	WriteRegStr HKLM "Software\SpaceCalibrator\Main"   "Version" "${VERSION}"

	WriteUninstaller "$INSTDIR\Uninstall.exe"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "DisplayName"     "Space Calibrator"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "DisplayVersion"  "${VERSION}"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "Publisher"       "RealWhyKnot"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""

	CreateShortCut "$SMPROGRAMS\Space Calibrator.lnk" "$INSTDIR\SpaceCalibrator.exe"

	; Register the overlay as a SteamVR-known application.
	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\SpaceCalibrator.exe" -installmanifest'
	nsExec::ExecToLog '"$INSTDIR\SpaceCalibrator.exe" -activatemultipledrivers'

SectionEnd

;--------------------------------
;Uninstaller

Section "Uninstall"
	FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
	StrCmp $0 0 +3
		MessageBox MB_OK|MB_ICONEXCLAMATION "SteamVR is still running. Please close SteamVR and try again."
		Abort

	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\SpaceCalibrator.exe" -removemanifest'

	; Remove our flag. The shared driver tree itself stays in place if
	; Smoothing's enable_smoothing.flag is still present; otherwise the user
	; can run the OpenVR-PairDriver uninstaller (or just delete the folder)
	; to remove the now-orphan tree.
	ReadRegStr $vrRuntimePath HKLM "Software\SpaceCalibrator\Driver" ""
	StrCmp $vrRuntimePath "" skipFlag
	Delete "$vrRuntimePath\drivers\01openvrpair\resources\enable_calibration.flag"
	DetailPrint "Removed enable_calibration.flag."
	IfFileExists "$vrRuntimePath\drivers\01openvrpair\resources\enable_smoothing.flag" 0 noOrphanWarn
		Goto skipFlag
	noOrphanWarn:
		DetailPrint "No remaining feature flags in 01openvrpair\resources\. The shared driver is now inert."
		DetailPrint "Run the OpenVR-PairDriver uninstaller or delete <SteamVR>\drivers\01openvrpair\ to remove the now-dormant driver tree."
	skipFlag:

	Delete "$INSTDIR\LICENSE"
	Delete "$INSTDIR\README.md"
	Delete "$INSTDIR\SpaceCalibrator.exe"
	Delete "$INSTDIR\openvr_api.dll"
	Delete "$INSTDIR\manifest.vrmanifest"
	Delete "$INSTDIR\icon.png"
	Delete "$INSTDIR\taskbar_icon.png"
	Delete "$INSTDIR\Uninstall.exe"
	RMDir "$INSTDIR"

	Delete "$SMPROGRAMS\Space Calibrator.lnk"

	DeleteRegKey HKLM "Software\SpaceCalibrator\Driver"
	DeleteRegKey HKLM "Software\SpaceCalibrator\Main"
	DeleteRegKey /ifempty HKLM "Software\SpaceCalibrator"
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator"

SectionEnd
