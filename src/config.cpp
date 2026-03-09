//
// config.cpp
//
// Mini-JV880pi - Roland JV880 synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
// Copyright (C) 2026  Plamikcho, Giulioz, Gene J.B. (Sterr1)
//
// Original author of this class:
//	R. Stange <rsta2@o2online.de>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "config.h"

CConfig::CConfig (FATFS *pFileSystem)
:	m_Properties ("minijv880.ini", pFileSystem)
{
}

CConfig::~CConfig (void)
{
}

void CConfig::Load (void)
{
	m_Properties.Load ();
	
	m_nMCUcycles = m_Properties.GetNumber ("MCUcycles", 11);
	m_nExpRom = m_Properties.GetNumber ("ExpRom", 0);
	m_SoundDevice = m_Properties.GetString ("SoundDevice", "pwm");
	m_nSampleRate = m_Properties.GetNumber ("SampleRate", 64000);

	if (m_SoundDevice == "hdmi") {
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 384*6);
	}
	else
	{
#ifdef ARM_ALLOW_MULTI_CORE
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 256);  // 128 per channel
#else
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 1024);
#endif
	}
	m_nDACI2CAddress = m_Properties.GetNumber ("DACI2CAddress", 0);
	m_bChannelsSwapped = m_Properties.GetNumber ("ChannelsSwapped", 0) != 0;

	m_nMIDIBaudRate = m_Properties.GetNumber ("MIDIBaudRate", 31250);


	m_bLCDEnabled = m_Properties.GetNumber ("LCDEnabled", 0) != 0;
	m_nLCDPinEnable = m_Properties.GetNumber ("LCDPinEnable", 4);
	m_nLCDPinRegisterSelect = m_Properties.GetNumber ("LCDPinRegisterSelect", 27);
	m_nLCDPinReadWrite = m_Properties.GetNumber ("LCDPinReadWrite", 0);
	m_nLCDPinData4 = m_Properties.GetNumber ("LCDPinData4", 22);
	m_nLCDPinData5 = m_Properties.GetNumber ("LCDPinData5", 23);
	m_nLCDPinData6 = m_Properties.GetNumber ("LCDPinData6", 24);
	m_nLCDPinData7 = m_Properties.GetNumber ("LCDPinData7", 25);
	m_nLCDI2CAddress = m_Properties.GetNumber ("LCDI2CAddress", 0);

	m_nSSD1306LCDI2CAddress = m_Properties.GetNumber ("SSD1306LCDI2CAddress", 0);
	m_nSSD1306LCDWidth = m_Properties.GetNumber ("SSD1306LCDWidth", 128);
	m_nSSD1306LCDHeight = m_Properties.GetNumber ("SSD1306LCDHeight", 32);
	m_bSSD1306LCDRotate = m_Properties.GetNumber ("SSD1306LCDRotate", 0) != 0;
	m_bSSD1306LCDMirror = m_Properties.GetNumber ("SSD1306LCDMirror", 0) != 0;

	m_nSPIBus = m_Properties.GetNumber ("SPIBus", SPI_INACTIVE);  // Disabled by default
	m_nSPIMode = m_Properties.GetNumber ("SPIMode", SPI_DEF_MODE);
	m_nSPIClockKHz = m_Properties.GetNumber ("SPIClockKHz", SPI_DEF_CLOCK);

	m_bST7789Enabled = m_Properties.GetNumber ("ST7789Enabled", 0) != 0;
	m_nST7789Data = m_Properties.GetNumber ("ST7789Data", 0);
	m_nST7789Select = m_Properties.GetNumber ("ST7789Select", 0);
	m_nST7789Reset = m_Properties.GetNumber ("ST7789Reset", 0);  // optional
	m_nST7789Backlight = m_Properties.GetNumber ("ST7789Backlight", 0);  // optional
	m_nST7789Width = m_Properties.GetNumber ("ST7789Width", 240);
	m_nST7789Height = m_Properties.GetNumber ("ST7789Height", 240);
	m_nST7789Rotation = m_Properties.GetNumber ("ST7789Rotation", 0);
	m_bST7789SmallFont = m_Properties.GetNumber ("ST7789SmallFont", 0) != 0;

	m_nLCDColumns = m_Properties.GetNumber ("LCDColumns", 16);
	m_nLCDRows = m_Properties.GetNumber ("LCDRows", 2);

	m_nButtonPinPreview = m_Properties.GetNumber ("ButtonPinPreview", 0);
	m_nButtonPinLeft = m_Properties.GetNumber ("ButtonPinLeft", 0);
	m_nButtonPinRight = m_Properties.GetNumber ("ButtonPinRight", 0);
	m_nButtonPinData = m_Properties.GetNumber ("ButtonPinData", 0);
	m_nButtonPinToneSelect = m_Properties.GetNumber ("ButtonPinToneSelect", 0);
	m_nButtonPinPatchPerform = m_Properties.GetNumber ("ButtonPinPatchPerform", 0);
	m_nButtonPinEdit = m_Properties.GetNumber ("ButtonPinEdit", 0);
	m_nButtonPinSystem = m_Properties.GetNumber ("ButtonPinSystem", 0);
	m_nButtonPinRhythm = m_Properties.GetNumber ("ButtonPinRhythm", 0);
	m_nButtonPinUtility = m_Properties.GetNumber ("ButtonPinUtility", 0);
	m_nButtonPinMute = m_Properties.GetNumber ("ButtonPinMute", 0);
	m_nButtonPinMonitor = m_Properties.GetNumber ("ButtonPinMonitor", 0);
	m_nButtonPinCompare = m_Properties.GetNumber ("ButtonPinCompare", 0);
	m_nButtonPinEnter = m_Properties.GetNumber ("ButtonPinEnter", 11);
	m_nButtonPinUp = m_Properties.GetNumber ("ButtonPinUp", 0);
	m_nButtonPinDown = m_Properties.GetNumber ("ButtonPinDown", 0);
	m_nButtonPinSaveNVRAM = m_Properties.GetNumber ("ButtonPinSaveNVRAM", 0);

	m_ButtonActionPreview = m_Properties.GetString ("ButtonActionPreview", "");
	m_ButtonActionLeft = m_Properties.GetString ("ButtonActionLeft", "");
	m_ButtonActionRight = m_Properties.GetString ("ButtonActionRight", "");
	m_ButtonActionData = m_Properties.GetString ("ButtonActionData", "");
	m_ButtonActionToneSelect = m_Properties.GetString ("ButtonActionToneSelect", "");
	m_ButtonActionPatchPerform = m_Properties.GetString ("ButtonActionPatchPerform", "");
	m_ButtonActionEdit = m_Properties.GetString ("ButtonActionEdit", "");
	m_ButtonActionSystem = m_Properties.GetString ("ButtonActionSystem", "");
	m_ButtonActionRhythm = m_Properties.GetString ("ButtonActionRhythm", "");
	m_ButtonActionUtility = m_Properties.GetString ("ButtonActionUtility", "");
	m_ButtonActionMute = m_Properties.GetString ("ButtonActionMute", "");
	m_ButtonActionMonitor = m_Properties.GetString ("ButtonActionMonitor", "");
	m_ButtonActionCompare = m_Properties.GetString ("ButtonActionCompare", "");
	m_ButtonActionEnter = m_Properties.GetString("ButtonActionEnter", "click");
	m_ButtonActionUp = m_Properties.GetString("ButtonActionUp", "click");
	m_ButtonActionDown = m_Properties.GetString("ButtonActionDown", "click");
	m_ButtonActionSaveNVRAM = m_Properties.GetString("ButtonActionSaveNVRAM", "longpress");

	// MIDI buttons
	m_nMIDIButtonCh = m_Properties.GetNumber ("MIDIButtonCh", 0);
	m_nMIDIButtonPreview = m_Properties.GetNumber("MIDIButtonPreview", 0);
	m_nMIDIButtonLeft = m_Properties.GetNumber("MIDIButtonLeft", 0);
	m_nMIDIButtonRight = m_Properties.GetNumber("MIDIButtonRight", 0);
	m_nMIDIButtonData = m_Properties.GetNumber("MIDIButtonData", 0);
	m_nMIDIButtonToneSelect = m_Properties.GetNumber("MIDIButtonToneSelect", 0);
	m_nMIDIButtonPatchPerform = m_Properties.GetNumber("MIDIButtonPatchPerform", 0);
	m_nMIDIButtonEdit = m_Properties.GetNumber("MIDIButtonEdit", 0);
	m_nMIDIButtonSystem = m_Properties.GetNumber("MIDIButtonSystem", 0);
	m_nMIDIButtonRhythm = m_Properties.GetNumber("MIDIButtonRhythm", 0);
	m_nMIDIButtonUtility = m_Properties.GetNumber("MIDIButtonUtility", 0);
	m_nMIDIButtonMute = m_Properties.GetNumber("MIDIButtonMute", 0);
	m_nMIDIButtonMonitor = m_Properties.GetNumber("MIDIButtonMonitor", 0);
	m_nMIDIButtonCompare = m_Properties.GetNumber("MIDIButtonCompare", 0);
	m_nMIDIButtonEnter = m_Properties.GetNumber("MIDIButtonEnter", 0);
	m_nMIDIButtonUp = m_Properties.GetNumber("MIDIButtonUp", 0);
	m_nMIDIButtonDown = m_Properties.GetNumber("MIDIButtonDown", 0);
	m_nMIDISaveNVRAM = m_Properties.GetNumber("MIDISaveNVRAM", 0);
	m_nMIDIEncoder = m_Properties.GetNumber("MIDIEncoder", 0);
	m_nMIDIEncoderCC = m_Properties.GetNumber("MIDIEncoderCC", 0);
	m_nMIDIEncoderUp = m_Properties.GetNumber("MIDIEncoderUp", 1);
	m_nMIDIEncoderDown = m_Properties.GetNumber("MIDIEncoderDown", 0);

	m_nDoubleClickTimeout = m_Properties.GetNumber ("DoubleClickTimeout", 400);
	m_nLongPressTimeout = m_Properties.GetNumber ("LongPressTimeout", 600);

	m_bEncoderEnabled = m_Properties.GetNumber ("EncoderEnabled", 0) != 0;
	m_nEncoderPinClock = m_Properties.GetNumber ("EncoderPinClock", 10);
	m_nEncoderPinData = m_Properties.GetNumber ("EncoderPinData", 9);

	m_bProfileEnabled = m_Properties.GetNumber ("ProfileEnabled", 0) != 0;

	// Network
	m_bNetworkEnabled  = m_Properties.GetNumber ("NetworkEnabled", 0) != 0;
	m_bNetworkDHCP  = m_Properties.GetNumber ("NetworkDHCP", 0) != 0;
	m_NetworkType = m_Properties.GetString ("NetworkType", "wlan");
	m_NetworkHostname = m_Properties.GetString ("NetworkHostname", "MiniJV880");
	if (const u8 *pIP = m_Properties.GetIPAddress("NetworkIPAddress")) m_INetworkIPAddress.Set (pIP);
	if (const u8 *pIP = m_Properties.GetIPAddress("NetworkSubnetMask")) m_INetworkSubnetMask.Set (pIP);
	if (const u8 *pIP = m_Properties.GetIPAddress("NetworkDefaultGateway")) m_INetworkDefaultGateway.Set (pIP);
	m_bSyslogEnabled  = m_Properties.GetNumber ("NetworkSyslogEnabled", 0) != 0;
	if (const u8 *pIP = m_Properties.GetIPAddress("NetworkDNSServer")) m_INetworkDNSServer.Set (pIP);
	m_bNetworkFTPEnabled = m_Properties.GetNumber("NetworkFTPEnabled", 0) != 0;
	if (const u8 *pIP = m_Properties.GetIPAddress ("NetworkSyslogServerIPAddress")) m_INetworkSyslogServerIPAddress.Set (pIP);
	m_bUDPMIDIEnabled = m_Properties.GetNumber("UDPMIDIEnabled", 0) != 0;
	if (const u8 *pIP = m_Properties.GetIPAddress("UDPMIDIIPAddress")) m_IUDPMIDIIPAddress.Set (pIP);


}

unsigned CConfig::GetMCUcycles (void) const
{
	return m_nMCUcycles;
}

unsigned CConfig::GetExpRom (void) const
{
	return m_nExpRom;
}

const char *CConfig::GetSoundDevice (void) const
{
	return m_SoundDevice.c_str ();
}

unsigned CConfig::GetSampleRate (void) const
{
	return m_nSampleRate;
}

unsigned CConfig::GetChunkSize (void) const
{
	return m_nChunkSize;
}

unsigned CConfig::GetDACI2CAddress (void) const
{
	return m_nDACI2CAddress;
}

bool CConfig::GetChannelsSwapped (void) const
{
	return m_bChannelsSwapped;
}

unsigned CConfig::GetMIDIBaudRate (void) const
{
	return m_nMIDIBaudRate;
}


bool CConfig::GetLCDEnabled (void) const
{
	return m_bLCDEnabled;
}

unsigned CConfig::GetLCDPinEnable (void) const
{
	return m_nLCDPinEnable;
}

unsigned CConfig::GetLCDPinRegisterSelect (void) const
{
	return m_nLCDPinRegisterSelect;
}

unsigned CConfig::GetLCDPinReadWrite (void) const
{
	return m_nLCDPinReadWrite;
}

unsigned CConfig::GetLCDPinData4 (void) const
{
	return m_nLCDPinData4;
}

unsigned CConfig::GetLCDPinData5 (void) const
{
	return m_nLCDPinData5;
}

unsigned CConfig::GetLCDPinData6 (void) const
{
	return m_nLCDPinData6;
}

unsigned CConfig::GetLCDPinData7 (void) const
{
	return m_nLCDPinData7;
}

unsigned CConfig::GetLCDI2CAddress (void) const
{
	return m_nLCDI2CAddress;
}

unsigned CConfig::GetSSD1306LCDI2CAddress (void) const
{
	return m_nSSD1306LCDI2CAddress;
}

unsigned CConfig::GetSSD1306LCDWidth (void) const
{
	return m_nSSD1306LCDWidth;
}

unsigned CConfig::GetSSD1306LCDHeight (void) const
{
	return m_nSSD1306LCDHeight;
}

bool CConfig::GetSSD1306LCDRotate (void) const
{
	return m_bSSD1306LCDRotate;
}

bool CConfig::GetSSD1306LCDMirror (void) const
{
	return m_bSSD1306LCDMirror;
}

unsigned CConfig::GetSPIBus (void) const
{
	return m_nSPIBus;
}

unsigned CConfig::GetSPIMode (void) const
{
	return m_nSPIMode;
}

unsigned CConfig::GetSPIClockKHz (void) const
{
	return m_nSPIClockKHz;
}

bool CConfig::GetST7789Enabled (void) const
{
	return m_bST7789Enabled;
}

unsigned CConfig::GetST7789Data (void) const
{
	return m_nST7789Data;
}

unsigned CConfig::GetST7789Select (void) const
{
	return m_nST7789Select;
}

unsigned CConfig::GetST7789Reset (void) const
{
	return m_nST7789Reset;
}

unsigned CConfig::GetST7789Backlight (void) const
{
	return m_nST7789Backlight;
}

unsigned CConfig::GetST7789Width (void) const
{
	return m_nST7789Width;
}

unsigned CConfig::GetST7789Height (void) const
{
	return m_nST7789Height;
}

unsigned CConfig::GetST7789Rotation (void) const
{
	return m_nST7789Rotation;
}

bool CConfig::GetST7789SmallFont (void) const
{
	return m_bST7789SmallFont;
}
unsigned CConfig::GetLCDColumns (void) const
{
	return m_nLCDColumns;
}

unsigned CConfig::GetLCDRows (void) const
{
	return m_nLCDRows;
}

unsigned CConfig::GetButtonPinPreview (void) const
{
	return m_nButtonPinPreview;
}

unsigned CConfig::GetButtonPinLeft (void) const
{
	return m_nButtonPinLeft;
}

unsigned CConfig::GetButtonPinRight (void) const
{
	return m_nButtonPinRight;
}

unsigned CConfig::GetButtonPinData (void) const
{
	return m_nButtonPinData;
}

unsigned CConfig::GetButtonPinToneSelect (void) const
{
	return m_nButtonPinToneSelect;
}

unsigned CConfig::GetButtonPinPatchPerform (void) const
{
	return m_nButtonPinPatchPerform;
}

unsigned CConfig::GetButtonPinEdit (void) const
{
	return m_nButtonPinEdit;
}

unsigned CConfig::GetButtonPinSystem (void) const
{
	return m_nButtonPinSystem;
}

unsigned CConfig::GetButtonPinRhythm (void) const
{
	return m_nButtonPinRhythm;
}

unsigned CConfig::GetButtonPinUtility (void) const
{
	return m_nButtonPinUtility;
}

unsigned CConfig::GetButtonPinMute (void) const
{
	return m_nButtonPinMute;
}

unsigned CConfig::GetButtonPinMonitor (void) const
{
	return m_nButtonPinMonitor;
}

unsigned CConfig::GetButtonPinCompare (void) const
{
	return m_nButtonPinCompare;
}

unsigned CConfig::GetButtonPinEnter (void) const
{
	return m_nButtonPinEnter;
}

unsigned CConfig::GetButtonPinUp (void) const
{
	return m_nButtonPinUp;
}

unsigned CConfig::GetButtonPinDown (void) const
{
	return m_nButtonPinDown;
}

unsigned CConfig::GetButtonPinSaveNVRAM (void) const
{
	return m_nButtonPinSaveNVRAM;
}

const char *CConfig::GetButtonActionPreview (void) const
{
	return m_ButtonActionPreview.c_str();
}

const char *CConfig::GetButtonActionLeft (void) const
{
	return m_ButtonActionLeft.c_str();
}

const char *CConfig::GetButtonActionRight (void) const
{
	return m_ButtonActionRight.c_str();
}

const char *CConfig::GetButtonActionData (void) const
{
	return m_ButtonActionData.c_str();
}

const char *CConfig::GetButtonActionToneSelect (void) const
{
	return m_ButtonActionToneSelect.c_str();
}

const char *CConfig::GetButtonActionPatchPerform (void) const
{
	return m_ButtonActionPatchPerform.c_str();
}

const char *CConfig::GetButtonActionEdit (void) const
{
	return m_ButtonActionEdit.c_str();
}

const char *CConfig::GetButtonActionSystem (void) const
{
	return m_ButtonActionSystem.c_str();
}

const char *CConfig::GetButtonActionRhythm (void) const
{
	return m_ButtonActionRhythm.c_str();
}

const char *CConfig::GetButtonActionUtility (void) const
{
	return m_ButtonActionUtility.c_str();
}

const char *CConfig::GetButtonActionMute (void) const
{
	return m_ButtonActionMute.c_str();
}

const char *CConfig::GetButtonActionMonitor (void) const
{
	return m_ButtonActionMonitor.c_str();
}

const char *CConfig::GetButtonActionCompare (void) const
{
	return m_ButtonActionCompare.c_str();
}

const char *CConfig::GetButtonActionEnter (void) const
{
	return m_ButtonActionEnter.c_str();
}

const char *CConfig::GetButtonActionUp (void) const
{
	return m_ButtonActionUp.c_str();
}

const char *CConfig::GetButtonActionDown (void) const
{
	return m_ButtonActionDown.c_str();
}

const char *CConfig::GetButtonActionSaveNVRAM (void) const
{
	return m_ButtonActionSaveNVRAM.c_str();
}

unsigned CConfig::GetMIDIButtonCh (void) const
{
	return m_nMIDIButtonCh;
}

unsigned CConfig::GetMIDIButtonPreview (void) const
{
    return m_nMIDIButtonPreview;
}

unsigned CConfig::GetMIDIButtonLeft (void) const
{
    return m_nMIDIButtonLeft;
}

unsigned CConfig::GetMIDIButtonRight (void) const
{
    return m_nMIDIButtonRight;
}

unsigned CConfig::GetMIDIButtonData (void) const
{
    return m_nMIDIButtonData;
}

unsigned CConfig::GetMIDIButtonToneSelect (void) const
{
    return m_nMIDIButtonToneSelect;
}

unsigned CConfig::GetMIDIButtonPatchPerform (void) const
{
    return m_nMIDIButtonPatchPerform;
}

unsigned CConfig::GetMIDIButtonEdit (void) const
{
    return m_nMIDIButtonEdit;
}

unsigned CConfig::GetMIDIButtonSystem (void) const
{
    return m_nMIDIButtonSystem;
}

unsigned CConfig::GetMIDIButtonRhythm (void) const
{
    return m_nMIDIButtonRhythm;
}

unsigned CConfig::GetMIDIButtonUtility (void) const
{
    return m_nMIDIButtonUtility;
}

unsigned CConfig::GetMIDIButtonMute (void) const
{
    return m_nMIDIButtonMute;
}

unsigned CConfig::GetMIDIButtonMonitor (void) const
{
    return m_nMIDIButtonMonitor;
}

unsigned CConfig::GetMIDIButtonCompare (void) const
{
    return m_nMIDIButtonCompare;
}

unsigned CConfig::GetMIDIButtonEnter (void) const
{
    return m_nMIDIButtonEnter;
}

unsigned CConfig::GetMIDIButtonUp (void) const
{
    return m_nMIDIButtonUp;
}

unsigned CConfig::GetMIDIButtonDown (void) const
{
    return m_nMIDIButtonDown;
}

unsigned CConfig::GetMIDISaveNVRAM (void) const
{
    return m_nMIDISaveNVRAM;
}

unsigned CConfig::GetMIDIEncoder (void) const
{
    return m_nMIDIEncoder;
}

unsigned CConfig::GetMIDIEncoderCC (void) const
{
    return m_nMIDIEncoderCC;
}

unsigned CConfig::GetMIDIEncoderUp (void) const
{
    return m_nMIDIEncoderUp;
}

unsigned CConfig::GetMIDIEncoderDown (void) const
{
    return m_nMIDIEncoderDown;
}

bool CConfig::GetEncoderEnabled (void) const
{
	return m_bEncoderEnabled;
}

unsigned CConfig::GetEncoderPinClock (void) const
{
	return m_nEncoderPinClock;
}

unsigned CConfig::GetEncoderPinData (void) const
{
	return m_nEncoderPinData;
}

unsigned CConfig::GetDoubleClickTimeout (void) const
{
	return m_nDoubleClickTimeout;
}

unsigned CConfig::GetLongPressTimeout (void) const
{
	return m_nLongPressTimeout;
}

bool CConfig::GetProfileEnabled (void) const
{
	return m_bProfileEnabled;
}

// Network
bool CConfig::GetNetworkEnabled (void) const
{
	return m_bNetworkEnabled;
}

bool CConfig::GetNetworkDHCP (void) const
{
	return m_bNetworkDHCP;
}

const char *CConfig::GetNetworkType (void) const
{
	return m_NetworkType.c_str();
}

const char *CConfig::GetNetworkHostname (void) const
{
	return m_NetworkHostname.c_str();
}

const CIPAddress& CConfig::GetNetworkIPAddress (void) const
{	
	return m_INetworkIPAddress;
}

const CIPAddress& CConfig::GetNetworkSubnetMask (void) const
{
	return m_INetworkSubnetMask;
}

const CIPAddress& CConfig::GetNetworkDefaultGateway (void) const
{
	return m_INetworkDefaultGateway;
}

const CIPAddress& CConfig::GetNetworkDNSServer (void) const
{
	return m_INetworkDNSServer;
}

bool CConfig::GetSyslogEnabled (void) const
{
	return m_bSyslogEnabled;
}

const CIPAddress& CConfig::GetNetworkSyslogServerIPAddress (void) const
{
	return m_INetworkSyslogServerIPAddress;
}

bool CConfig::GetNetworkFTPEnabled (void) const
{
	return m_bNetworkFTPEnabled;
}

bool CConfig::GetUDPMIDIEnabled (void) const
{
	return m_bUDPMIDIEnabled;
}

const CIPAddress& CConfig::GetUDPMIDIIPAddress (void) const
{
	return m_IUDPMIDIIPAddress;
}
