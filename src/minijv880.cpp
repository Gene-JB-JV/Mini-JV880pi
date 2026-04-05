//
// minijv880.cpp
//
// Mini-JV880pi - Roland JV880 synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
// Copyright (C) 2026  Plamikcho, Giulioz, Gene J.B. (Sterr1)
//
// Original MCU emulator replaced with native JV880Engine (jv880_engine.h)
// No MCU instruction execution. Audio generated directly via Tick().
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
#include "minijv880.h"
#include "midi.h"
#include "userinterface.h"
#include "jv880_engine.h"   // ← native engine (replaces mcu.*)
#include <assert.h>
#include <circle/memory.h>
#include <circle/devicenameservice.h>
#include <circle/gpiopin.h>
#include <circle/logger.h>
#include <circle/sound/hdmisoundbasedevice.h>
#include <circle/sound/i2ssoundbasedevice.h>
#include <circle/sound/pwmsoundbasedevice.h>
#include <circle/usb/usbmidihost.h>
#include <circle/net/syslogdaemon.h>
#include <circle/net/ipaddress.h>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <algorithm>

const char WLANFirmwarePath[] = "SD:firmware/";
const char WLANConfigFile[]   = "SD:wpa_supplicant.conf";
#define FTPUSERNAME "admin"
#define FTPPASSWORD "admin"

LOGMODULE("minijv880");

CMiniJV880 *CMiniJV880::s_pThis = 0;

// ─── ROM table (same as before — indices 0..26) ──────────────────────────────
CMiniJV880::RomInfo CMiniJV880::m_romInfos[27] = {
    m_romInfos[0]  = {sz32K,  "jv880_nvram.bin",   false, false, false, nullptr},
    m_romInfos[1]  = {sz32K,  "jv880_rom1.bin",    false, false, false, nullptr},  // not used by engine
    m_romInfos[2]  = {sz256K, "jv880_rom2.bin",    false, false, false, nullptr},
    m_romInfos[3]  = {sz2M,   "jv880_waverom1.bin",true,  false, true,  nullptr},
    m_romInfos[4]  = {sz2M,   "jv880_waverom2.bin",true,  false, true,  nullptr},
    m_romInfos[5]  = {sz128K, "rd500_patches.bin", false, false, false, nullptr},
    m_romInfos[6]  = {sz8M,   "rd500_expansion.bin",             true, false, true, nullptr},
    m_romInfos[7]  = {sz8M,   "SR-JV80-01 Pop - CS 0x3F1CF705.bin",               true, false, true, nullptr},
    m_romInfos[8]  = {sz8M,   "SR-JV80-02 Orchestral - CS 0x3F0E09E2.BIN",        true, false, true, nullptr},
    m_romInfos[9]  = {sz8M,   "SR-JV80-03 Piano - CS 0x3F8DB303.bin",             true, false, true, nullptr},
    m_romInfos[10] = {sz8M,   "SR-JV80-04 Vintage Synth - CS 0x3E23B90C.BIN",     true, false, true, nullptr},
    m_romInfos[11] = {sz8M,   "SR-JV80-05 World - CS 0x3E8E8A0D.bin",             true, false, true, nullptr},
    m_romInfos[12] = {sz8M,   "SR-JV80-06 Dance - CS 0x3EC462E0.bin",             true, false, true, nullptr},
    m_romInfos[13] = {sz8M,   "SR-JV80-07 Super Sound Set - CS 0x3F1EE208.bin",   true, false, true, nullptr},
    m_romInfos[14] = {sz8M,   "SR-JV80-08 Keyboards of the 60s and 70s - CS 0x3F1E3F0A.BIN", true, false, true, nullptr},
    m_romInfos[15] = {sz8M,   "SR-JV80-09 Session - CS 0x3F381791.BIN",           true, false, true, nullptr},
    m_romInfos[16] = {sz8M,   "SR-JV80-10 Bass & Drum - CS 0x3D83D02A.BIN",       true, false, true, nullptr},
    m_romInfos[17] = {sz8M,   "SR-JV80-11 Techno - CS 0x3F046250.bin",            true, false, true, nullptr},
    m_romInfos[18] = {sz8M,   "SR-JV80-12 HipHop - CS 0x3EA08A19.BIN",            true, false, true, nullptr},
    m_romInfos[19] = {sz8M,   "SR-JV80-13 Vocal - CS 0x3ECE78AA.bin",             true, false, true, nullptr},
    m_romInfos[20] = {sz8M,   "SR-JV80-14 Asia - CS 0x3C8A1582.bin",              true, false, true, nullptr},
    m_romInfos[21] = {sz8M,   "SR-JV80-15 Special FX - CS 0x3F591CE4.bin",        true, false, true, nullptr},
    m_romInfos[22] = {sz8M,   "SR-JV80-16 Orchestral II - CS 0x3F35B03B.bin",     true, false, true, nullptr},
    m_romInfos[23] = {sz8M,   "SR-JV80-17 Country - CS 0x3ED75089.bin",           true, false, true, nullptr},
    m_romInfos[24] = {sz8M,   "SR-JV80-18 Latin - CS 0x3EA51033.BIN",             true, false, true, nullptr},
    m_romInfos[25] = {sz8M,   "SR-JV80-19 House - CS 0x3E330C41.BIN",             true, false, true, nullptr},
    m_romInfos[26] = {sz8M,   "rd500_expansion.bin",             true, false, true, nullptr},
};

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
CMiniJV880::CMiniJV880(CConfig *pConfig, CInterruptSystem *pInterrupt,
                       CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster,
                       CSPIMaster *pSPIMaster, FATFS *pFileSystem,
                       CScreenDevice *mScreenUnbuffered)
    : CMultiCoreSupport(CMemorySystem::Get()),
      m_pConfig(pConfig),
      m_pFileSystem(pFileSystem),
      m_Serial(pInterrupt, TRUE),
      m_pSoundDevice(nullptr),
      screenUnbuffered(mScreenUnbuffered),
      m_bChannelsSwapped(pConfig->GetChannelsSwapped()),
      m_UI(this, pGPIOManager, pI2CMaster, pSPIMaster, pConfig),
      m_pNet(nullptr),
      m_pNetDevice(nullptr),
      m_WLAN(nullptr),
      m_WPASupplicant(nullptr),
      m_bNetworkReady(false),
      m_bNetworkInit(false),
      m_UDPMIDI(nullptr),
      m_pmDNSPublisher(nullptr),
      m_lastTick(0),
      m_lastTick1(0)
{
    CTimer::Get();
    assert(m_pConfig);
    s_pThis = this;

    // Wire encoder trigger from UI facade → engine patch navigation
    mcu.encoder_trigger_cb = [this](int dir)  { EncoderTrigger(dir); };
    mcu.tone_mute_cb       = [this](int tone) { ToggleToneMute(tone); };
    mcu.save_nvram_cb      = [this]()          { SaveNVRAMIncremental(); };

    m_nPendingBankSwitch.store(0xFF, std::memory_order_release);

    const char *pDeviceName = pConfig->GetSoundDevice();
    if (strcmp(pDeviceName, "i2s") == 0) {
        LOGNOTE("I2S mode");
        m_pSoundDevice = new CI2SSoundBaseDevice(
            pInterrupt, pConfig->GetSampleRate(), pConfig->GetChunkSize(),
            false, pI2CMaster, pConfig->GetDACI2CAddress(),
            CI2SSoundBaseDevice::DeviceModeTXOnly, 2);
    } else if (strcmp(pDeviceName, "hdmi") == 0) {
#if RASPPI == 5
        LOGNOTE("HDMI mode NOT supported on RPI 5.");
#else
        LOGNOTE("HDMI mode");
        m_pSoundDevice = new CHDMISoundBaseDevice(
            pInterrupt, pConfig->GetSampleRate(), pConfig->GetChunkSize());
        m_bChannelsSwapped = !m_bChannelsSwapped;
#endif
    } else {
        LOGNOTE("PWM mode");
        m_pSoundDevice = new CPWMSoundBaseDevice(
            pInterrupt, pConfig->GetSampleRate(), pConfig->GetChunkSize());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor
// ─────────────────────────────────────────────────────────────────────────────
CMiniJV880::~CMiniJV880()
{
    if (m_pmDNSPublisher) { delete m_pmDNSPublisher; m_pmDNSPublisher = nullptr; }
    if (m_pFTPDaemon)     { delete m_pFTPDaemon;     m_pFTPDaemon     = nullptr; }
    if (m_UDPMIDI)        { delete m_UDPMIDI;         m_UDPMIDI        = nullptr; }
    if (m_WPASupplicant)  { delete m_WPASupplicant;   m_WPASupplicant  = nullptr; }
    if (m_WLAN)           { delete m_WLAN;             m_WLAN           = nullptr; }
    if (m_pNet)           { delete m_pNet;             m_pNet           = nullptr; }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Initialize
// ─────────────────────────────────────────────────────────────────────────────
bool CMiniJV880::Initialize()
{
    assert(m_pConfig);
    assert(m_pSoundDevice);

    if (!m_UI.Initialize()) {
        LOGERR("Failed to initialize UI");
        return false;
    }

    if (!m_Serial.Initialize(m_pConfig->GetMIDIBaudRate())) {
        LOGERR("Failed to initialize Serial MIDI");
        return false;
    }
    unsigned ser_options = m_Serial.GetOptions();
    ser_options &= ~(SERIAL_OPTION_ONLCR);
    m_Serial.SetOptions(ser_options);
    LOGNOTE("Serial MIDI Initialized");

    InitBankMappings();
    midiParser.Init(this);

    // ── Load ROM files ────────────────────────────────────────────────────
    if (!LoadMainRoms(m_pConfig->GetExpRom()))
        return false;

    // ── Init JV880Engine (replaces mcu.startSC55) ────────────────────────
    {
        const uint8_t* nvram  = (const uint8_t*)m_romInfos[0].data;
        // rom1 is NOT used by the native engine (firmware runs natively)
        const uint8_t* rom2   = (const uint8_t*)m_romInfos[2].data;
        // JV-880 PCM ROM mapping (from pcm.h PCM_ReadROM):
        //   jv880_waverom1.bin → waverom_exp segment 0 (bank 3, hiaddr 6..7)
        //   jv880_waverom2.bin → waverom_exp segment 1 (bank 4, hiaddr 8..9)
        // All base JV-880 voices use hiaddr=6,7 (bank=3) → waverom1 covers all built-ins.
        // waverom_card (bank 2, hiaddr 4..5) and wrom[0,1] (bank 0,1) are not used.
        const uint8_t* pcm1   = (const uint8_t*)m_romInfos[3].data;  // → bank 3 (hiaddr 6,7)
        const uint8_t* pcm2   = (const uint8_t*)m_romInfos[4].data;  // → bank 4 (hiaddr 8,9)

        // Init() maps: waverom1 → wrom_exp_ (bank 3), waverom2 → wrom_exp2_ (bank 4)
        m_Engine.Init(rom2, pcm1, pcm2, nvram);

        // SR-JV80 expansion card overrides wrom_exp_ (bank 3) if present
        if (m_pConfig->GetExpRom() != 0) {
            const uint8_t* exp = (const uint8_t*)m_romInfos[m_pConfig->GetExpRom() + 6].data;
            m_Engine.SetExpansionWaverom(exp);
        }

        if (!m_Engine.IsReady()) {
            LOGERR("JV880Engine failed to initialize");
            return false;
        }
        LOGNOTE("JV880Engine started — patch: %.12s", m_Engine.PatchName());

        // Mark engine as "running" for UI (replaces mcu.mcu.pc != 0 check)
        mcu.mcu.pc = 1;
        SyncMCUCompat();  // initial LCD fill
    }

    // ── Audio device setup ────────────────────────────────────────────────
    int Channels = 2;
    if (!m_pSoundDevice->AllocateQueueFrames(
            2 * m_pConfig->GetChunkSize() / Channels)) {
        LOGERR("Cannot allocate sound queue");
        return false;
    }
    m_pSoundDevice->SetWriteFormat(SoundFormatSigned16, Channels);
    m_nQueueSizeFrames = m_pSoundDevice->GetQueueSizeFrames();
    m_pSoundDevice->Start();

    if (!CMultiCoreSupport::Initialize())
        return false;

    InitNetwork();
    LOGNOTE("CMiniJV880::Initialize: done");

    size_t free = CMemorySystem::Get()->GetHeapFreeSpace(HEAP_ANY);
    LOGNOTE("Free memory: %u bytes (%.2f MB)", free, (float)free / (1024.f * 1024.f));
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Process  (Core 0 — UI + MIDI receive)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::Process(bool bPlugAndPlayUpdated)
{
    CScheduler* const pScheduler = CScheduler::Get();

    SyncMCUCompat();  // refresh mcu.lcd.LCD_Data, mcu.jv880_led_state

    m_UI.Process();
    pScheduler->Yield();

    if (m_pNet)
        UpdateNetwork();
    pScheduler->Yield();

    // Pending bank switch (debounced)
    uint8_t pendingBank = m_nPendingBankSwitch.load(std::memory_order_acquire);
    if (pendingBank != 0xFF) {
        unsigned elapsed = CTimer::GetClockTicks()
                         - m_nBankSwitchTimestamp.load(std::memory_order_acquire);
        if (elapsed >= BANK_SWITCH_DEBOUNCE_US) {
            switchPatchBank(pendingBank);
            pScheduler->Yield();
            m_nPendingBankSwitch.store(0xFF, std::memory_order_release);
        }
    }

    // Serial MIDI receive
    int nRead = m_Serial.Read(m_MIDIBuffer, sizeof(m_MIDIBuffer));
    pScheduler->Yield();
    if (nRead > 0) {
        midiParser.FeedSerialBytes(m_MIDIBuffer, nRead);
        pScheduler->Yield();
    }

    if (!bPlugAndPlayUpdated)
        return;

    if (m_pMIDIDevice == nullptr) {
        m_pMIDIDevice = (CUSBMIDIDevice*)
            CDeviceNameService::Get()->GetDevice("umidi1", FALSE);
        if (m_pMIDIDevice) {
            m_pMIDIDevice->RegisterPacketHandler(USBMIDIMessageHandler);
            m_pMIDIDevice->RegisterRemovedHandler(DeviceRemovedHandler, this);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  USB MIDI handler
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::USBMIDIMessageHandler(unsigned /*nCable*/, u8 *pPacket,
                                       unsigned nLength)
{
    if (!pPacket || nLength == 0) return;
    s_pThis->midiParser.FeedUSBMIDIPacket(pPacket, nLength);
}

// ─────────────────────────────────────────────────────────────────────────────
//  HandleFullMIDIMessage  — called by midiParser after reassembly
//  Replaces: mcu.postMidiSC55(pData, nLength)
//  With:     for each byte → m_Engine.MidiIn(byte)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::HandleFullMIDIMessage(const uint8_t* pData, uint8_t nLength)
{
    if (nLength == 0) return;

    uint8_t status = pData[0];

    // ── Note On/Off ────────────────────────────────────────────────────────
    if (((status & 0xF0) == 0x80 || (status & 0xF0) == 0x90) && nLength == 3) {
        PostMIDIBytes(pData, nLength);
        return;
    }

    // ── Poly Aftertouch ───────────────────────────────────────────────────
    if ((status & 0xF0) == 0xA0 && nLength == 3) {
        PostMIDIBytes(pData, nLength);
        return;
    }

    // ── Channel Pressure (Aftertouch) ─────────────────────────────────────
    if ((status & 0xF0) == 0xD0 && nLength == 2) {
        PostMIDIBytes(pData, nLength);
        return;
    }

    // ── Pitch Bend ────────────────────────────────────────────────────────
    if ((status & 0xF0) == 0xE0 && nLength == 3) {
        PostMIDIBytes(pData, nLength);
        return;
    }

    // ── Modulation CC1 ────────────────────────────────────────────────────
    if ((status & 0xF0) == 0xB0 && nLength == 3 && pData[1] == 1) {
        PostMIDIBytes(pData, nLength);
        return;
    }

    // ── Bank Select ───────────────────────────────────────────────────────
    if ((status & 0xF0) == 0xB0 && nLength == 3) {
        uint8_t ch  = status & 0x0F;
        uint8_t cc  = pData[1];
        uint8_t val = pData[2] & 0x7F;

        if (cc == 0) {   // CC 0 MSB
            m_nBankMSB[ch] = val;
            if (val != 0) PostMIDIBytes(pData, nLength);
            return;
        }

        if (cc == 32) {  // CC 32 LSB
            uint8_t msb = m_nBankMSB[ch];
            if (msb != 0 || val == 80 || val == 81) {
                PostMIDIBytes(pData, nLength);
                return;
            }
            // MSB=0, LSB=0..79,82..127 → switch patch bank
            m_nPendingBankSwitch.store(val, std::memory_order_release);
            m_nBankSwitchTimestamp.store(CTimer::GetClockTicks(),
                                         std::memory_order_release);
            return;
        }
    }

    // ── NVRAM Save ────────────────────────────────────────────────────────
    if ((status & 0xF0) == 0xB0 && nLength == 3
        && pData[1] == m_UI.m_nMIDISaveNVRAM && pData[2] == 0) {
        SaveNVRAMIncremental();
        return;
    }

    // ── UI CC (buttons / encoder) ─────────────────────────────────────────
    if ((status & 0xF0) == 0xB0 && nLength == 3) {
        uint8_t ccNumber = pData[1];
        uint8_t ccValue  = pData[2];

        if (m_UI.m_nMIDIButtonChannel != 0) {
            uint8_t ch       = status & 0x0F;
            uint8_t expected = m_UI.m_nMIDIButtonChannel - 1;

            if (m_UI.m_nMIDIButtonChannel == 17 || expected == ch) {
                auto handleBtn = [this, ccNumber, ccValue](uint8_t confCC,
                                                           CUIButton::BtnEvent ev) {
                    if (ccNumber != confCC) return false;
                    if (ccValue < 64) m_UI.TriggerUIButtonEvent(ev);
                    else              m_UI.TriggerUIButtonEvent(CUIButton::BtnEventRelease);
                    return true;
                };

                if (handleBtn(m_UI.m_nMIDIPreview,      CUIButton::BtnEventPreview))      return;
                if (handleBtn(m_UI.m_nMIDILeft,         CUIButton::BtnEventLeft))          return;
                if (handleBtn(m_UI.m_nMIDIRight,        CUIButton::BtnEventRight))         return;
                if (handleBtn(m_UI.m_nMIDIData,         CUIButton::BtnEventData))          return;
                if (handleBtn(m_UI.m_nMIDIToneSelect,   CUIButton::BtnEventToneSelect))    return;
                if (handleBtn(m_UI.m_nMIDIPatchPerform, CUIButton::BtnEventPatchPerform))  return;
                if (handleBtn(m_UI.m_nMIDIEdit,         CUIButton::BtnEventEdit))          return;
                if (handleBtn(m_UI.m_nMIDISystem,       CUIButton::BtnEventSystem))        return;
                if (handleBtn(m_UI.m_nMIDIRhythm,       CUIButton::BtnEventRhythm))        return;
                if (handleBtn(m_UI.m_nMIDIUtility,      CUIButton::BtnEventUtility))       return;
                if (handleBtn(m_UI.m_nMIDIMute,         CUIButton::BtnEventMute))          return;
                if (handleBtn(m_UI.m_nMIDIMonitor,      CUIButton::BtnEventMonitor))       return;
                if (handleBtn(m_UI.m_nMIDICompare,      CUIButton::BtnEventCompare))       return;
                if (handleBtn(m_UI.m_nMIDIEnter,        CUIButton::BtnEventEnter))         return;

                // Encoder emulation via buttons
                if (ccNumber == m_UI.m_nMIDIUp && ccValue < 64) {
                    m_UI.TriggerUIButtonEvent(CUIButton::BtnEventRelease);
                    EncoderTrigger(1);
                    return;
                }
                if (ccNumber == m_UI.m_nMIDIDown && ccValue < 64) {
                    m_UI.TriggerUIButtonEvent(CUIButton::BtnEventRelease);
                    EncoderTrigger(0);
                    return;
                }
                // MIDI encoder CC
                if (m_UI.m_nMIDIEncoder) {
                    if (ccNumber == m_UI.m_nMIDIEncoderCC) {
                        bool up   = (m_UI.m_nMIDIEncoderUp   == 0) ? (ccValue < 64) : (ccValue > 64);
                        bool down = (m_UI.m_nMIDIEncoderDown == 0) ? (ccValue < 64) : (ccValue > 64);
                        if (up)   { m_UI.TriggerUIButtonEvent(CUIButton::BtnEventRelease); EncoderTrigger(1); return; }
                        if (down) { m_UI.TriggerUIButtonEvent(CUIButton::BtnEventRelease); EncoderTrigger(0); return; }
                    }
                }
            }
        }
    }

    // ── Roland SysEx (add checksum) ───────────────────────────────────────
    if (pData[0] == 0xF0 && nLength > 7 && pData[nLength - 1] == 0xF7
        && pData[1] == 0x41) {
        int chk_idx = nLength - 2;
        if (chk_idx >= 6) {
            int sum = 0;
            for (int i = 0; i < 5; i++) sum += pData[chk_idx - 5 + i];
            sum &= 0x7F;
            uint8_t cs = (128 - sum) & 0x7F;

            uint8_t out[256];
            memcpy(out, pData, nLength);
            out[chk_idx] = cs;
            PostMIDIBytes(out, nLength);
            return;
        }
    }

    // ── Everything else ────────────────────────────────────────────────────
    PostMIDIBytes(pData, nLength);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SyncMCUCompat  — copies engine state into the mcu.* compatibility facade
//  Called from Process() once per UI tick (~60 Hz) on Core 0.
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::SyncMCUCompat()
{
    if (!m_Engine.IsReady()) return;

    const JV880Engine::LCDState& lcd = m_Engine.GetLCD();

    // Fill LCD_Data rows 0 and 1 (each 40 chars, space-padded to full width)
    // Row 0: lcd.line[0] (20 chars) padded to 40
    memcpy(mcu.lcd.LCD_Data,      lcd.line[0], 20);
    memset(mcu.lcd.LCD_Data + 20, 0x20, 20);   // pad right half of row 0

    // Row 1: lcd.line[1] (20 chars) padded to 40
    memcpy(mcu.lcd.LCD_Data + 40, lcd.line[1], 20);
    memset(mcu.lcd.LCD_Data + 60, 0x20, 20);   // pad right half of row 1

    // LED state
    mcu.jv880_led_state = (uint16_t)(lcd.led_state & 0xFFFF);

    // Cursor: native engine has no hardware cursor
    mcu.lcd.LCD_DD_RAM = 0;
    mcu.lcd.LCD_C      = 0;

    // pc = 1 means "engine is running" (used by RenderDisplay emuActive check)
    mcu.mcu.pc = m_Engine.IsReady() ? 1 : 0;

}

// ─────────────────────────────────────────────────────────────────────────────
//  PostMIDIBytes  (replaces mcu.postMidiSC55)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::PostMIDIBytes(const uint8_t* pData, uint8_t nLength)
{
    for (int i = 0; i < nLength; i++)
        m_MIDIQueue.Push(pData[i]);
}

// EncoderTrigger and ToggleToneMute are inlined in minijv880.h

// ─────────────────────────────────────────────────────────────────────────────
//  Run  — multicore entry points
//
//  Core 0: Process() — UI + MIDI receive  (called from kernel)
//  Core 1: unused (available for future use)
//  Core 2: Audio output — fills sound device from engine Tick()
//  Core 3: not needed (PCM was separate in MCU mode; now merged into Core 2)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::Run(unsigned nCore)
{
    assert(1 <= nCore && nCore < CORES);

    if (nCore == 2)
    {
        // ── Core 2: Audio output ─────────────────────────────────────────
        // Drain MIDI queue into engine, then generate samples directly.
        // No MCU instruction execution. No ring buffer. Simple.
        //
        // Sample rate: JV880Engine::SAMPLE_RATE = 32000 Hz (32 kHz)
        // The sound device may run at a different rate (e.g. 44100).
        // We do simple nearest-neighbour upsampling here.
        // For better quality, replace with a linear interpolator.

        const uint32_t kEngineRate = 32000;
        const uint32_t kDeviceRate = m_pConfig->GetSampleRate();

        // Fixed-point ratio: how many engine samples per device sample (Q16)
        const uint32_t kRatio_Q16 = (kEngineRate << 16) / kDeviceRate;
        uint32_t phase_Q16 = 0;

        int16_t last_l = 0, last_r = 0;  // last engine sample (for upsampling)

        while (true)
        {
            if (m_bAudioPaused.load(std::memory_order_acquire)) {
                CTimer::SimpleMsDelay(1);
                continue;
            }

            // How many device frames the queue needs
            unsigned nFrames = m_nQueueSizeFrames
                             - m_pSoundDevice->GetQueueFramesAvail();
            if (nFrames < m_nQueueSizeFrames / 2) {
                CTimer::SimpleMsDelay(1);
                continue;
            }

            // Allocate output buffer (stereo int16)
            int nSamples = (int)nFrames * 2;  // L+R interleaved
            int16_t* out_buf = (int16_t*)malloc(nSamples * sizeof(int16_t));
            if (!out_buf) { CTimer::SimpleMsDelay(1); continue; }

            // Drain MIDI queue
            {
                uint8_t byte;
                while (m_MIDIQueue.Pop(byte))
                    m_Engine.MidiIn(byte);
            }

            // Generate device-rate samples via resampling
            for (int i = 0; i < nSamples; i += 2)
            {
                // Advance engine by however many 32kHz ticks correspond to
                // one device-rate sample
                phase_Q16 += kRatio_Q16;
                while (phase_Q16 >= 0x10000u) {
                    m_Engine.Tick(last_l, last_r);
                    phase_Q16 -= 0x10000u;
                }

                if (m_bChannelsSwapped) {
                    out_buf[i]     = last_r;
                    out_buf[i + 1] = last_l;
                } else {
                    out_buf[i]     = last_l;
                    out_buf[i + 1] = last_r;
                }
            }

            int len = nSamples * sizeof(int16_t);
            if (m_pSoundDevice->Write(out_buf, len) != len)
                LOGERR("Sound data dropped");
            free(out_buf);
        }
    }
    else if (nCore == 3)
    {
        // Core 3 is now idle — the engine's Tick() is called on Core 2.
        // Keep alive for future use (temperature monitor, MIDI thru, etc.)
        while (true)
            CTimer::SimpleMsDelay(100);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Bank switch  (replaces mcu.SC55_Reset + waverom_exp copy)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::switchPatchBank(int bankNumber)
{
    if (bankNumber < 0 || bankNumber > 99) return;

    int romIndex = -1;
    const char* nvramFilename = nullptr;
    for (unsigned i = 0; i < m_bankMappingsCount; i++) {
        if (m_bankMappings[i].bankNumber == bankNumber) {
            romIndex      = m_bankMappings[i].romIndex;
            nvramFilename = m_bankMappings[i].nvramFilename;
            break;
        }
    }
    if (romIndex == -1) { LOGNOTE("Bank %d not found", bankNumber); return; }
    if ((size_t)romIndex >= ROM_COUNT) {
        LOGERR("ROM index %d OOB for bank %d", romIndex, bankNumber);
        return;
    }

    RomInfo& rom = m_romInfos[romIndex];
    bool loadRomData = (bankNumber != 0);

    if (loadRomData && !rom.isLoaded && !LoadRom(romIndex)) {
        LOGERR("Failed to load ROM for bank %d", bankNumber);
        return;
    }

    // ── Pause audio ──────────────────────────────────────────────────────
    m_bAudioPaused.store(true, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    CTimer::SimpleMsDelay(100);  // let Core 2 exit its loop iteration

    // ── Load NVRAM and re-init ────────────────────────────────────────────
    // Init() always sets wrom_exp_ = built-in waverom1.
    // Then SetExpansionWaverom() overrides with SR-JV80 card if loaded.
    if (nvramFilename) {
        char path[64];
        snprintf(path, sizeof(path), "patch/%s", nvramFilename);

        FIL file;
        if (f_open(&file, path, FA_READ) == FR_OK) {
            UINT bytesRead;
            FRESULT res = f_read(&file, m_nvram, sizeof(m_nvram), &bytesRead);
            f_close(&file);
            if (res == FR_OK && bytesRead == sizeof(m_nvram)) {
                const uint8_t* rom2 = (const uint8_t*)m_romInfos[2].data;
                const uint8_t* pcm1 = (const uint8_t*)m_romInfos[3].data;
                const uint8_t* pcm2 = (const uint8_t*)m_romInfos[4].data;
                // Re-init: sets wrom_exp_ = built-in pcm1
                m_Engine.Init(rom2, pcm1, pcm2, m_nvram);
                // Then override with SR-JV80 expansion ROM if loaded
                if (loadRomData)
                    m_Engine.SetExpansionWaverom((const uint8_t*)rom.data);
            } else {
                LOGERR("Failed to read NVRAM from %s", nvramFilename);
                // Just silence voices without full re-init
                m_Engine.ReloadVoices();
            }
        } else {
            LOGERR("Cannot open NVRAM %s", path);
            m_Engine.ReloadVoices();
        }
    } else {
        m_Engine.ReloadVoices();
    }

    std::atomic_thread_fence(std::memory_order_seq_cst);
    CTimer::SimpleMsDelay(50);

    // ── Resume ────────────────────────────────────────────────────────────
    m_bAudioPaused.store(false, std::memory_order_release);

    size_t free = CMemorySystem::Get()->GetHeapFreeSpace(HEAP_ANY);
    LOGNOTE("=== BANK → %d ROM %d (%s) free=%.2f MB ===",
            bankNumber, romIndex, rom.filename,
            (float)free / (1024.f * 1024.f));
}

// ─────────────────────────────────────────────────────────────────────────────
//  NVRAM save
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::SaveNVRAMIncremental()
{
    char filename[64];

    FRESULT dirRes = f_mkdir("nvram");
    if (dirRes != FR_OK && dirRes != FR_EXIST) {
        LOGERR("Cannot create nvram directory: %d", dirRes);
        return;
    }

    FIL file;
    FRESULT res;
    do {
        sprintf(filename, "nvram/jv880_nvram%d.bin", ++m_nNVRAMSaveCounter);
        res = f_open(&file, filename, FA_READ);
        if (res == FR_OK) f_close(&file);
    } while (res == FR_OK);

    m_UI.LCDMessage("Saving NVRAM file\njv880_nvram%d.bin", m_nNVRAMSaveCounter);

    res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) { LOGERR("Cannot open %s: %d", filename, res); return; }

    UINT bytesWritten;
    res = f_write(&file, m_nvram, sizeof(m_nvram), &bytesWritten);
    f_close(&file);

    if (res == FR_OK && bytesWritten == sizeof(m_nvram)) {
        LOGNOTE("NVRAM saved to %s", filename);
        m_UI.LCDMessage("Saved NVRAM file\njv880_nvram%d.bin", m_nNVRAMSaveCounter);
    } else {
        LOGERR("NVRAM save failed: %s written=%u err=%d", filename, bytesWritten, res);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Device removed
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::DeviceRemovedHandler(CDevice *pDevice, void *pContext)
{
    CMiniJV880 *pThis = static_cast<CMiniJV880*>(pContext);
    assert(pThis);
    if (pDevice == pThis->m_pMIDIDevice)
        pThis->m_pMIDIDevice = nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ROM loading  (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
bool CMiniJV880::LoadMainRoms(uint8_t ExpRom)
{
    int indices[6];
    unsigned cr;

    if (ExpRom == 0 || ExpRom > 19) {
        const int idx[] = {0, 1, 2, 3, 4};
        memcpy(indices, idx, sizeof(idx));
        cr = 5;
    } else {
        const int idx[] = {0, 1, 2, 3, 4, ExpRom + 6};
        memcpy(indices, idx, sizeof(idx));
        cr = 6;
    }

    for (unsigned i = 0; i < cr; i++) {
        if (!LoadRom(indices[i])) {
            LOGERR("Failed to load ROM at index %d", indices[i]);
            return false;
        }
    }
    LOGNOTE("All main ROMs loaded");
    return true;
}

bool CMiniJV880::LoadRom(uint8_t rom_index)
{
    if (rom_index >= ROM_COUNT) { LOGERR("Invalid ROM index: %d", rom_index); return false; }

    RomInfo& rom = m_romInfos[rom_index];
    std::string fullPath = "roms/";
    fullPath += rom.filename;

    if (!FileExists(fullPath.c_str())) { LOGERR("ROM not found: %s", rom.filename); return false; }

    m_UI.LCDMessage("Loading file\n%s", rom.filename);
    m_UI.RenderDisplay();
    m_UI.GetLCDBuffered()->Update(256);

    if (rom.isLoaded) return true;

    rom.data = malloc(rom.size);
    if (!rom.data) { LOGERR("OOM for %s (%zu)", fullPath.c_str(), rom.size); return false; }

    if (!LoadFile(fullPath.c_str(), (uint8_t*)rom.data, rom.size)) {
        LOGERR("Cannot load %s", fullPath.c_str());
        free(rom.data); rom.data = nullptr;
        return false;
    }

    if (rom.needsUnscramble) {
        uint8_t* desc = (uint8_t*)malloc(rom.size);
        if (!desc) { LOGERR("OOM for descramble %s", fullPath.c_str()); free(rom.data); rom.data = nullptr; return false; }
        UnscrambleRom((uint8_t*)rom.data, desc, rom.size);
        free(rom.data);
        rom.data = desc;
    }

    LOGNOTE("Loaded %s", rom.filename);
    CTimer::SimpleMsDelay(300);
    rom.isLoaded = true;
    return true;
}

bool CMiniJV880::FileExists(const char* filename)
{
    FILE* f = fopen(filename, "rb");
    if (f) { fclose(f); return true; }
    return false;
}

bool CMiniJV880::LoadFile(const char* filename, uint8_t* buffer, size_t size)
{
    FIL f;
    UINT nRead = 0;
    if (f_open(&f, filename, FA_READ | FA_OPEN_EXISTING) != FR_OK) {
        LOGERR("Cannot open %s", filename);
        return false;
    }
    FRESULT fr = f_read(&f, buffer, size, &nRead);
    f_close(&f);
    if (fr != FR_OK) { LOGERR("f_read error %d for %s", fr, filename); return false; }
    if (nRead != size) { LOGERR("Size mismatch %s: expected %zu read %u", filename, size, nRead); return false; }
    return true;
}

void CMiniJV880::UnscrambleRom(const uint8_t *src, uint8_t *dst, int len)
{
    static const int aa[] = {2,0,3,4,1,9,13,10,18,17,6,15,11,16,8,5,12,7,14,19};
    static const int dd[] = {2,0,4,5,7,6,3,1};
    for (int i = 0; i < len; i++) {
        int address = i & ~0xfffff;
        for (int j = 0; j < 20; j++)
            if (i & (1 << j)) address |= 1 << aa[j];
        uint8_t srcdata = src[address];
        uint8_t data = 0;
        for (int j = 0; j < 8; j++)
            if (srcdata & (1 << dd[j])) data |= 1 << j;
        dst[i] = data;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Bank mappings  (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::InitBankMappings()
{
    m_bankMappingsCount    = 0;
    m_bankMappingsCapacity = 32;
    m_bankMappings         = new BankMapping[m_bankMappingsCapacity];

    DIR dir; FILINFO fno;
    if (f_opendir(&dir, "patch") != FR_OK) return;
    while (true) {
        if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0) break;
        if (strstr(fno.fname, "nvram") && strstr(fno.fname, ".bin"))
            ParseAndAddMapping(fno.fname);
    }
    f_closedir(&dir);
    LOGNOTE("Loaded %u bank mappings", m_bankMappingsCount);
}

void CMiniJV880::ParseAndAddMapping(const char* filename)
{
    if (strlen(filename) < 13) return;
    if (!isdigit(filename[0]) || !isdigit(filename[1])) return;
    int bankNumber = (filename[0]-'0')*10 + (filename[1]-'0');
    if (!isdigit(filename[7]) || !isdigit(filename[8])) return;
    int yyNumber = (filename[7]-'0')*10 + (filename[8]-'0');
    int romIndex = yyNumber + 6;

    if (m_bankMappingsCount >= m_bankMappingsCapacity) {
        m_bankMappingsCapacity *= 2;
        BankMapping* arr = new BankMapping[m_bankMappingsCapacity];
        memcpy(arr, m_bankMappings, m_bankMappingsCount * sizeof(BankMapping));
        delete[] m_bankMappings;
        m_bankMappings = arr;
    }

    m_bankMappings[m_bankMappingsCount].bankNumber = bankNumber;
    m_bankMappings[m_bankMappingsCount].romIndex   = romIndex;
    strncpy(m_bankMappings[m_bankMappingsCount].nvramFilename, filename,
            sizeof(m_bankMappings[0].nvramFilename) - 1);
    m_bankMappings[m_bankMappingsCount].nvramFilename[
        sizeof(m_bankMappings[0].nvramFilename) - 1] = '\0';
    m_bankMappingsCount++;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Logging stubs  (kept for compatibility)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::LogPCM(uint64_t) {}
void CMiniJV880::LogMCU(uint64_t, uint64_t, int, int) {}

// ─────────────────────────────────────────────────────────────────────────────
//  Network  (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
void CMiniJV880::UpdateNetwork()
{
    if (!m_pNet) return;

    bool bNetIsRunning = m_pNet->IsRunning();
    if (m_pNetDevice->GetType() == NetDeviceTypeEthernet)
        bNetIsRunning &= m_pNetDevice->IsLinkUp();
    else if (m_pNetDevice->GetType() == NetDeviceTypeWLAN)
        bNetIsRunning &= (m_WPASupplicant && m_WPASupplicant->IsConnected());

    if (!m_bNetworkInit && bNetIsRunning) {
        m_bNetworkInit = true;
        CString IPString;
        m_pNet->GetConfig()->GetIPAddress()->Format(&IPString);

        if (m_UDPMIDI) m_UDPMIDI->Initialize();

        if (m_pConfig->GetNetworkFTPEnabled()) {
            m_pFTPDaemon = new CFTPDaemon(FTPUSERNAME, FTPPASSWORD,
                                          m_pmDNSPublisher, m_pConfig);
            if (!m_pFTPDaemon->Initialize()) {
                LOGERR("FTP daemon init failed");
                delete m_pFTPDaemon; m_pFTPDaemon = nullptr;
            } else {
                LOGNOTE("FTP daemon initialized");
            }
        }

        if (IPString.GetLength() > 0)
            m_UI.LCDMessage("IP address is \n%s", (const char*)IPString);

        m_pmDNSPublisher = new CmDNSPublisher(m_pNet);
        assert(m_pmDNSPublisher);

        if (!m_pmDNSPublisher->PublishService(m_pConfig->GetNetworkHostname(),
                                               CmDNSPublisher::ServiceTypeAppleMIDI, 5004))
            LOGPANIC("Cannot publish mdns service");

        static constexpr const char *ServiceTypeFTP = "_ftp._tcp";
        static const char *ftpTxt[] = { "app=MiniJV880", nullptr };
        if (!m_pmDNSPublisher->PublishService(m_pConfig->GetNetworkHostname(),
                                               ServiceTypeFTP, 21, ftpTxt))
            LOGPANIC("Cannot publish mdns service");

        if (m_pConfig->GetSyslogEnabled()) {
            CIPAddress ServerIP = m_pConfig->GetNetworkSyslogServerIPAddress();
            if (ServerIP.IsSet() && !ServerIP.IsNull()) {
                static const u16 usServerPort = 514;
                CString IP2; ServerIP.Format(&IP2);
                LOGNOTE("Syslog → %s:%u", (const char*)IP2, (unsigned)usServerPort);
                new CSysLogDaemon(m_pNet, ServerIP, usServerPort);
            }
        }
        m_bNetworkReady = true;
    }

    if (m_bNetworkReady && !bNetIsRunning) {
        m_bNetworkReady = false;
        m_pmDNSPublisher->UnpublishService(m_pConfig->GetNetworkHostname());
        LOGNOTE("Network disconnected");
    } else if (!m_bNetworkReady && bNetIsRunning) {
        m_bNetworkReady = true;
        static constexpr const char *ServiceTypeFTP = "_ftp._tcp";
        static const char *ftpTxt[] = { "app=MiniJV880", nullptr };
        m_pmDNSPublisher->PublishService(m_pConfig->GetNetworkHostname(),
                                         CmDNSPublisher::ServiceTypeAppleMIDI, 5004);
        m_pmDNSPublisher->PublishService(m_pConfig->GetNetworkHostname(),
                                         ServiceTypeFTP, 21, ftpTxt);
        LOGNOTE("Network reconnected");
    }
}

bool CMiniJV880::InitNetwork()
{
    LOGNOTE("InitNetwork called");
    assert(m_pNet == nullptr);

    if (!m_pConfig->GetNetworkEnabled()) {
        LOGNOTE("Network disabled");
        return false;
    }

    TNetDeviceType NetDeviceType = NetDeviceTypeUnknown;

    if (strcmp(m_pConfig->GetNetworkType(), "wlan") == 0) {
        NetDeviceType = NetDeviceTypeWLAN;
        m_WLAN = new CBcm4343Device(WLANFirmwarePath);
        if (!m_WLAN || !m_WLAN->Initialize()) {
            LOGERR("WLAN init failed");
            delete m_WLAN; m_WLAN = nullptr;
            return false;
        }
    } else if (strcmp(m_pConfig->GetNetworkType(), "ethernet") == 0) {
        NetDeviceType = NetDeviceTypeEthernet;
    } else {
        LOGERR("Unknown network type");
        return false;
    }

    if (m_pConfig->GetNetworkDHCP()) {
        m_pNet = new CNetSubSystem(0, 0, 0, 0,
                                   m_pConfig->GetNetworkHostname(), NetDeviceType);
    } else {
        m_pNet = new CNetSubSystem(
            m_pConfig->GetNetworkIPAddress().Get(),
            m_pConfig->GetNetworkSubnetMask().Get(),
            m_pConfig->GetNetworkDefaultGateway().Get(),
            m_pConfig->GetNetworkDNSServer().Get(),
            m_pConfig->GetNetworkHostname(), NetDeviceType);
    }

    if (!m_pNet || !m_pNet->Initialize(false)) {
        LOGERR("Network subsystem init failed");
        delete m_pNet; m_pNet = nullptr;
        delete m_WLAN; m_WLAN = nullptr;
        return false;
    }

    if (NetDeviceType == NetDeviceTypeWLAN) {
        m_WPASupplicant = new CWPASupplicant(WLANConfigFile);
        if (!m_WPASupplicant || !m_WPASupplicant->Initialize()) {
            LOGERR("WPASupplicant init failed");
            delete m_WPASupplicant; m_WPASupplicant = nullptr;
        }
    }

    m_pNetDevice = CNetDevice::GetNetDevice(NetDeviceType);
    m_UDPMIDI = new CUDPMIDIDevice(this, m_pConfig);
    if (!m_UDPMIDI) LOGERR("Failed to allocate UDP MIDI");

    return m_pNet != nullptr;
}