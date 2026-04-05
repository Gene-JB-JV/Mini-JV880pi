#ifndef _minijv880_h
#define _minijv880_h

#ifndef ARM_ALLOW_MULTI_CORE
#define ARM_ALLOW_MULTI_CORE
#endif

#include "config.h"
#include "userinterface.h"
#include "midi.h"
#include "jv880_engine.h"
#include <circle/gpiomanager.h>
#include <circle/i2cmaster.h>
#include <circle/interrupt.h>
#include <circle/multicore.h>
#include <circle/screen.h>
#include <circle/sound/soundbasedevice.h>
#include <circle/spimaster.h>
#include <circle/spinlock.h>
#include <circle/types.h>
#include <circle/sched/scheduler.h>
#include <circle/net/netsubsystem.h>
#include <wlan/bcm4343.h>
#include <wlan/hostap/wpa_supplicant/wpasupplicant.h>
#include "net/mdnspublisher.h"
#include "udpmididevice.h"
#include "net/ftpdaemon.h"
#include <circle/usb/usbmidi.h>
#include <fatfs/ff.h>
#include <stdint.h>
#include <circle/serial.h>
#include <atomic>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
//  MCU compatibility facade
//  userinterface.cpp reads mcu.lcd.LCD_Data and mcu.jv880_led_state.
//  SyncMCUCompat() fills them from JV880Engine::GetLCD() each UI tick.
// ─────────────────────────────────────────────────────────────────────────────
struct LCD_Compat {
    uint8_t LCD_Data[80];  // 2 rows x 40 chars, space-padded
    uint8_t LCD_DD_RAM;
    uint8_t LCD_C;
    // userinterface.cpp calls mcu.lcd.LCD_Update() — no-op, data is
    // already fresh (filled by SyncMCUCompat before each UI tick)
    void LCD_Update() {}
};

struct mcu_compat_t {
    uint16_t pc;           // 0 = not running, 1 = running
};

struct MCU_Compat {
    mcu_compat_t mcu;
    LCD_Compat   lcd;
    uint16_t     jv880_led_state;
    uint32_t     mcu_button_pressed = 0;  // read by userinterface.cpp

    std::function<void(int)> encoder_trigger_cb;
    std::function<void(int)> tone_mute_cb;
    std::function<void()>    save_nvram_cb;

    // userinterface.cpp calls mcu.MCU_EncoderTrigger(dir) directly
    void MCU_EncoderTrigger(int dir) {
        if (encoder_trigger_cb) encoder_trigger_cb(dir);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Simple lock-free SPSC byte queue
// ─────────────────────────────────────────────────────────────────────────────
template<unsigned CAP>
class CSimpleQueue {
    static_assert((CAP & (CAP-1)) == 0, "CAP must be power of 2");
    uint8_t               buf_[CAP];
    std::atomic<unsigned> rd_{0}, wr_{0};
public:
    void Push(uint8_t b) {
        unsigned w = wr_.load(std::memory_order_relaxed);
        buf_[w & (CAP-1)] = b;
        wr_.store(w + 1, std::memory_order_release);
    }
    bool Pop(uint8_t& b) {
        unsigned r = rd_.load(std::memory_order_relaxed);
        if (r == wr_.load(std::memory_order_acquire)) return false;
        b = buf_[r & (CAP-1)];
        rd_.store(r + 1, std::memory_order_release);
        return true;
    }
};

// ─────────────────────────────────────────────────────────────────────────────

class CMiniJV880 : public CMultiCoreSupport {
public:
    CMiniJV880(CConfig *pConfig, CInterruptSystem *pInterrupt,
               CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster, CSPIMaster *pSPIMaster,
               FATFS *pFileSystem, CScreenDevice *mScreenUnbuffered);
    ~CMiniJV880(void);

    bool Initialize(void);
    void Process(bool bPlugAndPlayUpdated);

    virtual void Run(unsigned nCore) override;

    static void USBMIDIMessageHandler(unsigned nCable, u8 *pPacket, unsigned nLength);
    static void DeviceRemovedHandler(CDevice *pDevice, void *pContext);

    MidiParser midiParser;
    CSerialDevice& GetSerial()   { return m_Serial; }
    uint8_t* GetMIDIBuffer()     { return m_MIDIBuffer; }

    // MIDI
    void HandleFullMIDIMessage(const uint8_t* data, uint8_t length);

    void UnscrambleRom(const uint8_t *src, uint8_t *dst, int len);
    bool FileExists(const char* filename);
    bool LoadRom(uint8_t rom_index);
    bool LoadMainRoms(uint8_t ExpRom);
    bool LoadFile(const char* filename, uint8_t* data, size_t size);
    void LogMCU(uint64_t logcyc, uint64_t logwriteptr, int logsleep, int logex);
    void LogPCM(uint64_t logcyc1);
    int  s_log_counter = 0;
    void SaveNVRAMIncremental();
    void switchPatchBank(int bankNumber);
    void InitBankMappings();
    void ParseAndAddMapping(const char* filename);
    bool InitNetwork();
    void UpdateNetwork();

    // Engine callbacks (inlined — called from MIDI CC handler and constructor)
    void EncoderTrigger(int dir)  { m_Engine.EncoderTrigger(dir); }
    void ToggleToneMute(int tone) { m_Engine.ToggleToneMute(tone); }

    // Compatibility facade (read by userinterface.cpp)
    MCU_Compat     mcu;
    CScreenDevice *screenUnbuffered;

private:

    struct RomInfo {
        size_t      size;
        const char* filename;
        bool        isWaveRom;
        bool        isLoaded;
        bool        needsUnscramble;
        void*       data;
    };

    struct BankMapping {
        int  bankNumber;
        int  romIndex;
        char nvramFilename[32];
    };

    BankMapping* m_bankMappings         = nullptr;
    unsigned     m_bankMappingsCount    = 0;
    unsigned     m_bankMappingsCapacity = 0;
    int          m_currentExpansionRomIndex = 0;

    static constexpr size_t sz32K  =   32 * 1024;
    static constexpr size_t sz128K =  128 * 1024;
    static constexpr size_t sz256K =  256 * 1024;
    static constexpr size_t sz2M   = 2 * 1024 * 1024;
    static constexpr size_t sz8M   = 8 * 1024 * 1024;

    static RomInfo m_romInfos[27];
    static constexpr size_t ROM_COUNT = 27;

    CConfig  *m_pConfig;
    FATFS    *m_pFileSystem;

    CUSBMIDIDevice *volatile m_pMIDIDevice = nullptr;
    CSerialDevice   m_Serial;
    uint8_t         m_MIDIBuffer[256];
    uint8_t         m_nBankMSB[16] = {0};

    int lastEncoderPos = 0;

    CSoundBaseDevice *m_pSoundDevice    = nullptr;
    bool              m_bChannelsSwapped;
    unsigned          m_nQueueSizeFrames;
    CUserInterface    m_UI;

    // Network
    CNetSubSystem  *m_pNet           = nullptr;
    CNetDevice     *m_pNetDevice     = nullptr;
    CBcm4343Device *m_WLAN           = nullptr;
    CWPASupplicant *m_WPASupplicant  = nullptr;
    bool            m_bNetworkReady  = false;
    bool            m_bNetworkInit   = false;
    CUDPMIDIDevice *m_UDPMIDI        = nullptr;
    CFTPDaemon     *m_pFTPDaemon     = nullptr;
    CmDNSPublisher *m_pmDNSPublisher = nullptr;

    unsigned m_lastTick  = 0;
    unsigned m_lastTick1 = 0;

    static CMiniJV880 *s_pThis;
    unsigned   n_mMCUcycles          = 9;
    int        m_nNVRAMSaveCounter   = 0;
    std::atomic<int>      m_nPendingBankSwitch{-1};
    std::atomic<uint32_t> m_nBankSwitchTimestamp;
    static constexpr uint32_t BANK_SWITCH_DEBOUNCE_US = 300000;
    std::atomic<bool> m_bAudioPaused{false};

    // ── Engine (replaces MCU) ─────────────────────────────────────────────────
    JV880Engine        m_Engine;
    void               SyncMCUCompat();
    void               PostMIDIBytes(const uint8_t* pData, uint8_t nLength);
    CSimpleQueue<4096> m_MIDIQueue;

    // NVRAM buffer (loaded from SD, passed to m_Engine.Init)
    uint8_t m_nvram[sz32K]{};
};

#endif // _minijv880_h