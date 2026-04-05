//
// jv880_engine.h
//
// Bit-identical port of pcm.cpp PCM_Update for bare-metal Raspberry Pi.
// No MCU emulation. Audio output: 64 kHz stereo (two sample pairs per Tick()).
//
// Architecture:
//   - pcm_t state is embedded directly (same layout as in pcm.h)
//   - PCM_ReadROM dispatches to the same waverom banks as pcm.h
//   - Tick64k() runs exactly one PCM_Update iteration and fills two int16 pairs
//     (first half-step and second half-step, i.e. 64 kHz = 32 kHz × 2)
//   - MIDI and patch selection are handled by a minimal state machine layered
//     on top of direct pcm_t ram writes (replicating what the MCU firmware does)
//
// Copyright (C) 2026  Gene J.B. (Sterr1)
// Based on pcm.cpp by nukeykt (C) 2021, 2024
//
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Helper arithmetic  (verbatim from pcm.cpp)
// ─────────────────────────────────────────────────────────────────────────────
static inline int32_t jv_sx20(int32_t in) { return (in << 12) >> 12; }

static inline uint32_t jv_addclip20(uint32_t a, uint32_t b, uint32_t cin)
{
    uint32_t s = (a + b + cin) & 0xfffff;
    if ((a & 0x80000) && (b & 0x80000) && !(s & 0x80000)) s = 0x80000;
    else if (!(a & 0x80000) && !(b & 0x80000) && (s & 0x80000)) s = 0x7ffff;
    return s;
}

static inline int32_t jv_multi(int32_t val1, int8_t val2)
{
    return jv_sx20(val1) * val2;
}

static inline int jv_clamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int16_t jv_sat16(int v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// ─────────────────────────────────────────────────────────────────────────────
//  LUTs  (verbatim from pcm.cpp)
// ─────────────────────────────────────────────────────────────────────────────
static const int jv_interp_lut[3][128] = {
    { 3385, 3401, 3417, 3432, 3448, 3463, 3478, 3492, 3506, 3521, 3534, 3548,
      3562, 3575, 3588, 3601, 3614, 3626, 3638, 3650, 3662, 3673, 3685, 3696,
      3707, 3718, 3728, 3739, 3749, 3759, 3768, 3778, 3787, 3796, 3805, 3814,
      3823, 3831, 3839, 3847, 3855, 3863, 3870, 3878, 3885, 3892, 3899, 3905,
      3912, 3918, 3924, 3930, 3936, 3942, 3948, 3953, 3958, 3963, 3968, 3973,
      3978, 3983, 3987, 3991, 3995, 4000, 4004, 4007, 4011, 4015, 4018, 4022,
      4025, 4028, 4031, 4034, 4037, 4040, 4042, 4045, 4047, 4050, 4052, 4054,
      4057, 4059, 4061, 4063, 4064, 4066, 4068, 4070, 4071, 4073, 4074, 4076,
      4077, 4078, 4079, 4081, 4082, 4083, 4084, 4085, 4086, 4086, 4087, 4088,
      4089, 4089, 4090, 4091, 4091, 4092, 4092, 4093, 4093, 4094, 4094, 4094,
      4094, 4095, 4095, 4095, 4095, 4095, 4095, 4095 },
    {  710,  726,  742,  758,  775,  792,  809,  826,  844,  861,  879,  897,
       915,  933,  952,  971,  990, 1009, 1028, 1047, 1067, 1087, 1106, 1126,
      1147, 1167, 1188, 1208, 1229, 1250, 1271, 1292, 1314, 1335, 1357, 1379,
      1400, 1423, 1445, 1467, 1489, 1512, 1534, 1557, 1580, 1602, 1625, 1648,
      1671, 1695, 1718, 1741, 1764, 1788, 1811, 1835, 1858, 1882, 1906, 1929,
      1953, 1977, 2000, 2024, 2048, 2069, 2095, 2119, 2143, 2166, 2190, 2214,
      2237, 2261, 2284, 2308, 2331, 2355, 2378, 2401, 2425, 2448, 2471, 2494,
      2517, 2539, 2562, 2585, 2607, 2630, 2652, 2674, 2696, 2718, 2740, 2762,
      2783, 2805, 2826, 2847, 2868, 2889, 2910, 2931, 2951, 2971, 2991, 3011,
      3031, 3051, 3070, 3089, 3108, 3127, 3146, 3164, 3182, 3200, 3218, 3236,
      3253, 3271, 3288, 3304, 3321, 3338, 3354, 3370 },
    {   0,   0,   0,   1,   1,   1,   2,   2,   3,   3,   3,   4,   4,   5,
        5,   6,   6,   7,   8,   8,   9,  10,  10,  11,  12,  13,  14,  15,
       16,  17,  18,  19,  20,  22,  23,  24,  26,  27,  29,  30,  32,  34,
       36,  38,  40,  42,  44,  46,  49,  51,  53,  56,  59,  62,  65,  68,
       71,  74,  77,  81,  84,  88,  92,  96, 100, 104, 109, 113, 118, 122,
      127, 132, 137, 143, 148, 154, 160, 165, 171, 178, 184, 191, 197, 204,
      211, 219, 226, 234, 241, 249, 257, 266, 274, 283, 292, 301, 310, 319,
      329, 339, 349, 359, 369, 380, 391, 402, 413, 424, 436, 448, 460, 472,
      484, 497, 510, 523, 536, 549, 563, 577, 591, 605, 619, 634, 648, 663,
      679, 694 }
};

static const uint8_t jv_flip_nibble_lut[16] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};
static const uint8_t jv_tvc_lut[4]          = {3,15,63,127};

// ─────────────────────────────────────────────────────────────────────────────
//  calc_tv  (verbatim from pcm.cpp — not a single line changed)
// ─────────────────────────────────────────────────────────────────────────────
static inline void jv_calc_tv(
    uint32_t tv_counter,
    int e, int adjust, uint16_t* levelcur, int active, int* volmul)
{
    *levelcur &= 0x7fff;

    uint8_t speed           = adjust & 0xff;
    uint8_t speed_and_0x20  = speed & 0x20;
    uint8_t speed_and_0x40  = speed & 0x40;
    uint8_t speed_and_0x80  = speed & 0x80;
    uint16_t target         = (adjust >> 8) & 0xff;

    uint8_t w1 = !(speed & 0xf0);
    uint8_t w2 = w1 || (speed & 0x10);
    uint8_t w3 = (!speed_and_0x80 || (!speed_and_0x40 && (!w2 || !speed_and_0x20)));

    uint8_t type = w2 | (w3 << 3);
    if (speed_and_0x20)  type |= 2;
    if (!speed_and_0x80 || !speed_and_0x40) type |= 4;

    uint8_t write  = !active;
    uint8_t addlow = 0;
    if (type & 4) {
        addlow  = jv_flip_nibble_lut[tv_counter & 0x0f];
        write  |= 1;
    } else {
        uint8_t t = type & 3;
        addlow  = jv_flip_nibble_lut[(tv_counter >> (2 * t + 2)) & 0x0f];
        write  |= !(tv_counter & jv_tvc_lut[t]);
    }

    if ((type & 8) == 0)
    {
        int shift = (10 - (speed & 15)) & 15;
        int sum1  = (target << 11);
        if (e != 2 || active) sum1 -= (*levelcur << 4);
        int preshift = sum1;
        int shifted  = preshift >> shift;
        shifted -= sum1;
        int sum2 = (target << 11) + addlow + shifted;
        if (write)    *levelcur = (sum2 >> 4) & 0x7fff;
        if (e < 2)    *volmul   = (sum2 >> 4) & 0x7ffe;
    }
    else
    {
        int shift = (speed >> 4) & 14;
        shift |= w2;
        shift  = (10 - shift) & 15;
        int sum1 = target << 11;
        if (e != 2 || active) sum1 -= (*levelcur << 4);
        int neg      = (sum1 & 0x80000) != 0;
        int preshift = (speed & 15) << 9;
        if (!w1)   preshift |= 0x2000;
        if (neg)   preshift ^= ~0x3f;
        int shifted  = preshift >> shift;
        int sum2     = shifted;
        if (e != 2 || active) sum2 += (*levelcur << 4) | addlow;
        int sum2_l   = sum2 >> 4;
        int sum3     = (target << 11) - (sum2_l << 4);
        int neg2     = (sum3 & 0x80000) != 0;
        int xnor     = !(neg2 ^ neg);
        if (write) {
            if (xnor) *levelcur = sum2_l & 0x7fff;
            else      *levelcur = target << 7;
        }
        if (e == 0) {
            *volmul = sum2_l & 0x7ffe;
        } else if (e == 1) {
            if (xnor) *volmul = sum2_l & 0x7ffe;
            else      *volmul = target << 7;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  eram helpers  (verbatim from pcm.cpp)
// ─────────────────────────────────────────────────────────────────────────────
static inline int jv_eram_unpack(const uint16_t* eram, int addr, int type = 0)
{
    addr &= 0x3fff;
    int data = eram[addr];
    int val  = data & 0x3fff;
    int sh   = (data >> 14) & 3;
    val <<= 18;
    return val >> (18 - sh * 2 + type);
}

static inline void jv_eram_pack(uint16_t* eram, int addr, int val)
{
    addr &= 0x3fff;
    int sh   = 0;
    int top  = (val >> 13) & 0x7f;
    if (top & 0x40) top ^= 0x7f;
    if      (top >= 16) sh = 3;
    else if (top >=  4) sh = 2;
    else if (top >=  1) sh = 1;
    else                sh = 0;
    int data = (val >> (sh * 2)) & 0x3fff;
    data |= sh << 14;
    eram[addr] = (uint16_t)data;
}

// ─────────────────────────────────────────────────────────────────────────────
//  pcm_t  (same layout as pcm.h — copy-pasted to avoid the Circle dependency)
// ─────────────────────────────────────────────────────────────────────────────
struct jv_pcm_t {
    uint32_t ram1[32][8];
    uint16_t ram2[32][16];
    uint32_t select_channel;
    uint32_t voice_mask;
    uint32_t voice_mask_pending;
    uint32_t voice_mask_updating;
    uint32_t write_latch;
    uint32_t wave_read_address;
    uint8_t  wave_byte_latch;
    uint32_t read_latch;
    uint8_t  config_reg_3c;
    uint8_t  config_reg_3d;
    uint32_t irq_channel;
    uint32_t irq_assert;
    uint32_t nfs;
    uint32_t tv_counter;
    uint64_t cycles;
    uint16_t eram[0x4000];
    int      accum_l;
    int      accum_r;
    int      rcsum[2];
};

// ─────────────────────────────────────────────────────────────────────────────
//  ROM2 layout constants (for MIDI patch/NoteOn handling)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int kROM2_MULTISAMPLE_BASE = 0x00004; // 129 entries × 0x3C (skip 4-byte header)
static constexpr int kROM2_WAVE_BASE        = 0x01E40; // 128 entries × 0x12 (18 bytes)
static constexpr int kROM2_PATCH_A_BASE     = 0x10CE0; // bank A: 64 × 0x16a
static constexpr int kROM2_PATCH_B_BASE     = 0x18CE0; // bank B: 64 × 0x16a
static constexpr int kPATCH_SIZE            = 0x16a;   // 362 bytes per patch
static constexpr int kTONE_SIZE             = 84;      // 84 bytes per tone
static constexpr int kNUM_VOICES            = 28;
// Multisample entry layout (60 bytes):
//   [0..11]  name (12 bytes)
//   [12..27] hiKey[16]    — upper key boundary per zone (0x7F = no zone)
//   [28..59] sampleId[16] — 2-byte BE sample index per zone (0xFFFF = unused)
// Wave record layout (18 bytes):
//   [0]      volume (0-127)
//   [1..3]   start (24-bit BE, absolute in waverom space 0..0x3FFFFF)
//   [4..6]   loop  (24-bit BE, absolute)
//   [7..9]   end   (24-bit BE, absolute)
//   [10..12] flags/other
//   [13]     rootKey (MIDI note)
//   [14..15] fineTune (16-bit BE)
//   [16..17] loopFineTune (16-bit BE)
// Address → PCM bank mapping:
//   abs_addr < 0x200000 → wrom_exp_  (bank3, JV-880 waverom1), hiaddr = 6 + (abs_addr>>20)
//   abs_addr >= 0x200000 → wrom_exp2_ (bank4, JV-880 waverom2), hiaddr = 8 + ((abs_addr-0x200000)>>20)
//   20-bit addr for ram1[] = abs_addr & 0xFFFFF

// ─────────────────────────────────────────────────────────────────────────────
//  JV880Engine
// ─────────────────────────────────────────────────────────────────────────────
class JV880Engine
{
public:
    // ── LCD state (output) ───────────────────────────────────────────────────
    struct LCDState {
        char     line[2][20]; // space-padded, not NUL-terminated
        uint32_t led_state;
    };

    // ── Init / ROM loading ───────────────────────────────────────────────────
    // waverom1/waverom2: the JV-880 built-in PCM sample ROMs.
    // In the JV-880 all voices use hiaddr=6..7 → PCM_ReadROM bank=3
    // which reads from waverom_exp[]. So waverom1 is loaded as the
    // first segment of waverom_exp (covers all built-in sounds).
    // waverom2 covers waverom_exp segment 1 (hiaddr=8..9, not used by base JV-880).
    // Both waverom1 and waverom2 must be provided — they ARE the built-in ROM.
    bool Init(const uint8_t* rom2,
              const uint8_t* waverom1,   // JV-880 PCM ROM, part 1 (2MB) → waverom_exp seg 0
              const uint8_t* waverom2,   // JV-880 PCM ROM, part 2 (2MB) → waverom_exp seg 1
              const uint8_t* nvram)
    {
        memset(&pcm_, 0, sizeof(pcm_));

        if (!rom2 || !waverom1) return false;

        rom2_     = rom2;
        wrom_[0]  = nullptr;    // bank 0,1 (hiaddr 0..3) — not used by JV-880
        wrom_[1]  = nullptr;
        // jv880_waverom1.bin → waverom_exp segment 0 (bank 3, hiaddr 6..7)
        // jv880_waverom2.bin → waverom_exp segment 1 (bank 4, hiaddr 8..9)
        // All JV-880 base voices use hiaddr=6,7 → bank=3 → wrom_exp_
        // SetExpansionWaverom() overrides wrom_exp_ for SR-JV80 cards.
        wrom_exp_  = waverom1;  // segment 0 (first 2MB, bank 3)
        wrom_exp2_ = waverom2;  // segment 1 (next 2MB, bank 4)
        nvram_     = nvram;

        // JV-880 config registers (same values as firmware init)
        pcm_.config_reg_3c = 0xC0;
        pcm_.config_reg_3d = 0x1B; // 28 voices
        pcm_.voice_mask    = 0;
        pcm_.voice_mask_pending = 0;
        pcm_.nfs           = 0;

        // Load bank A patch 0 as default
        patch_index_ = 0;
        LoadPatch(0);

        ready_ = true;
        UpdateLCD();
        return true;
    }

    // Override built-in waverom with SR-JV80 expansion card ROM.
    // Call after Init(). The card ROM replaces segment 0 (bank 3).
    void SetExpansionWaverom(const uint8_t* exp) { wrom_exp_ = exp; }
    void ReloadVoices() { pcm_.voice_mask = 0; pcm_.voice_mask_pending = 0; }

    bool IsReady()         const { return ready_; }
    const char* PatchName()const { return patch_name_; }
    int  GetPatchIndex()   const { return patch_index_; }
    const LCDState& GetLCD() const { return lcd_; }

    // ── Tone mute control (Mute/Monitor/Compare/Enter buttons) ───────────────
    // Toggle user-mute for tone t (0-3). Only works if tone is ROM-active.
    // Returns new effective state: true = playing, false = muted.
    bool ToggleToneMute(int t)
    {
        if (t < 0 || t > 3) return false;
        if (!tones_[t].rom_active) return false; // can't toggle ROM-inactive tone
        tone_user_mute_[t] = !tone_user_mute_[t];
        // NOTE: currently playing voices are NOT stopped — they finish their release.
        // Mute takes effect on the NEXT NoteOn, matching original JV-880 behavior.
        UpdateLCD();
        return !tone_user_mute_[t];
    }

    // Returns bitmask of tones that are audible (bit0=tone1, bit1=tone2, ...)
    uint8_t GetActiveToneMask() const
    {
        uint8_t mask = 0;
        for (int t = 0; t < 4; t++)
            if (IsToneEnabled(t)) mask |= (1u << t);
        return mask;
    }

    // Returns bitmask of tones that exist in ROM for current patch
    uint8_t GetROMActiveToneMask() const
    {
        uint8_t mask = 0;
        for (int t = 0; t < 4; t++)
            if (tones_[t].rom_active) mask |= (1u << t);
        return mask;
    }

    void SelectPatchByIndex(int idx) {
        patch_index_ = (idx < 0) ? 0 : (idx > 127) ? 127 : idx;
        LoadPatch(patch_index_);
        UpdateLCD();
    }
    void NextPatch() { SelectPatchByIndex(patch_index_ + 1); }
    void PrevPatch() { SelectPatchByIndex(patch_index_ - 1); }

    // Encoder: dir=1 → next patch, dir=0 → prev patch
    void EncoderTrigger(int dir) { if (dir) NextPatch(); else PrevPatch(); }

    // ── MIDI input ───────────────────────────────────────────────────────────
    void MidiIn(uint8_t byte)
    {
        // Running-status MIDI parser
        if (byte & 0x80) {
            if (byte == 0xF7) { sysex_active_ = false; return; }
            if (byte == 0xF0) { sysex_active_ = true;  sysex_len_ = 0; return; }
            if (sysex_active_) return;
            midi_status_  = byte;
            midi_bytes_   = 0;
            return;
        }
        if (sysex_active_) {
            if (sysex_len_ < (int)sizeof(sysex_buf_))
                sysex_buf_[sysex_len_++] = byte;
            return;
        }
        midi_buf_[midi_bytes_++] = byte;

        uint8_t  cmd  = midi_status_ & 0xF0;
        uint8_t  ch   = midi_status_ & 0x0F;
        int      need = (cmd == 0xC0 || cmd == 0xD0) ? 1 : 2;

        if (midi_bytes_ < need) return;
        midi_bytes_ = 0; // consume

        uint8_t d0 = midi_buf_[0];
        uint8_t d1 = (need == 2) ? midi_buf_[1] : 0;

        switch (cmd) {
        case 0x90:
            if (d1 > 0) { NoteOn(ch, d0, d1); return; }
            // velocity 0 = note off (fall through)
        case 0x80: NoteOff(ch, d0);    break;
        case 0xA0: PolyAftertouch(ch, d0, d1); break; // poly aftertouch
        case 0xB0: Controller(ch, d0, d1); break;
        case 0xC0: ProgramChange(ch, d0);  break;
        case 0xD0: ChannelPressure(ch, d0); break;    // channel aftertouch
        case 0xE0: PitchBend(ch, (d0 | (d1 << 7)) - 8192); break;
        default: break;
        }
    }

    // ── Audio output: 64 kHz ─────────────────────────────────────────────────
    // Runs one PCM_Update iteration (= one 32 kHz slot cycle).
    // Produces two sample pairs at 64 kHz (half-step 0 and half-step 1).
    // Caller should call this at 32 kHz; each call provides 2 samples.
    void Tick64k(int16_t& l0, int16_t& r0, int16_t& l1, int16_t& r1)
    {
        PCM_Update_OneCycle(l0, r0, l1, r1);
    }

    // Legacy 32 kHz shim — averages the two 64 kHz sub-samples
    void Tick(int16_t& l, int16_t& r)
    {
        int16_t l0, r0, l1, r1;
        Tick64k(l0, r0, l1, r1);
        l = (int16_t)(((int)l0 + l1) >> 1);
        r = (int16_t)(((int)r0 + r1) >> 1);
    }

    // ── Debug accessors ───────────────────────────────────────────────────────
    uint32_t VoiceMask()              const { return pcm_.voice_mask; }
    uint16_t GetRam2(int s, int i)    const { return pcm_.ram2[s][i]; }
    uint32_t GetRam1(int s, int i)    const { return pcm_.ram1[s][i]; }

private:
    // ── State ────────────────────────────────────────────────────────────────
    jv_pcm_t pcm_{};

    const uint8_t* rom2_     = nullptr;
    const uint8_t* wrom_[2]   = {};   // bank 0,1 — not used by JV-880
    const uint8_t* wrom_exp_  = nullptr; // waverom_exp seg0 (bank 3) = jv880_waverom1
    const uint8_t* wrom_exp2_ = nullptr; // waverom_exp seg1 (bank 4) = jv880_waverom2
    const uint8_t* nvram_    = nullptr;

    bool ready_      = false;
    int  patch_index_= 0;
    char patch_name_[13]{};
    LCDState lcd_{};

    // ── Tone info for current patch (loaded from ROM2 at LoadPatch) ───────────
    // ─────────────────────────────────────────────────────────────────────────
    // Tone structure — offsets verified by ROM2 disassembly & patch comparison
    //
    // ROM2 patch layout (362 bytes):
    //   [0..11]   patch name (12 bytes ASCII)
    //   [12..25]  patch common (14 bytes: vol, chorus, reverb, pan, …)
    //   [26..109] tone 0  (84 bytes)
    //   [110..193] tone 1
    //   [194..277] tone 2
    //   [278..361] tone 3
    //
    // Tone layout (84 bytes) — verified offsets:
    //   [0]       bit7 = rom_active
    //   [1]       ms_index (multisample index)
    //   [4]       tone_level  (0-127)
    //   [5]       vel_curve   (0-3)
    //   [7]       key_follow
    //   [23]      tvf_cutoff  (0-127)
    //   [24]      tvf_resonance (0-127)
    //   [36..37]  adj_tva    = calc_tv sustain/release word (target<<8|speed) → ram2[3]
    //   [38..39]  adj_tva2   = calc_tv sustain/release word → ram2[4]
    //   [40..41]  adj_tvf    = calc_tv TVF word  → ram2[5]
    //   [42..43]  adj_iir    = IIR feedback word → ram2[6]
    //   [44]      tva_t1,  [45] tva_l1   (attack time / sustain level)
    //   [46]      tva_t2,  [47] tva_l2
    //   [48]      tva_t3,  [49] tva_l3
    //   [50]      tva_t4,  [51] tva_l4   (release time / release level)
    //   [52..53]  adj_tva_rel   = release calc_tv word (NoteOff) → ram2[3]
    //   [54..55]  adj_tva2_rel  → ram2[4]
    //   [56..57]  adj_tvf_rel   → ram2[5]
    //   [58..59]  adj_iir_rel   → ram2[6]
    //   [61]      tva_atk_speed  (attack speed factor: 0=slow, 127=fast)
    //   [68]      pan  (0=L, 64=center, 127=R,  128=random/undefined)
    //   [78]      chorus_send
    //   [79]      reverb_send
    // ─────────────────────────────────────────────────────────────────────────
    struct ToneInfo {
        bool    rom_active;     // tone[0] bit7
        uint8_t ms_index;       // tone[1] & 0x7F
        uint8_t vel_curve;      // tone[5]  (0-3)
        uint8_t key_follow;     // tone[7]
        // TVF (cutoff/resonance — filter parameters)
        uint8_t tvf_cutoff;     // tone[23] 0-127
        uint8_t tvf_resonance;  // tone[24] 0-127
        // calc_tv adjust words (sustain phase — written to ram2 at NoteOn)
        uint16_t adj_tva;       // tone[36..37]  BE  → ram2[slot][3]
        uint16_t adj_tva2;      // tone[38..39]  BE  → ram2[slot][4]
        uint16_t adj_tvf;       // tone[40..41]  BE  → ram2[slot][5]
        uint16_t adj_iir;       // tone[42..43]  BE  → ram2[slot][6] lo-word
        // TVA ADSR (T=time, L=level — confirmed by PP2/PP3 comparison)
        uint8_t tva_t1;         // tone[44]  (0=instant)
        uint8_t tva_l1;         // tone[45]  sustain level (0=natural decay, 63=hold)
        uint8_t tva_t2;         // tone[46]
        uint8_t tva_l2;         // tone[47]
        uint8_t tva_t3;         // tone[48]
        uint8_t tva_l3;         // tone[49]
        uint8_t tva_t4;         // tone[50]  release time
        uint8_t tva_l4;         // tone[51]  release level
        // calc_tv adjust words (release phase — written to ram2 at NoteOff)
        uint16_t adj_tva_rel;   // tone[52..53] BE → ram2[slot][3] on KeyOff
        uint16_t adj_tva2_rel;  // tone[54..55] BE → ram2[slot][4]
        uint16_t adj_tvf_rel;   // tone[56..57] BE → ram2[slot][5]
        uint16_t adj_iir_rel;   // tone[58..59] BE → ram2[slot][6]
        // Attack speed scale
        uint8_t tva_atk_speed;  // tone[61]  0=very slow, 127=fast
        // TVA level and pan
        uint8_t tone_level;     // tone[4]   0-127
        uint8_t pan;            // tone[68]  0=L,64=center,127=R
        // Effects
        uint8_t chorus_send;    // tone[78]
        uint8_t reverb_send;    // tone[79]
        int     rom2_base;
    };
    ToneInfo tones_[4]{};

    // ── User mute per tone (toggled by Mute/Monitor/Compare/Enter buttons) ───
    // false = pass-through (follow rom_active), true = force-muted by user
    bool tone_user_mute_[4]{};

    // Returns true if tone t should produce sound
    bool IsToneEnabled(int t) const {
        return tones_[t].rom_active && !tone_user_mute_[t];
    }

    // ── MIDI parser ───────────────────────────────────────────────────────────
    uint8_t  midi_status_  = 0;
    uint8_t  midi_buf_[2]  = {};
    int      midi_bytes_   = 0;
    bool     sysex_active_ = false;
    uint8_t  sysex_buf_[256]{};
    int      sysex_len_    = 0;

    // ── Voice allocation (28 PCM slots) ──────────────────────────────────────
    // Each slot plays one tone (0-3) of one MIDI note.
    // A NoteOn spawns up to 4 voices (one per active tone).
    struct VoiceSlot {
        bool    active   = false;
        uint8_t note     = 0;
        uint8_t vel      = 0;   // velocity at note-on
        uint8_t ch       = 0;
        int     tone_idx = 0;   // which patch tone (0-3)
        uint32_t age     = 0;
        // Wave info cached at note-on for pitch update
        uint8_t  root_key   = 60;
        uint16_t base_pitch = 0x0400;
    };
    VoiceSlot voices_[kNUM_VOICES]{};
    uint32_t  voice_age_ = 0;

    // Per-channel MIDI state
    int16_t  pitch_bend_[16]{};
    uint8_t  cc_volume_[16];
    uint8_t  cc_pan_[16];
    uint8_t  cc_expr_[16];
    uint8_t  cc_mod_[16]{};           // CC1 modulation
    uint8_t  ch_pressure_[16]{};      // channel aftertouch (0xD0)
    uint8_t  poly_pressure_[16][128]{}; // poly aftertouch (0xA0)
    bool     sustain_[16]{};

    void ResetChannelState() {
        for (int i = 0; i < 16; i++) {
            pitch_bend_[i] = 0;
            cc_volume_[i]  = 100;
            cc_pan_[i]     = 64;
            cc_expr_[i]    = 127;
            cc_mod_[i]     = 0;
            ch_pressure_[i]= 0;
            sustain_[i]    = false;
        }
        memset(poly_pressure_, 0, sizeof(poly_pressure_));
    }

    // ── ROM2 accessors ────────────────────────────────────────────────────────
    uint8_t R2(int off) const { return rom2_[off]; }
    uint16_t R2u16be(int off) const {
        return (uint16_t)((R2(off) << 8) | R2(off + 1));
    }

    // ── WaveROM ───────────────────────────────────────────────────────────────
    inline uint8_t PCM_ReadROM(uint32_t address) const
    {
        const int bank = (address >> 21) & 7;
        const uint32_t off = address & 0x1fffff;
        switch (bank) {
        case 0: return wrom_[0]  ? wrom_[0][off]  : 0;  // hiaddr 0,1 (not used by JV-880 base)
        case 1: return wrom_[1]  ? wrom_[1][off]  : 0;  // hiaddr 2,3
        case 2: return 0;                                 // waverom_card — not supported
        case 3: return wrom_exp_  ? wrom_exp_[off]  : 0; // hiaddr 6,7 = jv880_waverom1
        case 4: return wrom_exp2_ ? wrom_exp2_[off] : 0; // hiaddr 8,9 = jv880_waverom2
        case 5: case 6: return 0;                         // hiaddr 10..13 (8MB exp cards only)
        default: return 0;
        }
    }

    // ── Patch / tone loading ──────────────────────────────────────────────────
    // Confirmed ROM2 patch layout (362 bytes):
    //   [0..11]   name (12 bytes)
    //   [12..25]  patch common params (14 bytes)
    //   [26..]    4 tones × 84 bytes each
    //
    // Tone layout (84 bytes, offset from tone start):
    //   [0]   bit7 = active (ROM default), bits6:0 = misc flags
    //   [1]   bit7 = wave_group_type (0=int,1=exp), bits6:0 = multisample index
    //   [4]   tone_level (0-127)
    //   [5]   coarse_tune (int8, semitones)
    //   [7]   fine_tune   (int8, fine pitch)
    static constexpr int kPatchCommonSize = 14;   // after 12-byte name
    static constexpr int kToneSizeROM2   = 84;
    static constexpr int kToneOffset     = 12 + kPatchCommonSize; // = 26

    void LoadPatch(int idx)
    {
        int bank = (idx >= 64) ? 1 : 0;
        int slot = idx & 63;
        int base = (bank == 0 ? kROM2_PATCH_A_BASE : kROM2_PATCH_B_BASE)
                   + slot * kPATCH_SIZE;

        // Patch name
        for (int i = 0; i < 12; i++) {
            char c = (char)R2(base + i);
            patch_name_[i] = (c >= 0x20 && c <= 0x7e) ? c : ' ';
        }
        patch_name_[12] = '\0';

        // Load tone info — offsets verified by ROM2 disassembly
        for (int t = 0; t < 4; t++) {
            int tb = base + kToneOffset + t * kToneSizeROM2;
            tones_[t].rom2_base     = tb;
            tones_[t].rom_active    = (R2(tb + 0) & 0x80) != 0;
            tones_[t].ms_index      = R2(tb + 1) & 0x7F;
            // Level / sensitivity
            tones_[t].tone_level    = R2(tb + 4);
            tones_[t].vel_curve     = R2(tb + 5) & 0x03;
            tones_[t].key_follow    = R2(tb + 7);
            // TVF
            tones_[t].tvf_cutoff    = R2(tb + 23);
            tones_[t].tvf_resonance = R2(tb + 24);
            // calc_tv adjust words (sustain) — big-endian 16-bit
            tones_[t].adj_tva       = R2u16be(tb + 36);
            tones_[t].adj_tva2      = R2u16be(tb + 38);
            tones_[t].adj_tvf       = R2u16be(tb + 40);
            tones_[t].adj_iir       = R2u16be(tb + 42);
            // TVA ADSR envelope
            tones_[t].tva_t1        = R2(tb + 44);
            tones_[t].tva_l1        = R2(tb + 45);
            tones_[t].tva_t2        = R2(tb + 46);
            tones_[t].tva_l2        = R2(tb + 47);
            tones_[t].tva_t3        = R2(tb + 48);
            tones_[t].tva_l3        = R2(tb + 49);
            tones_[t].tva_t4        = R2(tb + 50);
            tones_[t].tva_l4        = R2(tb + 51);
            // calc_tv adjust words (release)
            tones_[t].adj_tva_rel   = R2u16be(tb + 52);
            tones_[t].adj_tva2_rel  = R2u16be(tb + 54);
            tones_[t].adj_tvf_rel   = R2u16be(tb + 56);
            tones_[t].adj_iir_rel   = R2u16be(tb + 58);
            // Attack speed and pan
            tones_[t].tva_atk_speed = R2(tb + 61);
            tones_[t].pan           = R2(tb + 68);
            // Effects sends
            tones_[t].chorus_send   = R2(tb + 78);
            tones_[t].reverb_send   = R2(tb + 79);
        }

        // Reset user mute on patch change
        for (int t = 0; t < 4; t++) tone_user_mute_[t] = false;

        // Kill all playing voices on patch change
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (voices_[i].active || (pcm_.voice_mask & (1u << i)))
                ForceKill(i);
        }
    }

    // ── Wave resolution ───────────────────────────────────────────────────────
    // Multisample table: base 0x0000, stride 0x3C
    //   [4..15]  name (12 bytes)
    //   [32..59] wave indices (up to 14 × uint16_t BE, 0xFFFF = unused)
    //
    // Wave record: base 0x1E40, stride 0x12 (18 bytes)
    //   [0]    flags: bits[7:4]=hiaddr, bit1=loop, bit0=reverse
    //   [1..3] start addr (20-bit, big-endian across 3 bytes)
    //   [4..6] loop  addr (20-bit)
    //   [7..9] end   addr (20-bit)
    //   [13]   root_key (MIDI note)
    //   [14..15] pitch_inc (uint16_t BE, Q14 sub-phase step at root)
    struct WaveInfo {
        uint32_t start;    // 20-bit addr for ram1 (abs & 0xFFFFF)
        uint32_t loop;
        uint32_t end;
        uint8_t  root_key;
        uint8_t  volume;
        uint8_t  hiaddr;   // bits[11:8] of ram2[7]: bank selector for PCM_ReadROM
        bool     valid;
    };

    // Resolve wave record for ms_index + note.
    // Multisample entry (60 bytes @ kROM2_MULTISAMPLE_BASE + ms*60):
    //   [0..11]  name
    //   [12..27] hiKey[16]    per zone (0x7F = last/unused)
    //   [28..59] sampleId[16] per zone (2-byte BE, 0xFFFF = unused)
    // Wave record (18 bytes @ kROM2_WAVE_BASE + id*18):
    //   [0]    volume
    //   [1..3] start (24-bit BE absolute, 0..0x3FFFFF across waverom1+waverom2)
    //   [4..6] loop  (24-bit BE absolute)
    //   [7..9] end   (24-bit BE absolute)
    //   [13]   rootKey
    WaveInfo ResolveWave(uint8_t ms_index, uint8_t note) const
    {
        WaveInfo wi{};
        wi.valid = false;

        int ms_base = kROM2_MULTISAMPLE_BASE + ms_index * 0x3C;

        // Find zone: hiKey[i] is upper boundary of zone i
        int best_sid = -1;
        for (int i = 0; i < 16; i++) {
            uint8_t  hikey = R2(ms_base + 12 + i);
            uint16_t sid   = R2u16be(ms_base + 28 + i * 2);
            if (sid == 0xFFFF) break;
            if (note <= hikey || hikey == 0x7F || i == 15) {
                best_sid = (int)sid;
                break;
            }
        }
        if (best_sid < 0) return wi;

        int wb = kROM2_WAVE_BASE + best_sid * 0x12;
        if (wb + 18 > 0x40000) return wi;

        // 24-bit absolute address in waverom space (0..0x3FFFFF)
        uint32_t abs_start = ((uint32_t)R2(wb+1)<<16)|((uint32_t)R2(wb+2)<<8)|R2(wb+3);
        uint32_t abs_loop  = ((uint32_t)R2(wb+4)<<16)|((uint32_t)R2(wb+5)<<8)|R2(wb+6);
        uint32_t abs_end   = ((uint32_t)R2(wb+7)<<16)|((uint32_t)R2(wb+8)<<8)|R2(wb+9);

        // Convert to 20-bit ram1 address + hiaddr for ram2[7]
        // waverom1 (wrom_exp_,  bank3): abs < 0x200000, hiaddr = 6 + (abs>>20)
        // waverom2 (wrom_exp2_, bank4): abs >= 0x200000, hiaddr = 8 + ((abs-0x200000)>>20)
        // hiaddr stored in ram2[7] bits[11:8], used by PCM_ReadROM as:
        //   full_address = (hiaddr << 20) | addr20
        //   bank = full_address >> 21  → 3 for waverom1, 4 for waverom2
        if (abs_start < 0x200000) {
            wi.hiaddr = (uint8_t)(6 + (abs_start >> 20));  // 6 or 7
        } else {
            wi.hiaddr = (uint8_t)(8 + ((abs_start - 0x200000) >> 20));  // 8 or 9
        }
        wi.start    = abs_start & 0xFFFFF;
        wi.loop     = abs_loop  & 0xFFFFF;
        wi.end      = abs_end   & 0xFFFFF;
        wi.volume   = R2(wb + 0);
        wi.root_key = R2(wb + 13);
        wi.valid    = true;
        return wi;
    }

    // ── Voice allocation / NoteOn / NoteOff ───────────────────────────────────
    void NoteOn(uint8_t ch, uint8_t note, uint8_t vel)
    {
        // Retrigger: принудительно убиваем предыдущее звучание этой ноты
        for (int i = 0; i < kNUM_VOICES; i++)
            if (voices_[i].note == note && voices_[i].ch == ch
                && (voices_[i].active || (pcm_.voice_mask & (1u << i))))
                ForceKill(i);

        // Count how many tones this patch needs
        int tones_needed = 0;
        for (int t = 0; t < 4; t++)
            if (IsToneEnabled(t)) tones_needed++;
        if (tones_needed == 0) return;

        // Count free slots — считаем слоты без active флага И без bit в voice_mask
        int free_count = 0;
        for (int i = 0; i < kNUM_VOICES; i++)
            if (!voices_[i].active && !(pcm_.voice_mask & (1u << i))) free_count++;

        // If not enough free slots, steal voices.
        // Priority: 1) releasing voices first (already dying, less audible)
        //           2) oldest active note (most natural to cut)
        while (free_count < tones_needed) {
            // Pass 1: steal any releasing slot (!active but still in voice_mask)
            bool stole_releasing = false;
            for (int i = 0; i < kNUM_VOICES && free_count < tones_needed; i++) {
                if (!voices_[i].active && (pcm_.voice_mask & (1u << i))) {
                    ForceKill(i);
                    free_count++;
                    stole_releasing = true;
                }
            }
            if (stole_releasing) continue;

            // Pass 2: steal the oldest active note (kill all its tones)
            uint32_t oldest_age = UINT32_MAX;
            uint8_t  steal_note = 0;
            uint8_t  steal_ch   = 0;
            for (int i = 0; i < kNUM_VOICES; i++) {
                if (voices_[i].active && voices_[i].age < oldest_age) {
                    oldest_age = voices_[i].age;
                    steal_note = voices_[i].note;
                    steal_ch   = voices_[i].ch;
                }
            }
            if (oldest_age == UINT32_MAX) break; // nothing to steal
            for (int i = 0; i < kNUM_VOICES; i++)
                if (voices_[i].active && voices_[i].note == steal_note && voices_[i].ch == steal_ch)
                    { ForceKill(i); free_count++; }
        }

        // Assign one free slot per active tone
        for (int t = 0; t < 4; t++) {
            if (!IsToneEnabled(t)) continue;
            for (int i = 0; i < kNUM_VOICES; i++) {
                if (!voices_[i].active && !(pcm_.voice_mask & (1u << i))) {
                    voices_[i] = { true, note, vel, ch, t, voice_age_++,
                                   0, 0 }; // root_key/base_pitch заполнятся в LoadVoiceToSlot
                    LoadVoiceToSlot(i, t, ch, note, vel);
                    break;
                }
            }
        }
    }

    void NoteOff(uint8_t ch, uint8_t note)
    {
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (voices_[i].active && voices_[i].note == note && voices_[i].ch == ch) {
                if (!sustain_[ch]) KeyOff(i);
                // else: leave active until sustain pedal released
            }
        }
    }

    void KeyOff(int slot)
    {
        voices_[slot].active = false;

        // Используем release adjust words из ROM2 напрямую.
        // Если adj_tva_rel != 0 — программируем release и оставляем голос звучать.
        // adj_tva_rel = (target<<8)|speed от firmware (tone[52..53]).
        const ToneInfo& tone = tones_[voices_[slot].tone_idx];
        uint16_t rel_adj = tone.adj_tva_rel;
        bool has_release = (rel_adj != 0) && (pcm_.voice_mask & (1u << slot));

        if (has_release) {
            // ROM2 @ 0x72a9..0x72bd KeyOff sequence (дизассемблировано):
            //   f016 ← r2  (ram2[3] = TVA release adjust)
            //   f018 ← r2  (ram2[4] = TVA2)
            //   f01a ← 0   (ram2[5] = TVF → 0)
            //   f01c        (ram2[6] = IIR, не меняем)
            //   f01e ← r2  (ram2[7] = release word, b5=0 → key_on снят)
            //   f030 ← r2  (ram2[8])
            pcm_.ram2[slot][3]  = rel_adj;
            pcm_.ram2[slot][4]  = tone.adj_tva2_rel;
            pcm_.ram2[slot][5]  = 0;          // TVF → 0 (ROM2 пишет r3=0)
            // ram2[6] (IIR) не трогаем — ROM2 читает текущий
            // ram2[7]: firmware пишет туда release_adj (b5=0, key_on снят)
            // Мы это эмулируем через voice_mask_pending:
            // voice_active = voice_mask & voice_mask_pending
            // Снимая pending → key=0 → write-back не восстановит b5
            // voice_mask остаётся → слот не переиспользуется до конца release
            pcm_.voice_mask_pending &= ~(1u << slot);
            // ram2[12] используем как счётчик фаз release для IRQ handler
            pcm_.ram2[slot][12] = 1;
        } else {
            // Нет release или adj=0 — убиваем мгновенно
            uint32_t mask = ~(1u << slot);
            pcm_.voice_mask         &= mask;
            pcm_.voice_mask_pending &= mask;
            memset(pcm_.ram2[slot], 0, sizeof(pcm_.ram2[slot]));
        }
    }

    // Принудительное убийство слота (при voice stealing, patch change и т.д.)
    void ForceKill(int slot)
    {
        voices_[slot].active = false;
        uint32_t mask = ~(1u << slot);
        pcm_.voice_mask         &= mask;
        pcm_.voice_mask_pending &= mask;
        memset(pcm_.ram2[slot], 0, sizeof(pcm_.ram2[slot]));
    }

    void Controller(uint8_t ch, uint8_t cc, uint8_t val)
    {
        switch (cc) {
        case 1:  // Modulation — масштабирует LFO depth у активных голосов
            cc_mod_[ch] = val;
            UpdateChannelModulation(ch);
            break;
        case 7:  cc_volume_[ch] = val; UpdateChannelVolume(ch); break;
        case 10: cc_pan_[ch]    = val; UpdateChannelPan(ch);    break;
        case 11: cc_expr_[ch]   = val; UpdateChannelVolume(ch); break;
        case 64: {
            bool was_on = sustain_[ch];
            sustain_[ch] = (val >= 64);
            // On sustain pedal release: key-off all held-but-not-active notes
            if (was_on && !sustain_[ch]) {
                for (int i = 0; i < kNUM_VOICES; i++) {
                    // A note is "held by pedal" if its voice slot is still in
                    // voice_mask but voices_[i].active is false (NoteOff arrived
                    // while pedal was down).
                    if (!voices_[i].active &&
                        voices_[i].ch == ch &&
                        (pcm_.voice_mask & (1u << i)) &&
                        pcm_.ram2[i][12] == 0)  // not already in release
                        KeyOff(i);
                }
            }
            break;
        }
        case 120: case 123:
            for (int i = 0; i < kNUM_VOICES; i++)
                if (voices_[i].ch == ch) KeyOff(i);
            break;
        default: break;
        }
    }

    // Poly Aftertouch — влияет на volume конкретной ноты
    void PolyAftertouch(uint8_t ch, uint8_t note, uint8_t pressure)
    {
        poly_pressure_[ch][note & 0x7F] = pressure;
        // Обновляем все голоса на этой ноте
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (voices_[i].active && voices_[i].ch == ch && voices_[i].note == note)
                UpdateVoiceAftertouch(i);
        }
    }

    // Channel Pressure (aftertouch) — влияет на volume всего канала
    void ChannelPressure(uint8_t ch, uint8_t pressure)
    {
        ch_pressure_[ch] = pressure;
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (voices_[i].active && voices_[i].ch == ch)
                UpdateVoiceAftertouch(i);
        }
    }

    // Применяем aftertouch к TVA голоса
    // Aftertouch увеличивает volume сверх базового уровня
    void UpdateVoiceAftertouch(int slot)
    {
        // Don't override release state
        if (pcm_.ram2[slot][12] != 0) return;

        const VoiceSlot& v  = voices_[slot];
        const ToneInfo& tone = tones_[v.tone_idx];
        // Берём max из channel и poly pressure
        uint8_t pressure = ch_pressure_[v.ch];
        uint8_t poly_p   = poly_pressure_[v.ch][v.note & 0x7F];
        if (poly_p > pressure) pressure = poly_p;

        // Базовый volume
        uint32_t vol = (uint32_t)tone.tone_level * v.vel / 127u;
        vol          = vol * cc_volume_[v.ch] / 127u;
        vol          = vol * cc_expr_[v.ch]   / 127u;
        // Aftertouch добавляет до 25% сверху
        vol += (uint32_t)pressure * vol / (127u * 4u);
        uint8_t target = (uint8_t)(vol > 127 ? 127 : vol);
        pcm_.ram2[slot][3] = (uint16_t)((target << 8) | 0x40); // плавно
    }

    // Modulation (CC1) — в JV-880 влияет на pitch LFO depth.
    // На уровне PCM-регистров нет прямого LFO — firmware делало это через
    // периодическое обновление ram2[slot][0] (pitch_inc). Мы имитируем это
    // через небольшую вариацию pitch_inc (вибрато).
    // Для минимальной реализации — просто помечаем что нужно обновить pitch.
    void UpdateChannelModulation(uint8_t ch)
    {
        // Modulation в JV-880 → LFO → pitch вибрато.
        // PCM-движок не имеет встроенного LFO — firmware обновляет pitch_inc
        // каждые несколько мс. Без таймера мы не можем сделать это правильно.
        // Сохраняем значение — будем применять в Tick() если добавим LFO таймер.
        (void)ch; // cc_mod_[ch] уже сохранён выше
    }

    void ProgramChange(uint8_t ch, uint8_t prog)
    {
        (void)ch;
        SelectPatchByIndex(prog & 0x7F);
    }

    void PitchBend(uint8_t ch, int bend)
    {
        pitch_bend_[ch] = (int16_t)bend;
        // Update all active voices on this channel
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (voices_[i].active && voices_[i].ch == ch)
                UpdateVoicePitch(i);
        }
    }

    // ── ROM2 speed table @ 0x04F64 ───────────────────────────────────────────
    // Converts time param (0-127) → speed byte for calc_tv adjust word.
    static constexpr int kSpeedTableBase = 0x04F64;

    uint8_t SpeedByte(uint8_t time_param) const {
        if (!rom2_) return 0xCA;
        return rom2_[kSpeedTableBase + (time_param & 0x7F)];
    }

    uint16_t MakeAdjustInstant(uint8_t target) const {
        return (uint16_t)((target << 8) | 0xCA); // 0xCA = instant convergence
    }

    // Pitch calculation: shift base_pitch_inc by (note - root_key) semitones
    // plus pitch bend (±8192 = ±2 semitones by Roland convention)
    uint16_t CalcPitchIncFromRoot(uint8_t note, uint8_t root_key,
                                  uint16_t base_pitch_inc,
                                  int16_t bend_14bit) const
    {
        double semi = (double)((int)note - (int)root_key);
        semi += (double)bend_14bit / 8192.0 * 2.0; // ±2 semitones
        double ratio;
        if (semi >= 0) {
            ratio = 1.0;
            for (int s = (int)semi; s > 0; --s) ratio *= 1.05946309436;
            ratio *= 1.0 + (semi - (int)semi) * 0.05946309436;
        } else {
            ratio = 1.0;
            double a = -semi;
            for (int s = (int)a; s > 0; --s) ratio /= 1.05946309436;
            ratio /= 1.0 + (a - (int)a) * 0.05946309436;
        }
        uint32_t pi = (uint32_t)((double)base_pitch_inc * ratio + 0.5);
        return (uint16_t)(pi > 0x3FFF ? 0x3FFF : pi);
    }

    void LoadVoiceToSlot(int slot, int tone_idx, uint8_t ch, uint8_t note, uint8_t vel)
    {
        const ToneInfo& tone = tones_[tone_idx];
        WaveInfo wi = ResolveWave(tone.ms_index, note);
        if (!wi.valid) return;  // no wave found — skip voice

        uint32_t* ram1 = pcm_.ram1[slot];
        uint16_t* ram2 = pcm_.ram2[slot];

        memset(ram1, 0, sizeof(pcm_.ram1[slot]));
        memset(ram2, 0, sizeof(pcm_.ram2[slot]));

        // JV-880 waverom samples always loop (forward loop from loop to end).
        // No reverse playback in base JV-880 voices.
        const bool loop_en = (wi.loop < wi.end);
        const bool reverse = false;

        // ── Addresses ────────────────────────────────────────────────────────
        // ram1[0] = address_end  (wrap trigger)
        // ram1[2] = address_loop (wrap-to target)
        // ram1[4] = address      (current / initial)
        ram1[4] = wi.start;
        ram1[0] = wi.end;
        ram1[2] = wi.loop;

        // ── Pitch (ram2[0]) ───────────────────────────────────────────────────
        // Base pitch_inc: PCM hardware uses Q14 sub-phase step.
        // At root_key, one sample per output step → pitch_inc = 0x0400 (Q14=1.0).
        // We derive from semitone offset: note - root_key.
        static constexpr uint16_t kBasePitchInc = 0x4000;
        uint16_t pitch_inc = CalcPitchIncFromRoot(
            note, wi.root_key, kBasePitchInc, pitch_bend_[ch]);
        ram2[0] = pitch_inc;

        // Cache for real-time pitch update
        voices_[slot].root_key   = wi.root_key;
        voices_[slot].base_pitch = kBasePitchInc;
        voices_[slot].vel        = vel;

        // ── Pan (ram2[1]) ─────────────────────────────────────────────────────
        // Tone pan: 0=L, 64=center, 127=R. 128 = random/patch-level pan.
        int tone_pan = (int)tone.pan;
        if (tone_pan > 127) tone_pan = 64;  // undefined → center
        int midi_pan = (int)cc_pan_[ch];    // MIDI CC10 (0-127, 64=center)
        // Combine: tone pan offsets MIDI pan around center
        int pan = tone_pan + (midi_pan - 64);
        if (pan < 0)   pan = 0;
        if (pan > 127) pan = 127;
        uint8_t pan_l, pan_r;
        if (pan <= 64) {
            pan_l = 127;
            pan_r = (uint8_t)(pan * 2);
        } else {
            pan_l = (uint8_t)((127 - pan) * 2);
            pan_r = 127;
        }
        ram2[1] = (uint16_t)((pan_l << 8) | pan_r);

        // ── Reverb/Chorus send (ram2[2]) ──────────────────────────────────────
        ram2[2] = (uint16_t)((tone.reverb_send << 8) | tone.chorus_send);

        // ── TVA volume (ram2[3], ram2[4]) ─────────────────────────────────────
        // Scale tone_level by velocity and MIDI volume/expression
        uint32_t vol = (uint32_t)tone.tone_level * vel / 127u;
        vol          = vol * cc_volume_[ch] / 127u;
        vol          = vol * cc_expr_[ch]   / 127u;
        uint8_t target_vol = (uint8_t)(vol > 127 ? 127 : vol);

        // ROM1 @ 0x385b (LoadVoiceToSlot, дизассемблировано):
        //   r1h ← SRAM[-0x723e+slot]  (TVA target из patch)
        //   r1l ← 0xba                (speed — ЖЁСТКО при Note On)
        //   → f016 (ram2[3]), f018 (ram2[4]), f01a (ram2[5]), f01c (ram2[6])
        // Т.е. speed byte = 0xba для всех TVA/TVF/IIR при Note On.
        // Target для ram2[3] масштабируем по velocity+level.
        static constexpr uint8_t kNoteOnSpeed = 0xba;

        // ram2[3] = TVA: target = velocity*level scaled, speed = 0xba
        ram2[3] = (uint16_t)((target_vol << 8) | kNoteOnSpeed);

        // ram2[4] = TVA2: target из adj_tva2 hi-byte, speed = 0xba
        ram2[4] = (uint16_t)(((tone.adj_tva2 >> 8) & 0xFF) << 8) | kNoteOnSpeed;

        // ── TVF cutoff (ram2[5]) ──────────────────────────────────────────────
        // ROM1 @ 0x385b: r1h ← TVF target из patch, r1l ← 0xba
        ram2[5] = (uint16_t)(((tone.adj_tvf >> 8) & 0xFF) << 8) | kNoteOnSpeed;

        // ── IIR feedback (ram2[6]) ────────────────────────────────────────────
        // ROM1 @ 0x385b: r1h ← IIR target из patch, r1l ← 0xba, bit0=IRQ enable
        ram2[6] = (uint16_t)((((tone.adj_iir >> 8) & 0xFF) << 8) | kNoteOnSpeed) | 0x0001;

        // ── ram2[7]: control flags + hiaddr ───────────────────────────────────
        // hiaddr bits[11:8]: selects waverom bank in PCM_ReadROM
        //   hiaddr=6,7 → bank3 → wrom_exp_ (jv880_waverom1)
        //   hiaddr=8,9 → bank4 → wrom_exp2_ (jv880_waverom2)
        ram2[7] = (uint16_t)(
            (slot & 0x1F)                    |  // sub-channel (pitch_inc src)
            (1u    << 5)                     |  // b5 = key on
            ((loop_en ? 1u : 0u) << 6)      |  // b6 = loop enable
            (0u    << 7)                     |  // b7 = reverse (always forward)
            ((uint16_t)(wi.hiaddr & 0xF) << 8) // bits[11:8] = hiaddr
        );

        // ── ram2[11]: TVF levelcur state ──────────────────────────────────────
        ram2[11] = 0;

        // ── Activate in voice mask ─────────────────────────────────────────────
        uint32_t bit = 1u << slot;
        pcm_.voice_mask         |= bit;
        pcm_.voice_mask_pending |= bit;
    }

    void UpdateVoicePitch(int slot)
    {
        const VoiceSlot& v = voices_[slot];
        pcm_.ram2[slot][0] = CalcPitchIncFromRoot(
            v.note, v.root_key, v.base_pitch, pitch_bend_[v.ch]);
    }

    void UpdateChannelVolume(uint8_t ch) {
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (!voices_[i].active || voices_[i].ch != ch) continue;
            if (pcm_.ram2[i][12] != 0) continue; // skip releasing voices
            const ToneInfo& tone = tones_[voices_[i].tone_idx];
            // Scale tone_level by velocity, MIDI vol, expression
            uint32_t vol = (uint32_t)tone.tone_level * voices_[i].vel / 127u;
            vol          = vol * cc_volume_[ch] / 127u;
            vol          = vol * cc_expr_[ch]   / 127u;
            uint8_t target = (uint8_t)(vol > 127 ? 127 : vol);
            // Use adj_tva speed byte from ROM2, override target
            uint8_t spd = tone.adj_tva & 0xFF;
            pcm_.ram2[i][3] = (uint16_t)((target << 8) | spd);
        }
    }

    void UpdateChannelPan(uint8_t ch) {
        for (int i = 0; i < kNUM_VOICES; i++) {
            if (!voices_[i].active || voices_[i].ch != ch) continue;
            const ToneInfo& tone = tones_[voices_[i].tone_idx];
            int tone_pan = (int)tone.pan;
            if (tone_pan > 127) tone_pan = 64;
            int pan = tone_pan + ((int)cc_pan_[ch] - 64);
            if (pan < 0)   pan = 0;
            if (pan > 127) pan = 127;
            uint8_t pan_l, pan_r;
            if (pan <= 64) { pan_l=127; pan_r=(uint8_t)(pan*2); }
            else { pan_l=(uint8_t)((127-pan)*2); pan_r=127; }
            pcm_.ram2[i][1] = (uint16_t)((pan_l << 8) | pan_r);
        }
    }

    // ── LCD ───────────────────────────────────────────────────────────────────
    void UpdateLCD() {
        char bank = (patch_index_ < 64) ? 'A' : 'B';
        int  num  = (patch_index_ < 64) ? patch_index_ + 1 : patch_index_ - 63;
        int  n    = patch_index_ + 1;

        // Line 0: "A01 PatchName       " (20 chars, space-padded)
        memset(lcd_.line[0], ' ', 20);
        lcd_.line[0][0] = bank;
        lcd_.line[0][1] = (char)('0' + num / 10);
        lcd_.line[0][2] = (char)('0' + num % 10);
        lcd_.line[0][3] = ' ';
        for (int i = 0; i < 12 && patch_name_[i]; i++)
            lcd_.line[0][4 + i] = patch_name_[i];

        // Line 1: "Patch  XX/128       " (20 chars, space-padded)
        memset(lcd_.line[1], ' ', 20);
        const char* prefix = "Patch ";
        for (int i = 0; i < 6; i++) lcd_.line[1][i] = prefix[i];
        lcd_.line[1][6]  = (char)('0' + (n / 100) % 10);
        lcd_.line[1][7]  = (char)('0' + (n /  10) % 10);
        lcd_.line[1][8]  = (char)('0' + (n       ) % 10);
        const char* suffix = "/128";
        for (int i = 0; i < 4; i++) lcd_.line[1][9 + i] = suffix[i];

        // LED state: per-tone active indicators
        // bit 0-3: tone 1-4 playing (ROM-active AND not user-muted)
        // bit 4-7: tone 1-4 ROM-active (exists in patch)
        // UI interprets: ROM-active+playing = lit, ROM-active+muted = dim, inactive = off
        lcd_.led_state = 0;
        for (int t = 0; t < 4; t++) {
            if (tones_[t].rom_active)    lcd_.led_state |= (1u << (t + 4)); // "exists" bit
            if (IsToneEnabled(t))        lcd_.led_state |= (1u <<  t);      // "playing" bit
        }
    }

    // ── PCM_Update — direct port ──────────────────────────────────────────────
    // Runs one complete PCM_Update cycle (32 kHz slot), outputs two 64 kHz
    // sample pairs (matching the two MCU_PostSample calls in pcm.cpp).
    void PCM_Update_OneCycle(int16_t& out_l0, int16_t& out_r0,
                              int16_t& out_l1, int16_t& out_r1)
    {
        jv_pcm_t& p = pcm_;

        constexpr int REG_SLOTS  = 28;
        constexpr int WRITE_MASK = 3;

        const int voice_active = (int)(p.voice_mask & p.voice_mask_pending);

        // ── final mix / LFSR (half-step 0) ────────────────────────────────────
        {
            int shifter = (int)p.ram2[30][10];
            int xr = ((shifter>>0)^(shifter>>1)^(shifter>>7)^(shifter>>12)) & 1;
            shifter = (shifter >> 1) | (xr << 15);
            p.ram2[30][10] = (uint16_t)shifter;

            p.accum_l = jv_addclip20(p.accum_l, p.ram1[30][0], 0);
            p.accum_r = jv_addclip20(p.accum_r, p.ram1[30][1], 0);

            p.ram1[30][2] = jv_addclip20(p.accum_l, 0, 0);
            p.ram1[30][4] = jv_addclip20(p.accum_r, 0, 0);

            p.ram1[30][0] = p.accum_l & WRITE_MASK;
            p.ram1[30][1] = p.accum_r & WRITE_MASK;

            int tt0 = (int)((p.ram1[30][2] & ~(uint32_t)WRITE_MASK) << 12);
            int tt1 = (int)((p.ram1[30][4] & ~(uint32_t)WRITE_MASK) << 12);
            out_l0 = jv_sat16(tt0 >> 16);
            out_r0 = jv_sat16(tt1 >> 16);

            // half-step 1 LFSR advance
            xr = ((shifter>>0)^(shifter>>1)^(shifter>>7)^(shifter>>12)) & 1;
            shifter = (shifter >> 1) | (xr << 15);
            p.ram2[30][10] = (uint16_t)shifter;

            p.accum_l = jv_addclip20(p.accum_l, p.ram1[30][0], 0);
            p.accum_r = jv_addclip20(p.accum_r, p.ram1[30][1], 0);

            p.ram1[30][3] = jv_addclip20(p.accum_l, 0, 0);
            p.ram1[30][5] = jv_addclip20(p.accum_r, 0, 0);

            p.ram1[30][0] = p.accum_l & WRITE_MASK;
            p.ram1[30][1] = p.accum_r & WRITE_MASK;

            int tt2 = (int)((p.ram1[30][3] & ~(uint32_t)WRITE_MASK) << 12);
            int tt3 = (int)((p.ram1[30][5] & ~(uint32_t)WRITE_MASK) << 12);
            out_l1 = jv_sat16(tt2 >> 16);
            out_r1 = jv_sat16(tt3 >> 16);
        }

        // ── TV counter ────────────────────────────────────────────────────────
        {
            if (!p.nfs) p.tv_counter = p.ram2[31][8];
            p.tv_counter = (p.tv_counter - 1) & 0x3fff;
        }

        // ── chorus/reverb LFO address split ──────────────────────────────────
        {
            int r318 = (int)p.ram2[31][8];
            int a    = (r318 & 0x7fff);
            int b    = ((0x4000 - r318) & 0x7fff);
            if (r318 & 0x8000)          p.ram2[31][9]  = (uint16_t)a;
            else                        p.ram2[31][10] = (uint16_t)a;
            if ((0x4000 - r318) & 0x8000) p.ram2[31][10] = (uint16_t)b;
            else                          p.ram2[31][9]  = (uint16_t)b;
        }

        // ── pre-FX mixing ────────────────────────────────────────────────────
        {
            int v1 = p.ram2[31][1];
            int m1 = jv_multi(p.ram1[29][1], v1 >> 8) >> 5;
            int m2 = jv_multi(p.rcsum[1],    v1 & 255) >> 5;
            p.ram1[29][1] = jv_addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);
        }
        {
            int okey   = (p.ram2[31][7] & 0x20) != 0;
            int active = okey;
            int u = 0;
            jv_calc_tv(p.tv_counter, 1, p.ram2[30][0], &p.ram2[30][9], active, &u);
        }
        {
            int v1 = p.ram2[30][1];
            int m1 = jv_multi(p.ram1[29][0], v1 >> 8) >> 5;
            int m2 = jv_multi(p.rcsum[0],    v1 & 255) >> 5;
            p.ram1[29][0] = jv_addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);
        }

        // ── rcadd arrays ─────────────────────────────────────────────────────
        int rcadd [6] = {};
        int rcadd2[6] = {};

        // ── FX block (reverb/chorus delay network) ────────────────────────────
        // Verbatim from PCM_Update blocks 1-31
        {
            // 1
            { const int v1=p.ram2[30][4]; const int m1=jv_multi(p.ram1[29][0],(v1>>8))>>6;
              const int s1=jv_eram_unpack(p.eram,p.ram2[28][1]+p.tv_counter,1);
              const int s2=jv_eram_unpack(p.eram,p.ram2[28][1]+p.tv_counter);
              const int v2=((v1&0x30)!=0)?s1:0;
              const int v3=jv_addclip20(m1,(v2^0xfffff),1); p.ram1[29][4]=v3;
              const int m2=jv_multi(v3,v1&255)>>5;
              p.ram1[29][5]=jv_addclip20(m2>>1,s2,m2&1); }
            // 2
            { const int v1=p.ram2[30][4];
              const int s1=jv_eram_unpack(p.eram,p.ram2[28][2]+p.tv_counter,1);
              const int s2=jv_eram_unpack(p.eram,p.ram2[28][2]+p.tv_counter);
              const int v2=((v1&0x30)!=0)?s1:0;
              const int v3=jv_addclip20(p.ram1[29][5],(v2^0xfffff),1); p.ram1[29][5]=v3;
              const int m2=jv_multi(v3,v1&255)>>5;
              p.ram1[28][0]=jv_addclip20(m2>>1,s2,m2&1); }
            // 3
            { const int v1=p.ram2[30][4];
              const int s1=jv_eram_unpack(p.eram,p.ram2[28][3]+p.tv_counter,1);
              const int s2=jv_eram_unpack(p.eram,p.ram2[28][3]+p.tv_counter);
              const int v2=((v1&0x30)!=0)?s1:0;
              const int v3=jv_addclip20(p.ram1[28][0],(v2^0xfffff),1); p.ram1[28][0]=v3;
              const int m2=jv_multi(v3,v1&255)>>5;
              p.ram1[28][1]=jv_addclip20(m2>>1,s2,m2&1);
              p.ram1[28][2]=jv_eram_unpack(p.eram,p.ram2[28][5]+p.tv_counter); }
            // 4
            { const int v1=p.ram2[30][5];
              const int s1=jv_eram_unpack(p.eram,p.ram2[28][4]+p.tv_counter,1);
              const int s2=jv_eram_unpack(p.eram,p.ram2[28][4]+p.tv_counter);
              const int v2=((v1&0x30)!=0)?s1:0;
              const int v3=jv_addclip20(p.ram1[28][1],(v2^0xfffff),1); p.ram1[28][1]=v3;
              const int m2=jv_multi(v3,v1&255)>>5;
              p.ram1[28][3]=jv_addclip20(m2>>1,s2,m2&1);
              p.ram1[28][4]=jv_eram_unpack(p.eram,p.ram2[29][1]+p.tv_counter); }
            // 5
            { const int v1=p.ram2[30][7];
              const int m1=jv_multi(p.ram1[29][2],(v1>>8))>>5;
              const int s1=jv_eram_unpack(p.eram,p.ram2[29][0]+p.tv_counter);
              const int m2=jv_multi(s1,v1&255)>>5;
              p.ram1[29][2]=jv_addclip20(m1>>1,m2>>1,(m1|m2)&1);
              jv_eram_pack(p.eram,p.ram2[28][0]+p.tv_counter,p.ram1[29][4]); }
            // 6
            { const int v1=p.ram2[30][8];
              const int m1=jv_multi(p.ram1[29][3],(v1>>8))>>5;
              const int s1=jv_eram_unpack(p.eram,p.ram2[29][8]+p.tv_counter);
              const int m2=jv_multi(s1,v1&255)>>5;
              p.ram1[29][3]=jv_addclip20(m1>>1,m2>>1,(m1|m2)&1);
              jv_eram_pack(p.eram,p.ram2[28][1]+p.tv_counter,p.ram1[29][5]);
              jv_eram_pack(p.eram,p.ram2[28][2]+p.tv_counter,p.ram1[28][0]); }
            // 7
            { const int v1=p.ram2[30][9]; const int v2=p.ram1[28][3];
              const int m1=jv_multi(p.ram1[29][2],(v1>>8))>>5;
              const int m2=jv_multi(p.ram1[29][3],(v1>>8))>>5;
              p.ram1[28][3]=jv_addclip20(v2,m1>>1,m1&1);
              p.ram1[28][5]=jv_addclip20(v2,m2>>1,m2&1);
              jv_eram_pack(p.eram,p.ram2[28][3]+p.tv_counter,p.ram1[28][1]); }
            // 8
            { const int v1=p.ram2[30][6];
              const int m1=jv_multi(p.ram1[28][2],v1>>8)>>5;
              const int v2=jv_addclip20(p.ram1[28][3],m1>>1,m1&1); p.ram1[28][3]=v2;
              const int m2=jv_multi(v2,v1&255)>>5;
              p.ram1[28][2]=jv_addclip20(p.ram1[28][2],m2>>1,m2&1);
              p.ram1[28][1]=jv_eram_unpack(p.eram,p.ram2[28][9]+p.tv_counter); }
            // 9
            { const int v1=p.ram2[30][6];
              const int m1=jv_multi(p.ram1[28][4],v1>>8)>>5;
              const int v2=jv_addclip20(p.ram1[28][5],m1>>1,m1&1); p.ram1[28][5]=v2;
              const int m2=jv_multi(v2,v1&255)>>5;
              p.ram1[28][4]=jv_addclip20(p.ram1[28][4],m2>>1,m2&1);
              p.ram1[29][4]=jv_eram_unpack(p.eram,p.ram2[29][5]+p.tv_counter); }
            // 10
            { const int v1=p.ram2[30][6]; const int v2=p.ram1[28][1];
              const int m1=jv_multi(v2,v1>>8)>>5;
              const int s1=jv_eram_unpack(p.eram,p.ram2[28][8]+p.tv_counter);
              const int v3=jv_addclip20(m1>>1,s1,m1&1); p.ram1[28][1]=v3;
              const int m2=jv_multi(v3,v1&255)>>5;
              p.ram1[29][5]=jv_addclip20(m2>>1,v2,m2&1);
              jv_eram_pack(p.eram,p.ram2[28][4]+p.tv_counter,p.ram1[28][3]); }
            // 11
            { const int v1=p.ram2[30][6]; const int v2=p.ram1[29][4];
              const int m1=jv_multi(v2,v1>>8)>>5;
              const int s1=jv_eram_unpack(p.eram,p.ram2[29][4]+p.tv_counter);
              const int v3=jv_addclip20(m1>>1,s1,m1&1); p.ram1[29][4]=v3;
              const int m2=jv_multi(v3,v1&255)>>5;
              p.ram1[28][0]=jv_addclip20(m2>>1,v2,m2&1);
              jv_eram_pack(p.eram,p.ram2[28][5]+p.tv_counter,p.ram1[28][2]);
              jv_eram_pack(p.eram,p.ram2[29][0]+p.tv_counter,p.ram1[28][5]); }
            // 12
            { p.ram1[28][5]=jv_eram_unpack(p.eram,p.ram2[28][6]+p.tv_counter); }
            // 13
            { const int s1=jv_eram_unpack(p.eram,p.ram2[28][10]+p.tv_counter);
              p.ram1[28][5]=jv_addclip20(p.ram1[28][5],s1,0);
              p.ram1[28][2]=jv_eram_unpack(p.eram,p.ram2[29][2]+p.tv_counter); }
            // 14
            { const int s1=jv_eram_unpack(p.eram,p.ram2[29][6]+p.tv_counter);
              const int t1=jv_addclip20(s1,p.ram1[28][2],0);
              p.ram1[28][5]=jv_addclip20(t1,p.ram1[28][5],0);
              p.ram1[28][2]=jv_eram_unpack(p.eram,p.ram2[28][7]+p.tv_counter); }
            // 15
            { const int s1=jv_eram_unpack(p.eram,p.ram2[28][11]+p.tv_counter);
              p.ram1[28][2]=jv_addclip20(p.ram1[28][2],s1,0);
              p.ram1[28][3]=jv_eram_unpack(p.eram,p.ram2[29][3]+p.tv_counter); }
            // 16
            { const int s1=jv_eram_unpack(p.eram,p.ram2[29][7]+p.tv_counter);
              const int t1=jv_addclip20(s1,p.ram1[28][2],0);
              p.ram1[28][2]=jv_addclip20(t1,p.ram1[28][3],0);
              jv_eram_pack(p.eram,p.ram2[29][1]+p.tv_counter,p.ram1[28][4]);
              jv_eram_pack(p.eram,p.ram2[28][8]+p.tv_counter,p.ram1[28][1]); }
            // 17
            { const int v1=p.ram2[30][2]; const int v2=p.ram1[28][5];
              const int m1=jv_multi(v2,v1>>8)>>5;
              rcadd[0]=m1; rcadd2[0]=jv_multi(v2,v1&255)>>5;
              const int t1=jv_eram_unpack(p.eram,p.ram2[29][10]+p.tv_counter+1);
              jv_eram_pack(p.eram,p.ram2[28][9]+p.tv_counter,p.ram1[29][5]);
              p.ram1[29][5]=t1; }
            // 18
            { const int v1=p.ram2[30][3]; const int v2=p.ram1[28][2];
              const int m1=jv_multi(v2,v1>>8)>>5;
              rcadd[1]=m1; rcadd2[1]=jv_multi(v2,v1&255)>>5;
              p.ram1[28][1]=jv_eram_unpack(p.eram,p.ram2[29][11]+p.tv_counter+1); }
            // 19
            { const int v1=p.ram2[31][9];
              const int s1=jv_eram_unpack(p.eram,p.ram2[29][10]+p.tv_counter);
              jv_eram_pack(p.eram,p.ram2[29][4]+p.tv_counter,p.ram1[29][4]);
              const int m1=jv_multi(s1,v1>>8)>>5;
              const int m2=jv_multi(p.ram1[29][5],v1>>8)>>5;
              const int t2=jv_addclip20(s1,((m1>>1)^0xfffff),1);
              p.ram1[29][5]=jv_addclip20(t2,m2>>1,m2&1); }
            // 20
            { const int v1=p.ram2[31][10];
              const int s1=jv_eram_unpack(p.eram,p.ram2[29][11]+p.tv_counter);
              jv_eram_pack(p.eram,p.ram2[29][5]+p.tv_counter,p.ram1[28][0]);
              const int m1=jv_multi(s1,v1>>8)>>5;
              const int m2=jv_multi(p.ram1[28][1],v1>>8)>>5;
              const int t2=jv_addclip20(s1,((m1>>1)^0xfffff),1);
              p.ram1[28][1]=jv_addclip20(t2,m2>>1,m2&1);
              jv_eram_pack(p.eram,p.ram2[29][9]+p.tv_counter,p.ram1[29][1]); }
            // 21
            { const int v1=p.ram2[31][2]; const int v2=p.ram1[29][5];
              rcadd[2]=jv_multi(v2,v1>>8)>>5; rcadd2[2]=jv_multi(v2,v1&255)>>5; }
            // 22
            { const int v1=p.ram2[31][3]; const int v2=p.ram1[29][5];
              rcadd[3]=jv_multi(v2,v1>>8)>>5; rcadd2[3]=jv_multi(v2,v1&255)>>5; }
            // 23
            { const int v1=p.ram2[31][4]; const int v2=p.ram1[28][1];
              rcadd[4]=jv_multi(v2,v1>>8)>>5; rcadd2[4]=jv_multi(v2,v1&255)>>5; }
            // 31
            { const int v1=p.ram2[31][5]; const int v2=p.ram1[28][1];
              rcadd[5]=jv_multi(v2,v1>>8)>>5; rcadd2[5]=jv_multi(v2,v1&255)>>5;

              // address generator for slot 31 (LFO/chorus)
              const int key=1, okey=(p.ram2[31][7]&0x20)!=0;
              const int active31=(key&&okey), kon31=(key&&!okey);
              bool b15_31=(p.ram2[31][8]&0x8000)!=0;
              const bool b6_31=(p.ram2[31][7]&0x40)!=0;
              const bool b7_31=(p.ram2[31][7]&0x80)!=0;
              const int ae31=p.ram1[31][0], al31=p.ram1[31][2];
              int sp31=(p.ram2[31][8]&0x3fff);
              const int step31=p.ram2[p.ram2[31][7]&31][0];
              sp31+=step31;
              const int spo31=(sp31>>14)&7;
              if(p.nfs){ p.ram2[31][8]=(p.ram2[31][8]&~0x3fff)|(sp31&0x3fff); }
              int ac31=p.ram1[31][4];
              const int c1a31=b15_31?al31:ae31;
              bool acmp31=((c1a31&0xfffff)==(ac31&0xfffff));
              int na31=ac31; bool nb15_31=b15_31;
              int ac2_31=(kon31||(!b6_31&&acmp31))?((!b6_31&&acmp31)?al31:ac31):ac31;
              int aa31=((!acmp31&&b6_31&&!b15_31)||(!acmp31&&!b6_31))?1:0;
              int as31=(!acmp31&&b6_31&&b15_31)?1:0;
              ac31=b7_31?((ac2_31-(aa31-as31))&0xfffff):((ac2_31+(aa31-as31))&0xfffff);
              b15_31=b6_31&&(b15_31^acmp31);
              const int c1b31=b15_31?al31:ae31;
              acmp31=((c1b31&0xfffff)==(ac31&0xfffff));
              if(spo31>=1){na31=ac31;nb15_31=b15_31;}
              if(active31&&p.nfs) p.ram1[31][4]=na31;
              if(p.nfs){ p.ram2[31][8]=(p.ram2[31][8]&~0x8000)|(int(nb15_31)<<15); }
              p.ram2[29][10]=p.ram1[31][0]-p.ram1[31][4]+p.ram1[31][2];
              p.ram2[29][11]=p.ram1[31][4];
            }
        }

        // ── clear accumulator sum registers ──────────────────────────────────
        p.ram1[31][1] = 0;
        p.ram1[31][3] = 0;
        p.rcsum[0] = 0;
        p.rcsum[1] = 0;

        // ── voice loop (slots 0-27) ───────────────────────────────────────────
        for (int slot = 0; slot < REG_SLOTS; ++slot)
        {
            uint32_t* ram1 = p.ram1[slot];
            uint16_t* ram2 = p.ram2[slot];

            const int okey   = (ram2[7] & 0x20) != 0;
            const int key    = (voice_active >> slot) & 1;
            const int active = (okey & key);
            const int kon    = (key & !okey);

            bool b15     = (ram2[8] & 0x8000) != 0;
            const bool b6= (ram2[7] & 0x40) != 0;
            const bool b7= (ram2[7] & 0x80) != 0;
            const int hiaddr   = (ram2[7] >> 8) & 15;
            const int old_nib  = (ram2[7] >> 12) & 15;

            const int address_end  = (int)ram1[0];
            const int address_loop = (int)ram1[2];
            const int address      = (int)ram1[4];

            // ── nibble address + newnibble ────────────────────────────────────
            const int cmp1n        = b15 ? address_loop : address_end;
            const bool nibble_cmp1 = ((cmp1n & 0xffff0) == (address & 0xffff0));

            int irq_flag;
            if (kon) irq_flag = (((cmp1n + address_loop) & 0x100000) != 0);
            else     irq_flag = (((address + ((-address_loop) & 0xfffff)) & 0x100000) != 0);
            irq_flag ^= (int)b7;

            int nibble_address = (!b6 && nibble_cmp1) ? address_loop : address;
            const int address_b4 = (nibble_address & 0x10) != 0;
            int wave_address = (nibble_address >> 5) & 0xfffff;
            const int xor2  = (address_b4 ^ (int)b7);
            const int check1= (xor2 & active);
            const int xor1  = ((int)b15 ^ (int)(!nibble_cmp1));
            const int nibble_add = b6 ? (check1 && xor1) : (!nibble_cmp1 && check1);
            const int nibble_sub = (b6 && !xor1 && active && !xor2);
            wave_address = b7 ? ((wave_address - (nibble_add - nibble_sub)) & 0xfffff)
                               : ((wave_address + (nibble_add - nibble_sub)) & 0xfffff);

            int newnibble = (int)PCM_ReadROM((uint32_t)(hiaddr << 20) | (uint32_t)wave_address);
            const int newnibble_sel = address_b4 ^ ((b6 || !nibble_cmp1) && okey);
            newnibble = newnibble_sel ? ((newnibble >> 4) & 15) : (newnibble & 15);

            // ── sub-phase / pitch ─────────────────────────────────────────────
            int sub_phase = (ram2[8] & 0x3fff);
            const int step= (int)p.ram2[ram2[7] & 31][0];
            const int interp_ratio = (sub_phase >> 7) & 127;
            sub_phase += step;
            const int sub_phase_of = (sub_phase >> 14) & 7;
            if (p.nfs) {
                ram2[8] = (uint16_t)((ram2[8] & ~0x3fff) | (sub_phase & 0x3fff));
            }

            // ── 5-step address generator ──────────────────────────────────────
#define JV_ADDR_STEP(samp, nc, sub_thr) \
            { bool acmp = (((b15?address_loop:address_end)&0xfffff)==(address_cnt&0xfffff)); \
              if (sub_phase_of >= (sub_thr)) { next_address=address_cnt; usenew=!nc; next_b15=b15; } \
              int ac2=(kon||(!b6&&acmp))?((!b6&&acmp)?address_loop:address_cnt):address_cnt; \
              int aa=((!acmp&&b6&&!b15)||(!acmp&&!b6))?1:0; \
              int as2=(!acmp&&b6&&b15)?1:0; \
              address_cnt=b7?((ac2-(aa-as2))&0xfffff):((ac2+(aa-as2))&0xfffff); \
              b15=b6&&(b15^acmp); \
              samp=(int8_t)PCM_ReadROM((uint32_t)(hiaddr<<20)|(uint32_t)address_cnt); \
              nc=((address&0xffff0)==(address_cnt&0xffff0)); }

            int address_cnt = address;
            const int samp0 = (int8_t)PCM_ReadROM((uint32_t)(hiaddr << 20) | (uint32_t)address_cnt);
            bool nibble_cmp2 = ((address & 0xffff0) == (address_cnt & 0xffff0));
            {   // advance address_cnt to position for samp1 (same as step 0 in original)
                bool acmp = (((b15?address_loop:address_end)&0xfffff)==(address_cnt&0xfffff));
                int ac2=(kon||(!b6&&acmp))?((!b6&&acmp)?address_loop:address_cnt):address_cnt;
                int aa=((!acmp&&b6&&!b15)||(!acmp&&!b6))?1:0;
                int as2=(!acmp&&b6&&b15)?1:0;
                address_cnt=b7?((ac2-(aa-as2))&0xfffff):((ac2+(aa-as2))&0xfffff);
                b15=b6&&(b15^acmp);
            }
            int next_address = address_cnt;
            bool usenew = !nibble_cmp2;
            bool next_b15 = b15;

            const int samp1=(int8_t)PCM_ReadROM((uint32_t)(hiaddr<<20)|(uint32_t)address_cnt);
            bool nibble_cmp3=((address&0xffff0)==(address_cnt&0xffff0));
            { bool acmp=(((b15?address_loop:address_end)&0xfffff)==(address_cnt&0xfffff));
              if(sub_phase_of>=1){next_address=address_cnt;usenew=!nibble_cmp3;next_b15=b15;}
              int ac2=(kon||(!b6&&acmp))?((!b6&&acmp)?address_loop:address_cnt):address_cnt;
              int aa=((!acmp&&b6&&!b15)||(!acmp&&!b6))?1:0;
              int as2=(!acmp&&b6&&b15)?1:0;
              address_cnt=b7?((ac2-(aa-as2))&0xfffff):((ac2+(aa-as2))&0xfffff);
              b15=b6&&(b15^acmp); }

            const int samp2=(int8_t)PCM_ReadROM((uint32_t)(hiaddr<<20)|(uint32_t)address_cnt);
            bool nibble_cmp4=((address&0xffff0)==(address_cnt&0xffff0));
            { bool acmp=(((b15?address_loop:address_end)&0xfffff)==(address_cnt&0xfffff));
              if(sub_phase_of>=2){next_address=address_cnt;usenew=!nibble_cmp4;next_b15=b15;}
              int ac2=(kon||(!b6&&acmp))?((!b6&&acmp)?address_loop:address_cnt):address_cnt;
              int aa=((!acmp&&b6&&!b15)||(!acmp&&!b6))?1:0;
              int as2=(!acmp&&b6&&b15)?1:0;
              address_cnt=b7?((ac2-(aa-as2))&0xfffff):((ac2+(aa-as2))&0xfffff);
              // note: b15 not updated on this step per original }
            }

            const int samp3=(int8_t)PCM_ReadROM((uint32_t)(hiaddr<<20)|(uint32_t)address_cnt);
            bool nibble_cmp5=((address&0xffff0)==(address_cnt&0xffff0));
            { bool acmp=(((b15?address_loop:address_end)&0xfffff)==(address_cnt&0xfffff));
              if(sub_phase_of>=3){next_address=address_cnt;usenew=!nibble_cmp5;next_b15=b15;}
              int ac2=(kon||(!b6&&acmp))?((!b6&&acmp)?address_loop:address_cnt):address_cnt;
              int aa=((!acmp&&b6&&!b15)||(!acmp&&!b6))?1:0;
              int as2=(!acmp&&b6&&b15)?1:0;
              address_cnt=b7?((ac2-(aa-as2))&0xfffff):((ac2+(aa-as2))&0xfffff); }

            bool nibble_cmp6=((address&0xffff0)==(address_cnt&0xffff0));
            if(sub_phase_of>=4){next_address=address_cnt;usenew=!nibble_cmp6;}

#undef JV_ADDR_STEP

            if (active && p.nfs) ram1[4] = (uint32_t)next_address;
            if (p.nfs)           ram2[8] = (uint16_t)((ram2[8]&~0x8000)|(int(next_b15)<<15));

            // ── DPCM ─────────────────────────────────────────────────────────
            int reference = (int)ram1[5];
            { int ps=(samp0<<10); int sn=nibble_cmp2?old_nib:newnibble;
              int sh=(10-sn)&15; int sv=(ps<<1)>>sh;
              if(sub_phase_of>=1) reference=jv_addclip20(reference,sv>>1,sv&1); }
            { int ps=(samp1<<10); int sn=nibble_cmp3?old_nib:newnibble;
              int sh=(10-sn)&15; int sv=(ps<<1)>>sh;
              if(sub_phase_of>=2) reference=jv_addclip20(reference,sv>>1,sv&1); }
            { int ps=(samp2<<10); int sn=nibble_cmp4?old_nib:newnibble;
              int sh=(10-sn)&15; int sv=(ps<<1)>>sh;
              if(sub_phase_of>=3) reference=jv_addclip20(reference,sv>>1,sv&1); }
            { int ps=(samp3<<10); int sn=nibble_cmp5?old_nib:newnibble;
              int sh=(10-sn)&15; int sv=(ps<<1)>>sh;
              if(sub_phase_of>=4) reference=jv_addclip20(reference,sv>>1,sv&1); }

            // ── 3-point Lagrange interpolation ───────────────────────────────
            int test = (int)ram1[5];
            { int st=jv_multi(jv_interp_lut[0][interp_ratio]<<6,samp0)>>8;
              int sn=nibble_cmp2?old_nib:newnibble; int sh=(10-sn)&15;
              st=(st<<1)>>sh; test=jv_addclip20(test,st>>1,st&1); }
            { int st=jv_multi(jv_interp_lut[1][interp_ratio]<<6,samp1)>>8;
              int sn=nibble_cmp3?old_nib:newnibble; int sh=(10-sn)&15;
              st=(st<<1)>>sh; test=jv_addclip20(test,st>>1,st&1); }
            { int st=jv_multi(jv_interp_lut[2][interp_ratio]<<6,samp2)>>8;
              int sn=nibble_cmp4?old_nib:newnibble; int sh=(10-sn)&15;
              st=(st<<1)>>sh;

              // ── IIR TVF filter (JV-880 path, 32-bit hack) ─────────────────
              const int reg1   = (int)ram1[1];
              const int reg3   = (int)ram1[3];
              const int reg2_6 = (ram2[6] >> 8) & 127;
              test = jv_addclip20(test, st>>1, st&1);

              const int filter = (int)ram2[11];
              const int mult1 = reg1 * (int8_t)(filter >> 8);
              const int mult2 = reg1 * (int8_t)((filter >> 1) & 127);
              const int mult3 = reg1 * (int8_t)reg2_6;
              int v2    = reg3 + (mult1>>6) + ((mult1>>5)&1);
              int v1    = v2   + (mult2>>13) + ((mult2>>12)&1);
              int subvar= v1   + (mult3>>6)  + ((mult3>>5)&1);
              ram1[3] = (uint32_t)jv_clamp(v1, -0x80000, 0x7ffff);
              int tests = (test<<12)>>12;
              int v3    = tests - subvar;
              const int mult4 = v3 * (int8_t)(filter >> 8);
              const int mult5 = v3 * (int8_t)((filter >> 1) & 127);
              int v4    = reg1 + (mult4>>6)  + ((mult4>>5)&1);
              int v5    = v4   + (mult5>>13) + ((mult5>>12)&1);
              ram1[1] = (uint32_t)jv_clamp(v5, -0x80000, 0x7ffff);
              ram1[5] = (uint32_t)reference;

              // ── IRQ ──────────────────────────────────────────────────────
              if (active && (ram2[6]&1) && !(ram2[8]&0x4000) && !p.irq_assert && irq_flag) {
                  if (p.nfs) ram2[8] |= 0x4000;
                  p.irq_assert  = 1;
                  p.irq_channel = (uint32_t)slot;
              }

              // ── TVA/TVP/TVF envelopes ─────────────────────────────────────
              int volmul1=0, volmul2=0;
              jv_calc_tv(p.tv_counter, 0, ram2[3], &ram2[9],  active, &volmul1);
              jv_calc_tv(p.tv_counter, 1, ram2[4], &ram2[10], active, &volmul2);
              jv_calc_tv(p.tv_counter, 2, ram2[5], &ram2[11], active, nullptr);

              // ── output sample select (LPF=ram1[3], HPF=v3) ───────────────
              const int sample = ((ram2[6]&2)==0) ? (int)ram1[3] : v3;

              // ── volume scaling ────────────────────────────────────────────
              const int mv1=jv_multi(sample,(volmul1>>8));
              const int mv2=jv_multi(sample,(volmul1>>1)&127);
              const int s2=jv_addclip20(mv1>>6,mv2>>13,((mv2>>12)|(mv1>>5))&1);
              const int mv3=jv_multi(s2,(volmul2>>8));
              const int mv4=jv_multi(s2,(volmul2>>1)&127);
              const int s3=jv_addclip20(mv3>>6,mv4>>13,((mv4>>12)|(mv3>>5))&1);

              // ── pan + reverb/chorus send ──────────────────────────────────
              const int pan = active ? (int)ram2[1] : 0;
              const int rc  = active ? (int)ram2[2] : 0;
              const int sampl = jv_multi(s3,(pan>>8)&255);
              const int sampr = jv_multi(s3,(pan>>0)&255);
              const int rc0   = jv_multi(s3,(rc>>8)&255) >> 5;
              const int rc1   = jv_multi(s3,(rc>>0)&255) >> 5;

              // ── key state write-back ──────────────────────────────────────
              // key = (voice_mask & voice_mask_pending) >> slot
              // При release: voice_mask_pending снят → key=0 → b5 не восстанавливается
              // При sustain: key=1 → b5 устанавливается (key_on активен)
              if (key && p.nfs) {
                  ram2[7] &= ~0xf020;
                  ram2[7] |= (uint16_t)(((usenew||kon)?newnibble:old_nib) << 12);
                  ram2[7] |= (uint16_t)(key << 5);
              }
              if (!active) {
                  if (p.nfs) { ram1[1]=0; ram1[3]=0; ram1[5]=0; }
                  ram2[8]=0; ram2[9]=0; ram2[10]=0;
              }

              // ── mix into accumulator ──────────────────────────────────────
              const int next_slot = (slot==REG_SLOTS-1) ? 31 : (slot+1);
              switch (next_slot) {
              case 17: p.ram1[31][1]=jv_addclip20(p.ram1[31][1],rcadd[0]>>1,rcadd[0]&1);
                       p.rcsum[1]   =jv_addclip20(p.rcsum[1],rcadd2[0]>>1,rcadd2[0]&1); break;
              case 18: p.ram1[31][3]=jv_addclip20(p.ram1[31][3],rcadd[1]>>1,rcadd[1]&1);
                       p.rcsum[1]   =jv_addclip20(p.rcsum[1],rcadd2[1]>>1,rcadd2[1]&1); break;
              case 21: p.ram1[31][1]=jv_addclip20(p.ram1[31][1],rcadd[2]>>1,rcadd[2]&1);
                       p.rcsum[0]   =jv_addclip20(p.rcsum[0],rcadd2[2]>>1,rcadd2[2]&1); break;
              case 22: p.ram1[31][3]=jv_addclip20(p.ram1[31][3],rcadd[3]>>1,rcadd[3]&1);
                       p.rcsum[1]   =jv_addclip20(p.rcsum[1],rcadd2[3]>>1,rcadd2[3]&1); break;
              case 23: p.ram1[31][1]=jv_addclip20(p.ram1[31][1],rcadd[4]>>1,rcadd[4]&1);
                       p.rcsum[0]   =jv_addclip20(p.rcsum[0],rcadd2[4]>>1,rcadd2[4]&1); break;
              case 31: p.ram1[31][3]=jv_addclip20(p.ram1[31][3],rcadd[5]>>1,rcadd[5]&1);
                       p.rcsum[1]   =jv_addclip20(p.rcsum[1],rcadd2[5]>>1,rcadd2[5]&1); break;
              default: break;
              }
              p.rcsum[0]=jv_addclip20(p.rcsum[0],rc0>>1,rc0&1);
              p.rcsum[1]=jv_addclip20(p.rcsum[1],rc1>>1,rc1&1);
              const int suml=jv_addclip20(p.ram1[31][1],sampl>>6,(sampl>>5)&1);
              const int sumr=jv_addclip20(p.ram1[31][3],sampr>>6,(sampr>>5)&1);
              if (slot != REG_SLOTS-1) {
                  p.ram1[31][1]=(uint32_t)suml;
                  p.ram1[31][3]=(uint32_t)sumr;
              } else {
                  p.accum_l = suml;
                  p.accum_r = sumr;
              }
            }
        } // slot loop

        if (p.nfs) p.ram2[31][7] |= 0x20;
        p.nfs = 1;

        // ── IRQ handling (no MCU — we respond inline) ─────────────────────────
        // Real firmware (ROM2 @ 0x70ee, 0x7104):
        //   PCM INT → MCU reads irq_channel → runs envelope state machine.
        // При release (voice_mask_pending снят, key=0):
        //   active=0, calc_tv продолжает работать к target
        //   IRQ срабатывает когда sample достигает loop/end point
        //   MCU проверяет: если envelope близок к нулю → убивает слот
        if (p.irq_assert) {
            const uint32_t si = p.irq_channel;
            if (si < 28) {
                uint16_t* r2 = p.ram2[si];
                // pending бит: если снят → мы в release фазе
                const bool releasing = !(p.voice_mask_pending & (1u << si));
                const bool loop_en   = (r2[7] & 0x40) != 0;
                // Clear IRQ-pending latch
                r2[8] &= ~(uint16_t)0x4000;

                bool kill = false;

                if (!loop_en) {
                    // One-shot sample finished → kill
                    kill = true;
                } else if (releasing) {
                    // Release phase: проверяем envelope amplitude
                    // ram2[9] = levelcur TVA (Q15), ram2[12] = release stage
                    const uint16_t stage = r2[12];
                    if (stage == 1) {
                        // Переходим в финальный decay к 0
                        r2[3]  = (uint16_t)0x00CA; // target=0, fast speed
                        r2[12] = 2;
                    } else if (stage == 2) {
                        // Финальный decay завершён → убиваем
                        kill = true;
                    } else {
                        // Нет stage → убиваем если amplitude у нуля
                        if (((uint8_t)(r2[3] >> 8)) < 4) kill = true;
                    }
                }
                // else: sustain loop с key_on → IRQ просто очищён, продолжаем

                if (kill) {
                    const uint32_t bit = 1u << si;
                    p.voice_mask         &= ~bit;
                    p.voice_mask_pending &= ~bit;
                    memset(p.ram2[si], 0, sizeof(p.ram2[si]));
                    memset(p.ram1[si], 0, sizeof(p.ram1[si]));
                    // Очищаем VoiceSlot
                    if (si < (uint32_t)kNUM_VOICES) voices_[si] = VoiceSlot{};
                }
            }
            p.irq_assert = 0;
        }
    }
};


