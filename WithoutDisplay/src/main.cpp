/*
  ESP32 Bluetooth speaker: A2DP sink -> software EQ -> PCM5102 (I2S)

  Hardware (original ESP32 only, NOT S2/S3/C3):
    PCM5102 module:  BCK -> GPIO26   LCK/WS -> GPIO25   DIN -> GPIO22
                     VIN -> 3.3V (or 5V per module)  GND -> GND
                     SCK -> GND, FMT -> GND, XSMT -> 3.3V (unmute), FLT -> GND, DEMP -> GND
    ADS1115 (on 5V rail, I2C through a 3.3V <-> 5V level shifter, ADDR -> GND = 0x48):
                     SDA -> GPIO21   SCL -> GPIO19
                     A0 = volume pot, 0..5V
                     A1 = button ladder 1 (idle 5V):  PREV 0.85V  PLAY 1.685V  NEXT 2.5V
                     A2 = button ladder 2 (idle 5V):  -  0.03V  AUDIO 0.865V  +  1.688V  TP 3.34V

  Controls:
    AUDIO toggles whether +/- adjust bass or treble (range -6..+6).
    Beeps: low tone = bass, high tone = treble (on AUDIO and on every +/- press).
    Hold TP while powering on -> setup mode instead of Bluetooth:
      WiFi AP "<BT name> Setup" (password SETUP_AP_PASS), browse to http://192.168.4.1
      Settings page for BT name / bass / treble, firmware update at /update (ElegantOTA).

  Serial commands (115200 baud): B/b = bass up/down, T/t = treble up/down.
  On power-up the speaker reconnects to the last connected phone.
  Settings are saved to flash and restored on boot.
*/

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Adafruit_ADS1X15.h>
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"

// ---------------------------------------------------------------- pins
#define MY_I2S_BCK    26
#define MY_I2S_WS     25
#define MY_I2S_DOUT   22

#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   19
#define I2C_FREQ      100000   // conservative for MOSFET level shifters
#define ADS_ADDR      0x48

// ---------------------------------------------------------------- tuning
#define EQ_MAX          6        // bass/treble steps: -EQ_MAX..+EQ_MAX
#define DB_PER_STEP     2.0f     // each bass/treble step = 2 dB  (+-6 -> +-12 dB)
#define BASS_FREQ_HZ    200.0f   // low-shelf corner
#define TREBLE_FREQ_HZ  4000.0f  // high-shelf corner
#define I2S_PORT        I2S_NUM_0

// Default volume range in dB (digital attenuation below full scale), editable on the setup page.
// Knob value 1 = min, 127 = max, 0 = mute. Raise min if the low end is still inaudible,
// lower max if the top end distorts (the amp input is being overdriven).
#define VOL_MIN_DB      (-45.0f)
#define VOL_MAX_DB      (-4.0f)
#define VOL_DB_LIMIT    (-80.0f)  // lowest value accepted on the setup page

#define POT_VMAX        5.0f     // pot voltage that means full volume
#define OFF_VOL_OFF     5        // knob below this (0..127) = radio off: muted, buttons ignored
#define OFF_VOL_ON      7        // knob must reach this to turn back on (hysteresis)
#define BTN_TOLERANCE_V 0.25f    // +- window around each ladder voltage
#define BTN_DEBOUNCE_MS 30

#define BEEP_LOW_HZ     500.0f   // bass
#define BEEP_HIGH_HZ    1500.0f  // treble
#define BEEP_MS         70
#define BEEP_LEVEL      0.12f    // fraction of full scale, independent of volume

#define DEFAULT_BT_NAME "E36Compact"
#define SETUP_AP_PASS   "speaker123"  // min. 8 characters

BluetoothA2DPSink a2dp_sink;
Preferences prefs;
Adafruit_ADS1115 ads;
WebServer server(80);

// ---------------------------------------------------------------- state
volatile int   g_bass   = 0;          // -EQ_MAX..+EQ_MAX
volatile int   g_treble = 0;          // -EQ_MAX..+EQ_MAX
volatile bool  g_coeffsDirty = true;  // set when EQ or sample rate changes
volatile uint32_t g_sampleRate = 44100;
volatile int   g_volume = 64;         // speaker (knob) volume 0..127, independent of the phone
volatile float g_gain   = 0.25f;      // linear gain derived from g_volume
volatile uint32_t g_lastAudioMs = 0;  // last time the BT stream delivered audio

enum EqSel { SEL_BASS, SEL_TREBLE };
EqSel  g_eqSel = SEL_BASS;
String g_btName = DEFAULT_BT_NAME;
float  g_volMinDb = VOL_MIN_DB;
float  g_volMaxDb = VOL_MAX_DB;
bool   g_adsOk = false;
bool   g_setupMode = false;
bool   g_off = false;                 // knob turned to minimum
bool   g_connBeeps = true;            // chime on Bluetooth connect / disconnect
volatile bool g_btConnected = false;

// Phone's play state from AVRCP notifications. The A2DP stream state lags behind a pause
// by several seconds, so it is only used until the phone has reported a status.
volatile bool g_playStatusKnown = false;
volatile bool g_playing = false;
// ---------------------------------------------------------------- biquad
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float x1[2] = {0, 0}, x2[2] = {0, 0}, y1[2] = {0, 0}, y2[2] = {0, 0};

  inline float process(int ch, float x) {
    float y = b0 * x + b1 * x1[ch] + b2 * x2[ch] - a1 * y1[ch] - a2 * y2[ch];
    x2[ch] = x1[ch]; x1[ch] = x;
    y2[ch] = y1[ch]; y1[ch] = y;
    return y;
  }
  void setCoeffs(float nb0, float nb1, float nb2, float na0, float na1, float na2) {
    b0 = nb0 / na0; b1 = nb1 / na0; b2 = nb2 / na0;
    a1 = na1 / na0; a2 = na2 / na0;
  }
};

Biquad bassFilter, trebleFilter;
float  headroom = 1.0f;  // pre-attenuation so boosts don't clip as often

// RBJ audio-EQ-cookbook shelving filters (shelf slope S = 1)
void makeLowShelf(Biquad &f, float fs, float f0, float dB) {
  float A = powf(10.0f, dB / 40.0f);
  float w0 = 2.0f * (float)M_PI * f0 / fs;
  float cw = cosf(w0), sw = sinf(w0);
  float alpha = sw / 2.0f * sqrtf(2.0f);
  float tsA = 2.0f * sqrtf(A) * alpha;
  f.setCoeffs(
    A * ((A + 1) - (A - 1) * cw + tsA),
    2 * A * ((A - 1) - (A + 1) * cw),
    A * ((A + 1) - (A - 1) * cw - tsA),
    (A + 1) + (A - 1) * cw + tsA,
    -2 * ((A - 1) + (A + 1) * cw),
    (A + 1) + (A - 1) * cw - tsA);
}

void makeHighShelf(Biquad &f, float fs, float f0, float dB) {
  float A = powf(10.0f, dB / 40.0f);
  float w0 = 2.0f * (float)M_PI * f0 / fs;
  float cw = cosf(w0), sw = sinf(w0);
  float alpha = sw / 2.0f * sqrtf(2.0f);
  float tsA = 2.0f * sqrtf(A) * alpha;
  f.setCoeffs(
    A * ((A + 1) + (A - 1) * cw + tsA),
    -2 * A * ((A - 1) + (A + 1) * cw),
    A * ((A + 1) + (A - 1) * cw - tsA),
    (A + 1) - (A - 1) * cw + tsA,
    2 * ((A - 1) - (A + 1) * cw),
    (A + 1) - (A - 1) * cw - tsA);
}

void updateCoeffs() {
  float fs = (float)g_sampleRate;
  float bassDb   = g_bass   * DB_PER_STEP;
  float trebleDb = g_treble * DB_PER_STEP;
  makeLowShelf(bassFilter, fs, BASS_FREQ_HZ, bassDb);
  makeHighShelf(trebleFilter, fs, TREBLE_FREQ_HZ, trebleDb);
  float maxBoost = fmaxf(0.0f, fmaxf(bassDb, trebleDb));
  headroom = powf(10.0f, -maxBoost / 20.0f);
  g_coeffsDirty = false;
}

// ---------------------------------------------------------------- volume
// Speaker (knob) volume only. The phone's own volume is applied separately by the
// A2DP library to the stream before audioCallback(), so the two multiply.
void setVolume(int v) {
  v = constrain(v, 0, 127);
  float dB = 0.0f;
  float gain = 0.0f;  // 0 = mute
  if (v > 0) {
    dB = g_volMinDb + (g_volMaxDb - g_volMinDb) * (v - 1) / 126.0f;  // linear in dB = natural feel
    gain = powf(10.0f, dB / 20.0f);
  }
  if (v != g_volume) {
    if (v > 0) Serial.printf("Volume: %d/127 (%.1f dB)\n", v, dB);
    else       Serial.println("Volume: 0/127 (muted)");
  }
  g_volume = v;
  g_gain = gain;
}

// ---------------------------------------------------------------- I2S
void i2sInit(uint32_t rate) {
  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = rate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 512;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = true;  // output silence on underrun instead of repeating

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = MY_I2S_BCK;
  pins.ws_io_num = MY_I2S_WS;
  pins.data_out_num = MY_I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

// Called by the library if the phone switches to e.g. 48 kHz
void onSampleRateChange(uint16_t rate) {
  g_sampleRate = rate;
  i2s_set_sample_rates(I2S_PORT, rate);
  g_coeffsDirty = true;
}

// ---------------------------------------------------------------- beep
// Mixed into the BT stream while it is running, otherwise written to I2S from loop().
struct Note { float freq; uint16_t ms; };  // freq 0 = rest

const Note CHIME_CONNECT[]    = {{880, 60}, {0, 25}, {1319, 90}};  // rising fifth
const Note CHIME_DISCONNECT[] = {{1319, 60}, {0, 25}, {880, 90}};  // falling fifth

#define MAX_NOTES 4
Note     g_notes[MAX_NOTES];
volatile uint8_t  g_noteCount = 0, g_noteIdx = 0;
volatile uint32_t g_beepLeft = 0;   // samples left in the current note
uint32_t g_beepLen = 0;
float    g_beepPhase = 0, g_beepInc = 0;

void playNotes(const Note *notes, uint8_t count) {
  g_beepLeft = 0;
  g_noteCount = 0;
  count = min<uint8_t>(count, MAX_NOTES);
  for (uint8_t i = 0; i < count; i++) g_notes[i] = notes[i];
  g_noteIdx = 0;
  g_noteCount = count;
}

void startBeep(float freq) {
  Note n = {freq, BEEP_MS};
  playNotes(&n, 1);
}

void stopBeep() {
  g_noteCount = 0;
  g_beepLeft = 0;
}

inline bool beepActive() { return g_beepLeft > 0 || g_noteIdx < g_noteCount; }

// Returns the next beep sample scaled to int16 range, 0 when no beep is active.
inline float nextBeepSample() {
  uint32_t left = g_beepLeft;
  if (left == 0) {
    if (g_noteIdx >= g_noteCount) return 0.0f;
    const Note &n = g_notes[g_noteIdx++];
    float fs = (float)g_sampleRate;
    g_beepInc = 2.0f * (float)M_PI * n.freq / fs;
    g_beepPhase = 0;
    g_beepLen = left = (uint32_t)(fs * n.ms / 1000.0f);
    if (left == 0) return 0.0f;
  }
  float s = 0.0f;
  if (g_beepInc > 0.0f) {
    uint32_t pos = g_beepLen - left;
    float fade = g_sampleRate * 0.005f;  // 5 ms fade in/out against clicks
    float env = fminf(1.0f, fminf(pos / fade, left / fade));
    s = sinf(g_beepPhase) * env * BEEP_LEVEL * 32767.0f;
    g_beepPhase += g_beepInc;
    if (g_beepPhase > 2.0f * (float)M_PI) g_beepPhase -= 2.0f * (float)M_PI;
  }
  g_beepLeft = left - 1;
  return s;
}

void playBeepDirect() {
  static int16_t buf[256 * 2];
  while (beepActive()) {
    for (int i = 0; i < 256; i++) {
      int16_t s = (int16_t)nextBeepSample();
      buf[i * 2] = s;
      buf[i * 2 + 1] = s;
    }
    size_t written;
    i2s_write(I2S_PORT, buf, sizeof(buf), &written, portMAX_DELAY);
  }
}

void beepBlocking(float freq) {
  startBeep(freq);
  playBeepDirect();
}

// ---------------------------------------------------------------- audio path
// Receives decoded 16-bit stereo PCM from Bluetooth, applies EQ + volume,
// writes to I2S. Runs in the Bluetooth/audio task, so keep it fast.
void audioCallback(const uint8_t *data, uint32_t len) {
  g_lastAudioMs = millis();
  if (g_coeffsDirty) updateCoeffs();

  const int16_t *in = (const int16_t *)data;
  uint32_t frames = len / 4;  // 2 channels * 2 bytes
  static int16_t out[512 * 2];
  const float target = g_gain * headroom;
  static float curGain = 0.0f;  // smoothed gain, avoids clicks when volume changes

  while (frames > 0) {
    uint32_t n = frames > 512 ? 512 : frames;
    for (uint32_t i = 0; i < n; i++) {
      curGain += (target - curGain) * 0.002f;  // ~10 ms smoothing at 44.1 kHz
      float beep = nextBeepSample();
      for (int ch = 0; ch < 2; ch++) {
        float s = (float)in[i * 2 + ch];
        s = bassFilter.process(ch, s);
        s = trebleFilter.process(ch, s);
        s = s * curGain + beep;
        if (s > 32767.0f) s = 32767.0f;
        else if (s < -32768.0f) s = -32768.0f;
        out[i * 2 + ch] = (int16_t)s;
      }
    }
    size_t written;
    i2s_write(I2S_PORT, out, n * 4, &written, portMAX_DELAY);
    in += n * 2;
    frames -= n;
  }
}

// ---------------------------------------------------------------- metadata
void onMetadata(uint8_t id, const uint8_t *text) {
  switch (id) {
    case ESP_AVRC_MD_ATTR_TITLE:  Serial.printf("Title : %s\n", (const char *)text); break;
    case ESP_AVRC_MD_ATTR_ARTIST: Serial.printf("Artist: %s\n", (const char *)text); break;
    case ESP_AVRC_MD_ATTR_ALBUM:  Serial.printf("Album : %s\n", (const char *)text); break;
    default: break;
  }
}

// Runs in the Bluetooth task. Disconnect only chimes after a real connection, so the
// failed reconnect attempts at power-up stay silent.
void onConnectionState(esp_a2d_connection_state_t state, void *) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED && !g_btConnected) {
    g_btConnected = true;
    Serial.println("Bluetooth connected");
    if (g_connBeeps && !g_off) playNotes(CHIME_CONNECT, 3);
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED && g_btConnected) {
    g_btConnected = false;
    g_playStatusKnown = false;
    Serial.println("Bluetooth disconnected");
    if (g_connBeeps && !g_off) playNotes(CHIME_DISCONNECT, 3);
  }
}

void onPlayStatus(esp_avrc_playback_stat_t status) {
  g_playStatusKnown = true;
  g_playing = (status == ESP_AVRC_PLAYBACK_PLAYING);
}

// ---------------------------------------------------------------- EQ settings
void saveEq() {
  prefs.putChar("bass", (int8_t)g_bass);
  prefs.putChar("treble", (int8_t)g_treble);
}

void changeEq(int dBass, int dTreble) {
  g_bass   = constrain(g_bass + dBass, -EQ_MAX, EQ_MAX);
  g_treble = constrain(g_treble + dTreble, -EQ_MAX, EQ_MAX);
  g_coeffsDirty = true;
  saveEq();
  Serial.printf("Bass %+d  Treble %+d\n", (int)g_bass, (int)g_treble);
}

// ---------------------------------------------------------------- analog buttons
// A resistor ladder on one ADS1115 input: idle = 5V, each button pulls it to its own voltage.
struct ButtonLadder {
  const float *volts;
  uint8_t count;
  int stable = -1, candidate = -1;  // button index, -1 = none
  uint32_t since = 0;

  ButtonLadder(const float *v, uint8_t n) : volts(v), count(n) {}

  int decode(float v) const {
    for (uint8_t i = 0; i < count; i++)
      if (fabsf(v - volts[i]) < BTN_TOLERANCE_V) return i;
    return -1;
  }

  // returns the button index once per press (debounced), -1 otherwise
  int update(float v) {
    int b = decode(v);
    uint32_t now = millis();
    if (b != candidate) { candidate = b; since = now; }
    if ((now - since) >= BTN_DEBOUNCE_MS && candidate != stable) {
      stable = candidate;
      return stable;
    }
    return -1;
  }
};

enum { BTN1_PREV, BTN1_PLAY, BTN1_NEXT };
const float LADDER1_V[] = {0.85f, 1.685f, 2.5f};
enum { BTN2_MINUS, BTN2_AUDIO, BTN2_PLUS, BTN2_TP };
const float LADDER2_V[] = {0.03f, 0.865f, 1.688f, 3.34f};

ButtonLadder ladder1{LADDER1_V, 3}, ladder2{LADDER2_V, 4};

const uint16_t ADS_MUX[] = {
  ADS1X15_REG_CONFIG_MUX_SINGLE_0,
  ADS1X15_REG_CONFIG_MUX_SINGLE_1,
  ADS1X15_REG_CONFIG_MUX_SINGLE_2,
};

void beepForSelection() {
  startBeep(g_eqSel == SEL_BASS ? BEEP_LOW_HZ : BEEP_HIGH_HZ);
}

void handlePot(float v) {
  static float potSmooth = -1;
  static int lastPotVol = -100;
  potSmooth = (potSmooth < 0) ? v : potSmooth + 0.1f * (v - potSmooth);
  int vol = constrain((int)(potSmooth * 127.0f / POT_VMAX + 0.5f), 0, 127);

  bool off = g_off ? (vol < OFF_VOL_ON) : (vol < OFF_VOL_OFF);
  if (off != g_off) {
    g_off = off;
    Serial.println(off ? "Radio off (knob at minimum)" : "Radio on");
    if (off) stopBeep();
    lastPotVol = vol;
    setVolume(off ? 0 : vol);
    return;
  }
  if (!g_off && abs(vol - lastPotVol) >= 2) {  // only act when the knob really moved
    lastPotVol = vol;
    setVolume(vol);
  }
}

void handleLadder1(float v) {
  int b = ladder1.update(v);  // keep debouncing even while off
  if (g_off) return;
  switch (b) {
    case BTN1_PLAY: {
      bool playing = g_playStatusKnown ? g_playing
                                       : a2dp_sink.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED;
      if (playing) a2dp_sink.pause();
      else a2dp_sink.play();
      g_playing = !playing;  // assume it worked until the phone reports otherwise
      break;
    }
    case BTN1_NEXT: a2dp_sink.next(); break;
    case BTN1_PREV: a2dp_sink.previous(); break;
  }
}

void handleLadder2(float v) {
  int d = 0;
  int b = ladder2.update(v);  // keep debouncing even while off
  if (g_off) return;
  switch (b) {
    case BTN2_AUDIO:
      g_eqSel = (g_eqSel == SEL_BASS) ? SEL_TREBLE : SEL_BASS;
      Serial.printf("Adjusting %s\n", g_eqSel == SEL_BASS ? "bass" : "treble");
      beepForSelection();
      return;
    case BTN2_PLUS:  d = +1; break;
    case BTN2_MINUS: d = -1; break;
    default: return;
  }
  int before = (g_eqSel == SEL_BASS) ? g_bass : g_treble;
  if (g_eqSel == SEL_BASS) changeEq(d, 0);
  else changeEq(0, d);
  int value = (g_eqSel == SEL_BASS) ? g_bass : g_treble;
  if (value != before && value != 0) beepForSelection();  // silent at the limits and at flat (0)
}

// Reads the three inputs round-robin without blocking (one conversion every ~2 ms).
void pollAds() {
  static uint8_t ch = 0;
  static bool busy = false;
  static uint32_t t0 = 0;
  if (!g_adsOk) return;
  if (!busy) {
    ads.startADCReading(ADS_MUX[ch], false);
    t0 = micros();
    busy = true;
    return;
  }
  if (micros() - t0 < 2000) return;  // 860 SPS -> 1.16 ms per conversion
  busy = false;
  float v = ads.computeVolts(ads.getLastConversionResults());
  switch (ch) {
    case 0: handlePot(v); break;
    case 1: handleLadder1(v); break;
    case 2: handleLadder2(v); break;
  }
  ch = (ch + 1) % 3;
}

bool tpHeldAtBoot() {
  if (!g_adsOk) return false;
  for (int i = 0; i < 5; i++) {
    float v = ads.computeVolts(ads.readADC_SingleEnded(2));
    if (ladder2.decode(v) != BTN2_TP) return false;
    delay(10);
  }
  return true;
}

// ---------------------------------------------------------------- setup mode (WiFi)
String htmlEscape(const String &s) {
  String o;
  for (char c : s) {
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      default:  o += c;
    }
  }
  return o;
}

String eqOptions(int current) {
  String o;
  for (int i = -EQ_MAX; i <= EQ_MAX; i++) {
    o += "<option value=\"" + String(i) + "\"";
    if (i == current) o += " selected";
    o += ">" + String(i > 0 ? "+" : "") + String(i) + "</option>";
  }
  return o;
}

void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Speaker setup</title><style>"
    "body{font-family:sans-serif;max-width:420px;margin:2em auto;padding:0 1em}"
    "label{display:block;margin-top:1em}input,select{width:100%;padding:.4em;font-size:1em}"
    "button{margin-top:1.5em;padding:.6em 1.2em;font-size:1em}.ok{color:green}"
    "</style></head><body><h2>Speaker setup</h2>";
  if (server.hasArg("saved")) html += "<p class=\"ok\">Saved. The Bluetooth name applies after reboot.</p>";
  html +=
    "<form method=\"POST\" action=\"/save\">"
    "<label>Bluetooth name<input name=\"name\" maxlength=\"32\" required value=\"" + htmlEscape(g_btName) + "\"></label>"
    "<label>Bass<select name=\"bass\">" + eqOptions(g_bass) + "</select></label>"
    "<label>Treble<select name=\"treble\">" + eqOptions(g_treble) + "</select></label>"
    "<label>Volume at knob minimum (dB)<input name=\"volmin\" type=\"number\" step=\"0.5\" min=\"" + String(VOL_DB_LIMIT, 0) +
      "\" max=\"0\" value=\"" + String(g_volMinDb, 1) + "\"></label>"
    "<label>Volume at knob maximum (dB)<input name=\"volmax\" type=\"number\" step=\"0.5\" min=\"" + String(VOL_DB_LIMIT, 0) +
      "\" max=\"0\" value=\"" + String(g_volMaxDb, 1) + "\"></label>"
    "<label style=\"display:flex;align-items:center;gap:.5em\"><input name=\"connbeep\" type=\"checkbox\" style=\"width:auto\"" +
      String(g_connBeeps ? " checked" : "") + ">Chime on Bluetooth connect / disconnect</label>"
    "<button type=\"submit\">Save</button></form>"
    "<p><a href=\"/update\">Firmware update</a></p>"
    "<form method=\"POST\" action=\"/reboot\"><button type=\"submit\">Reboot to speaker mode</button></form>"
    "</body></html>";
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("name")) {
    String n = server.arg("name");
    n.trim();
    if (n.length() > 0 && n.length() <= 32) {
      g_btName = n;
      prefs.putString("btname", g_btName);
    }
  }
  if (server.hasArg("bass"))   g_bass   = constrain(server.arg("bass").toInt(), -EQ_MAX, EQ_MAX);
  if (server.hasArg("treble")) g_treble = constrain(server.arg("treble").toInt(), -EQ_MAX, EQ_MAX);
  saveEq();
  if (server.hasArg("volmin") && server.hasArg("volmax")) {
    float lo = constrain(server.arg("volmin").toFloat(), VOL_DB_LIMIT, 0.0f);
    float hi = constrain(server.arg("volmax").toFloat(), VOL_DB_LIMIT, 0.0f);
    if (lo < hi) {
      g_volMinDb = lo;
      g_volMaxDb = hi;
      prefs.putFloat("volmin", g_volMinDb);
      prefs.putFloat("volmax", g_volMaxDb);
    }
  }
  g_connBeeps = server.hasArg("connbeep");  // unchecked boxes are not submitted
  prefs.putBool("connbeep", g_connBeeps);
  Serial.printf("Saved: name '%s'  bass %+d  treble %+d  volume %.1f..%.1f dB  chime %s\n", g_btName.c_str(),
                (int)g_bass, (int)g_treble, g_volMinDb, g_volMaxDb, g_connBeeps ? "on" : "off");
  server.sendHeader("Location", "/?saved=1");
  server.send(303);
}

void handleReboot() {
  server.send(200, "text/html", "<p>Rebooting into speaker mode...</p>");
  delay(500);
  ESP.restart();
}

void startSetupMode() {
  String ssid = g_btName + " Setup";
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(), SETUP_AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reboot", HTTP_POST, handleReboot);
  ElegantOTA.begin(&server);
  server.begin();

  Serial.printf("Setup mode: join WiFi '%s' (password '%s'), open http://%s\n",
                ssid.c_str(), SETUP_AP_PASS, WiFi.softAPIP().toString().c_str());

  beepBlocking(BEEP_HIGH_HZ);
  delay(80);
  beepBlocking(BEEP_HIGH_HZ);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);

  prefs.begin("audio", false);
  g_bass   = constrain((int)prefs.getChar("bass", 0), -EQ_MAX, EQ_MAX);
  g_treble = constrain((int)prefs.getChar("treble", 0), -EQ_MAX, EQ_MAX);
  g_btName = prefs.getString("btname", DEFAULT_BT_NAME);
  g_volMinDb = prefs.getFloat("volmin", VOL_MIN_DB);
  g_volMaxDb = prefs.getFloat("volmax", VOL_MAX_DB);
  if (!(g_volMinDb < g_volMaxDb)) { g_volMinDb = VOL_MIN_DB; g_volMaxDb = VOL_MAX_DB; }
  g_connBeeps = prefs.getBool("connbeep", true);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
  g_adsOk = ads.begin(ADS_ADDR, &Wire);
  if (g_adsOk) {
    ads.setGain(GAIN_TWOTHIRDS);  // +-6.144V full scale, covers the 0..5V inputs
    ads.setDataRate(RATE_ADS1115_860SPS);
  } else {
    Serial.println("ADS1115 not found - buttons and volume knob disabled");
  }

  i2sInit(44100);

  if (tpHeldAtBoot()) {
    g_setupMode = true;
    startSetupMode();
    return;
  }

  setVolume(64);

  a2dp_sink.set_avrc_metadata_attribute_mask(
      ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM);
  a2dp_sink.set_avrc_metadata_callback(onMetadata);
  a2dp_sink.set_avrc_rn_playstatus_callback(onPlayStatus);
  a2dp_sink.set_on_connection_state_changed(onConnectionState);
  a2dp_sink.set_sample_rate_callback(onSampleRateChange);
  a2dp_sink.set_stream_reader(audioCallback, false);  // false = we do the I2S output ourselves
  a2dp_sink.start(g_btName.c_str(), true);  // true = reconnect to the last phone on power-up

  Serial.printf("Ready. Pair with '%s'.\n", g_btName.c_str());
}

void loop() {
  if (g_setupMode) {
    server.handleClient();
    ElegantOTA.loop();
    delay(2);
    return;
  }

  pollAds();

  // beep requested while no stream is running -> nobody else is writing to I2S
  if (beepActive() && (g_lastAudioMs == 0 || millis() - g_lastAudioMs > 200)) playBeepDirect();

  // --- bass / treble over serial
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'B': changeEq(+1, 0); break;
      case 'b': changeEq(-1, 0); break;
      case 'T': changeEq(0, +1); break;
      case 't': changeEq(0, -1); break;
    }
  }

  delay(1);
}
