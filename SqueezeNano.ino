/*
 * Squeeze Nano - minimal SlimProto (Logitech Media Server) player for ESP32 boards with I2S DAC
 * Multi room audio supported
 * ESP32 boards : Requires ESP32-S2 with 2MB PSRAM at a minimum; S3 (dual core) is preferable
 * 
 * Arduino IDE settings:
 *   Board            : ESP32 S2/S3
 *   PSRAM            : QSPI PSRAM Enabled
 *   CPU Frequency    : 240MHz
 *   USB CDC On Boot  : Enabled (for Serial output)
 *   Flash size       : 4MB
 *   Partition scheme : Default 4MB with spiffs
 *   Library          : "ESP8266Audio" by Earle F. Philhower, III (install via Library Manager)
 *
 * Audio path:
 *   HTTP stream -> ring buffer (PSRAM) -> decoder -> PCM FIFO (PSRAM)
 *               -> dedicated I2S task (new driver on core 3.x, legacy on 2.x)
 *   The I2S task starts output at the timestamp sent by LMS ("strm u"), so
 *   several players can start in the same millisecond. The elapsed time reported
 *   to LMS is derived from the number of frames actually handed to the I2S DMA.
 *
 * I2S Pins configuration:
 *    PIN_I2S_BCLK
 *    PIN_I2S_LRC
 *    PIN_I2S_DOUT
 * 
 * Limitations:
 *   - support mp3, flac and aac only, no raw PCM
 *   - no gapless / crossfade
 *   - HTTP chunked transfer and ICY metadata are not supported
 *   - sample rate is limited by advertising MaxSampleRate to the server,
 *     so LMS resamples before sending (S2:48000 S3:96000)
 */

// ---------------------------------------------------------------- Configuration
#define WIFI_SSID        "your-ssid"
#define WIFI_PASSWORD    "your-password"

#define LMS_HOST         ""            // LMS IP or host name. Empty = UDP discovery
#define PLAYER_NAME      "SqueezeNano"

#define STREAM_BUF_SIZE  (768 * 1024)  // network ring buffer, allocated in PSRAM
#define PCM_FIFO_FRAMES  65536         // decoded PCM FIFO, MUST be a power of two
                                       // (mono: 128 KB, stereo: 256 KB, in PSRAM)
#define MONO_OUTPUT      0             // 1 = downmix L+R to mono (same signal on both I2S slots)
                                       // This also halves the PCM FIFO memory.

#define ENABLE_FLAC      1             // flac decoding needs more RAM (uses PSRAM)
#define ENABLE_AAC       1

// I2S DMA geometry. Its length adds a fixed output latency that is compensated
// when starting at a timestamp. Keep the same values on every player of a group.
#define I2S_DMA_COUNT    8
#define I2S_DMA_FRAMES   256

// Fine tuning of the start time, in ms. Positive = start earlier (use it when this
// player is heard LATE compared with the others), negative = start later.
// Example: if it lags by about 30 ms, set 30. The LMS per-player start delay works too.
#define START_ADVANCE_MS 0

// ESP32-S2 has a single core: the I2S task shares it with the loop task and WiFi.
// MP3 is comfortable there; flac may glitch (set ENABLE_FLAC to 0 if it does).
#if CONFIG_IDF_TARGET_ESP32S2
  #define SQZ_SINGLE_CORE 1
  #define PIN_I2S_BCLK    33
  #define PIN_I2S_LRC     4
  #define PIN_I2S_DOUT    35
  #define MAX_SAMPLE_RATE  48000         // advertised to LMS (it resamples above this)
#else
  #define SQZ_SINGLE_CORE 0
  #define PIN_I2S_BCLK    35
  #define PIN_I2S_LRC     5
  #define PIN_I2S_DOUT    36
  #define MAX_SAMPLE_RATE  96000         // advertised to LMS (it resamples above this)
#endif

// ---------------------------------------------------------------- Includes
#include <WiFi.h>
#include <WiFiUdp.h>
#include <atomic>
#include "AudioFileSource.h"
#include "AudioOutput.h"
#include "AudioGeneratorMP3.h"
#if ENABLE_FLAC
#include "AudioGeneratorFLAC.h"
#endif
#if ENABLE_AAC
#include "AudioGeneratorAAC.h"
#endif

// The loop task also runs the decoders; give it more stack than the 8 KB default
#ifdef SET_LOOP_TASK_STACK_SIZE
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

// ---------------------------------------------------------------- Helpers
static inline void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
static inline void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static inline uint16_t get16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static inline uint32_t get32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline size_t minz(size_t a, size_t b) { return a < b ? a : b; }

// ---------------------------------------------------------------- Stream ring buffer (PSRAM)
static uint8_t *rb = nullptr;
static size_t rbHead = 0, rbTail = 0, rbCount = 0;

static size_t rbFree() { return STREAM_BUF_SIZE - rbCount; }

static size_t rbWrite(const uint8_t *src, size_t n) {
  n = minz(n, rbFree());
  size_t first = minz(n, STREAM_BUF_SIZE - rbHead);
  memcpy(rb + rbHead, src, first);
  memcpy(rb, src + first, n - first);
  rbHead = (rbHead + n) % STREAM_BUF_SIZE;
  rbCount += n;
  return n;
}

static size_t rbRead(uint8_t *dst, size_t n) {
  n = minz(n, rbCount);
  size_t first = minz(n, STREAM_BUF_SIZE - rbTail);
  memcpy(dst, rb + rbTail, first);
  memcpy(dst + first, rb, n - first);
  rbTail = (rbTail + n) % STREAM_BUF_SIZE;
  rbCount -= n;
  return n;
}

static void rbClear() { rbHead = rbTail = rbCount = 0; }

// ---------------------------------------------------------------- PCM output: FIFO + I2S task
/*
 * Single producer (decoder, loop task) / single consumer (I2S task) FIFO using
 * monotonic frame counters. All control flags are atomics. The I2S task is the
 * only one that touches the read index, so a flush is requested via a flag.
 */
#define I2S_CHUNK_FRAMES  128
#define PCM_SAMPLES_FRAME (MONO_OUTPUT ? 1 : 2)

/*
 * Minimal I2S HAL. Arduino-ESP32 3.x (ESP-IDF 5) must use the new driver: the
 * ESP8266Audio library is linked into the sketch and pulls in the new driver, and
 * the new and legacy drivers abort at boot when both are present
 * ("CONFLICT! The new i2s driver can't work along with the legacy i2s driver").
 * The I2S task writes continuously (silence when idle), so no auto-clear is needed.
 */
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
#include "driver/i2s_std.h"

static i2s_chan_handle_t i2sTx = NULL;
static i2s_std_config_t i2sCfg;

static bool i2sHalInit(int rate) {
  i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan.dma_desc_num = I2S_DMA_COUNT;
  chan.dma_frame_num = I2S_DMA_FRAMES;
  if (i2s_new_channel(&chan, &i2sTx, NULL) != ESP_OK) return false;

  memset(&i2sCfg, 0, sizeof(i2sCfg));
  i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
  i2s_std_slot_config_t slot =
      I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  i2sCfg.clk_cfg = clk;
  i2sCfg.slot_cfg = slot;
  i2sCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  i2sCfg.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  i2sCfg.gpio_cfg.ws = (gpio_num_t)PIN_I2S_LRC;
  i2sCfg.gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
  i2sCfg.gpio_cfg.din = I2S_GPIO_UNUSED;
  if (i2s_channel_init_std_mode(i2sTx, &i2sCfg) != ESP_OK) return false;
  return i2s_channel_enable(i2sTx) == ESP_OK;
}

static void i2sHalSetRate(int rate) {
  i2s_channel_disable(i2sTx);
  i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
  i2s_channel_reconfig_std_clock(i2sTx, &clk);
  i2s_channel_enable(i2sTx);
}

// Writes interleaved stereo 16-bit frames, returns the number of frames accepted
static size_t i2sHalWrite(const int16_t *buf, size_t frames) {
  size_t bw = 0;
  i2s_channel_write(i2sTx, buf, frames * 4, &bw, 100);
  return bw / 4;
}

#else  // Arduino-ESP32 2.x: legacy driver
#include "driver/i2s.h"

static bool i2sHalInit(int rate) {
  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = rate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = I2S_DMA_COUNT;
  cfg.dma_buf_len = I2S_DMA_FRAMES;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return false;

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_LRC;
  pins.data_out_num = PIN_I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;
  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

static void i2sHalSetRate(int rate) { i2s_set_sample_rates(I2S_NUM_0, rate); }

static size_t i2sHalWrite(const int16_t *buf, size_t frames) {
  size_t bw = 0;
  i2s_write(I2S_NUM_0, buf, frames * 4, &bw, pdMS_TO_TICKS(100));
  return bw / 4;
}
#endif

class PcmOutput : public AudioOutput {
public:
  // ---- AudioOutput interface (called by the decoder in the loop task)
  bool begin() { return true; }           // I2S is owned by this class
  bool stop() { return true; }            // keep the FIFO alive while it drains
  bool SetRate(int hz) { desiredRate = hz; return true; }
  bool ConsumeSample(int16_t s[2]) {
    uint32_t w = wIdx.load(std::memory_order_relaxed);
    uint32_t r = rIdx.load(std::memory_order_acquire);
    if (w - r >= PCM_FIFO_FRAMES) return false;  // FIFO full, decoder retries later
    uint32_t i = w & (PCM_FIFO_FRAMES - 1);
#if MONO_OUTPUT
    fifo[i] = (int16_t)(((int32_t)s[0] + (int32_t)s[1]) / 2);
#else
    fifo[2 * i] = s[0];
    fifo[2 * i + 1] = s[1];
#endif
    wIdx.store(w + 1, std::memory_order_release);
    return true;
  }

  // ---- Setup
  bool initI2S() {
    fifo = (int16_t *)ps_malloc((size_t)PCM_FIFO_FRAMES * PCM_SAMPLES_FRAME * sizeof(int16_t));
    if (!fifo) return false;

    if (!i2sHalInit(44100)) return false;
    appliedRate = 44100;

    // Dual core: run on core 1 (WiFi lives on core 0). Single core: the only core, 0.
    xTaskCreatePinnedToCore(taskEntry, "i2s", 4096, this, 5, NULL, SQZ_SINGLE_CORE ? 0 : 1);
    return true;
  }

  // ---- Control (called from the loop task)
  void reset() {
    armed = false;
    paused = false;
    decoderDone = false;
    flushReq = true;
    uint32_t t0 = millis();
    while (flushReq && millis() - t0 < 200) delay(1);
  }
  // startAt: player clock (millis) at which sound must come out, 0 = as soon as possible
  void arm(uint32_t startAt) {
    startAtMs = startAt;
    armed = true;
  }
  void setPause(bool p) { paused = p; }
  void skipAheadMs(uint32_t ms) {
    skipReq += (uint32_t)((uint64_t)ms * desiredRate.load() / 1000);
  }
  void setDecoderDone() { decoderDone = true; }
  void setVolume(float v) {
    int32_t g = (int32_t)(v * 256.0f);
    if (g < 0) g = 0;
    if (g > 1024) g = 1024;
    gainQ8 = g;
  }

  // ---- Status
  bool isDrained() { return drained; }
  bool consumeStarted() { return started.exchange(false); }
  int32_t lastStartLagMs() { return startLagMs; }
  uint32_t availFrames() { return wIdx.load() - rIdx.load(); }
  uint32_t fillBytes() { return availFrames() * PCM_SAMPLES_FRAME * 2; }
  bool nearlyFull() { return availFrames() > (PCM_FIFO_FRAMES / 4) * 3; }
  uint32_t dmaMs() {
    int rate = appliedRate;
    return rate ? (uint32_t)I2S_DMA_COUNT * I2S_DMA_FRAMES * 1000 / rate : 0;
  }
  // Position of the sample that is audible now, derived from frames given to the DMA
  uint32_t elapsedMs() {
    int rate = appliedRate;
    if (!rate || !running) return 0;
    uint32_t played = framesPlayed;
    uint32_t inDma = paused ? 0 : (uint32_t)I2S_DMA_COUNT * I2S_DMA_FRAMES;
    uint32_t f = played > inDma ? played - inDma : 0;
    return (uint32_t)((uint64_t)f * 1000 / rate);
  }

private:
  int16_t *fifo = nullptr;
  std::atomic<uint32_t> wIdx{0}, rIdx{0};
  std::atomic<bool> armed{false}, running{false}, paused{false};
  std::atomic<bool> decoderDone{false}, drained{false}, started{false}, flushReq{false};
  std::atomic<int> desiredRate{44100};
  std::atomic<uint32_t> framesPlayed{0};
  std::atomic<uint32_t> skipReq{0};   // frames to drop from the FIFO head (skip ahead)
  volatile uint32_t startAtMs = 0;
  volatile int32_t gainQ8 = 128;
  volatile int32_t startLagMs = 0;
  int appliedRate = 44100;

  static void taskEntry(void *arg) { ((PcmOutput *)arg)->run(); }

  // Feeds one chunk of zeros. The blocking write also paces the task at the I2S rate.
  static void writeSilence() {
    static int16_t zeros[I2S_CHUNK_FRAMES * 2];  // zero-initialised
    if (i2sHalWrite(zeros, I2S_CHUNK_FRAMES) == 0) vTaskDelay(1);
  }

  void run() {
    static int16_t ob[I2S_CHUNK_FRAMES * 2];
    for (;;) {
      if (flushReq) {
        rIdx.store(wIdx.load());
        running = false;
        started = false;
        drained = false;
        framesPlayed = 0;
        skipReq = 0;
        flushReq = false;
      }

      if (!running) {
        if (armed) {
          uint32_t avail = wIdx.load() - rIdx.load();
          int rate = desiredRate;
          uint32_t latency = (uint32_t)I2S_DMA_COUNT * I2S_DMA_FRAMES * 1000 / rate;
          uint32_t at = startAtMs;
          // Start early by the DMA latency so the sound is audible at the requested time
          int32_t lag = (int32_t)(millis() + latency - at);  // > 0: later than the target time
          bool timeOk = (at == 0) || (lag + START_ADVANCE_MS >= 0);
          // Immediate start waits for some PCM; timestamped start is never delayed
          bool dataOk = (at != 0) || avail >= 2048 || decoderDone;
          if (timeOk && dataOk) {
            if (rate != appliedRate) {
              i2sHalSetRate(rate);
              appliedRate = rate;
            }
            startLagMs = (at == 0) ? 0 : lag;
            if (at != 0 && lag > 0) {
              // The target time has already passed: skip the audio that should have
              // been played by now so this player lines up with the others
              uint32_t capMs = lag > 3000 ? 3000 : (uint32_t)lag;
              skipReq += (uint32_t)((uint64_t)capMs * rate / 1000);
            }
            running = true;
            started = true;
          }
        }
        if (!running) { writeSilence(); continue; }
      }

      if (paused) { writeSilence(); continue; }

      uint32_t r = rIdx.load(std::memory_order_relaxed);
      uint32_t w = wIdx.load(std::memory_order_acquire);
      uint32_t sk = skipReq.exchange(0);
      if (sk) {
        uint32_t d = w - r;
        if (d > sk) d = sk;
        r += d;
        rIdx.store(r, std::memory_order_release);
        framesPlayed += d;               // the audible position moves forward as well
        if (sk > d) skipReq += sk - d;   // not enough data yet, keep the rest pending
      }
      uint32_t n = w - r;
      if (n > I2S_CHUNK_FRAMES) n = I2S_CHUNK_FRAMES;
      if (n == 0) {
        if (decoderDone) drained = true;  // everything has been handed to the DMA
        writeSilence();                   // underrun or end of track
        continue;
      }

      int32_t g = gainQ8;
      for (uint32_t i = 0; i < n; i++) {
        uint32_t idx = (r + i) & (PCM_FIFO_FRAMES - 1);
#if MONO_OUTPUT
        int32_t l = fifo[idx], rr = l;
#else
        int32_t l = fifo[2 * idx], rr = fifo[2 * idx + 1];
#endif
        l = (l * g) >> 8;
        rr = (rr * g) >> 8;
        if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
        if (rr > 32767) rr = 32767; else if (rr < -32768) rr = -32768;
        ob[2 * i] = (int16_t)l;
        ob[2 * i + 1] = (int16_t)rr;
      }
      uint32_t done = (uint32_t)i2sHalWrite(ob, n);
      if (done == 0) vTaskDelay(1);  // write failed or timed out, avoid a busy loop
      rIdx.store(r + done, std::memory_order_release);
      framesPlayed += done;
    }
  }
};

static PcmOutput pcm;

// ---------------------------------------------------------------- Global state
enum StreamState { ST_NONE, ST_HEADERS, ST_BODY };
enum PlayState   { PS_IDLE, PS_BUFFERING, PS_WAIT_UNPAUSE, PS_PLAYING, PS_PAUSED, PS_DRAINING };

static WiFiClient   ctl;           // SlimProto control connection (TCP 3483)
static WiFiClient   streamClient;  // HTTP audio stream
static IPAddress    serverIP;

static StreamState  streamState = ST_NONE;
static PlayState    playState = PS_IDLE;
static String       hdrBuf;
static uint32_t     streamStartMs = 0;
static bool         streamEnded = false;
static uint64_t     bytesReceived = 0;

static char         curFormat = 0;       // 'm' mp3, 'f' flac, 'a' aac
static char         autostartMode = '1';
static size_t       thresholdBytes = 0;

static uint32_t     resumeAtMs = 0;      // timed pause ("strm p" with interval)
static uint32_t     lastStatMs = 0;
static float        volume = 0.5f;

static AudioGenerator *dec = nullptr;

static void pumpStream();
static void sendStat(const char *event, uint32_t serverTs = 0);

// ---------------------------------------------------------------- Audio source fed by the ring buffer
class RingSource : public AudioFileSource {
public:
  bool open(const char *) { pos = 0; return true; }
  bool close() { return true; }
  bool seek(int32_t, int) { return false; }
  // The FLAC decoder treats "getPos() >= getSize()" as end of stream, so a live stream must
  // always report a size ahead of the read position until it has ended and drained.
  uint32_t getSize() { return pos + rbCount + (streamEnded ? 0 : (1u << 28)); }
  uint32_t getPos() { return pos; }
  // Stays "open" until the stream has ended and the buffer is drained
  bool isOpen() { return rbCount > 0 || !streamEnded; }
  uint32_t read(void *data, uint32_t len) {
    // Wait briefly for the network so decoders never see a false EOF on a short underrun
    uint32_t t0 = millis();
    while (rbCount == 0 && !streamEnded && millis() - t0 < 3000) {
      pumpStream();
      delay(1);
    }
    size_t n = rbRead((uint8_t *)data, len);
    pos += n;
    return n;
  }
private:
  uint32_t pos = 0;
};
static RingSource src;

// ---------------------------------------------------------------- STAT message to the server
static void sendStat(const char *event, uint32_t serverTs) {
  if (!ctl.connected()) return;
  uint32_t el = pcm.elapsedMs();
  uint8_t b[61];
  memset(b, 0, sizeof(b));
  memcpy(b, "STAT", 4);
  put32(b + 4, 53);                          // payload length (without opcode + length)
  memcpy(b + 8, event, 4);
  put32(b + 15, STREAM_BUF_SIZE);            // stream buffer size
  put32(b + 19, rbCount);                    // stream buffer fullness
  put32(b + 23, (uint32_t)(bytesReceived >> 32));
  put32(b + 27, (uint32_t)bytesReceived);
  put16(b + 31, 0xFFFF);                     // signal strength (unused)
  put32(b + 33, millis());                   // jiffies: the clock LMS uses for "strm u"
  put32(b + 37, (uint32_t)PCM_FIFO_FRAMES * PCM_SAMPLES_FRAME * 2);  // output buffer size
  put32(b + 41, pcm.fillBytes());            // output buffer fullness
  put32(b + 45, el / 1000);
  put32(b + 51, el);
  put32(b + 55, serverTs);                   // echo of the server timestamp
  ctl.write(b, sizeof(b));
}

// ---------------------------------------------------------------- Playback control
static void deleteDecoder() {
  if (dec) {
    dec->stop();
    delete dec;
    dec = nullptr;
  }
}

static void stopAll() {
  deleteDecoder();
  pcm.reset();
  streamClient.stop();
  streamState = ST_NONE;
  streamEnded = false;
  hdrBuf = "";
  rbClear();
  resumeAtMs = 0;
  playState = PS_IDLE;
}

// Create the decoder and let it pre-fill the PCM FIFO. Output stays gated until pcm.arm().
static bool startDecoder() {
  deleteDecoder();
  pcm.reset();
  switch (curFormat) {
    case 'm': dec = new AudioGeneratorMP3(); break;
#if ENABLE_FLAC
    case 'f': dec = new AudioGeneratorFLAC(); break;
#endif
#if ENABLE_AAC
    case 'a': dec = new AudioGeneratorAAC(); break;
#endif
    default: break;
  }
  if (!dec) {
    Serial.printf("Unsupported format '%c'\n", curFormat);
    sendStat("STMn");
    stopAll();
    return false;
  }
  src.open("");
  pcm.setVolume(volume);
  if (!dec->begin(&src, &pcm)) {
    Serial.println("Decoder begin() failed");
    sendStat("STMn");
    stopAll();
    return false;
  }
  return true;
}

static void decodeStep() {
  if (!dec) return;
  if (!dec->isRunning() || !dec->loop()) {
    deleteDecoder();
    pcm.setDecoderDone();
    Serial.println("Decoder finished");
  } else if (pcm.nearlyFull()) {
    delay(2);  // FIFO is full: yield instead of spinning
  }
}

static void finishTrack() {
  streamClient.stop();
  streamState = ST_NONE;
  streamEnded = false;
  rbClear();
  pcm.reset();
  playState = PS_IDLE;
  sendStat("STMd");  // decode finished
  sendStat("STMu");  // underrun / track finished
  Serial.println("Track finished");
}

static void startStream(const uint8_t *d, size_t len) {
  stopAll();
  curFormat = (char)d[6];
  autostartMode = (char)d[5];
  thresholdBytes = (size_t)d[11] * 1024;
  if (thresholdBytes > STREAM_BUF_SIZE / 2) thresholdBytes = STREAM_BUF_SIZE / 2;
  uint16_t port = get16(d + 22);
  IPAddress ip = get32(d + 24) ? IPAddress(d[24], d[25], d[26], d[27]) : serverIP;

  Serial.printf("Stream: format '%c' from %s:%u\n", curFormat, ip.toString().c_str(), port);
  if (!streamClient.connect(ip, port, 3000)) {
    Serial.println("Stream connect failed");
    sendStat("STMn");
    return;
  }
  streamClient.setNoDelay(true);
  streamClient.write(d + 28, len - 28);  // HTTP request supplied by the server
  bytesReceived = 0;
  streamEnded = false;
  hdrBuf = "";
  streamState = ST_HEADERS;
  streamStartMs = millis();
  playState = PS_BUFFERING;
  sendStat("STMc");  // connected
}

// ---------------------------------------------------------------- HTTP stream reader
static void streamError(const char *msg) {
  Serial.println(msg);
  sendStat("STMn");
  stopAll();
}

static void pumpStream() {
  if (streamState == ST_NONE) return;

  if (streamState == ST_HEADERS) {
    if (millis() - streamStartMs > 10000) { streamError("HTTP header timeout"); return; }
    while (streamClient.available()) {
      hdrBuf += (char)streamClient.read();
      if (hdrBuf.endsWith("\r\n\r\n")) {
        String low = hdrBuf;
        low.toLowerCase();
        if (!hdrBuf.startsWith("HTTP/1.") || hdrBuf.substring(9, 12) != "200") {
          streamError("HTTP error response");
          return;
        }
        if (low.indexOf("transfer-encoding: chunked") >= 0) {
          streamError("Chunked transfer is not supported");
          return;
        }
        streamState = ST_BODY;
        sendStat("STMh");  // headers received
        break;
      }
    }
    if (streamState != ST_BODY) return;
  }

  // Body: move network data into the ring buffer
  uint8_t tmp[1024];
  for (int i = 0; i < 8; i++) {
    int avail = streamClient.available();
    if (avail <= 0 || rbFree() == 0) break;
    int r = streamClient.read(tmp, minz(minz(avail, sizeof(tmp)), rbFree()));
    if (r <= 0) break;
    rbWrite(tmp, r);
    bytesReceived += r;
  }
  if (!streamClient.connected() && streamClient.available() == 0) streamEnded = true;
}

// ---------------------------------------------------------------- SlimProto
static String buildCaps() {
  String c = String("Model=squeezelite,ModelName=" PLAYER_NAME
                    ",AccuratePlayPoints=1,HasDigitalOut=0,MaxSampleRate=") + String(MAX_SAMPLE_RATE);
  c += ",mp3";
#if ENABLE_FLAC
  c += ",flc";
#endif
#if ENABLE_AAC
  c += ",aac";
#endif
  return c;
}

static void sendHelo() {
  String caps = buildCaps();
  uint8_t b[400];
  memset(b, 0, sizeof(b));
  size_t capLen = minz(caps.length(), sizeof(b) - 44);
  memcpy(b, "HELO", 4);
  put32(b + 4, 36 + capLen);
  b[8] = 12;                       // device id (squeezelite)
  b[9] = 0;                        // revision
  WiFi.macAddress(b + 10);         // player id
  memcpy(b + 42, "en", 2);         // language
  memcpy(b + 44, caps.c_str(), capLen);
  ctl.write(b, 44 + capLen);
}

static void sendSetdName() {
  size_t n = strlen(PLAYER_NAME);
  uint8_t b[64];
  memcpy(b, "SETD", 4);
  put32(b + 4, 1 + n + 1);
  b[8] = 0;                        // id 0 = player name
  memcpy(b + 9, PLAYER_NAME, n);
  b[9 + n] = 0;
  ctl.write(b, 10 + n);
}

static void onStrm(const uint8_t *d, size_t len) {
  if (len < 28) return;
  char cmd = (char)d[4];
  uint32_t field = get32(d + 18);  // replay gain slot: timestamp / interval depending on the command
  switch (cmd) {
    case 's':
      startStream(d, len);
      break;
    case 'p':  // pause (field = pause duration in ms, 0 = until unpause)
      if (playState == PS_PLAYING) {
        pcm.setPause(true);
        playState = PS_PAUSED;
        resumeAtMs = field ? millis() + field : 0;
      }
      sendStat("STMp");
      break;
    case 'u':  // unpause (field = player clock time at which to start, 0 = now)
      resumeAtMs = 0;
      if (playState == PS_PAUSED) {
        pcm.setPause(false);
        playState = PS_PLAYING;
        sendStat("STMr");
      } else if (playState == PS_WAIT_UNPAUSE) {
        pcm.arm(field);
        playState = PS_PLAYING;
        sendStat("STMr");
        Serial.printf("Start at %lu (now %lu)\n", (unsigned long)field, (unsigned long)millis());
      } else if (playState == PS_BUFFERING) {
        autostartMode = '1';  // start as soon as the threshold is reached
      }
      break;
    case 'q':  // stop
    case 'f':  // flush
      stopAll();
      sendStat("STMf");
      break;
    case 't':  // status request, server timestamp is carried in the replay gain field
      sendStat("STMt", field);
      break;
    case 'a':  // skip ahead (field = ms), used by LMS to correct drift
      if (playState == PS_PLAYING) pcm.skipAheadMs(field);
      break;
    default:
      break;
  }
}

static void dispatch(const uint8_t *d, size_t len) {
  if (len < 4) return;
  if (!memcmp(d, "strm", 4)) {
    onStrm(d, len);
  } else if (!memcmp(d, "audg", 4) && len >= 14) {
    // Digital volume: 16.16 fixed point gain. dvc == 0 means "no volume control".
    float g = 1.0f;
    if (d[12] && len >= 18) g = (float)get32(d + 14) / 65536.0f;
    if (g > 4.0f) g = 4.0f;
    volume = g;
    pcm.setVolume(volume);
  } else if (!memcmp(d, "setd", 4)) {
    if (len == 5 && d[4] == 0) sendSetdName();
  }
  // aude, vers, serv, cont and display commands are ignored
}

// Parse length-prefixed frames from the server: [len:2][opcode:4][data...]
static uint8_t rxBuf[2048];
static size_t rxExpected = 0, rxPos = 0;
static bool rxSkip = false;

static void handleControl() {
  while (ctl.connected() && ctl.available()) {
    if (rxExpected == 0) {
      if (ctl.available() < 2) return;
      uint8_t h[2];
      ctl.read(h, 2);
      rxExpected = get16(h);
      rxPos = 0;
      rxSkip = rxExpected > sizeof(rxBuf);
      if (rxExpected == 0) continue;
    }
    size_t want = rxExpected - rxPos;
    int r;
    if (rxSkip) {
      uint8_t scratch[128];
      r = ctl.read(scratch, minz(want, sizeof(scratch)));
    } else {
      r = ctl.read(rxBuf + rxPos, want);
    }
    if (r <= 0) return;
    rxPos += r;
    if (rxPos == rxExpected) {
      if (!rxSkip) dispatch(rxBuf, rxExpected);
      rxExpected = 0;
    }
  }
}

// ---------------------------------------------------------------- Server discovery / connection
static bool discoverServer() {
  if (strlen(LMS_HOST) > 0) {
    IPAddress ip;
    if (ip.fromString(LMS_HOST) || WiFi.hostByName(LMS_HOST, ip) == 1) {
      serverIP = ip;
      return true;
    }
    return false;
  }
  WiFiUDP udp;
  if (!udp.begin(3484)) return false;
  static const uint8_t req[] = {'e', 'I', 'P', 'A', 'D', 0, 'N', 'A', 'M', 'E', 0, 'J', 'S', 'O', 'N', 0};
  for (int attempt = 0; attempt < 3; attempt++) {
    udp.beginPacket(IPAddress(255, 255, 255, 255), 3483);
    udp.write(req, sizeof(req));
    udp.endPacket();
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      if (udp.parsePacket() > 0) {
        serverIP = udp.remoteIP();
        udp.stop();
        return true;
      }
      delay(10);
    }
  }
  udp.stop();
  return false;
}

static bool connectControl() {
  if (!discoverServer()) {
    Serial.println("LMS server not found");
    return false;
  }
  Serial.printf("Connecting to LMS at %s\n", serverIP.toString().c_str());
  if (!ctl.connect(serverIP, 3483, 3000)) {
    Serial.println("LMS connect failed");
    return false;
  }
  ctl.setNoDelay(true);
  rxExpected = 0;
  rxPos = 0;
  sendHelo();
  lastStatMs = millis();
  Serial.println("Connected to LMS");
  return true;
}

// ---------------------------------------------------------------- Playback tick
static void playbackTick() {
  switch (playState) {
    case PS_BUFFERING:
      if (streamState == ST_BODY && (rbCount >= thresholdBytes || streamEnded)) {
        if (!startDecoder()) break;  // pre-fills the PCM FIFO, output still gated
        if (autostartMode == '1' || autostartMode == '3') {
          pcm.arm(0);
          playState = PS_PLAYING;
        } else {
          sendStat("STMl");  // buffer threshold reached, wait for the server's "strm u"
          playState = PS_WAIT_UNPAUSE;
        }
      }
      break;
    case PS_WAIT_UNPAUSE:
      decodeStep();
      break;
    case PS_PLAYING:
      decodeStep();
      if (!dec) playState = PS_DRAINING;
      break;
    case PS_PAUSED:
      decodeStep();
      if (resumeAtMs && (int32_t)(millis() - resumeAtMs) >= 0) {
        resumeAtMs = 0;
        pcm.setPause(false);
        playState = PS_PLAYING;
      }
      break;
    case PS_DRAINING:
      if (pcm.isDrained()) {
        delay(pcm.dmaMs() + 20);  // let the I2S DMA play out its last buffers
        finishTrack();
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------- Arduino entry points
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("SqueezeTiny starting");

  if (!psramFound()) Serial.println("WARNING: PSRAM not found. Select 'QSPI PSRAM' in the Tools menu.");
  rb = (uint8_t *)ps_malloc(STREAM_BUF_SIZE);
  if (!rb) {
    Serial.println("Failed to allocate the stream buffer");
    while (true) delay(1000);
  }
  if (!pcm.initI2S()) {
    Serial.println("Failed to initialise I2S / PCM FIFO");
    while (true) delay(1000);
  }
  pcm.setVolume(volume);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // WiFi power save causes audio dropouts and timing jitter
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected, IP %s\n", WiFi.localIP().toString().c_str());
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    delay(500);
    return;
  }
  if (!ctl.connected()) {
    stopAll();
    ctl.stop();
    if (!connectControl()) delay(3000);
    return;
  }

  handleControl();
  pumpStream();
  playbackTick();

  // The I2S task raises this flag when the first frame is handed to the DMA
  if (pcm.consumeStarted()) {
    sendStat("STMs");  // track started
    Serial.printf("Playback started at %lu, %ld ms after the target time\n",
                  (unsigned long)millis(), (long)pcm.lastStartLagMs());
  }

  // Periodic status keeps the server informed about the elapsed time
  if (millis() - lastStatMs >= 1000) {
    lastStatMs = millis();
    sendStat("STMt");
  }

#if SQZ_SINGLE_CORE
  delay(1);  // let the idle task and the WiFi stack run (avoids watchdog resets)
#endif
}
