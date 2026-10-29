/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Real-Time Harmonic Distortion (THD) Analyzer
  *                   STM32F411CEU6 (Black Pill), ARM Cortex-M4F @ 96 MHz
  ******************************************************************************
  *  P mode : TIM3 PWM (PA6) --jumper--> ADC1 IN1 (PA1), sampled at 10 kHz
  *           by TIM2 TRGO, moved by DMA2 (circular, ping-pong halves).
  *           PA0 KEY: short press = next load profile (frequency + duty),
  *                    long press (>0.8 s) = toggle FFT / DFT engine.
  *  A mode : samples streamed from the laptop over USB CDC at 16 kHz
  *           (test tone / mic / WAV). Takes over automatically while data
  *           arrives; P mode resumes 0.5 s after it stops.
  *
  *  On-chip DSP per 1024-sample frame:
  *    DC removal -> Hann window -> radix-2 FFT or DFT (FPU) -> magnitude ->
  *    fundamental detection + parabolic interpolation -> THD %
  *  LED PC13 = THD ALARM (on when THD > THD_LIMIT_PCT).
  *  Results streamed to the PC over USB CDC (virtual COM port).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <math.h>

/* Spectrum engine is selectable at run time (long-press KEY):
 *   FFT : radix-2 Cooley-Tukey, O(N log N)  (default)
 *   DFT : direct summation,     O(N^2)      (for speed comparison)    */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_FFT          1024U
#define ADC_FS_HZ      10000U        /* TIM2 trigger rate              */
#define AUDIO_FS_HZ    16000U        /* rate the PC script sends at    */
#define TIM_TICK_HZ    1000000UL     /* TIM3 tick after PSC=95 @96 MHz */
#define TWO_PI_F       6.28318530718f
#define THD_LIMIT_PCT  60.0f         /* demo alarm threshold           */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* ADC (PWM) path */
static uint16_t adc_buf[2 * N_FFT];
static volatile int8_t ready_half = -1;

/* USB audio path (filled from the USB interrupt) */
static int16_t usb_frames[2][N_FFT];
static volatile uint8_t  usb_wr_frame = 0;
static volatile uint32_t usb_wr_idx = 0;
static volatile int8_t   usb_frame_ready = -1;
static volatile uint32_t last_usb_rx_ms = 0;
static uint8_t pend_lo = 0, have_pend = 0;

/* DSP buffers */
static float in_buf[N_FFT];
static int16_t raw_buf[N_FFT];              /* untouched copy of the frame (for display) */
static float mag[N_FFT / 2];
static float hann[N_FFT];
static float tw_cos[N_FFT], tw_sin[N_FFT];  /* twiddle factors e^(-j2*pi*k/N) */
static float fft_im[N_FFT];                 /* imaginary part for the FFT      */
static uint16_t bitrev[N_FFT];              /* bit-reversal permutation        */
static volatile uint8_t use_fft = 1;        /* 1 = FFT, 0 = DFT                */

/* Load profiles selected by the PA0 button.
 * Theoretical THD of an ideal rectangular wave:
 *   50 % duty ~ 48 %,  25 % duty ~ 92 %,  10 % duty ~ 191 %          */
typedef struct { uint32_t freq_hz; uint32_t duty_pct; } profile_t;
static const profile_t profiles[] = {
  {   50, 50 },   /* square-wave inverter output        */
  {   50, 25 },   /* phase-chopped load                 */
  {   50, 10 },   /* pulsed / rectifier-type load       */
  { 1000, 50 },   /* higher-frequency switching signal  */
};
#define N_PROFILES (sizeof(profiles) / sizeof(profiles[0]))
static uint8_t p_idx = 0;

static char tx_buf[(N_FFT / 2) * 11 + N_FFT * 7 + 96];  /* header + 512 mags (max 10 digits) + 1024 samples */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Called from CDC_Receive_FS (USB interrupt): raw int16 little-endian */
void audio_rx_bytes(const uint8_t *buf, uint32_t len)
{
  last_usb_rx_ms = HAL_GetTick();
  for (uint32_t i = 0; i < len; i++)
  {
    if (!have_pend) { pend_lo = buf[i]; have_pend = 1; continue; }
    have_pend = 0;
    int16_t s = (int16_t)((uint16_t)pend_lo | ((uint16_t)buf[i] << 8));
    usb_frames[usb_wr_frame][usb_wr_idx++] = s;
    if (usb_wr_idx >= N_FFT)
    {
      usb_frame_ready = (int8_t)usb_wr_frame;
      usb_wr_frame ^= 1U;
      usb_wr_idx = 0;
    }
  }
}

static void set_pwm(uint32_t freq_hz, uint32_t duty_pct)
{
  uint32_t arr = TIM_TICK_HZ / freq_hz - 1U;          /* 50 Hz -> 19999 */
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ((arr + 1U) * duty_pct) / 100U);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
}

static char *put_uint(char *p, uint32_t v)
{
  char tmp[11];
  int i = 0;
  do { tmp[i++] = (char)('0' + v % 10U); v /= 10U; } while (v);
  while (i) *p++ = tmp[--i];
  return p;
}

static char *put_int(char *p, int32_t v)
{
  if (v < 0) { *p++ = '-'; v = -v; }
  return put_uint(p, (uint32_t)v);
}

static int usb_ready(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) &&
         (hUsbDeviceFS.pClassData != NULL);
}

static int usb_wait_idle(uint32_t timeout_ms)
{
  if (!usb_ready()) return 0;
  USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  uint32_t t0 = HAL_GetTick();
  while (h->TxState != 0U && (HAL_GetTick() - t0) < timeout_ms) { }
  return (h->TxState == 0U);
}

static void usb_send(char *buf, uint16_t len)
{
  if (!usb_ready()) return;
  uint32_t t0 = HAL_GetTick();
  while (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_BUSY)
  {
    if ((HAL_GetTick() - t0) > 50U) return;
  }
}

static void dsp_init(void)
{
  for (uint32_t n = 0; n < N_FFT; n++)
    hann[n] = 0.5f * (1.0f - cosf(TWO_PI_F * (float)n / (float)(N_FFT - 1U)));
  for (uint32_t n = 0; n < N_FFT; n++)
  {
    tw_cos[n] = cosf(TWO_PI_F * (float)n / (float)N_FFT);
    tw_sin[n] = sinf(TWO_PI_F * (float)n / (float)N_FFT);
  }
  /* bit-reversal table for log2(N_FFT) = 10 bits */
  uint32_t bits = 0;
  while ((1UL << bits) < N_FFT) bits++;
  for (uint32_t i = 0; i < N_FFT; i++)
  {
    uint32_t r = 0;
    for (uint32_t b = 0; b < bits; b++)
      if (i & (1UL << b)) r |= 1UL << (bits - 1U - b);
    bitrev[i] = (uint16_t)r;
  }
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* DWT cycle counter */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* In-place iterative radix-2 decimation-in-time FFT (re[] real, im[] imag) */
static void fft_radix2(float *re, float *im)
{
  for (uint32_t i = 0; i < N_FFT; i++)            /* bit-reverse reorder */
  {
    uint32_t j = bitrev[i];
    if (j > i)
    {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (uint32_t len = 2; len <= N_FFT; len <<= 1) /* log2(N) stages */
  {
    uint32_t half = len >> 1;
    uint32_t step = N_FFT / len;
    for (uint32_t i = 0; i < N_FFT; i += len)
    {
      for (uint32_t j = 0; j < half; j++)          /* butterflies */
      {
        float wr =  tw_cos[j * step];
        float wi = -tw_sin[j * step];
        uint32_t a = i + j, b = a + half;
        float tr = re[b] * wr - im[b] * wi;
        float ti = re[b] * wi + im[b] * wr;
        re[b] = re[a] - tr;  im[b] = im[a] - ti;
        re[a] += tr;         im[a] += ti;
      }
    }
  }
}

/* Energy in +/-2 bins around bin kc (Hann main lobe) */
static float band_power(int32_t kc)
{
  float p = 0.0f;
  for (int32_t k = kc - 2; k <= kc + 2; k++)
    if (k >= 1 && k < (int32_t)(N_FFT / 2)) p += mag[k] * mag[k];
  return p;
}

/* in_buf[] must hold N_FFT raw samples (float).
 * Returns fundamental frequency (Hz); writes transform cycles and THD (%). */
static float analyze(uint32_t fs, uint32_t *cycles, float *thd_pct)
{
  /* 1. remove DC, apply Hann window */
  float mean = 0.0f;
  for (uint32_t n = 0; n < N_FFT; n++) mean += in_buf[n];
  mean /= (float)N_FFT;
  for (uint32_t n = 0; n < N_FFT; n++)
    in_buf[n] = (in_buf[n] - mean) * hann[n];

  /* 2. spectrum, timed with the DWT cycle counter */
  uint32_t c0 = DWT->CYCCNT;
  if (use_fft)
  {
    for (uint32_t n = 0; n < N_FFT; n++) fft_im[n] = 0.0f;
    fft_radix2(in_buf, fft_im);
    for (uint32_t k = 0; k < N_FFT / 2; k++)
      mag[k] = sqrtf(in_buf[k] * in_buf[k] + fft_im[k] * fft_im[k]);
  }
  else
  {
    for (uint32_t k = 0; k < N_FFT / 2; k++)
    {
      float re = 0.0f, im = 0.0f;
      uint32_t idx = 0;
      for (uint32_t n = 0; n < N_FFT; n++)
      {
        re += in_buf[n] * tw_cos[idx];
        im -= in_buf[n] * tw_sin[idx];
        idx = (idx + k) & (N_FFT - 1U);
      }
      mag[k] = sqrtf(re * re + im * im);
    }
  }
  *cycles = DWT->CYCCNT - c0;
  mag[0] = 0.0f;                      /* DC                         */
  mag[1] = 0.0f;                      /* residual DC leakage (Hann) */

  /* 3. fundamental: global max is the fallback; prefer a lower strong
   *    local peak (>= 50 % of max) so a strong 2nd harmonic is not
   *    mistaken for the fundamental                                   */
  uint32_t kmax = 2;
  for (uint32_t k = 3; k < N_FFT / 2 - 1; k++)
    if (mag[k] > mag[kmax]) kmax = k;
  float mmax = mag[kmax];

  uint32_t k0 = kmax;
  for (uint32_t k = 2; k < kmax; k++)
  {
    if (mag[k] >= 0.5f * mmax && mag[k] >= mag[k - 1] && mag[k] >= mag[k + 1])
    { k0 = k; break; }
  }

  /* parabolic interpolation, offset clamped to +/-0.5 bin */
  float a = mag[k0 - 1], b = mag[k0], c = mag[k0 + 1];
  float denom = a - 2.0f * b + c;
  float delta = (denom < 0.0f) ? 0.5f * (a - c) / denom : 0.0f;
  if (delta >  0.5f) delta =  0.5f;
  if (delta < -0.5f) delta = -0.5f;
  float df = (float)fs / (float)N_FFT;
  float f0 = ((float)k0 + delta) * df;   /* k0 >= 2, so f0 >= 1.5 bins */

  /* 4. THD = sqrt(sum of harmonic powers) / fundamental, up to Nyquist */
  *thd_pct = 0.0f;
  if (mmax < 1.0f) return f0;            /* no signal: nothing to measure */
  float p1 = band_power((int32_t)(f0 / df + 0.5f));
  float ph = 0.0f;
  for (uint32_t h = 2; h <= 200U && (float)h * f0 < (float)fs * 0.5f - 3.0f * df; h++)
    ph += band_power((int32_t)((float)h * f0 / df + 0.5f));
  *thd_pct = (p1 > 0.0f) ? 100.0f * sqrtf(ph / p1) : 0.0f;

  return f0;
}

/* F,<src>,<fs>,<set_hz>,<duty>,<f0*10>,<thd*10>,<cycles>,<alg>,<512 magnitudes>,<1024 raw samples>\n
 * alg: 1 = FFT, 0 = DFT                                                        */
static void send_frame(char src, uint32_t fs, uint32_t set_hz, uint32_t duty,
                       float f0, float thd, uint32_t cycles)
{
  if (!usb_wait_idle(20)) return;    /* PC still reading the last frame: skip this one, never overwrite tx_buf */
  char *p = tx_buf;
  *p++ = 'F'; *p++ = ','; *p++ = src; *p++ = ',';
  p = put_uint(p, fs);                         *p++ = ',';
  p = put_uint(p, set_hz);                     *p++ = ',';
  p = put_uint(p, duty);                       *p++ = ',';
  p = put_uint(p, (uint32_t)(f0 * 10.0f));     *p++ = ',';
  p = put_uint(p, (uint32_t)(thd * 10.0f));    *p++ = ',';
  p = put_uint(p, cycles);                     *p++ = ',';
  *p++ = use_fft ? '1' : '0';
  for (uint32_t k = 0; k < N_FFT / 2; k++)
  {
    *p++ = ',';
    p = put_uint(p, (uint32_t)mag[k]);
  }
  for (uint32_t n = 0; n < N_FFT; n++)         /* original time-domain signal */
  {
    *p++ = ',';
    p = put_int(p, raw_buf[n]);
  }
  *p++ = '\n';
  usb_send(tx_buf, (uint16_t)(p - tx_buf));
}

static void set_alarm_led(float thd)
{
  /* PC13 LED is active-low: RESET = ON */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                    (thd > THD_LIMIT_PCT) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  dsp_init();
  set_pwm(profiles[p_idx].freq_hz, profiles[p_idx].duty_pct);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 2 * N_FFT);
  HAL_TIM_Base_Start(&htim2);        /* starts 10 kHz ADC triggering */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* PA0 button (active low):
     *   short press (< 0.8 s) -> next load profile
     *   long press (>= 0.8 s) -> toggle FFT / DFT engine                 */
    static uint8_t  btn_last = 1;
    static uint32_t btn_down_t = 0;
    uint8_t btn_now = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    if (btn_last == 1U && btn_now == 0U)
    {
      btn_down_t = HAL_GetTick();                      /* pressed  */
    }
    else if (btn_last == 0U && btn_now == 1U)
    {
      uint32_t held = HAL_GetTick() - btn_down_t;      /* released */
      if (held >= 800U)
      {
        use_fft ^= 1U;
      }
      else if (held >= 30U)                            /* debounce */
      {
        p_idx = (uint8_t)((p_idx + 1U) % N_PROFILES);
        set_pwm(profiles[p_idx].freq_hz, profiles[p_idx].duty_pct);
      }
    }
    btn_last = btn_now;

    uint8_t audio_mode = (last_usb_rx_ms != 0U) &&
                         ((HAL_GetTick() - last_usb_rx_ms) < 500U);
    uint32_t cycles;
    float thd;

    /* A mode: one frame of samples received over USB */
    if (usb_frame_ready >= 0)
    {
      int8_t f = usb_frame_ready;
      usb_frame_ready = -1;
      for (uint32_t n = 0; n < N_FFT; n++)
      {
        raw_buf[n] = usb_frames[(uint8_t)f][n];
        in_buf[n] = (float)raw_buf[n];
      }
      float f0 = analyze(AUDIO_FS_HZ, &cycles, &thd);
      set_alarm_led(thd);
      send_frame('A', AUDIO_FS_HZ, 0, 0, f0, thd, cycles);
    }

    /* P mode: one half-buffer of ADC samples from DMA */
    if (ready_half >= 0)
    {
      int8_t h = ready_half;
      ready_half = -1;
      if (!audio_mode)
      {
        const uint16_t *src = &adc_buf[(uint32_t)h * N_FFT];
        for (uint32_t n = 0; n < N_FFT; n++)
        {
          raw_buf[n] = (int16_t)src[n];
          in_buf[n] = (float)src[n];
        }
        float f0 = analyze(ADC_FS_HZ, &cycles, &thd);
        set_alarm_led(thd);
        send_frame('P', ADC_FS_HZ, profiles[p_idx].freq_hz,
                   profiles[p_idx].duty_pct, f0, thd, cycles);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSE 25 MHz -> PLL (M=25, N=192, P=2, Q=4)
  *        SYSCLK = HCLK = 96 MHz, APB1 = 48 MHz, APB2 = 96 MHz, USB = 48 MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  *        12-bit, single channel IN1 (PA1), triggered by TIM2 TRGO (10 kHz),
  *        results moved by DMA in circular mode.
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 96/4 = 24 MHz (max 36 MHz) */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  *        96 MHz / (95+1) / (99+1) = 10 kHz update -> TRGO -> ADC trigger
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  *        PWM on CH1 (PA6); period and duty are set at runtime by set_pwm()
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
	__HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  *        PC13 = LED (output, active low), PA0 = KEY (input, pull-up)
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) ready_half = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) ready_half = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
