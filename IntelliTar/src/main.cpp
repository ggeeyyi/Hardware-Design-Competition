// ————————————————————————————————————————————————头文件————————————————————————————————————————————————//
#include <intelliTar.h>
#include "gui_guider.h"
#include "events_init.h"
#include "custom.h"
#include <vector>
#include <uklScore.h>
#include <testscore.cpp>
#include <arduinoFFT.h>
// ————————————————————————————————————————————————变量定义————————————————————————————————————————————————//
//  定义分辨率
static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;

// 定义屏幕缓冲
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

// 旋转编码器输入
int lastpos = 0;
int time0 = 0;
int time1 = 0;
int click = 0;
int posState = 0;

// 节拍器变量
int MetroPointerAngle = 100; // 节拍器角度
int bpm;
int beepPin = 13;    // 蜂鸣器引脚
int switchState = 0; // 开关状态

// 调音器变量
int frequency = 0;
int TunerPointerAngle = 100; // 调音器角度
int targetFrequency = 255;   // 目标频率(默认中央C)
int currentFrequency = 90;   // 当前频率
const int bufferSize = 1024;
const float cmndfThreshold = 0.3;
int sampleRate = 11000; // 采样率
int buffer[bufferSize];
float df[bufferSize / 2];    // 差函数的数组
float cmndf[bufferSize / 2]; // 累积平方差函数的数组
int tauEstimate = 0;
float pitchEstimate = 0;

// 乐谱展示器变量
int pauseState = 0;        // 暂停状态
int continueState = 0;     // 继续状态
int displayerState = 0;    // 展示器状态
int pagehome_ddlist_index; // 乐谱选择
int noteIndex = 0;         // 音符索引
int lastNoteIndex = 0;     // 上一个音符索引
int displayerMode = 0;     // 展示器模式
int displayerbpm = 80;     // 展示器节拍
int displayert;            // 展示器间隔时间
int displayGapTime = 0;    // 展示器自动模式间隔时间
int LEDGapTime = 0;        // LED自动模式间隔时间
int LEDState = 0;          // LED状态(0:关闭 1:演奏引导模式 2:FFT模式)
u_int8_t testnote[8][5][5] = {
    {{0, 0, 0, 0, 0},
     {0, 255, 255, 1, 0},
     {0, 1, 255, 255, 1},
     {0, 255, 255, 0, 1},
     {0, 255, 255, 255, 1}},
    {{255, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {0, 0, 1, 1, 0}},
    {{1, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {1, 0, 1, 1, 1}},
    {{2, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {1, 1, 0, 0, 1}},
    {{3, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {1, 1, 1, 1, 1}},
    {{4, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {0, 1, 1, 1, 0}},
    {{5, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {0, 1, 1, 1, 0}},
    {{6, 0, 0, 0, 0},
     {0, 0, 1, 1, 0},
     {0, 1, 0, 0, 1},
     {0, 0, 0, 0, 0},
     {0, 1, 1, 1, 0}}};
std::vector<section<4>> testSec;
int totalIndex = 28;
UklScore test; // 测试乐谱1

// arduino FFT变量
arduinoFFT FFT = arduinoFFT();
const uint16_t samples = 1024;    // This value MUST ALWAYS be a power of 2
double samplingFrequency = 10000; // Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];
double magnitudeSpectrum[samples];
double noteFrequencies[12];
double chromagram[12];
double lastChromagram[12];
double normalizedChromagram[12];
double targetChromagram[12];
double referenceFrequency = 262.8;

// —————————————————————————————————————————————————函数定义——————————————————————————————————————————————————//
void FFTsetup()
{
    for (int i = 0; i < 12; i++)
    {
        noteFrequencies[i] = referenceFrequency * pow(2, (((float)i) / 12));
    }
}

void FFTloop()
{
    if (analogRead(MIC) > 1500)
    {
        int time0 = millis();
        /*SAMPLING*/
        for (int i = 0; i < samples; i++)
        {
            vReal[i] = analogRead(MIC);
            vImag[i] = 0;
        }

        samplingFrequency = samples * 1000 / (millis() - time0);

        FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */

        FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */

        for (int i = 0; i < samples; i++)
        {
            magnitudeSpectrum[i] = sqrt(pow(vReal[i], 2) + pow(vImag[i], 2));
            magnitudeSpectrum[i] = sqrt(magnitudeSpectrum[i]);
            // Serial.println(magnitudeSpectrum[i]);
        }

        double x = FFT.MajorPeak(vReal, samples, samplingFrequency);

        double divisorRatio = (((double)samplingFrequency)) / ((double)samples);
        for(int i=0;i<12;i++)
        {
            lastChromagram[i] = chromagram[i];
        }

        for (int n = 0; n < 12; n++)
        {
            double chromaSum = 0.0;

            for (int octave = 1; octave <= 2; octave++)
            {
                double noteSum = 0.0;

                for (int harmonic = 1; harmonic <= 2; harmonic++)
                {
                    int centerBin = round((noteFrequencies[n] * octave * harmonic) / divisorRatio);
                    int minBin = centerBin - (octave * harmonic);
                    int maxBin = centerBin + (octave * harmonic);
                    if (centerBin > 361)
                    {
                        centerBin = round((noteFrequencies[n] * octave * harmonic) / divisorRatio * 4);
                        minBin = centerBin - (octave * harmonic) * 4;
                        maxBin = centerBin + (octave * harmonic) * 4;
                    }
                    double maxVal = 0.0;

                    for (int k = minBin; k < maxBin; k++)
                    {
                        if (magnitudeSpectrum[k] > maxVal)
                        {
                            maxVal = magnitudeSpectrum[k];
                        }
                    }

                    noteSum += (maxVal / (double)harmonic);
                }

                chromaSum += noteSum;
            }

            chromagram[n] = chromaSum;
        }

        double minValue = 1000;
        int minIndex = 0;
        for (int n = 0; n < 12; n++)
        {
            if (chromagram[n] < minValue)
            {
                minValue = chromagram[n];
                minIndex = n;
            }
        }
        for (int i = 0; i < 12; i++)
        {
            normalizedChromagram[i] = chromagram[i] / minValue;
            if (normalizedChromagram[i] > 4)
                normalizedChromagram[i] = 1;
            else
                normalizedChromagram[i] = 0;
            // Serial.print(normalizedChromagram[i]);
            // Serial.print(" ");
        }
        Serial.println();
    }
}
void generateLEDDisplay(int noteIndex)
{
    LEDclear();
    LEDStateSetAll(0);
    if (testnote[noteIndex / 4][noteIndex % 4 + 1][1] != 255)
        LEDStateSet(testnote[noteIndex / 4][noteIndex % 4 + 1][1] + 1, 1, true);
    if (testnote[noteIndex / 4][noteIndex % 4 + 1][2] != 255)
        LEDStateSet(testnote[noteIndex / 4][noteIndex % 4 + 1][2] + 1, 2, true);
    if (testnote[noteIndex / 4][noteIndex % 4 + 1][3] != 255)
        LEDStateSet(testnote[noteIndex / 4][noteIndex % 4 + 1][3] + 1, 3, true);
    if (testnote[noteIndex / 4][noteIndex % 4 + 1][4] != 255)
        LEDStateSet(testnote[noteIndex / 4][noteIndex % 4 + 1][4] + 1, 4, true);
}
void generateLEDFFTDisplay()
{
    LEDclear();
    LEDStateSetAll(0);
    double mag[13];
    for (int n = 0; n < 13; n++)
    {
        for (int i = 0; i < 78; i++)
        {
            mag[n] += magnitudeSpectrum[i + n * 78];
        }
    }
    int minMag = 10;
    int maxMag = 0;
    for (int i = 1; i < 13; i++)
    {
        if (mag[i] < minMag)
        {
            minMag = mag[i];
        }
        if (mag[i] > maxMag)
        {
            maxMag = mag[i];
        }
    }
    for (int i = 1; i < 13; i++)
    {
        mag[i] = 4 * (mag[i] - minMag) / (maxMag - minMag);
    }
    int LEDMag[12];
    for(int i = 0; i < 12; i++)
    {
        LEDMag[i] = mag[i + 1];
    }
    for (int i = 1; i < 12; i++)
    {
        Serial.print(LEDMag[i]);
        Serial.print(" ");
    }
}
void generateLEDChromagramDisplay()
{
    LEDclear();
    LEDStateSetAll(0);
    for (int i = 0; i < 12; i++)
    {
        Serial.print(chromagram[i]);
        Serial.print(" ");
    }
    for(int i = 0; i < 12; i++)
    {
        if(chromagram[i] > 400)
        {
            LEDStateSet(i + 1, 1, true);
            LEDStateSet(i + 1, 2, true);
            LEDStateSet(i + 1, 3, true);
            LEDStateSet(i + 1, 4, true);
        }
        else if(chromagram[i] > 300&&(chromagram[i] -lastChromagram[i]>=50))
        {
            LEDStateSet(i + 1, 1, true);
            LEDStateSet(i + 1, 2, true);
            LEDStateSet(i + 1, 3, true);
        }
        else if(chromagram[i] > 200&&(chromagram[i] -lastChromagram[i]>=20))
        {
            LEDStateSet(i + 1, 1, true);
            LEDStateSet(i + 1, 2, true);
        }
        else if(chromagram[i] > 100)
        {
            LEDStateSet(i + 1, 1, true);
        }
    }
}
void generateTargetChromagram(int noteIndex)
{
    for (int i = 0; i < 5; i++)
    {
        Serial.print(testnote[noteIndex / 4][noteIndex % 4 + 1][i]);
        if (i == 1)
        {
            switch (testnote[noteIndex / 4][noteIndex % 4 + 1][i])
            {
            case 0:
                targetChromagram[7] = 1;
                break;
            case 1:
                targetChromagram[8] = 1;
                break;
            case 2:
                targetChromagram[9] = 1;
                break;
            case 3:
                targetChromagram[10] = 1;
                break;
            case 4:
                targetChromagram[11] = 1;
                break;
            case 5:
                targetChromagram[0] = 1;
                break;
            case 6:
                targetChromagram[1] = 1;
                break;
            case 7:
                targetChromagram[2] = 1;
                break;
            case 8:
                targetChromagram[3] = 1;
                break;
            case 9:
                targetChromagram[4] = 1;
                break;
            case 10:
                targetChromagram[5] = 1;
                break;
            case 11:
                targetChromagram[6] = 1;
                break;
            default:
                break;
            }
        }
        if (i == 2)
        {
            switch (testnote[noteIndex / 4][noteIndex % 4 + 1][i])
            {
            case 0:
                targetChromagram[0] = 1;
                break;
            case 1:
                targetChromagram[1] = 1;
                break;
            case 2:
                targetChromagram[2] = 1;
                break;
            case 3:
                targetChromagram[3] = 1;
                break;
            case 4:
                targetChromagram[4] = 1;
                break;
            case 5:
                targetChromagram[5] = 1;
                break;
            case 6:
                targetChromagram[6] = 1;
                break;
            case 7:
                targetChromagram[7] = 1;
                break;
            case 8:
                targetChromagram[8] = 1;
                break;
            case 9:
                targetChromagram[9] = 1;
                break;
            case 10:
                targetChromagram[10] = 1;
                break;
            case 11:
                targetChromagram[11] = 1;
                break;
            default:
                break;
            }
        }
        if (i == 3)
        {
            switch (testnote[noteIndex / 4][noteIndex % 4 + 1][i])
            {
            case 0:
                targetChromagram[4] = 1;
                break;
            case 1:
                targetChromagram[5] = 1;
                break;
            case 2:
                targetChromagram[6] = 1;
                break;
            case 3:
                targetChromagram[7] = 1;
                break;
            case 4:
                targetChromagram[8] = 1;
                break;
            case 5:
                targetChromagram[9] = 1;
                break;
            case 6:
                targetChromagram[10] = 1;
                break;
            case 7:
                targetChromagram[11] = 1;
                break;
            case 8:
                targetChromagram[0] = 1;
                break;
            case 9:
                targetChromagram[1] = 1;
                break;
            case 10:
                targetChromagram[2] = 1;
                break;
            case 11:
                targetChromagram[3] = 1;
                break;
            default:
                break;
            }
        }
        if (i == 4)
        {
            switch (testnote[noteIndex / 4][noteIndex % 4 + 1][i])
            {
            case 0:
                targetChromagram[9] = 1;
                break;
            case 1:
                targetChromagram[10] = 1;
                break;
            case 2:
                targetChromagram[11] = 1;
                break;
            case 3:
                targetChromagram[1] = 1;
                break;
            case 4:
                targetChromagram[2] = 1;
                break;
            case 5:
                targetChromagram[3] = 1;
                break;
            case 6:
                targetChromagram[4] = 1;
                break;
            case 7:
                targetChromagram[5] = 1;
                break;
            case 8:
                targetChromagram[6] = 1;
                break;
            case 9:
                targetChromagram[7] = 1;
                break;
            case 10:
                targetChromagram[8] = 1;
                break;
            case 11:
                targetChromagram[9] = 1;
                break;
            default:
                break;
            }
        }
    }
}

#if LV_USE_LOG != 0
/* Serial debugging  串口调试用*/
void my_print(const char *buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing 显示填充 与LCD驱动关联*/
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
/*输入设备，读取触摸板*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;

    bool touched = tft.getTouch(&touchX, &touchY, 600);

    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
    }
}
float yinAlgorithm()
{
    // 计算差函数
    for (int tau = 0; tau < bufferSize / 2; tau++)
    {
        df[tau] = 0; // 初始化差函数
        for (int i = 0; i < bufferSize / 2; i++)
        {
            int index1 = i;
            int index2 = i + tau;
            df[tau] += (buffer[index1] - buffer[index2]) * (buffer[index1] - buffer[index2]);
        }
    }
    // 计算累积平方差函数
    cmndf[0] = df[0];
    for (int tau = 1; tau < bufferSize / 2; tau++)
    {
        float cm = 0;
        for (int j = 1; j < tau; j++)
        {
            cm += df[j] / tau;
        }
        cmndf[tau] = df[tau] / cm;
    }

    int taumax = 0;

    for (int i = 2; i < bufferSize / 2; i++)
    {
        if (cmndf[i] < cmndfThreshold && cmndf[i] < cmndf[i - 1] && cmndf[i] < cmndf[i + 1])
        {
            taumax = i;
            break;
        }
    }

    if (taumax == 0)
    {
        Serial.println("No pitch detected");
        return 0;
    }

    // 插值计算精确的周期
    if (taumax != bufferSize / 2 && taumax)
    {
        float delta = df[taumax + 1] - df[taumax - 1];
        pitchEstimate = sampleRate / (taumax + (delta / (2 * delta - df[taumax])));
    }
    else
    {
        pitchEstimate = 0;
    }

    return pitchEstimate;
}
// 控制蜂鸣器发声

void tft_init()
{

    tft.begin();        /* TFT init TFT初始化*/
    tft.setRotation(1); /* Landscape orientation, flipped 设置方向*/

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    /*设置触摸屏校准数据，
      可以使用获取显示的实际数据
      TFT_eSPI 库中的 Generic -> Touch_calibrate 示例*/
    uint16_t calData[5] = {187, 3596, 387, 3256, 5};
    tft.setTouch(calData);
}

void lvgl_setup()
{
    lv_init();
    // lv_port_disp_init();  /*Initialize the display 初始化显示*/
    // lv_port_indev_init(); /*Initialize the input devices 初始化输入设备*/
#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging 注册打印功能以进行调试*/
#endif

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

    /*Initialize the display*/
    /*初始化显示*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    /*将以下行更改为您的显示分辨率*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    /*初始化（虚拟）输入设备驱动程序*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
}

lv_ui guider_ui; // 结构体包含所有屏幕与部件，必不可少，且不能放到setup里 。使用该指针可以找到程序里任何对象

void lvglSetUp()
{
    lv_init();
#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

    tft.begin();        /* TFT init */
    tft.setRotation(3); /* Landscape orientation, flipped */

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    uint16_t calData[5] = {275, 3620, 264, 3532, 1};
    tft.setTouch(calData);

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    /*Create a GUI-Guider app */
    setup_ui(&guider_ui);
    events_init(&guider_ui); // 事件初始化
    custom_init(&guider_ui); // 运行自定义代码，例如将实际输出跟gui显示的数值关联起来
}
void musicSheetPresenterInit()
{
    for (int i = 0; i < 8; i++)
    {
        section<4> s;
        for (int j = 0; j < 5; j++)
        {
            for (int k = 0; k < 5; k++)
            {
                s.musicScore[j][k] = testnote[i][j][k];
            }
        }
        testSec.push_back(s);
    }
    test.UklScoreSetUp(4, testSec, &tft);
}
void lv_port_indev_init(void)
{
    // 初识化编码器
    my_encoder_init();
    // 注册输入设备
    static lv_indev_drv_t indev_drv2;
    lv_indev_drv_init(&indev_drv2);
    indev_drv2.type = LV_INDEV_TYPE_ENCODER;
    indev_drv2.read_cb = my_encoder_read;
    lv_indev_drv_register(&indev_drv2);
    lv_group_t *group = lv_group_create();
    lv_group_set_default(group);
    lv_group_remove_all_objs(group);
    lv_group_add_obj(group, guider_ui.screen_btn_1);
    lv_group_add_obj(group, guider_ui.screen_btn_2);
    lv_group_add_obj(group, guider_ui.screen_btn_3);
    lv_indev_set_group(lv_indev_drv_register(&indev_drv2), group);
}
void encoderSetup()
{
    lv_port_indev_init();
}

void setup()
{
    intelliTarSetup();
    lvglSetUp();
    pinMode(beepPin, OUTPUT);
    encoderSetup();
    musicSheetPresenterInit();
    FFTsetup();
    generateTargetChromagram(0);
    // lv_obj_clear_flag(guider_ui.Tunerhome, LV_OBJ_FLAG_SCROLLABLE);
}

void loop()
{

    lv_timer_handler(); /* let the GUI do its work 让 GUI 完成它的工作 */
    FFTloop();
    int temp = keyDown();
    if (temp != 0)
    {
        Serial.println(temp);
        click = temp;
    }
    if (pos != lastpos)
    {
        Serial.println(pos);
        lastpos = pos;
    }

    // 控制屏幕显示
    if (lv_scr_act() == guider_ui.Metronomehome && switchState == 1) // 节拍器页面
    {
    }

    else if (lv_scr_act() == guider_ui.Tunerhome) // 手动调音器页面(默认)
    {
        if (analogRead(MIC) > 1500)
        {
            pitchEstimate = 0;
            int time0 = millis();
            for (int i = 0; i < bufferSize; i++)
            {
                buffer[i] = analogRead(MIC);
            }
            sampleRate = bufferSize * 1000 / (millis() - time0);
            pitchEstimate = yinAlgorithm(); // yin算法计算频率
            if (pitchEstimate > 200 && pitchEstimate < 500)
            {
                currentFrequency = pitchEstimate;
            }
        }
        lv_meter_set_indicator_value(guider_ui.Tunerhome_meter_1, guider_ui.Tunerhome_meter_1_scale_1_ndline_0, currentFrequency - targetFrequency);
    }
    if (lv_scr_act() == guider_ui.TurnerAutoHome) // 自动调音器页面
    {
        if (analogRead(MIC) > 1400)
        {
            pitchEstimate = 0;
            int time0 = millis();
            for (int i = 0; i < bufferSize; i++)
            {
                buffer[i] = analogRead(MIC);
            }
            sampleRate = bufferSize * 1000 / (millis() - time0);
            pitchEstimate = yinAlgorithm();
            Serial.println("Estimated Pitch: ");

            Serial.println(pitchEstimate);
            Serial.println(sampleRate);
            // 主循环代码（用于更新音高显示和指针位置）
            if (pitchEstimate > 200 && pitchEstimate < 500)
            {
                currentFrequency = pitchEstimate;
            }
        }
        if (currentFrequency > 220 && currentFrequency < 293)
        {
            targetFrequency = 255;
            lv_label_set_text(guider_ui.TurnerAutoHome_label_3, "Now:\nC\n");
            lv_meter_set_indicator_value(guider_ui.TurnerAutoHome_meter_1, guider_ui.TurnerAutoHome_meter_1_scale_1_ndline_0, currentFrequency - targetFrequency);
        }
        else if (currentFrequency > 293 && currentFrequency < 369)
        {
            targetFrequency = 322;
            lv_label_set_text(guider_ui.TurnerAutoHome_label_3, "Now:\nE\n");
            lv_meter_set_indicator_value(guider_ui.TurnerAutoHome_meter_1, guider_ui.TurnerAutoHome_meter_1_scale_1_ndline_0, currentFrequency - targetFrequency);
        }
        else if (currentFrequency > 369 && currentFrequency < 415)
        {
            targetFrequency = 392;
            lv_label_set_text(guider_ui.TurnerAutoHome_label_3, "Now:\nG\n");
            lv_meter_set_indicator_value(guider_ui.TurnerAutoHome_meter_1, guider_ui.TurnerAutoHome_meter_1_scale_1_ndline_0, currentFrequency - targetFrequency);
        }
        else if (currentFrequency > 415 && currentFrequency < 493)
        {
            targetFrequency = 440;
            lv_label_set_text(guider_ui.TurnerAutoHome_label_3, "Now:\nA\n");
            lv_meter_set_indicator_value(guider_ui.TurnerAutoHome_meter_1, guider_ui.TurnerAutoHome_meter_1_scale_1_ndline_0, currentFrequency - targetFrequency);
        }
        else
        {
            lv_label_set_text(guider_ui.TurnerAutoHome_label_3, "Not\nrecognized\n");
        }
    }
    if (lv_scr_act() == guider_ui.Pagehome)
    {
        pagehome_ddlist_index = lv_dropdown_get_selected(guider_ui.Pagehome_ddlist_1);
        // Serial.println(pagehome_ddlist_index);
    }
    if (lv_scr_act() == guider_ui.Musicsheetpresenter)
    {
        if (pagehome_ddlist_index == 0) // 乐谱演示list1
        {
        }
        if (pagehome_ddlist_index == 1) // 乐谱演示list2
        {
            test.drawMusic();
            if (displayerState == 1) // 乐谱演示list2的开关
            {

                if (displayerMode == 0) // AUTO模式（自动播放）
                {
                    int displayerTime = 60 * 1000 / displayerbpm;
                    test.drawMusic();
                    if (millis() - displayert > displayerTime)
                    {
                        test.forwardPos();
                        tft.fillRect(0, 60, 480, 260, TFT_WHITE);
                        displayert = millis();
                        lastNoteIndex = noteIndex;
                        noteIndex++;
                        if (noteIndex >= totalIndex)
                        {
                            noteIndex = 0;
                        }
                    }
                }
                if (displayerMode == 1) // TRIG模式（手动播放）
                {
                    int sameCnt = 0;
                    int notZeroCnt = 0;
                    for (int i = 0; i < 12; i++)
                    {
                        if (normalizedChromagram[i] == targetChromagram[i] && targetChromagram[i] != 0)
                        {
                            sameCnt++;
                        }
                        if (targetChromagram[i] != 0)
                        {
                            notZeroCnt++;
                        }
                    }
                    test.drawMusic();

                    if (((notZeroCnt == 1 && sameCnt == 1) || (notZeroCnt >= 2 && sameCnt >= 2)) && (millis() - displayGapTime > 1000))
                    {
                        test.forwardPos();
                        tft.fillRect(0, 60, 480, 260, TFT_WHITE);
                        lastNoteIndex = noteIndex;
                        noteIndex++;
                        if (noteIndex >= totalIndex)
                        {
                            noteIndex = 0;
                        }
                        for (int i = 0; i < 12; i++)
                        {
                            targetChromagram[i] = 0;
                        }
                        generateTargetChromagram(noteIndex);
                        displayGapTime = millis();
                    }
                }
            }
            if (posState == 1) // 检测到旋钮转动
            {
                displayerState = 0; // 关闭乐谱演示
                test.forwardPos();
                tft.fillRect(0, 60, 480, 260, TFT_WHITE);
                lv_label_set_text(guider_ui.Musicsheetpresenter_btn_3_label, "PAUSED");
                posState = 0;
                lastNoteIndex = noteIndex;

                noteIndex++;
                if (noteIndex >= totalIndex)
                {
                    noteIndex = 0;
                }
            }
            if (posState == -1) // 检测到旋钮转动
            {
                displayerState = 0; // 关闭乐谱演示
                test.backwardPos();
                tft.fillRect(0, 60, 480, 260, TFT_WHITE);
                lv_label_set_text(guider_ui.Musicsheetpresenter_btn_3_label, "PAUSED");
                posState = 0;
                if (noteIndex > 0)
                {
                    lastNoteIndex = noteIndex;
                    noteIndex--;
                }
            }
        }
        if (pagehome_ddlist_index == 2)
        {
            if (continueState == 1)
            {
                test.drawMusic();
                if (pos != lastpos)
                {
                    test.backwardPos();
                    tft.fillRect(0, 60, 480, 260, TFT_WHITE);
                }
            }
        }
    }
    if(lv_scr_act() == guider_ui.Settings)
    {
        Serial.println(lv_textarea_get_text(guider_ui.Settings_ta_1));
        Serial.println(lv_textarea_get_text(guider_ui.Settings_ta_2));
    }

    // 控制LED显示
    if (LEDState == 0) // 不显示
    {
        LEDclear();
    }
    else if (LEDState == 1 && lv_scr_act() == guider_ui.Musicsheetpresenter) // 乐谱演示模式
    {
        if (lastNoteIndex != noteIndex)
        {
            generateLEDDisplay(noteIndex);
            lastNoteIndex = noteIndex;
        }
    }
    else if (LEDState == 2 && lv_scr_act() == guider_ui.Musicsheetpresenter) // 频谱显示模式
    {
        if (millis() - LEDGapTime > 100)
        {
            generateLEDChromagramDisplay();
            LEDGapTime = millis();
        }
    }
    delay(5);
}

void LedLoop(void *pvParameters)
{
    while (1)
    {
        LEDdisplay();
        if (lv_scr_act() == guider_ui.Metronomehome && switchState == 1)
        {
            bpm = MetroPointerAngle;
            int time = 60 * 1000 / bpm;
            if (millis() - time0 > time)
            {
                tone(beepPin, 300);
                time0 = millis();
                time1 = time0;
            }
            if (millis() - time1 > 50)
            {
                noTone(beepPin);
                time1 = millis();
            }
        }
    }
}