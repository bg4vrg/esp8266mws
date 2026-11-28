/*
    Esp8266MWS -> Solar Low-Power APRS Station (v2.1)
    Original Author : BG4UVR @ 2021.5 (based on ver 0.17a)
    Modified by     : BG4VRG @ 2026.11.28
    Features        :
      1. 3-Mode Sensor Switch (-t 0: Off | 1: Comment text [Default, S-] | 2: Standard APRS WX [R_])
      2. AHT20 primary temperature (with BMP280 fallback & range sanity checks)
      3. 3-Tier Adaptive Solar Sleep (Night [Night] | Rainy/Cloudy [Cloudy] | LowVolt [LowPower] / Cutoff)
      4. Built-in Web Configuration UI (http://<IP>) + TCP Debug CLI + Hold FLASH button at boot
      5. APRS Standard Telemetry (T# + PARM/UNIT/EQNS) with RTC sequence memory
*/
// #define DEBUG_MODE
// #define EEPROM_CLEAR

const char fw[20] = {"v2.1"};

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <Adafruit_AHTX0.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

ADC_MODE(ADC_VCC);

#define FLASH_BTN_PIN  0      // 板载 FLASH 按键 (GPIO0)
#define LOW_POWER_VOLT 3.15f  // 低电压阈值 (切换为1小时发送一次)
#define CUTOFF_VOLT    3.00f  // 极端欠压保护阈值 (不开Wi-Fi直接休眠1小时)

#define DBGPRINT(x)             \
    if (client_dbg.connected()) \
        client_dbg.print(x);

#define DBGPRINTLN(x)           \
    if (client_dbg.connected()) \
        client_dbg.println(x);

typedef enum
{
    SYS_FAIL,
    SYS_CFG,
    SYS_RUN,
} sys_mode_t;

typedef enum
{
    CN = 0,
    EN = 1,
} language_t;

#define LANGUAGE_NUM 2

// EEPROM 配置结构体
typedef struct
{
    uint32_t crc;
    float lon;
    float lat;
    uint16_t password;
    uint16_t send_interval;
    uint16_t aprs_server_port;
    uint16_t debug_server_port;
    char aprs_server_addr[26];
    char debug_server_addr[26];
    char callsign[8];
    char ssid[4];
    sys_mode_t sysstate;
    language_t language;
    char symbol[4];
    char comment[40];
    uint8_t wx_mode; // 0: 纯固定站(关传感器), 1: 固定站+注释带温湿度气压(默认), 2: 标准APRS气象站
} cfg_t;

// RTC 断电记忆结构体
typedef struct
{
    uint32_t magic;
    uint16_t seq;
} rtc_data_t;

cfg_t mycfg;
rtc_data_t rtcData;
WiFiClient client_aprs, client_dbg;
ESP8266WebServer webServer(80);
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;

float voltage;
uint32_t last_send;
uint32_t sleepsec;
char msgbuf[240] = {0};
bool bm280_state = false;
bool aht20_state = false;
bool web_started = false;

// 当次唤醒的环境与电源状态标志
bool low_power_mode = false;
bool night_mode = false;
bool cloudy_mode = false;
int current_wmo_code = -1;

// ======================== CRC32 校验 ========================
uint32_t mycrc32(uint8_t *data, int length)
{
    uint32_t crc = 0xffffffff;
    uint8_t *ldata = data;
    while (length--)
    {
        uint8_t c = *(ldata++);
        for (uint32_t i = 0x80; i > 0; i >>= 1)
        {
            bool bit = crc & 0x80000000;
            if (c & i)
                bit = !bit;
            crc <<= 1;
            if (bit)
                crc ^= 0x04c11db7;
        }
    }
    return crc;
}

// ======================== RTC 遥测序号管理 ========================
uint16_t get_next_telemetry_seq()
{
    ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData));
    if (rtcData.magic != 0x20261128)
    {
        rtcData.magic = 0x20261128;
        rtcData.seq = 1;
    }
    else
    {
        rtcData.seq++;
        if (rtcData.seq > 999)
            rtcData.seq = 1;
    }
    ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData));
    return rtcData.seq;
}

// ======================== 坐标转换与日出日落天文计算 ========================
float aprsToDecimal(float aprsCoord)
{
    int deg = (int)(aprsCoord / 100.0f);
    float min = aprsCoord - (deg * 100.0f);
    return (float)deg + (min / 60.0f);
}

bool check_is_night(struct tm *timenow)
{
    if (timenow->tm_year + 1900 < 2024)
        return false;

    float lat_deg = aprsToDecimal(mycfg.lat);
    float lon_deg = aprsToDecimal(mycfg.lon);
    int doy = timenow->tm_yday + 1;

    double gamma = 2.0 * PI / 365.0 * (doy - 1);
    double eqtime = 229.18 * (0.000075 + 0.001868 * cos(gamma) - 0.032077 * sin(gamma)
                              - 0.014615 * cos(2.0 * gamma) - 0.040849 * sin(2.0 * gamma));
    double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma)
                  - 0.006758 * cos(2.0 * gamma) + 0.000907 * sin(2.0 * gamma)
                  - 0.002697 * cos(3.0 * gamma) + 0.00148 * sin(3.0 * gamma);

    double lat_rad = lat_deg * DEG_TO_RAD;
    double cos_ha = (cos(90.833 * DEG_TO_RAD) / (cos(lat_rad) * cos(decl))) - tan(lat_rad) * tan(decl);

    if (cos_ha > 1.0)
        return true;
    if (cos_ha < -1.0)
        return false;

    double ha_deg = acos(cos_ha) * RAD_TO_DEG;

    int sunrise_utc = (int)round(720.0 - 4.0 * (lon_deg + ha_deg) - eqtime);
    int sunset_utc  = (int)round(720.0 - 4.0 * (lon_deg - ha_deg) - eqtime);

    sunrise_utc = (sunrise_utc % 1440 + 1440) % 1440;
    sunset_utc  = (sunset_utc  % 1440 + 1440) % 1440;

    int now_utc_min = timenow->tm_hour * 60 + timenow->tm_min;

    if (sunrise_utc < sunset_utc)
        return (now_utc_min < sunrise_utc || now_utc_min > sunset_utc);
    else
        return (now_utc_min > sunset_utc && now_utc_min < sunrise_utc);
}

// ======================== 白天请求 Open-Meteo 判断阴雨天 ========================
bool check_cloudy_weather()
{
    float lat_deg = aprsToDecimal(mycfg.lat);
    float lon_deg = aprsToDecimal(mycfg.lon);

    char url[140];
    snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current=weather_code",
             lat_deg, lon_deg);

    WiFiClient httpClient;
    HTTPClient http;
    http.setTimeout(3000);

    if (http.begin(httpClient, url))
    {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK)
        {
            String payload = http.getString();
            int idx = payload.lastIndexOf("\"weather_code\":");
            if (idx != -1)
            {
                current_wmo_code = payload.substring(idx + 15).toInt();
                http.end();
                return (current_wmo_code >= 3);
            }
        }
        http.end();
    }
    current_wmo_code = -1;
    return false;
}

// ======================== 自动配网 ========================
void WiFisetup()
{
    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(300);

    if (!wifiManager.autoConnect("Esp8266MWS-SET"))
    {
        delay(3000);
        ESP.reset();
        delay(5000);
    }
}

// ======================== 传感器初始化与读取（含异常值过滤） ========================
void init_sensors()
{
    Wire.begin(12, 14);
    Wire.setClock(50000);
    bm280_state = bmp.begin();
    if (!bm280_state)
        bm280_state = bmp.begin(BMP280_ADDRESS_ALT);
    aht20_state = aht.begin();
}

bool read_bmp280(float *temperature, float *pressure)
{
    if (!bm280_state)
        return false;
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
    float t = bmp.readTemperature();
    float p = bmp.readPressure();
    if (isnan(t) || isnan(p) || t < -40.0f || t > 85.0f || p < 30000.0f || p > 110000.0f)
        return false;
    *temperature = t;
    *pressure = p;
    return true;
}

bool read_aht20(float *temperature, float *humidity)
{
    if (!aht20_state)
        return false;
    sensors_event_t humAHT, tempAHT;
    aht.getEvent(&humAHT, &tempAHT);
    float t = tempAHT.temperature;
    float h = humAHT.relative_humidity;
    if (isnan(t) || isnan(h) || t < -40.0f || t > 85.0f || h < 0.0f || h > 100.0f)
        return false;
    *temperature = t;
    *humidity = h;
    return true;
}

// ======================== 校验与保存配置 ========================
bool is_valid_symbol(const char *sym)
{
    if (strlen(sym) != 2)
        return false;
    char table = sym[0];
    char code = sym[1];
    bool valid_table = (table == '/' || table == '\\' ||
                        (table >= '0' && table <= '9') ||
                        (table >= 'A' && table <= 'Z'));
    bool valid_code = (code >= 0x21 && code <= 0x7E);
    return valid_table && valid_code;
}

void eeprom_save()
{
    mycfg.crc = mycrc32((uint8_t *)&mycfg + 4, sizeof(mycfg) - 4);
    for (uint8_t i = 0; i < sizeof(cfg_t); i++)
        EEPROM.write(i, ((uint8_t *)&mycfg)[i]);
    EEPROM.commit();
}

void cfg_init()
{
    mycfg.lon = 11818.88f;
    mycfg.lat = 3357.77f;
    mycfg.send_interval = 900;
    mycfg.password = 0;
    mycfg.aprs_server_port = 14580;
    mycfg.debug_server_port = 12345;
    strcpy(mycfg.aprs_server_addr, "rotate.aprs2.net");
    strcpy(mycfg.debug_server_addr, "192.168.1.125");
    strcpy(mycfg.callsign, "NOCALL");
    strcpy(mycfg.ssid, "13");
    mycfg.sysstate = SYS_CFG;
    mycfg.language = CN;
    mycfg.wx_mode = 1; // 默认模式1: 固定站 + 注释附带温湿度气压
    strcpy(mycfg.symbol, "S-");
    strcpy(mycfg.comment, "ESP Solar Sta.");
    eeprom_save();
}

// ======================== 内置网页配置后台 ========================
void handleWebRoot()
{
    voltage = (float)ESP.getVcc() / 1000.0f;
    int32_t rssi = WiFi.RSSI();
    String pwrState = (voltage < LOW_POWER_VOLT) ? "低电压保护 (1小时/次)" : "智能光控+气象调度 (晴天" + String(mycfg.send_interval / 60) + "分钟 / 夜间及阴雨1小时)";

    String sensorInfo = "未启用传感器 (模式 0)";
    if (mycfg.wx_mode > 0)
    {
        if (!aht20_state && !bm280_state)
            init_sensors();
        float tAht = 0, hAht = 0, tBmp = 0, pBmp = 0;
        bool okAht = read_aht20(&tAht, &hAht);
        bool okBmp = read_bmp280(&tBmp, &pBmp);
        sensorInfo = "";
        if (okAht)
            sensorInfo += "AHT20: <b>" + String(tAht, 1) + "°C / " + String((int)round(hAht)) + "%</b> &nbsp;|&nbsp; ";
        else
            sensorInfo += "AHT20: <span style='color:#d93025'>未连接</span> &nbsp;|&nbsp; ";

        if (okBmp)
            sensorInfo += "BMP280: <b>" + String(pBmp / 100.0f, 1) + "hPa</b> (参考温" + String(tBmp, 1) + "°C)";
        else
            sensorInfo += "BMP280: <span style='color:#d93025'>未连接</span>";
    }

    String html = F("<!DOCTYPE html><html lang='zh-CN'><head><meta charset='UTF-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<title>太阳能低功耗 APRS 固定台配置</title>"
                    "<style>"
                    "body{font-family:system-ui,-apple-system,sans-serif;background:#f2f4f8;margin:0;padding:16px;color:#222;}"
                    ".card{max-width:480px;margin:0 auto;background:#fff;padding:20px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,.08);}"
                    "h2{margin:0 0 6px;font-size:1.3rem;color:#1a73e8;}"
                    ".sub{font-size:0.85rem;color:#666;margin-bottom:16px;}"
                    ".status{background:#eef5ff;padding:10px 12px;border-radius:8px;font-size:0.88rem;margin-bottom:16px;line-height:1.65;}"
                    "label{display:block;font-size:0.88rem;font-weight:600;margin:10px 0 4px;}"
                    "input,select{width:100%;box-sizing:border-box;padding:9px 10px;border:1px solid #ccc;border-radius:6px;font-size:0.95rem;}"
                    ".row{display:flex;gap:10px;}.row>div{flex:1;}"
                    "button{width:100%;margin-top:18px;padding:11px;background:#1a73e8;color:#fff;border:none;border-radius:8px;font-size:1rem;font-weight:600;cursor:pointer;}"
                    "button:hover{background:#1558b0;}"
                    ".btn-def{background:#d93025;margin-top:10px;}"
                    ".btn-def:hover{background:#a50e0e;}"
                    "</style></head><body><div class='card'>"
                    "<h2>太阳能低功耗 APRS 固定台</h2>"
                    "<div class='sub'>Ver: ");
    html += fw;
    html += F(" | Mod: BG4VRG | Orig: BG4UVR</div><div class='status'>");
    html += "当前电压: <b>" + String(voltage, 2) + " V</b> &nbsp;|&nbsp; Wi-Fi 信号: <b>" + String(rssi) + " dBm</b><br>";
    html += "传感器状态: " + sensorInfo + "<br>";
    html += "休眠策略: <b>" + pwrState + "</b><br>";
    html += "运行状态: <b>" + String(mycfg.sysstate == SYS_RUN ? "正常运行中" : "等待配置") + "</b> &nbsp;|&nbsp; IP: " + WiFi.localIP().toString();
    html += F("</div><form action='/save' method='POST'>"
              "<div class='row'><div><label>呼号 (Callsign)*</label><input name='call' maxlength='6' required value='");
    html += String(mycfg.callsign);
    html += F("'></div><div><label>SSID</label><input name='ssid' maxlength='2' value='");
    html += String(mycfg.ssid);
    html += F("'></div></div>"
              "<label>APRS-IS 验证码 (Passcode)*</label><input type='number' name='pass' required value='");
    html += String(mycfg.password);
    html += F("'><div class='row'><div><label>纬度 (ddmm.mm)</label><input type='number' step='0.01' name='lat' value='");
    html += String(mycfg.lat, 2);
    html += F("'></div><div><label>经度 (dddmm.mm)</label><input type='number' step='0.01' name='lon' value='");
    html += String(mycfg.lon, 2);
    html += F("'></div></div><div class='row'><div><label>台站图标 (2位字符)</label><input name='sym' maxlength='2' value='");
    html += String(mycfg.symbol);
    html += F("'></div><div><label>传感器工作模式</label><select name='wx'>"
              "<option value='0'");
    if (mycfg.wx_mode == 0)
        html += F(" selected");
    html += F(">0 - 关闭 (仅电压)</option><option value='1'");
    if (mycfg.wx_mode == 1)
        html += F(" selected");
    html += F(">1 - 注释附带温湿度气压 (默认)</option><option value='2'");
    if (mycfg.wx_mode == 2)
        html += F(" selected");
    html += F(">2 - 标准APRS气象站 (R_)</option></select></div></div>"
              "<label>注释文字 (Comment)</label><input name='cmt' maxlength='38' value='");
    html += String(mycfg.comment);
    html += F("'><div class='row'><div><label>白天晴天间隔 (600-1200秒)</label><input type='number' min='600' max='1200' name='intv' value='");
    html += String(mycfg.send_interval);
    html += F("'></div><div><label>APRS 端口</label><input type='number' name='port' value='");
    html += String(mycfg.aprs_server_port);
    html += F("'></div></div>"
              "<label>APRS 服务器地址</label><input name='srv' maxlength='25' value='");
    html += String(mycfg.aprs_server_addr);
    html += F("'><div class='row'><div><label>调试主机 IP</label><input name='dip' maxlength='25' value='");
    html += String(mycfg.debug_server_addr);
    html += F("'></div><div><label>调试端口</label><input type='number' name='dport' value='");
    html += String(mycfg.debug_server_port);
    html += F("'></div></div>"
              "<button type='submit'>保存配置并立即进入运行模式</button></form>"
              "<form action='/reset' method='POST' onsubmit=\"return confirm('确定恢复出厂设置并重启？');\">"
              "<button type='submit' class='btn-def'>恢复出厂默认配置</button></form>"
              "</div></body></html>");

    webServer.send(200, "text/html", html);
}

void handleWebSave()
{
    String call = webServer.arg("call");
    call.trim();
    call.toUpperCase();
    uint16_t pass = webServer.arg("pass").toInt();

    if (call.length() < 4 || call.length() > 6 || call == "NOCALL" || pass == 0)
    {
        webServer.send(400, "text/html; charset=UTF-8", "<h3>参数错误：呼号(4-6位)和有效验证码不能为空！<a href='/'>返回</a></h3>");
        return;
    }

    strcpy(mycfg.callsign, call.c_str());
    mycfg.password = pass;

    String ssid = webServer.arg("ssid");
    ssid.trim();
    if (ssid.length() > 0 && ssid.length() <= 2)
        strcpy(mycfg.ssid, ssid.c_str());

    mycfg.lat = webServer.arg("lat").toFloat();
    mycfg.lon = webServer.arg("lon").toFloat();

    int wx_in = webServer.arg("wx").toInt();
    uint8_t new_wx = (wx_in >= 0 && wx_in <= 2) ? (uint8_t)wx_in : 1;
    String sym = webServer.arg("sym");
    sym.trim();

    if (new_wx != mycfg.wx_mode && sym == String(mycfg.symbol))
    {
        strcpy(mycfg.symbol, new_wx == 2 ? "R_" : "S-");
    }
    else if (is_valid_symbol(sym.c_str()))
    {
        strcpy(mycfg.symbol, sym.c_str());
    }
    mycfg.wx_mode = new_wx;

    String cmt = webServer.arg("cmt");
    cmt.trim();
    if (cmt.length() < sizeof(mycfg.comment))
        strcpy(mycfg.comment, cmt.c_str());

    int intv = webServer.arg("intv").toInt();
    if (intv >= 600 && intv <= 1200)
        mycfg.send_interval = intv;

    String srv = webServer.arg("srv");
    srv.trim();
    if (srv.length() > 0 && srv.length() <= 25)
        strcpy(mycfg.aprs_server_addr, srv.c_str());

    int port = webServer.arg("port").toInt();
    if (port >= 20 && port <= 20000)
        mycfg.aprs_server_port = port;

    String dip = webServer.arg("dip");
    dip.trim();
    if (dip.length() > 0 && dip.length() <= 25)
        strcpy(mycfg.debug_server_addr, dip.c_str());

    int dport = webServer.arg("dport").toInt();
    if (dport >= 1024 && dport <= 65000)
        mycfg.debug_server_port = dport;

    mycfg.sysstate = SYS_RUN;
    eeprom_save();

    webServer.send(200, "text/html; charset=UTF-8",
                   "<h3>配置已保存！设备将在 2 秒后重启并按新参数上报数据...</h3>");
    delay(2000);
    ESP.reset();
}

void handleWebReset()
{
    cfg_init();
    webServer.send(200, "text/html; charset=UTF-8",
                   "<h3>已恢复出厂默认配置，设备正在重启进入配置模式...</h3>");
    delay(2000);
    ESP.reset();
}

void startWebServer()
{
    if (web_started)
        return;
    webServer.on("/", HTTP_GET, handleWebRoot);
    webServer.on("/save", HTTP_POST, handleWebSave);
    webServer.on("/reset", HTTP_POST, handleWebReset);
    webServer.begin();
    web_started = true;
}

// ======================== 发送 APRS 位置、状态与遥测报文 ========================
void send_data()
{
    voltage = (float)ESP.getVcc() / 1000.0f;
    low_power_mode = (voltage < LOW_POWER_VOLT);
    int32_t rssi = WiFi.RSSI();
    uint16_t seq = get_next_telemetry_seq();

    // 1. NTP 授时
    uint8_t sync_timeout = 50;
    configTime(0, 0, "ntp.aliyun.com", "time.asia.apple.com", "pool.ntp.org");
    DBGPRINTLN("Waiting for time sync from NTP server");
    while (time(nullptr) <= 100000 && sync_timeout--)
    {
        DBGPRINT(".");
        delay(100);
    }
    time_t now = time(nullptr);
    struct tm *timenow = gmtime(&now);
    snprintf(msgbuf, sizeof(msgbuf), "\nGMT time is: %s", asctime(timenow));
    DBGPRINTLN(msgbuf);

    // 2. 判断昼夜状态与白天当地天气状态
    night_mode = check_is_night(timenow);
    if (!night_mode)
    {
        cloudy_mode = check_cloudy_weather();
        if (client_dbg.connected())
            client_dbg.printf("Daytime check -> WMO code: %d (%s)\n",
                              current_wmo_code, cloudy_mode ? "Cloudy/Rainy -> 1h sleep" : "Sunny -> Normal interval");
    }
    else
    {
        cloudy_mode = false;
        DBGPRINTLN("Nighttime check -> After sunset -> 1h sleep");
    }

    bool need_long_sleep = (low_power_mode || night_mode || cloudy_mode);
    sleepsec = need_long_sleep ? 3600 : mycfg.send_interval;

    // 3. 首次开机 (seq == 1) 或每隔 3 小时发送状态报文与遥测定义（PARM / UNIT / EQNS）
    if (seq == 1 || ((timenow->tm_hour % 3 == 0) && (timenow->tm_min < (sleepsec / 60) + 1)))
    {
        snprintf(msgbuf, sizeof(msgbuf), "%s-%s>APUVR:>太阳能低功耗APRS固定台 %s by BG4VRG (Orig: BG4UVR)",
                 mycfg.callsign, mycfg.ssid, fw);
#ifndef DEBUG_MODE
        client_aprs.println(msgbuf);
#endif
        DBGPRINTLN(msgbuf);
        delay(200);

        char call_ssid[12];
        snprintf(call_ssid, sizeof(call_ssid), "%s-%s", mycfg.callsign, mycfg.ssid);

        snprintf(msgbuf, sizeof(msgbuf), "%s>APUVR::%-9s:PARM.BatVolt,WiFiRSSI,WakeCnt,Spare1,Spare2,WX_Std,LowPwr,Night,Cloudy",
                 call_ssid, call_ssid);
#ifndef DEBUG_MODE
        client_aprs.println(msgbuf);
#endif
        DBGPRINTLN(msgbuf);
        delay(200);

        snprintf(msgbuf, sizeof(msgbuf), "%s>APUVR::%-9s:UNIT.V,dBm,cnt,none,none,ON,LOW,NGT,CLD",
                 call_ssid, call_ssid);
#ifndef DEBUG_MODE
        client_aprs.println(msgbuf);
#endif
        DBGPRINTLN(msgbuf);
        delay(200);

        snprintf(msgbuf, sizeof(msgbuf), "%s>APUVR::%-9s:EQNS.0,0.01,0,0,-1,0,0,1,0,0,1,0,0,1,0",
                 call_ssid, call_ssid);
#ifndef DEBUG_MODE
        client_aprs.println(msgbuf);
#endif
        DBGPRINTLN(msgbuf);
        delay(200);
    }

    // 4. 读取本地传感器（仅在模式 1 或模式 2 下读取，以 AHT20 温度为准，BMP280 仅作故障兜底）
    float tempAHT = 0, humAHT = 0, tempBMP = 0, pressBMP = 0;
    bool aht_ok = false, bmp_ok = false, has_temp = false;
    float final_temp = 0.0f;

    if (mycfg.wx_mode == 1 || mycfg.wx_mode == 2)
    {
        aht_ok = read_aht20(&tempAHT, &humAHT);
        bmp_ok = read_bmp280(&tempBMP, &pressBMP);

        if (client_dbg.connected())
        {
            if (aht_ok)
                client_dbg.printf("AHT20 -> Temp: %.2f C, Hum: %.1f %%\n", tempAHT, humAHT);
            if (bmp_ok)
                client_dbg.printf("BMP280 -> Press: %.2f hPa (Ref Temp: %.2f C)\n", pressBMP / 100.0f, tempBMP);
        }

        if (aht_ok)
        {
            final_temp = tempAHT;
            has_temp = true;
        }
        else if (bmp_ok)
        {
            final_temp = tempBMP;
            has_temp = true;
        }
    }

    // 5. 拼接状态标签 ([LowPower] / [Night] / [Cloudy])
    char status_tag[32] = {0};
    if (low_power_mode)
        strcat(status_tag, " [LowPower]");
    if (night_mode)
        strcat(status_tag, " [Night]");
    else if (cloudy_mode)
        strcat(status_tag, " [Cloudy]");

    // 6. 按当前模式组装并发送位置报文
    if (mycfg.wx_mode == 2)
    {
        // 模式 2: 标准 APRS 气象报文格式
        char temperatureS[4] = {"..."};
        char humidityS[3] = {".."};
        char pressureS[6] = {"....."};

        if (has_temp)
            snprintf(temperatureS, sizeof(temperatureS), "%03d", (int8_t)round(final_temp * 9.0f / 5.0f + 32.0f));
        if (aht_ok)
            snprintf(humidityS, sizeof(humidityS), "%02d", (uint8_t)round(humAHT));
        if (bmp_ok)
            snprintf(pressureS, sizeof(pressureS), "%05d", (uint16_t)round(pressBMP / 10.0f));

        snprintf(msgbuf, sizeof(msgbuf),
                 "%s-%s>APUVR:!%07.2f%c%c%08.2f%c%c.../...g...t%sr...p...h%sb%s%s%sBat:%0.2fV%s",
                 mycfg.callsign, mycfg.ssid,
                 mycfg.lat, mycfg.lat > 0 ? 'N' : 'S', mycfg.symbol[0],
                 mycfg.lon, mycfg.lon > 0 ? 'E' : 'W', mycfg.symbol[1],
                 temperatureS, humidityS, pressureS,
                 mycfg.comment, strlen(mycfg.comment) > 0 ? " " : "", voltage, status_tag);
    }
    else if (mycfg.wx_mode == 1)
    {
        // 模式 1 (默认): 固定站图标 + 注释后附带温湿度气压文本
        char env_str[40] = {0};
        char tmp[16];
        if (has_temp)
        {
            snprintf(tmp, sizeof(tmp), " %.1fC", final_temp);
            strcat(env_str, tmp);
        }
        if (aht_ok)
        {
            snprintf(tmp, sizeof(tmp), " %d%%", (int)round(humAHT));
            strcat(env_str, tmp);
        }
        if (bmp_ok)
        {
            snprintf(tmp, sizeof(tmp), " %dhPa", (int)round(pressBMP / 100.0f));
            strcat(env_str, tmp);
        }

        snprintf(msgbuf, sizeof(msgbuf),
                 "%s-%s>APUVR:!%07.2f%c%c%08.2f%c%c%s%sBat:%0.2fV%s%s",
                 mycfg.callsign, mycfg.ssid,
                 mycfg.lat, mycfg.lat > 0 ? 'N' : 'S', mycfg.symbol[0],
                 mycfg.lon, mycfg.lon > 0 ? 'E' : 'W', mycfg.symbol[1],
                 mycfg.comment, strlen(mycfg.comment) > 0 ? " " : "", voltage, env_str, status_tag);
    }
    else
    {
        // 模式 0: 纯固定站（不读取传感器，仅上报电压）
        snprintf(msgbuf, sizeof(msgbuf),
                 "%s-%s>APUVR:!%07.2f%c%c%08.2f%c%c%s%sBat:%0.2fV%s",
                 mycfg.callsign, mycfg.ssid,
                 mycfg.lat, mycfg.lat > 0 ? 'N' : 'S', mycfg.symbol[0],
                 mycfg.lon, mycfg.lon > 0 ? 'E' : 'W', mycfg.symbol[1],
                 mycfg.comment, strlen(mycfg.comment) > 0 ? " " : "", voltage, status_tag);
    }

#ifndef DEBUG_MODE
    client_aprs.println(msgbuf);
#endif
    DBGPRINTLN(msgbuf);
    delay(200);

    // 7. 发送标准 APRS 遥测数据帧 (T#)，Spare1 与 Spare2 保持 000
    uint16_t v_val = (uint16_t)(voltage * 100.0f);
    uint16_t rssi_val = (rssi < 0) ? (uint16_t)(-rssi) : 0;
    if (rssi_val > 999)
        rssi_val = 999;

    snprintf(msgbuf, sizeof(msgbuf),
             "%s-%s>APUVR:T#%03d,%03d,%03d,%03d,000,000,%d%d%d%d0000",
             mycfg.callsign, mycfg.ssid,
             seq, v_val, rssi_val, seq,
             mycfg.wx_mode == 2 ? 1 : 0,
             low_power_mode ? 1 : 0,
             night_mode ? 1 : 0,
             cloudy_mode ? 1 : 0);

#ifndef DEBUG_MODE
    client_aprs.println(msgbuf);
#endif
    DBGPRINTLN(msgbuf);
}

// ======================== 登录 APRS-IS 服务器 ========================
bool loginAPRS()
{
    DBGPRINTLN(mycfg.language == CN ? "正在连接APRS服务器" : "Connecting to the APRS server");

    uint8_t timeout = 0;
    while (timeout++ < 5)
    {
        if (client_aprs.connect(mycfg.aprs_server_addr, mycfg.aprs_server_port))
        {
            DBGPRINTLN(mycfg.language == CN ? "APRS服务器已连接" : "The APRS server is connected");
            timeout = 0;
            while (timeout++ < 100)
            {
                if (client_aprs.available())
                {
                    String line = client_aprs.readStringUntil('\n');
                    DBGPRINTLN(line);

                    if (line.indexOf("aprsc") != -1 || line.indexOf("javAPRSSrvr") != -1)
                    {
                        sprintf(msgbuf, "user %s-%s pass %d vers esp8266solar %s",
                                mycfg.callsign, mycfg.ssid, mycfg.password, fw);
                        client_aprs.println(msgbuf);
                        DBGPRINTLN(msgbuf);
                        timeout = 0;
                    }
                    else if (line.indexOf("verified") != -1)
                    {
                        DBGPRINTLN(mycfg.language == CN ? "APRS服务器登录成功" : "APRS server login successful");
                        send_data();
                        if (client_aprs.flush(10000))
                        {
                            DBGPRINTLN(mycfg.language == CN ? "数据发送成功" : "data sent success");
                            return true;
                        }
                        else
                        {
                            DBGPRINTLN(mycfg.language == CN ? "数据发送超时" : "data sending timeout");
                            return false;
                        }
                    }
                    else if (line.indexOf("Server full") != -1 || line.indexOf("Port full") != -1)
                    {
                        DBGPRINTLN(mycfg.language == CN ? "服务器已满，稍后重试" : "Server full");
                        return false;
                    }
                }
                delay(100);
            }
            DBGPRINTLN(mycfg.language == CN ? "错误：接收APRS服务器数据超时" : "Error: Receive timeout");
            return false;
        }
        else
            delay(2000);
    }
    return false;
}

// ======================== TCP 调试主机配置逻辑 ========================
void dispset()
{
    const char *mode_desc[] = {"0 (关闭传感器)", "1 (注释附带温湿度气压)", "2 (标准APRS气象站)"};
    client_dbg.println(mycfg.language == CN ? "系统当前配置：" : "System current configuration:");
    client_dbg.printf("APRS服务器:\t%s:%d\n", mycfg.aprs_server_addr, mycfg.aprs_server_port);
    client_dbg.printf("调试主机:\t%s:%d\n", mycfg.debug_server_addr, mycfg.debug_server_port);
    client_dbg.printf("呼号-SSID:\t%s-%s (Pass: %d)\n", mycfg.callsign, mycfg.ssid, mycfg.password);
    client_dbg.printf("传感器模式:\t%s\n", mode_desc[mycfg.wx_mode <= 2 ? mycfg.wx_mode : 1]);
    client_dbg.printf("台站图标:\t%s\n", mycfg.symbol);
    client_dbg.printf("注释文字:\t%s\n", mycfg.comment);
    client_dbg.printf("晴天发送间隔:\t%d 秒 (夜间/阴雨/低电自动切为3600秒)\n", mycfg.send_interval);
    client_dbg.printf("经度/纬度:\t%0.2f / %0.2f\n", mycfg.lon, mycfg.lat);
}

void dispsysinfo()
{
    client_dbg.printf("\n太阳能低功耗APRS固定台 (Ver:%s | Mod:BG4VRG | Orig:BG4UVR)\n", fw);
    client_dbg.print("网页配置后台地址: http://");
    client_dbg.println(WiFi.localIP());
    dispset();
    client_dbg.println("\n支持命令行: cfg -c 呼号 -w 验证码 -a 纬度 -o 经度 -t 0/1/2 -i 图标 -m \"注释\" | rst | def\n");
}

uint8_t split_args(char *str, char **argv, uint8_t max_args)
{
    uint8_t argc = 0;
    char *p = str;
    while (*p && argc < max_args)
    {
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            break;

        if (*p == '"')
        {
            p++;
            argv[argc++] = p;
            while (*p && *p != '"')
                p++;
            if (*p == '"')
                *p++ = '\0';
        }
        else
        {
            argv[argc++] = p;
            while (*p && *p != ' ' && *p != '\t')
                p++;
            if (*p)
                *p++ = '\0';
        }
    }
    return argc;
}

void set_cfg()
{
    if (!client_dbg.available())
        return;

    String line = client_dbg.readStringUntil('\n');
    line.trim();
    client_dbg.println(line);

    char *buf = new char[line.length() + 1];
    strcpy(buf, line.c_str());

    if (strcmp(buf, "rst") == 0)
    {
        DBGPRINTLN("重启中...");
        delete[] buf;
        delay(1000);
        ESP.reset();
    }

    if (strcmp(buf, "def") == 0)
    {
        DBGPRINTLN("恢复出厂设置并重启...");
        cfg_init();
        delete[] buf;
        delay(1000);
        ESP.reset();
    }

    if (strncmp(buf, "cfg ", 4) != 0)
    {
        DBGPRINTLN("命令格式错误（支持 cfg / rst / def）");
        delete[] buf;
        return;
    }

    char *cmd[50] = {0};
    uint8_t cnt = split_args(buf, cmd, 50);

    optind = 0;
    int ch;
    bool fail = false, ok = false, symbol_explicit = false;
    int wx_opt = -1;

    while ((ch = getopt(cnt, cmd, "c:w:o:a:s:d:p:e:g:n:l:i:m:t:")) != -1)
    {
        switch (ch)
        {
        case 'c':
            if (strlen(optarg) >= 4 && strlen(optarg) <= 6)
            {
                strcpy(mycfg.callsign, optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'w':
            if (atol(optarg) > 0 && atol(optarg) <= 32768)
            {
                mycfg.password = atoi(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'o':
            if (atof(optarg) <= 18000.0f && atof(optarg) >= -18000.0f)
            {
                mycfg.lon = atof(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'a':
            if (atof(optarg) <= 9000.0f && atof(optarg) >= -9000.0f)
            {
                mycfg.lat = atof(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 's':
            if (strlen(optarg) <= 25)
            {
                strcpy(mycfg.aprs_server_addr, optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'd':
            if (strlen(optarg) > 0 && strlen(optarg) <= 2)
            {
                strcpy(mycfg.ssid, optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 't':
            if (strcmp(optarg, "0") == 0 || strcmp(optarg, "1") == 0 || strcmp(optarg, "2") == 0)
            {
                wx_opt = atoi(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'i':
            if (is_valid_symbol(optarg))
            {
                strcpy(mycfg.symbol, optarg);
                symbol_explicit = true;
                ok = true;
            }
            else
                fail = true;
            break;
        case 'm':
            if (strlen(optarg) < sizeof(mycfg.comment))
            {
                strcpy(mycfg.comment, optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'p':
            if (atoi(optarg) >= 20 && atoi(optarg) <= 20000)
            {
                mycfg.aprs_server_port = atoi(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'g':
            if (strlen(optarg) <= 25)
            {
                strcpy(mycfg.debug_server_addr, optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'e':
            if (atoi(optarg) >= 1024 && atoi(optarg) <= 65000)
            {
                mycfg.debug_server_port = atoi(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'n':
            if (atoi(optarg) >= 600 && atoi(optarg) <= 1200)
            {
                mycfg.send_interval = atoi(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        case 'l':
            if (atoi(optarg) >= 0 && atoi(optarg) < LANGUAGE_NUM)
            {
                mycfg.language = (language_t)atoi(optarg);
                ok = true;
            }
            else
                fail = true;
            break;
        default:
            break;
        }
    }
    delete[] buf;

    if (fail || !ok)
    {
        DBGPRINTLN("参数有误，未保存");
        return;
    }

    if (wx_opt != -1)
    {
        mycfg.wx_mode = (uint8_t)wx_opt;
        if (mycfg.wx_mode > 0)
            init_sensors();

        if (!symbol_explicit)
            strcpy(mycfg.symbol, mycfg.wx_mode == 2 ? "R_" : "S-");
    }

    if (strcmp(mycfg.callsign, "NOCALL") == 0 || strlen(mycfg.callsign) < 4 || mycfg.password == 0)
    {
        DBGPRINTLN("请先设置呼号(-c)和验证码(-w)");
        return;
    }

    mycfg.sysstate = SYS_RUN;
    eeprom_save();
    dispset();
    DBGPRINTLN("设置已保存！");
}

void freeloop()
{
    set_cfg();
    if (web_started)
        webServer.handleClient();
    ArduinoOTA.handle();
    delay(10);
}

// ======================== 初始化与主循环 ========================
void setup()
{
    pinMode(2, OUTPUT);
    pinMode(FLASH_BTN_PIN, INPUT_PULLUP);

    // 1. 极端欠压保护检测（在开启 Wi-Fi 射频前立即检测电压）
    voltage = (float)ESP.getVcc() / 1000.0f;
    if (voltage < CUTOFF_VOLT && digitalRead(FLASH_BTN_PIN) == HIGH)
    {
        digitalWrite(LED_BUILTIN, 1);
        ESP.deepSleep(3600ULL * 1000000ULL);
    }

    digitalWrite(LED_BUILTIN, 0);

    WiFisetup();
    ArduinoOTA.setHostname("Esp8266MWS-OTA");
    ArduinoOTA.begin();
    EEPROM.begin(256);

#ifdef DEBUG_MODE
#ifdef EEPROM_CLEAR
    for (uint8_t i = 0; i < 128; i++)
        EEPROM.write(i, 0xff);
    EEPROM.commit();
#endif
#endif

    for (uint8_t i = 0; i < sizeof(cfg_t); i++)
        *((uint8_t *)&mycfg + i) = EEPROM.read(i);

    if (mycrc32((uint8_t *)&mycfg + 4, sizeof(mycfg) - 4) != mycfg.crc)
        cfg_init();

    // 检测开机时是否按住了板载 FLASH 键，若按住则强制进入网页配置模式
    if (digitalRead(FLASH_BTN_PIN) == LOW)
    {
        mycfg.sysstate = SYS_CFG;
    }

    if (mycfg.wx_mode > 0)
        init_sensors();
}

void loop()
{
    voltage = (float)ESP.getVcc() / 1000.0f;
    low_power_mode = (voltage < LOW_POWER_VOLT);
    sleepsec = low_power_mode ? 3600 : mycfg.send_interval;

    switch (mycfg.sysstate)
    {
    case SYS_RUN:
        client_dbg.connect(mycfg.debug_server_addr, mycfg.debug_server_port);
        dispsysinfo();

        do
        {
            if (!loginAPRS())
            {
                bool is_eco = (low_power_mode || night_mode || cloudy_mode);
                sleepsec = is_eco ? 1800 : 60;
            }

            client_aprs.stop();
            last_send = millis();

            while (millis() - last_send < sleepsec * 1000 && client_dbg.connected())
                freeloop();
        } while (client_dbg.connected());

        client_aprs.stop();
        delay(100);

        digitalWrite(LED_BUILTIN, 1);
        ESP.deepSleep((uint64_t)sleepsec * 1000 * 1000);
        break;

    case SYS_CFG:
    default:
        startWebServer();

        while (mycfg.sysstate != SYS_RUN)
        {
            if (!client_dbg.connected())
            {
                if (client_dbg.connect(mycfg.debug_server_addr, mycfg.debug_server_port))
                    dispsysinfo();
            }
            freeloop();
        }
        break;
    }
}