/*
 * Описание:
 *  Прошивка ESP32 (push-to монтировка sv225) для программы Stellarium.
 *  Протокол Meade LX200 (compatible): USB serial (9600 8N1).
 *  Обе оси (MT6701) пересчитываются в RA/Dec (J2000) на самой ESP32,
 *  привязка к звёздам (Sync) строит модель поправок.
 *
 *  Время и место наблюдения задаются в runtime командами (см. HELP):
 *    T YYYY-MM-DD HH:MM:SS   - время UTC
 *    SLOT <n> <lat> <lon> [название] - сохранить слот места (n = 1..3)
 *    SLOTUSE <n>             - сделать слот текущим местом
 *    SLOTCLEAR <n>           - очистить слот
 *    AZDIR / ALTDIR 1|-1     - направление осей
 *    AZZERO / ALTZERO <°>    - смещение нуля осей
 *    STATUS, CLEAR, HELP
 *
 *  Те же параметры можно задавать с телефона или ноутбука через веб-интерфейс:
 *  ESP32 поднимает точку доступа Wi-Fi (redstar01) и отдаёт страницу настроек
 *  по адресу https://192.168.4.1/ (самоподписанный сертификат) - веб работает
 *  одновременно с LX200 по USB. На странице есть и кнопка взятия
 *  GPS-координат (браузеры отдают их только по HTTPS). Место хранится
 *  в трёх слотах с названиями (NVS), текущее место - активный слот.
 *
 * Автор: Красноперов Павел
 *   https://t.me/redstar01
 *
 * Документация, новые версии программы:
 *   https://github.com/redstar01/sv225_push_to_mount
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

// ----------------------------- настройки -----------------------------

#define WEB_UI 1  // веб-настройка по Wi-Fi (0 - отключить Wi-Fi вместе со страницей)

#if WEB_UI
#include <WiFi.h>
#include <esp_https_server.h>
#endif

static const uint32_t SERIAL_BAUD      = 9600;              // Stellarium открывает serial на 9600 8N1
static const uint16_t ENCODER_STEPS    = 16384;             // MT6701: 14 бит
static const uint32_t ENCODER_READ_MS  = 50;
static const uint8_t  MAX_ALIGN_POINTS = 6;                 // 1 - смещения, 2 - среднее, от 3 - линейная модель (МНК)
static const double   PT_REPLACE_DEG   = 2.0;               // Sync рядом с точкой - обновление, а не дубликат
static const uint8_t  MAX_SLOTS        = 3;                 // слоты мест наблюдения (NVS)
static const uint8_t  SLOT_NAME_BYTES  = 48;                // имя слота (UTF-8, включая NUL)
static const uint8_t  CMD_BUF_SIZE     = 192;               // имя слота в URL-кодировке
static const uint32_t TEXT_CMD_TIMEOUT_MS = 500;            // сброс недополученной текстовой строки

#if WEB_UI
static const char     AP_SSID[]        = "redstar01";       // точка доступа для страницы настроек
static const char     AP_PASS[]        = "1234567890";
static const uint16_t WEB_PORT         = 443;               // HTTPS; на 80 порту - редирект
#endif

static const double DEG2RAD = 0.017453292519943295;
static const double RAD2DEG = 57.29577951308232;

// ----------------------------- состояние -----------------------------

struct Link {
  Stream* io;
  char    buf[CMD_BUF_SIZE];
  uint8_t len;
  uint8_t mode;  // 0 - ожидание, 1 - команда LX200 (до '#'), 2 - текстовая команда (до перевода строки)
  uint32_t lastMs;
};

struct AlignPoint {
  double rawAz, rawAlt;    // углы осей (после AZDIR/ALTDIR/AZZERO/ALTZERO)
  double trueAz, trueAlt;  // истинные углы, посчитанные по звезде
};

static Link        g_links[1];
static Preferences g_prefs;

// места наблюдения: три слота с названиями (NVS); текущее место - активный слот
struct SiteSlot {
  bool   set;
  char   name[SLOT_NAME_BYTES];
  double lat, lon;
};

static SiteSlot g_slots[MAX_SLOTS];
static int8_t   g_activeSlot = -1;  // -1 - место не задано

// координаты активного слота (для расчётов)
static double g_latDeg = 0.0;
static double g_lonDeg = 0.0;

// ориентация энкодеров (AZDIR/ALTDIR/AZZERO/ALTZERO, хранится в NVS)
static int8_t g_azDir = 1;
static int8_t g_altDir = 1;
static double g_azZeroDeg = 0.0;
static double g_altZeroDeg = 0.0;

// время UTC
static time_t   g_utcEpoch = 0;
static uint32_t g_utcAtMs = 0;
static bool     g_timeSet = false;

// показания энкодеров
static int    g_rawAz = 0;
static int    g_rawAlt = 0;
static double g_encAz = 0.0;
static double g_encAlt = 0.0;

// «старт от полюса»: пока нет точек привязки, считаем, что в момент включения
// (или после CLEAR) труба смотрела на полюс мира - крест в Stellarium сразу
// у полюса (на Полярной) и далее едет за монтировкой; первый Sync заменяет
// эту грубую модель.
static double g_polarAz = 0.0;   // g_encAz в момент старта
static double g_polarAlt = 0.0;  // g_encAlt в момент старта

// цель :Sr/:Sd (J2000)
static double g_targetRa = 0.0;
static double g_targetDec = 0.0;
static bool   g_targetRaValid = false;
static bool   g_targetDecValid = false;

// модель привязки
static AlignPoint g_pts[MAX_ALIGN_POINTS];
static uint8_t    g_nPts = 0;

struct Model {
  uint8_t n;
  bool    affine;
  double  refAz, refAlt;       // опорная точка (первая)
  double  cAz0, cAz1, cAz2;    // поправка азимута: e = c0 + c1*du + c2*dv
  double  cAlt0, cAlt1, cAlt2; // поправка высоты
};

static Model g_model;

// прототипы
static void readEncoders();
static void saveConfig();
static void saveSlotData(uint8_t i);
static void selectSlot(uint8_t i);
static void clearSlotData(uint8_t i);
static void currentRaDec(double& raJ2000, double& decJ2000);
static void formatRa(double raRad, char* out);
static void formatDec(double decRad, char* out);

// --------------------------- утилиты углов ---------------------------

static inline double clampd(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline double norm360(double deg)
{
  deg = fmod(deg, 360.0);
  if (deg < 0.0) deg += 360.0;
  return deg;
}

static inline double norm180(double deg)
{
  deg = norm360(deg);
  if (deg > 180.0) deg -= 360.0;
  return deg;
}

static inline double normRad(double rad)
{
  rad = fmod(rad, 2.0 * M_PI);
  if (rad < 0.0) rad += 2.0 * M_PI;
  return rad;
}

// ------------------------------- время -------------------------------

static long long daysFromCivil(int y, int m, int d)
{
  y -= (m <= 2);
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned  yoe = (unsigned)(y - era * 400);
  const unsigned  doy = (unsigned)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
  const unsigned  doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long long)doe - 719468;
}

static void civilFromDays(long long z, int& y, int& m, int& d)
{
  z += 719468;
  const long long era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned  doe = (unsigned)(z - era * 146097);
  const unsigned  yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const long long yy  = (long long)yoe + era * 400;
  const unsigned  doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned  mp  = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp < 10 ? mp + 3 : mp - 9);
  y = (int)(yy + (m <= 2));
}

static void setUtc(time_t t)
{
  g_utcEpoch = t;
  g_utcAtMs = millis();
}

static time_t utcNow()
{
  return g_utcEpoch + (time_t)((millis() - g_utcAtMs) / 1000);
}

static void formatTimeUtc(char* out, size_t n)
{
  time_t t = utcNow();
  long long days = (long long)t / 86400;
  long rem = (long)t % 86400;
  if (rem < 0) { rem += 86400; days--; }
  int y, m, d;
  civilFromDays(days, y, m, d);
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d",
           y, m, d, (int)(rem / 3600), (int)((rem / 60) % 60), (int)(rem % 60));
}

// запасное время - момент компиляции (считаем его UTC, точность не гарантируется)
static void loadCompileTime()
{
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {0};
  int day = 0, year = 0, h = 0, mi = 0, s = 0;
  if (sscanf(__DATE__, "%3s %d %d", mon, &day, &year) == 3 &&
      sscanf(__TIME__, "%d:%d:%d", &h, &mi, &s) == 3) {
    int m = 0;
    for (int i = 0; i < 12; i++) {
      if (!strncmp(mon, months + i * 3, 3)) { m = i + 1; break; }
    }
    if (m > 0) {
      setUtc(daysFromCivil(year, m, day) * 86400 + h * 3600 + mi * 60 + s);
      return;
    }
  }
  setUtc(daysFromCivil(2000, 1, 1) * 86400);
}

// ----------------------------- энкодеры ------------------------------

static int readAngle(TwoWire& wire)
{
  wire.beginTransmission(0b0000110);
  wire.write(0x03);
  wire.endTransmission(false);
  wire.requestFrom((int)0b0000110, 2);
  if (wire.available() < 2) {
    return -1;
  }
  int angleH = wire.read();
  int angleL = wire.read();
  return (angleH << 6) | (angleL >> 2);
}

static void readEncoders()
{
  int rawAz = readAngle(Wire1);
  if (rawAz >= 0) g_rawAz = rawAz;
  int rawAlt = readAngle(Wire);
  if (rawAlt >= 0) g_rawAlt = rawAlt;

  double azDeg  = (double)g_rawAz * 360.0 / ENCODER_STEPS;
  double altDeg = (double)g_rawAlt * 360.0 / ENCODER_STEPS;
  g_encAz  = norm360((double)g_azDir * azDeg + g_azZeroDeg);
  g_encAlt = norm180((double)g_altDir * altDeg + g_altZeroDeg);
}

// запомнить текущее положение как «труба смотрит на полюс мира»
static void setPolarStart()
{
  g_polarAz = g_encAz;
  g_polarAlt = g_encAlt;
}

// ------------------------------ модель -------------------------------

static bool solve3(const double m[3][3], const double b[3], double x[3])
{
  double a[3][4];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) a[i][j] = m[i][j];
    a[i][3] = b[i];
  }
  for (int col = 0; col < 3; col++) {
    int piv = col;
    for (int r = col + 1; r < 3; r++) {
      if (fabs(a[r][col]) > fabs(a[piv][col])) piv = r;
    }
    if (fabs(a[piv][col]) < 1e-9) return false;
    if (piv != col) {
      for (int j = 0; j < 4; j++) {
        double t = a[col][j]; a[col][j] = a[piv][j]; a[piv][j] = t;
      }
    }
    for (int r = 0; r < 3; r++) {
      if (r == col) continue;
      double f = a[r][col] / a[col][col];
      for (int j = col; j < 4; j++) a[r][j] -= f * a[col][j];
    }
  }
  for (int i = 0; i < 3; i++) x[i] = a[i][3] / a[i][i];
  return true;
}

static void rebuildModel()
{
  g_model.n = g_nPts;
  g_model.affine = false;
  if (g_nPts == 0) return;

  g_model.refAz  = g_pts[0].rawAz;
  g_model.refAlt = g_pts[0].rawAlt;

  double eAz[MAX_ALIGN_POINTS], eAlt[MAX_ALIGN_POINTS];
  double du[MAX_ALIGN_POINTS], dv[MAX_ALIGN_POINTS];
  for (uint8_t i = 0; i < g_nPts; i++) {
    du[i]   = norm180(g_pts[i].rawAz - g_model.refAz);
    dv[i]   = g_pts[i].rawAlt - g_model.refAlt;
    eAz[i]  = norm180(g_pts[i].trueAz - g_pts[i].rawAz);
    eAlt[i] = g_pts[i].trueAlt - g_pts[i].rawAlt;
  }

  if (g_nPts == 1) {
    g_model.cAz0  = eAz[0];
    g_model.cAlt0 = eAlt[0];
    return;
  }

  if (g_nPts == 2) {
    // для азимута - круговое среднее (учитывает переход через 360°)
    g_model.cAz0 = atan2((sin(eAz[0] * DEG2RAD) + sin(eAz[1] * DEG2RAD)) / 2.0,
                         (cos(eAz[0] * DEG2RAD) + cos(eAz[1] * DEG2RAD)) / 2.0) * RAD2DEG;
    g_model.cAlt0 = 0.5 * (eAlt[0] + eAlt[1]);
    return;
  }

  // от 3 точек: линейная модель e = c0 + c1*du + c2*dv (МНК по всем точкам)
  double m[3][3] = {{0}};
  double bAz[3] = {0}, bAlt[3] = {0};
  for (uint8_t i = 0; i < g_nPts; i++) {
    double u = du[i], v = dv[i];
    m[0][0] += 1.0; m[0][1] += u; m[0][2] += v;
    m[1][1] += u * u; m[1][2] += u * v; m[2][2] += v * v;
    bAz[0]  += eAz[i];  bAz[1]  += eAz[i] * u;  bAz[2]  += eAz[i] * v;
    bAlt[0] += eAlt[i]; bAlt[1] += eAlt[i] * u; bAlt[2] += eAlt[i] * v;
  }
  m[1][0] = m[0][1]; m[2][0] = m[0][2]; m[2][1] = m[1][2];

  double cAz[3], cAlt[3];
  if (solve3(m, bAz, cAz) && solve3(m, bAlt, cAlt)) {
    g_model.affine = true;
    g_model.cAz0 = cAz[0];  g_model.cAz1 = cAz[1];  g_model.cAz2 = cAz[2];
    g_model.cAlt0 = cAlt[0]; g_model.cAlt1 = cAlt[1]; g_model.cAlt2 = cAlt[2];
  } else {
    double sumSin = 0.0, sumCos = 0.0, sumAlt = 0.0;
    for (uint8_t i = 0; i < g_nPts; i++) {
      sumSin += sin(eAz[i] * DEG2RAD);
      sumCos += cos(eAz[i] * DEG2RAD);
      sumAlt += eAlt[i];
    }
    g_model.cAz0  = atan2(sumSin / g_nPts, sumCos / g_nPts) * RAD2DEG;
    g_model.cAlt0 = sumAlt / g_nPts;
  }
}

static void addAlignPoint(double trueAz, double trueAlt, double rawAz, double rawAlt)
{
  // Повторная привязка рядом с уже сохранённой точкой обновляет её, а не
  // добавляет почти такой же дубликат: иначе набор забьётся точками в одной
  // области неба, а линейная модель станет вырожденной.
  int8_t nearIdx = -1;
  double best = PT_REPLACE_DEG;
  for (uint8_t i = 0; i < g_nPts; i++) {
    double dAz  = norm180(rawAz - g_pts[i].rawAz);
    double dAlt = rawAlt - g_pts[i].rawAlt;
    double d    = sqrt(dAz * dAz + dAlt * dAlt);
    if (d < best) { best = d; nearIdx = (int8_t)i; }
  }

  if (nearIdx >= 0) {
    g_pts[nearIdx].rawAz   = rawAz;
    g_pts[nearIdx].rawAlt  = rawAlt;
    g_pts[nearIdx].trueAz  = trueAz;
    g_pts[nearIdx].trueAlt = trueAlt;
  } else {
    if (g_nPts >= MAX_ALIGN_POINTS) {
      for (uint8_t i = 1; i < MAX_ALIGN_POINTS; i++) g_pts[i - 1] = g_pts[i];
      g_nPts = MAX_ALIGN_POINTS - 1;
    }
    g_pts[g_nPts].rawAz   = rawAz;
    g_pts[g_nPts].rawAlt  = rawAlt;
    g_pts[g_nPts].trueAz  = trueAz;
    g_pts[g_nPts].trueAlt = trueAlt;
    g_nPts++;
  }
  rebuildModel();
}

static void removeAlignPoint(uint8_t i)
{
  if (i >= g_nPts) return;
  for (uint8_t k = i + 1; k < g_nPts; k++) g_pts[k - 1] = g_pts[k];
  g_nPts--;
  rebuildModel();
}

// поправки модели для углов осей (без сворачивания через зенит/надир):
// используются и в applyModel(), и для невязок точек привязки
static void modelErrors(double rawAz, double rawAlt, double& eAz, double& eAlt)
{
  eAz = 0.0; eAlt = 0.0;
  if (g_model.n == 0) {
    // нет точек привязки: отсчёт от полюса мира (AZ = 0°, ALT = широта места)
    eAz  = norm180(-g_polarAz);
    eAlt = g_latDeg - g_polarAlt;
  } else if (g_model.n == 1) {
    eAz = g_model.cAz0;
    eAlt = g_model.cAlt0;
  } else if (g_model.affine) {
    double u = norm180(rawAz - g_model.refAz);
    double v = rawAlt - g_model.refAlt;
    eAz  = g_model.cAz0 + g_model.cAz1 * u + g_model.cAz2 * v;
    eAlt = g_model.cAlt0 + g_model.cAlt1 * u + g_model.cAlt2 * v;
  } else {
    eAz = g_model.cAz0;
    eAlt = g_model.cAlt0;
  }
}

// невязка точки привязки: насколько модель ошибается в самой точке, градусы
static void pointResidualDeg(uint8_t i, double& errAz, double& errAlt)
{
  double eAz, eAlt;
  modelErrors(g_pts[i].rawAz, g_pts[i].rawAlt, eAz, eAlt);
  errAz  = norm180(g_pts[i].trueAz - norm360(g_pts[i].rawAz + eAz));
  errAlt = g_pts[i].trueAlt - (g_pts[i].rawAlt + eAlt);
}

static void applyModel(double rawAz, double rawAlt, double& azOut, double& altOut)
{
  double eAz, eAlt;
  modelErrors(rawAz, rawAlt, eAz, eAlt);
  azOut  = norm360(rawAz + eAz);
  altOut = rawAlt + eAlt;

  // Перевал через зенит/надир: ось высоты может уйти за ±90°, но труба смотрит
  // в ту же точку неба, что (азимут+180°, 180°−высота). Обрезать высоту на ±90°
  // нельзя: в зените RA/Dec вырождаются, и крест замирает - перестаёт
  // реагировать на вращение обеих осей, пока монтировку не вернут обратно.
  if (altOut > 90.0) {
    altOut = 180.0 - altOut;
    azOut  = norm360(azOut + 180.0);
  } else if (altOut < -90.0) {
    altOut = -180.0 - altOut;
    azOut  = norm360(azOut + 180.0);
  }
}

// ------------------------- астрономия/позиция -------------------------

static double julianCenturies(time_t t)
{
  double jd = 2440587.5 + (double)t / 86400.0;
  return (jd - 2451545.0) / 36525.0;
}

static double gmstRad(time_t t)
{
  double jd = 2440587.5 + (double)t / 86400.0;
  double d = jd - 2451545.0;
  double T = d / 36525.0;
  double deg = 280.46061837 + 360.98564736629 * d + 0.000387933 * T * T - T * T * T / 38710000.0;
  return norm360(deg) * DEG2RAD;
}

static void precessEquatorial(double& ra, double& dec, double T, bool inverse = false)
{
  const double arcsec = DEG2RAD / 3600.0;
  double zeta  = (2306.2181 + 0.30188 * T + 0.017998 * T * T) * T * arcsec;
  double z     = (2306.2181 + 1.09468 * T + 0.018203 * T * T) * T * arcsec;
  double theta = (2004.3109 - 0.42665 * T - 0.041833 * T * T) * T * arcsec;

  double a, b, c;
  if (inverse) {
    a = cos(dec) * sin(ra - z);
    b = cos(theta) * cos(dec) * cos(ra - z) + sin(theta) * sin(dec);
    c = -sin(theta) * cos(dec) * cos(ra - z) + cos(theta) * sin(dec);
    ra  = normRad(atan2(a, b) - zeta);
  } else {
    a = cos(dec) * sin(ra + zeta);
    b = cos(theta) * cos(dec) * cos(ra + zeta) - sin(theta) * sin(dec);
    c = sin(theta) * cos(dec) * cos(ra + zeta) + cos(theta) * sin(dec);
    ra  = normRad(atan2(a, b) + z);
  }
  dec = asin(clampd(c, -1.0, 1.0));
}

static void raDecToAltAz(double ra, double dec, double lat, double lst,
                         double& alt, double& az)
{
  double H = lst - ra;
  double sinAlt = sin(lat) * sin(dec) + cos(lat) * cos(dec) * cos(H);
  alt = asin(clampd(sinAlt, -1.0, 1.0));
  az  = atan2(-cos(dec) * sin(H), sin(dec) * cos(lat) - cos(dec) * sin(lat) * cos(H));
  az  = normRad(az);
}

static void altAzToRaDec(double alt, double az, double lat, double lst,
                         double& ra, double& dec)
{
  double sinDec = sin(lat) * sin(alt) + cos(lat) * cos(alt) * cos(az);
  dec = asin(clampd(sinDec, -1.0, 1.0));
  double H = atan2(-sin(az) * cos(alt), cos(lat) * sin(alt) - sin(lat) * cos(alt) * cos(az));
  ra = normRad(lst - H);
}

// текущая позиция монтировки в J2000 (радианы)
static void currentRaDec(double& raJ2000, double& decJ2000)
{
  time_t t = utcNow();
  double T = julianCenturies(t);
  double lst = gmstRad(t) + g_lonDeg * DEG2RAD;
  double az, alt;
  applyModel(g_encAz, g_encAlt, az, alt);
  double ra, dec;
  altAzToRaDec(alt * DEG2RAD, az * DEG2RAD, g_latDeg * DEG2RAD, lst, ra, dec);
  precessEquatorial(ra, dec, T, true);  // из даты в J2000
  raJ2000 = ra;
  decJ2000 = dec;
}

// --------------------------- форматирование ---------------------------

static void formatRa(double raRad, char* out)
{
  double hours = norm360(raRad * RAD2DEG) / 15.0;
  long total = lround(hours * 3600.0) % 86400;
  if (total < 0) total += 86400;
  sprintf(out, "%02d:%02d:%02d", (int)(total / 3600), (int)((total / 60) % 60), (int)(total % 60));
}

static void formatDec(double decRad, char* out)
{
  long signedTotal = lround(decRad * RAD2DEG * 3600.0);
  char sign = (signedTotal < 0) ? '-' : '+';
  long total = labs(signedTotal);
  sprintf(out, "%c%02d%c%02d:%02d", sign, (int)(total / 3600), (char)0xDF,
          (int)((total / 60) % 60), (int)(total % 60));
}

// ------------------------------ привязка -----------------------------

static void doSync()
{
  if (!g_targetRaValid || !g_targetDecValid) return;

  time_t t = utcNow();
  double T = julianCenturies(t);
  double lst = gmstRad(t) + g_lonDeg * DEG2RAD;

  double ra = g_targetRa, dec = g_targetDec;
  precessEquatorial(ra, dec, T);  // J2000 -> дата

  double az, alt;
  raDecToAltAz(ra, dec, g_latDeg * DEG2RAD, lst, alt, az);
  addAlignPoint(norm360(az * RAD2DEG), alt * RAD2DEG, g_encAz, g_encAlt);
}

// --------------------------- команды LX200 ---------------------------

static bool parseSetRa(const char* s)
{
  int h, m, sec;
  if (sscanf(s, "%d:%d:%d", &h, &m, &sec) != 3) return false;
  if (h < 0 || h > 23 || m < 0 || m > 59 || sec < 0 || sec > 59) return false;
  g_targetRa = (h + m / 60.0 + sec / 3600.0) * 15.0 * DEG2RAD;
  g_targetRaValid = true;
  return true;
}

static bool parseSetDec(const char* s)
{
  const char* p = s;
  int sign = 1;
  if (*p == '+') p++;
  else if (*p == '-') { sign = -1; p++; }
  else return false;

  if (!isdigit((unsigned char)*p)) return false;
  int d = 0;
  while (isdigit((unsigned char)*p)) { d = d * 10 + (*p - '0'); p++; }

  if (*p != '*' && (uint8_t)*p != 0xDF) return false;
  p++;

  if (!isdigit((unsigned char)*p)) return false;
  int m = 0;
  while (isdigit((unsigned char)*p)) { m = m * 10 + (*p - '0'); p++; }

  int sec = 0;
  if (*p == ':' || *p == '\'') {
    p++;
    if (!isdigit((unsigned char)*p)) return false;
    while (isdigit((unsigned char)*p)) { sec = sec * 10 + (*p - '0'); p++; }
  }
  if (*p != 0) return false;
  if (d > 90 || m > 59 || sec > 59) return false;

  double deg = d + m / 60.0 + sec / 3600.0;
  if (deg > 90.0) return false;
  g_targetDec = sign * deg * DEG2RAD;
  g_targetDecValid = true;
  return true;
}

static void handleLx200(Link& l, const char* cmd)
{
  if (!strcmp(cmd, ":GR")) {
    double ra, dec;
    currentRaDec(ra, dec);
    char b[16];
    formatRa(ra, b);
    l.io->print(b);
    l.io->print('#');
  } else if (!strcmp(cmd, ":GD")) {
    double ra, dec;
    currentRaDec(ra, dec);
    char b[16];
    formatDec(dec, b);
    l.io->print(b);
    l.io->print('#');
  } else if (!strncmp(cmd, ":Sr", 3)) {
    l.io->print(parseSetRa(cmd + 3) ? '1' : '0');
  } else if (!strncmp(cmd, ":Sd", 3)) {
    l.io->print(parseSetDec(cmd + 3) ? '1' : '0');
  } else if (!strcmp(cmd, ":CM")) {
    doSync();
    l.io->print('0');  // принимается и старыми, и новыми версиями Stellarium
  } else if (!strcmp(cmd, ":MS")) {
    l.io->print('0');  // push-to: наведение выполняет наблюдатель
  } else if (!strcmp(cmd, ":Q") || !strcmp(cmd, ":U")) {
    // нет ответа
  } else if (!strcmp(cmd, ":GVP")) {
    l.io->print("SV225 PushTo#");
  }
}

// ------------------------- текстовые команды -------------------------

static void printHelp(Link& l)
{
  l.io->println();
  l.io->println("Команды настройки (примеры):");
  l.io->println("  T 2026-09-17 18:00:00      - время UTC");
  l.io->println("  SLOT 1 55.0583 73.2950 Дача - слот места: широта (север +,");
  l.io->println("                               юг -), долгота (восток +, запад -),");
  l.io->println("                               название (необязательно)");
  l.io->println("  SLOTUSE 1                  - сделать слот 1 текущим местом");
  l.io->println("  SLOTCLEAR 1                - очистить слот 1");
  l.io->println("  AZDIR 1 | AZDIR -1         - направление оси азимута");
  l.io->println("  ALTDIR 1 | ALTDIR -1       - направление оси высоты");
  l.io->println("  AZZERO 0.0                 - смещение нуля энкодера азимута, градусы");
  l.io->println("  ALTZERO 0.0                - смещение нуля энкодера высоты, градусы");
  l.io->println("  STATUS                     - текущее состояние");
  l.io->println("  PTDEL 1 .. PTDEL 6         - удалить точку привязки (номера в STATUS)");
  l.io->println("  CLEAR                      - сбросить привязку (текущее положение = полюс мира)");
  l.io->println("  HELP                       - эта справка");
  l.io->println("Пока нет точек привязки, считается, что в момент включения");
  l.io->println("труба смотрела на полюс мира: крест в Stellarium сразу на");
  l.io->println("Полярной и следует за монтировкой; первый Sync заменяет это.");
  l.io->println("Хранится до 6 точек; Sync рядом с существующей (до ~5°)");
  l.io->println("обновляет её, а не вытесняет остальные.");
  l.io->println("Место хранится в трёх слотах (NVS), текущее место - активный");
  l.io->println("слот; слоты, названия и направление осей сохраняются в памяти");
  l.io->println("ESP32. Время после каждого включения задавайте командой T.");
}

static void printStatus(Link& l)
{
  char tbuf[24];
  formatTimeUtc(tbuf, sizeof(tbuf));
  l.io->print("Время UTC: ");
  l.io->print(tbuf);
  l.io->println(g_timeSet ? " (задано командой T)" : " (время не установлено, задайте T)");

  if (g_activeSlot >= 0) {
    const SiteSlot& s = g_slots[g_activeSlot];
    l.io->print("Место: слот ");
    l.io->print(g_activeSlot + 1);
    if (s.name[0]) {
      l.io->print(" (");
      l.io->print(s.name);
      l.io->print(")");
    }
    l.io->print(", широта ");
    l.io->print(g_latDeg, 6);
    l.io->print(", долгота ");
    l.io->println(g_lonDeg, 6);
  } else {
    l.io->println("Место: НЕ ЗАДАНО (сохраните слот и выполните SLOTUSE)");
  }

  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    l.io->print("Слот ");
    l.io->print(i + 1);
    l.io->print(": ");
    if (!g_slots[i].set) {
      l.io->println("пусто");
    } else {
      l.io->print(g_slots[i].name[0] ? g_slots[i].name : "(без имени)");
      l.io->print(" - ");
      l.io->print(g_slots[i].lat, 6);
      l.io->print(", ");
      l.io->print(g_slots[i].lon, 6);
      if (g_activeSlot == (int8_t)i) l.io->print(" [активен]");
      l.io->println();
    }
  }

  l.io->print("Направление осей: AZ ");
  l.io->print((int)g_azDir);
  l.io->print(", ALT ");
  l.io->println((int)g_altDir);

  l.io->print("Нули энкодеров: AZ ");
  l.io->print(g_azZeroDeg, 3);
  l.io->print("°, ALT ");
  l.io->print(g_altZeroDeg, 3);
  l.io->println("°");

  l.io->print("Энкодер AZ: raw ");
  l.io->print(g_rawAz);
  l.io->print(" (");
  l.io->print((double)g_rawAz * 360.0 / ENCODER_STEPS, 2);
  l.io->print("°), после настройки: ");
  l.io->print(g_encAz, 3);
  l.io->println("°");

  l.io->print("Энкодер ALT: raw ");
  l.io->print(g_rawAlt);
  l.io->print(" (");
  l.io->print((double)g_rawAlt * 360.0 / ENCODER_STEPS, 2);
  l.io->print("°), после настройки: ");
  l.io->print(g_encAlt, 3);
  l.io->println("°");

  double az, alt;
  applyModel(g_encAz, g_encAlt, az, alt);
  l.io->print("Углы монтировки: AZ ");
  l.io->print(az, 3);
  l.io->print("°, ALT ");
  l.io->print(alt, 3);
  l.io->println("°");

  double ra, dec;
  currentRaDec(ra, dec);
  char b[16];
  l.io->print("LX200 (J2000): RA ");
  formatRa(ra, b);
  l.io->print(b);
  l.io->print(", Dec ");
  formatDec(dec, b);
  for (const char* p = b; *p; p++) {
    if ((uint8_t)*p == 0xDF) l.io->print("°");
    else l.io->print(*p);
  }
  l.io->println();

  l.io->print("Точек привязки: ");
  l.io->print(g_nPts);
  if (g_model.n == 0) {
    l.io->print(" (нет привязки: текущее положение - полюс мира, AZ=0°, ALT=");
    l.io->print(g_latDeg, 3);
    l.io->println("°)");
  } else {
    l.io->println();
  }
  for (uint8_t i = 0; i < g_nPts; i++) {
    double rAz, rAlt;
    pointResidualDeg(i, rAz, rAlt);
    l.io->print("  ");
    l.io->print(i + 1);
    l.io->print(": оси AZ ");
    l.io->print(g_pts[i].rawAz, 2);
    l.io->print("°, ALT ");
    l.io->print(g_pts[i].rawAlt, 2);
    l.io->print("° -> AZ ");
    l.io->print(norm360(g_pts[i].trueAz), 2);
    l.io->print("°, ALT ");
    l.io->print(g_pts[i].trueAlt, 2);
    l.io->print("°, невязка ");
    l.io->print(rAz * 60.0, 1);
    l.io->print("'/");
    l.io->print(rAlt * 60.0, 1);
    l.io->println("'");
  }
}

// имя слота: до cap-1 байт, целыми UTF-8-символами, без управляющих байтов
static void copySlotName(char* dst, const char* src, size_t cap)
{
  size_t w = 0;
  while (*src && w + 1 < cap) {
    uint8_t c = (uint8_t)*src;
    if (c < 0x20 || c == 0x7F) { src++; continue; }
    size_t len = 1;
    if ((c & 0xE0) == 0xC0) len = 2;
    else if ((c & 0xF0) == 0xE0) len = 3;
    else if ((c & 0xF8) == 0xF0) len = 4;
    if (w + len + 1 > cap) break;
    bool ok = true;
    for (size_t k = 1; k < len; k++) {
      if (((uint8_t)src[k] & 0xC0) != 0x80) { ok = false; break; }
    }
    if (!ok) { src++; continue; }
    for (size_t k = 0; k < len; k++) dst[w++] = src[k];
    src += len;
  }
  dst[w] = 0;
}

static void handleText(Link& l, char* line)
{
  char kw[16];
  uint8_t i = 0;
  while (line[i] && line[i] != ' ' && i < sizeof(kw) - 1) {
    kw[i] = (char)toupper((unsigned char)line[i]);
    i++;
  }
  kw[i] = 0;
  const char* arg = line + i;
  while (*arg == ' ') arg++;

  if (!strcmp(kw, "T")) {
    int y, mo, d, h, mi, s;
    if (sscanf(arg, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) == 6 &&
        y >= 2000 && y <= 2099 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31 &&
        h >= 0 && h < 24 && mi >= 0 && mi < 60 && s >= 0 && s < 60) {
      setUtc(daysFromCivil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s);
      g_timeSet = true;
      char tbuf[24];
      formatTimeUtc(tbuf, sizeof(tbuf));
      l.io->print("OK: время UTC ");
      l.io->println(tbuf);
    } else {
      l.io->println("Ошибка формата. Пример: T 2026-09-17 18:00:00 (UTC)");
    }
  } else if (!strcmp(kw, "SLOT")) {
    int idx = 0, pos = 0;
    double la = 0.0, lo = 0.0;
    if (sscanf(arg, "%d %lf %lf%n", &idx, &la, &lo, &pos) == 3 &&
        idx >= 1 && idx <= (int)MAX_SLOTS &&
        la >= -90.0 && la <= 90.0 && lo >= -180.0 && lo <= 180.0) {
      uint8_t si = (uint8_t)(idx - 1);
      const char* name = arg + pos;
      while (*name == ' ') name++;
      g_slots[si].set = true;
      g_slots[si].lat = la;
      g_slots[si].lon = lo;
      copySlotName(g_slots[si].name, name, sizeof(g_slots[si].name));
      saveSlotData(si);
      if (g_activeSlot == (int8_t)si) {
        g_latDeg = la;  // текущее место - данные активного слота
        g_lonDeg = lo;
      }
      l.io->print("OK: слот ");
      l.io->print(idx);
      l.io->print(" сохранён");
      if (g_slots[si].name[0]) {
        l.io->print(": ");
        l.io->print(g_slots[si].name);
      }
      l.io->println();
    } else {
      l.io->println("Ошибка. Пример: SLOT 1 55.0583 73.2950 Дача");
    }
  } else if (!strcmp(kw, "SLOTUSE")) {
    int idx = 0;
    if (sscanf(arg, "%d", &idx) == 1 && idx >= 1 && idx <= (int)MAX_SLOTS &&
        g_slots[idx - 1].set) {
      uint8_t si = (uint8_t)(idx - 1);
      selectSlot(si);
      l.io->print("OK: место - слот ");
      l.io->print(idx);
      if (g_slots[si].name[0]) {
        l.io->print(" (");
        l.io->print(g_slots[si].name);
        l.io->print(")");
      }
      l.io->println();
    } else {
      l.io->println("Ошибка. Пример: SLOTUSE 1 (слот должен быть сохранён)");
    }
  } else if (!strcmp(kw, "SLOTCLEAR")) {
    int idx = 0;
    if (sscanf(arg, "%d", &idx) == 1 && idx >= 1 && idx <= (int)MAX_SLOTS) {
      clearSlotData((uint8_t)(idx - 1));
      l.io->print("OK: слот ");
      l.io->print(idx);
      l.io->println(" очищен");
    } else {
      l.io->println("Ошибка. Пример: SLOTCLEAR 1");
    }
  } else if (!strcmp(kw, "AZDIR") || !strcmp(kw, "ALTDIR")) {
    int v;
    if (sscanf(arg, "%d", &v) == 1 && (v == 1 || v == -1)) {
      if (!strcmp(kw, "AZDIR")) g_azDir = (int8_t)v;
      else g_altDir = (int8_t)v;
      saveConfig();
      l.io->print("OK: ");
      l.io->print(kw);
      l.io->print(" ");
      l.io->println(v);
    } else {
      l.io->println("Ошибка. Пример: AZDIR 1 или AZDIR -1");
    }
  } else if (!strcmp(kw, "AZZERO") || !strcmp(kw, "ALTZERO")) {
    double v;
    if (sscanf(arg, "%lf", &v) == 1) {
      if (!strcmp(kw, "AZZERO")) g_azZeroDeg = norm360(v);
      else g_altZeroDeg = norm180(v);
      readEncoders();
      saveConfig();
      l.io->print("OK: ");
      l.io->print(kw);
      l.io->print(" ");
      l.io->println(v, 3);
    } else {
      l.io->println("Ошибка. Пример: AZZERO 0.0");
    }
  } else if (!strcmp(kw, "CLEAR")) {
    g_nPts = 0;
    rebuildModel();
    readEncoders();
    setPolarStart();
    l.io->println("OK: привязка сброшена (текущее положение - полюс мира)");
  } else if (!strcmp(kw, "PTDEL")) {
    int idx = 0;
    if (sscanf(arg, "%d", &idx) == 1 && idx >= 1 && idx <= (int)g_nPts) {
      removeAlignPoint((uint8_t)(idx - 1));
      l.io->print("OK: точка ");
      l.io->print(idx);
      l.io->print(" удалена, осталось точек: ");
      l.io->println(g_nPts);
    } else {
      l.io->println("Ошибка. Пример: PTDEL 1 (номер точки смотрите в STATUS)");
    }
  } else if (!strcmp(kw, "STATUS")) {
    printStatus(l);
  } else if (!strcmp(kw, "HELP") || !strcmp(kw, "?")) {
    printHelp(l);
  } else if (kw[0] != 0) {
    l.io->println("Неизвестная команда. HELP - список команд.");
  }
}

// --------------------------- веб-интерфейс ---------------------------
// ESP32 поднимает точку доступа Wi-Fi и отдаёт страницу настроек
// (https://192.168.4.1/). Команды со страницы идут через тот же handleText(),
// что и текстовые команды serial - логика не дублируется.

#if WEB_UI

// приёмник вывода handleText()/printStatus(): собирает ответ в строку
class StringStream : public Stream {
public:
  size_t write(uint8_t c) override { out += (char)c; return 1; }
  size_t write(const uint8_t* data, size_t len) override {
    for (size_t i = 0; i < len; i++) out += (char)data[i];
    return len;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  const String& str() const { return out; }
private:
  String out;
};

static httpd_handle_t g_web = NULL;

// строка в JSON: экранируем кавычки и обратные слэши (имя слота задаёт пользователь)
static void appendJsonStr(String& j, const char* s)
{
  for (const char* p = s; *p; p++) {
    if (*p == '"' || *p == '\\') j += '\\';
    j += *p;
  }
}

static String buildStatusJson()
{
  char tbuf[24], rab[16], decb[16];
  formatTimeUtc(tbuf, sizeof(tbuf));

  double az, alt;
  applyModel(g_encAz, g_encAlt, az, alt);
  double ra, dec;
  currentRaDec(ra, dec);
  formatRa(ra, rab);
  formatDec(dec, decb);
  String decU(decb);
  decU.replace("\xDF", "\xC2\xB0");  // 0xDF -> UTF-8 "°"

  String j = "{";
  j += "\"timeUtc\":\"";   j += tbuf;   j += "\",";
  j += "\"timeSet\":";     j += (g_timeSet ? "true" : "false"); j += ",";
  j += "\"lat\":";         j += String(g_latDeg, 6);  j += ",";
  j += "\"lon\":";         j += String(g_lonDeg, 6);  j += ",";
  j += "\"geoSet\":";      j += (g_activeSlot >= 0 ? "true" : "false");  j += ",";
  j += "\"activeSlot\":";  j += (int)g_activeSlot;  j += ",";
  j += "\"siteName\":\"";  appendJsonStr(j, g_activeSlot >= 0 ? g_slots[g_activeSlot].name : "");  j += "\",";
  j += "\"azDir\":";       j += (int)g_azDir;  j += ",";
  j += "\"altDir\":";      j += (int)g_altDir; j += ",";
  j += "\"azZero\":";      j += String(g_azZeroDeg, 3);  j += ",";
  j += "\"altZero\":";     j += String(g_altZeroDeg, 3); j += ",";
  j += "\"rawAz\":";       j += g_rawAz;  j += ",";
  j += "\"rawAlt\":";      j += g_rawAlt; j += ",";
  j += "\"rawAzDeg\":";    j += String((double)g_rawAz * 360.0 / ENCODER_STEPS, 3);  j += ",";
  j += "\"rawAltDeg\":";   j += String((double)g_rawAlt * 360.0 / ENCODER_STEPS, 3); j += ",";
  j += "\"encAz\":";       j += String(g_encAz, 3);  j += ",";
  j += "\"encAlt\":";      j += String(g_encAlt, 3); j += ",";
  j += "\"mountAz\":";     j += String(az, 3);  j += ",";
  j += "\"mountAlt\":";    j += String(alt, 3); j += ",";
  j += "\"raStr\":\"";     j += rab;  j += "\",";
  j += "\"decStr\":\"";    j += decU; j += "\",";
  j += "\"nPts\":";        j += g_nPts;  j += ",";
  j += "\"pts\":[";
  for (uint8_t i = 0; i < g_nPts; i++) {
    if (i) j += ",";
    double rAz, rAlt;
    pointResidualDeg(i, rAz, rAlt);
    j += "{\"rawAz\":";   j += String(g_pts[i].rawAz, 2);
    j += ",\"rawAlt\":";  j += String(g_pts[i].rawAlt, 2);
    j += ",\"trueAz\":";  j += String(norm360(g_pts[i].trueAz), 2);
    j += ",\"trueAlt\":"; j += String(g_pts[i].trueAlt, 2);
    j += ",\"errAz\":";   j += String(rAz * 60.0, 1);
    j += ",\"errAlt\":";  j += String(rAlt * 60.0, 1);
    j += "}";
  }
  j += "],";
  j += "\"polar\":";       j += (g_model.n == 0 ? "true" : "false");  j += ",";
  j += "\"slots\":[";
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (i) j += ",";
    j += "{\"set\":";
    j += (g_slots[i].set ? "true" : "false");
    j += ",\"name\":\"";
    appendJsonStr(j, g_slots[i].name);
    j += "\",\"lat\":";
    j += String(g_slots[i].lat, 6);
    j += ",\"lon\":";
    j += String(g_slots[i].lon, 6);
    j += "}";
  }
  j += "]";
  j += "}";
  return j;
}

static const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SV225 - настройка</title>
<style>
body{font-family:system-ui,sans-serif;background:#12151a;color:#e6e9ee;margin:0;padding:12px}
h1{font-size:1.15rem;margin:4px 0 12px}
.card{background:#1c2027;border-radius:10px;padding:12px;margin-bottom:12px}
h2{font-size:1rem;margin:0 0 8px;color:#9fb3c8}
table{width:100%;border-collapse:collapse;font-size:.9rem}
td{padding:3px 0;vertical-align:top}
td:first-child{color:#9fb3c8;white-space:nowrap;padding-right:8px}
input,select{background:#12151a;color:#e6e9ee;border:1px solid #333a45;border-radius:6px;padding:6px;font-size:.95rem;box-sizing:border-box;width:100%;margin:2px 0 8px}
input[type=checkbox]{width:auto;margin:0 6px 0 0;vertical-align:middle}
label{font-size:.95rem}
button{background:#2b3648;color:#e6e9ee;border:1px solid #3a4a63;border-radius:6px;padding:8px 10px;font-size:.9rem;margin:2px 4px 2px 0;cursor:pointer}
button:active{background:#3a4a63}
.row{display:flex;gap:8px}.row>div{flex:1}
.hint{font-size:.8rem;color:#8a97a8;margin:8px 0 0}
.warn{color:#ff6b6b}
.slot{border:1px solid #333a45;border-radius:8px;padding:10px;margin-bottom:10px}
.slot.active{border-color:#4caf78;background:#18251d}
.slot-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.badge{font-size:.75rem;padding:2px 8px;border-radius:10px;background:#333a45;color:#9fb3c8}
.slot.active .badge{background:#4caf78;color:#10151a}
.pts{font-size:.85rem;margin-bottom:8px}
.pts button{padding:4px 8px;margin:0}
.pts td{padding:3px 6px 3px 0}
pre{white-space:pre-wrap;word-break:break-word;font-size:.85rem;margin:0;color:#b7e5a1}
</style>
</head>
<body>
<h1>SV225 Push-To - Stellarium</h1>

<div class="card">
<h2>Состояние</h2>
<table>
<tr><td>Время UTC</td><td id="s-time">-</td></tr>
<tr><td>Место</td><td id="s-geo">-</td></tr>
<tr><td>Оси</td><td id="s-dir">-</td></tr>
<tr><td>Нули</td><td id="s-zero">-</td></tr>
<tr><td>Энкодер AZ</td><td id="s-az">-</td></tr>
<tr><td>Энкодер ALT</td><td id="s-alt">-</td></tr>
<tr><td>Монтировка</td><td id="s-mount">-</td></tr>
<tr><td>J2000</td><td id="s-radec">-</td></tr>
<tr><td>Точек привязки</td><td id="s-npts">-</td></tr>
</table>
</div>

<div class="card">
<h2>Время UTC</h2>
<input type="datetime-local" id="dt" step="1">
<button onclick="timeNow()">Взять время с этого устройства</button>
<button onclick="applyTime()">Применить</button>
<p class="hint">Поле и ввод - в UTC. Кнопка «Взять время с этого устройства»
подставляет в поле текущее время браузера; чтобы оно попало в ESP32,
нажмите «Применить».</p>
</div>

<div class="card">
<h2>Место наблюдения</h2>
<div class="slot" id="slot0">
<div class="slot-head"><b>Слот 1</b><span class="badge" id="s0state">пусто</span></div>
<input type="text" id="s0name" maxlength="20" placeholder="Название (необязательно)">
<div class="row">
<div>Широта (север +)<input type="number" id="s0lat" step="0.000001"></div>
<div>Долгота (восток +)<input type="number" id="s0lon" step="0.000001"></div>
</div>
<button onclick="gpsSlot(0)">Взять GPS</button>
<button onclick="saveSlot(0)">Сохранить</button>
<button onclick="useSlot(0)">Использовать</button>
<button onclick="clearSlot(0)">Очистить</button>
</div>
<div class="slot" id="slot1">
<div class="slot-head"><b>Слот 2</b><span class="badge" id="s1state">пусто</span></div>
<input type="text" id="s1name" maxlength="20" placeholder="Название (необязательно)">
<div class="row">
<div>Широта (север +)<input type="number" id="s1lat" step="0.000001"></div>
<div>Долгота (восток +)<input type="number" id="s1lon" step="0.000001"></div>
</div>
<button onclick="gpsSlot(1)">Взять GPS</button>
<button onclick="saveSlot(1)">Сохранить</button>
<button onclick="useSlot(1)">Использовать</button>
<button onclick="clearSlot(1)">Очистить</button>
</div>
<div class="slot" id="slot2">
<div class="slot-head"><b>Слот 3</b><span class="badge" id="s2state">пусто</span></div>
<input type="text" id="s2name" maxlength="20" placeholder="Название (необязательно)">
<div class="row">
<div>Широта (север +)<input type="number" id="s2lat" step="0.000001"></div>
<div>Долгота (восток +)<input type="number" id="s2lon" step="0.000001"></div>
</div>
<button onclick="gpsSlot(2)">Взять GPS</button>
<button onclick="saveSlot(2)">Сохранить</button>
<button onclick="useSlot(2)">Использовать</button>
<button onclick="clearSlot(2)">Очистить</button>
</div>
<p class="hint">Место хранится в трёх слотах в памяти ESP32. «Сохранить»
записывает слот, «Использовать» делает его текущим местом (активный слот
подсвечен). «Взять GPS» подставляет координаты браузера в поля слота -
после этого нажмите «Сохранить» или «Использовать»; разрешите доступ к
геопозиции.</p>
</div>

<div class="card">
<h2>Оси и нули энкодеров</h2>
<div class="row">
<div><label><input type="checkbox" id="azinv"> Инверсия оси AZM</label></div>
<div><label><input type="checkbox" id="altinv"> Инверсия оси ALT</label></div>
</div>
<div class="row">
<div>AZZERO, °<input type="number" id="azzero" step="0.001"></div>
<div>ALTZERO, °<input type="number" id="altzero" step="0.001"></div>
</div>
<button onclick="zeroAxis('az')">AZ = 0 сейчас</button>
<button onclick="zeroAxis('alt')">ALT = 0 сейчас</button><br>
<button onclick="saveAxes()">Сохранить оси</button>
</div>

<div class="card">
<h2>Привязка</h2>
<table class="pts">
<thead><tr><td>Точка</td><td>Углы осей</td><td>Истинные (по звезде)</td><td>Невязка</td><td></td></tr></thead>
<tbody id="ptbody"><tr><td colspan="5">нет точек</td></tr></tbody>
</table>
<button onclick="cmd('CLEAR')">Сбросить привязку</button>
<button onclick="cmd('STATUS')">Статус текстом</button>
<button onclick="cmd('HELP')">Справка</button>
<p class="hint">Точки появляются после Sync из Stellarium. «Углы осей» - то, что
показывают энкодеры сейчас (как физически повёрнуты оси). «Истинные» - настоящие
координаты звезды, которую вы синхронизировали: их и передал Stellarium.
Невязка - ошибка модели в этой точке в угловых минутах: 0' - модель проходит
точно через точку, чем больше - тем сильнее точка выбивается (кривой Sync,
ошибка наведения, гнутие). Повторный Sync рядом с уже привязанной звездой
(до 2° по осям) обновляет её точку, а не добавляет дубликат; когда точек уже 6,
новый Sync в стороне заменяет самую старую. «Сбросить привязку» удаляет
все точки.</p>
</div>

<div class="card">
<h2>Ответ</h2>
<pre id="log">-</pre>
</div>

<script>
const $ = id => document.getElementById(id);
let st = null, first = true;

async function api(path, data){
  const r = await fetch(path, {method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:new URLSearchParams(data)});
  return await r.text();
}
function log(s){ $('log').textContent = s || 'OK'; }
async function cmd(c){ log(await api('/api/cmd', {cmd:c})); refresh(); }
function setVal(id, v){
  const e = $(id);
  if (!e.dataset.dirty && document.activeElement !== e) e.value = v;
}
function setChk(id, v){
  const e = $(id);
  if (!e.dataset.dirty && document.activeElement !== e) e.checked = v;
}
function p2(n){ return String(n).padStart(2,'0'); }
function utcStr(d){
  return d.getUTCFullYear()+'-'+p2(d.getUTCMonth()+1)+'-'+p2(d.getUTCDate())+'T'+
         p2(d.getUTCHours())+':'+p2(d.getUTCMinutes())+':'+p2(d.getUTCSeconds());
}
function clearDirty(ids){ ids.forEach(id => { delete $(id).dataset.dirty; }); }
function slotIds(i){ return ['s'+i+'name', 's'+i+'lat', 's'+i+'lon']; }
function refreshSlots(s){
  s.slots.forEach((sl, i) => {
    setVal('s'+i+'name', sl.set ? sl.name : '');
    setVal('s'+i+'lat', sl.set ? sl.lat : '');
    setVal('s'+i+'lon', sl.set ? sl.lon : '');
    const card = $('slot'+i), badge = $('s'+i+'state');
    if (i === s.activeSlot){
      card.classList.add('active');
      badge.textContent = 'используется';
    } else {
      card.classList.remove('active');
      badge.textContent = sl.set ? 'сохранён' : 'пусто';
    }
  });
}
function refreshPts(s){
  const b = $('ptbody');
  if (!s.pts || !s.pts.length){
    b.innerHTML = '<tr><td colspan="5">нет точек</td></tr>';
    return;
  }
  b.innerHTML = s.pts.map((p, i) =>
    '<tr><td>'+(i+1)+'</td>'+
    '<td>'+p.rawAz.toFixed(2)+'° / '+p.rawAlt.toFixed(2)+'°</td>'+
    '<td>'+p.trueAz.toFixed(2)+'° / '+p.trueAlt.toFixed(2)+'°</td>'+
    '<td>'+p.errAz.toFixed(1)+"' / "+p.errAlt.toFixed(1)+"'</td>"+
    '<td><button onclick="delPt('+(i+1)+')">Удалить</button></td></tr>').join('');
}
async function delPt(n){
  if (!confirm('Удалить точку '+n+'?')) return;
  log(await api('/api/cmd', {cmd:'PTDEL '+n}));
  refresh();
}
async function refresh(){
  let s;
  try { s = await (await fetch('/api/status')).json(); }
  catch(e) { return; }
  st = s;
  $('s-time').innerHTML = s.timeUtc + (s.timeSet ? '' : ' <span class="warn">(время не установлено)</span>');
  if (s.geoSet){
    const nm = s.siteName || ('Слот '+(s.activeSlot+1));
    $('s-geo').textContent = nm+' ('+s.lat.toFixed(6)+', '+s.lon.toFixed(6)+')';
  } else {
    $('s-geo').textContent = 'НЕ ЗАДАНО - сохраните слот и нажмите «Использовать»';
  }
  $('s-dir').textContent = 'AZ '+(s.azDir<0?'инверсия':'норма')+', ALT '+(s.altDir<0?'инверсия':'норма');
  $('s-zero').textContent = 'AZ '+s.azZero.toFixed(3)+'°, ALT '+s.altZero.toFixed(3)+'°';
  $('s-az').textContent = s.rawAz+' ('+s.rawAzDeg.toFixed(2)+'°) -> '+s.encAz.toFixed(3)+'°';
  $('s-alt').textContent = s.rawAlt+' ('+s.rawAltDeg.toFixed(2)+'°) -> '+s.encAlt.toFixed(3)+'°';
  $('s-mount').textContent = 'AZ '+s.mountAz.toFixed(3)+'°, ALT '+s.mountAlt.toFixed(3)+'°';
  $('s-radec').textContent = s.raStr+'   '+s.decStr;
  $('s-npts').textContent = s.nPts + (s.polar ? ' (нет привязки: старт от полюса)' : '');
  if (first){ first = false; setVal('dt', utcStr(new Date())); }
  setVal('azzero', s.azZero.toFixed(3));
  setVal('altzero', s.altZero.toFixed(3));
  setChk('azinv', s.azDir < 0);
  setChk('altinv', s.altDir < 0);
  refreshSlots(s);
  refreshPts(s);
}
function timeNow(){
  const e = $('dt');
  e.value = utcStr(new Date());
  delete e.dataset.dirty;
  log('Время подставлено. Нажмите «Применить», чтобы сохранить.');
}
async function applyTime(){
  let v = $('dt').value.replace('T',' ').trim();
  if (v.length === 16) v += ':00';
  if (!v){ log('Укажите время UTC.'); return; }
  clearDirty(['dt']);
  await cmd('T '+v);
}
function slotCoords(i){
  const la = parseFloat($('s'+i+'lat').value);
  const lo = parseFloat($('s'+i+'lon').value);
  if (!isFinite(la) || la < -90 || la > 90){ log('Укажите широту от -90 до 90.'); return null; }
  if (!isFinite(lo) || lo < -180 || lo > 180){ log('Укажите долготу от -180 до 180.'); return null; }
  return {la:la, lo:lo};
}
function slotCmd(i, c){
  const name = $('s'+i+'name').value.trim();
  return 'SLOT '+(i+1)+' '+c.la.toFixed(6)+' '+c.lo.toFixed(6)+(name ? ' '+name : '');
}
async function saveSlot(i){
  const c = slotCoords(i);
  if (!c) return;
  clearDirty(slotIds(i));
  log(await api('/api/cmd', {cmd: slotCmd(i, c)}));
  refresh();
}
async function useSlot(i){
  const c = slotCoords(i);
  if (!c) return;
  clearDirty(slotIds(i));
  let r = await api('/api/cmd', {cmd: slotCmd(i, c)});
  r += await api('/api/cmd', {cmd:'SLOTUSE '+(i+1)});
  log(r);
  refresh();
}
async function clearSlot(i){
  if (!confirm('Очистить слот '+(i+1)+'?')) return;
  clearDirty(slotIds(i));
  log(await api('/api/cmd', {cmd:'SLOTCLEAR '+(i+1)}));
  refresh();
}
function gpsSlot(i){
  if (!navigator.geolocation){ log('Браузер не поддерживает геолокацию.'); return; }
  log('Запрашиваю GPS-координаты... разрешите доступ к геопозиции.');
  navigator.geolocation.getCurrentPosition(p => {
    const la = p.coords.latitude.toFixed(6);
    const lo = p.coords.longitude.toFixed(6);
    $('s'+i+'lat').value = la;
    $('s'+i+'lat').dataset.dirty = '1';
    $('s'+i+'lon').value = lo;
    $('s'+i+'lon').dataset.dirty = '1';
    log('Координаты подставлены в слот '+(i+1)+' (±'+(p.coords.accuracy||0).toFixed(0)+
        ' м). Нажмите «Сохранить» или «Использовать».');
  }, e => {
    log('Не удалось получить координаты: '+e.message+
        '\nРазрешите доступ к геопозиции в настройках браузера.');
  }, {enableHighAccuracy:true, timeout:15000, maximumAge:0});
}
function zeroAxis(axis){
  if (!st) return;
  if (axis === 'az') cmd('AZZERO '+(-st.azDir*st.rawAzDeg).toFixed(3));
  else cmd('ALTZERO '+(-st.altDir*st.rawAltDeg).toFixed(3));
}
async function saveAxes(){
  const list = ['AZDIR '+($('azinv').checked?-1:1), 'ALTDIR '+($('altinv').checked?-1:1),
                'AZZERO '+$('azzero').value, 'ALTZERO '+$('altzero').value];
  let r = '';
  for (const c of list) r += await api('/api/cmd', {cmd:c});
  clearDirty(['azinv','altinv','azzero','altzero']);
  log(r); refresh();
}
document.querySelectorAll('input,select').forEach(e =>
  e.addEventListener('input', () => { e.dataset.dirty = '1'; }));
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
)HTML";

// ------------------------- обработчики страницы -------------------------
// Всё общение со страницей идёт по HTTPS: браузеры отдают GPS
// (navigator.geolocation) только в защищённом контексте. При входе браузер
// предупредит о самоподписанном сертификате («Дополнительно» -> «Перейти»).
// Команды со страницы не переписываются: они идут в handleText() через
// StringStream, как и на serial.

static esp_err_t handleWebRoot(httpd_req_t* req)
{
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleWebStatus(httpd_req_t* req)
{
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  return httpd_resp_send(req, buildStatusJson().c_str(), HTTPD_RESP_USE_STRLEN);
}

// тело запроса x-www-form-urlencoded (поле cmd)
static void urlDecode(char* s)
{
  char* w = s;
  for (char* p = s; *p; p++) {
    if (*p == '+') {
      *w++ = ' ';
    } else if (*p == '%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])) {
      int hi = isdigit((unsigned char)p[1]) ? p[1] - '0' : toupper((unsigned char)p[1]) - 'A' + 10;
      int lo = isdigit((unsigned char)p[2]) ? p[2] - '0' : toupper((unsigned char)p[2]) - 'A' + 10;
      *w++ = (char)((hi << 4) | lo);
      p += 2;
    } else {
      *w++ = *p;
    }
  }
  *w = 0;
}

static esp_err_t handleWebCmd(httpd_req_t* req)
{
  char body[CMD_BUF_SIZE + 16];
  int total = req->content_len;
  if (total <= 0 || total >= (int)sizeof(body)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad request");
    return ESP_FAIL;
  }
  int len = 0;
  while (len < total) {
    int n = httpd_req_recv(req, body + len, total - len);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (n <= 0) {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
      return ESP_FAIL;
    }
    len += n;
  }
  body[len] = 0;

  const char* c = strstr(body, "cmd=");
  if (!c) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no cmd");
    return ESP_FAIL;
  }
  c += 4;
  char* amp = strchr(c, '&');
  if (amp) *amp = 0;
  urlDecode((char*)c);

  StringStream ss;
  if (*c) {
    Link l;
    l.io = &ss;
    l.len = 0;
    l.mode = 0;
    l.lastMs = millis();
    handleText(l, (char*)c);
  }
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  return httpd_resp_send(req, ss.str().c_str(), HTTPD_RESP_USE_STRLEN);
}

// http://192.168.4.1/... -> https://192.168.4.1/
static esp_err_t handleHttpRedirect(httpd_req_t* req)
{
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "https://192.168.4.1/");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static void addUri(httpd_handle_t srv, const char* uri, httpd_method_t method,
                   esp_err_t (*handler)(httpd_req_t*))
{
  httpd_uri_t u = {};
  u.uri = uri;
  u.method = method;
  u.handler = handler;
  esp_err_t err = httpd_register_uri_handler(srv, &u);
  if (err != ESP_OK) Serial.printf("Не зарегистрирован %s: %s\n", uri, esp_err_to_name(err));
}

static void registerWebHandlers(httpd_handle_t srv)
{
  addUri(srv, "/",           HTTP_GET,  handleWebRoot);
  addUri(srv, "/index.html", HTTP_GET,  handleWebRoot);
  addUri(srv, "/api/status", HTTP_GET,  handleWebStatus);
  addUri(srv, "/api/cmd",    HTTP_POST, handleWebCmd);
}

// --------------------------- HTTPS-сервер ---------------------------
// Весь веб-интерфейс отдаётся по HTTPS с самоподписанным сертификатом
// (браузеры разрешают GPS только в защищённом контексте, а заодно это
// защищает настройки в локальной сети). На 80 порту - только редирект.
//
// Сертификат локальный (SAN IP 192.168.4.1, срок до 2056 г.), приватный
// ключ секретом не является: устройство отдаёт в своей сети только эту
// страницу. Перегенерировать при необходимости:
//   openssl ecparam -genkey -name prime256v1 -noout -out sv225_key.pem
//   openssl req -x509 -new -key sv225_key.pem -days 10957 \
//     -subj "/C=RU/O=SV225 PushTo/CN=192.168.4.1" -sha256 \
//     -addext "subjectAltName=IP:192.168.4.1,IP:127.0.0.1" -out sv225_cert.pem

static const char SV225_CERT_PEM[] = R"CERT(-----BEGIN CERTIFICATE-----
MIIB4jCCAYigAwIBAgIUM4+EWRkruSJU2Bf2oDX8vjKY9lwwCgYIKoZIzj0EAwIw
OjELMAkGA1UEBhMCUlUxFTATBgNVBAoMDFNWMjI1IFB1c2hUbzEUMBIGA1UEAwwL
MTkyLjE2OC40LjEwIBcNMjYwOTE3MDg0MTQ3WhgPMjA1NjA5MTYwODQxNDdaMDox
CzAJBgNVBAYTAlJVMRUwEwYDVQQKDAxTVjIyNSBQdXNoVG8xFDASBgNVBAMMCzE5
Mi4xNjguNC4xMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEOjqBuOndV77bwT6p
naxUN/u6x5pZlLNHqJlgThjxev+5b06ez9EvmZ6cNGt3BNQ/oEqEYorEWuRNC3Mt
qWttDqNqMGgwHQYDVR0OBBYEFLtFI/aGpdckL8elsYbWVxK5LfGgMB8GA1UdIwQY
MBaAFLtFI/aGpdckL8elsYbWVxK5LfGgMA8GA1UdEwEB/wQFMAMBAf8wFQYDVR0R
BA4wDIcEwKgEAYcEfwAAATAKBggqhkjOPQQDAgNIADBFAiA0t3/PiAVB7psjLU3+
BnmUTyiAgrcA+1ybJmUjNne2FQIhAL78oRLNCja04XvZt/99OuzQw4AyWoVLgYyz
V2tNFgEp
-----END CERTIFICATE-----
)CERT";

static const char SV225_KEY_PEM[] = R"KEY(-----BEGIN EC PRIVATE KEY-----
MHcCAQEEIMA8yBTyF30UoZ1qOKXg/f+iohk24WIErCZssCUEFaEVoAoGCCqGSM49
AwEHoUQDQgAEOjqBuOndV77bwT6pnaxUN/u6x5pZlLNHqJlgThjxev+5b06ez9Ev
mZ6cNGt3BNQ/oEqEYorEWuRNC3MtqWttDg==
-----END EC PRIVATE KEY-----
)KEY";

static bool g_https = false;

static void startWebServer()
{
  // HTTPS: основная страница, /api/status, /api/cmd
  httpd_ssl_config_t sconf = HTTPD_SSL_CONFIG_DEFAULT();
  // ВАЖНО: mbedTLS для PEM ждёт длину вместе с завершающим NUL (внутри strstr),
  // иначе x509_crt_parse вернёт MBEDTLS_ERR_X509_INVALID_FORMAT.
  sconf.servercert     = (const uint8_t*)SV225_CERT_PEM;
  sconf.servercert_len = strlen(SV225_CERT_PEM) + 1;
  sconf.prvtkey_pem    = (const uint8_t*)SV225_KEY_PEM;
  sconf.prvtkey_len    = strlen(SV225_KEY_PEM) + 1;
  sconf.httpd.max_uri_handlers = 4;
  sconf.httpd.max_open_sockets = 2;
  sconf.httpd.stack_size = 10240;
  sconf.port_secure = WEB_PORT;
  esp_err_t err = httpd_ssl_start(&g_web, &sconf);
  if (err == ESP_OK) {
    g_https = true;
    registerWebHandlers(g_web);
  } else {
    // если TLS не поднялся, страница всё равно должна работать - по HTTP
    g_web = NULL;
    Serial.printf("HTTPS не запустился: %s. Веб-интерфейс по HTTP (GPS будет недоступен)\n",
                  esp_err_to_name(err));
    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.max_uri_handlers = 4;
    if (httpd_start(&g_web, &conf) != ESP_OK) {
      g_web = NULL;
      Serial.println("Веб-сервер не запустился.");
    } else {
      registerWebHandlers(g_web);
    }
    return;
  }

  // HTTP (порт 80): только редирект на HTTPS, чтобы адрес можно было
  // набирать без схемы. Отдельный ctrl_port - у HTTPS он занят (32769).
  httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
  conf.server_port = 80;
  conf.ctrl_port = ESP_HTTPD_DEF_CTRL_PORT;
  conf.max_uri_handlers = 1;
  conf.max_open_sockets = 2;
  // Без этого URI сравниваются посимвольно и "/*" не совпадёт ни с одним
  // адресом: в HTTPD_DEFAULT_CONFIG() uri_match_fn = NULL.
  conf.uri_match_fn = httpd_uri_match_wildcard;
  httpd_handle_t http = NULL;
  esp_err_t rerr = httpd_start(&http, &conf);
  if (rerr == ESP_OK) {
    addUri(http, "/*", HTTP_GET, handleHttpRedirect);
  } else {
    Serial.printf("HTTP-редирект на HTTPS не запустился: %s\n", esp_err_to_name(rerr));
  }
}

#endif  // WEB_UI

// ------------------------------- NVS ---------------------------------

static void slotKey(char* out, size_t n, uint8_t i, const char* what)
{
  snprintf(out, n, "s%u%s", (unsigned)i, what);
}

// слоты и активный слот из NVS; координаты текущего места - из активного
static void loadSlots()
{
  char k[12];
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    slotKey(k, sizeof(k), i, "set");
    g_slots[i].set = g_prefs.getBool(k, false);
    g_slots[i].name[0] = 0;
    g_slots[i].lat = 0.0;
    g_slots[i].lon = 0.0;
    if (!g_slots[i].set) continue;
    slotKey(k, sizeof(k), i, "name");
    strlcpy(g_slots[i].name, g_prefs.getString(k, "").c_str(), sizeof(g_slots[i].name));
    slotKey(k, sizeof(k), i, "lat");
    g_slots[i].lat = g_prefs.getFloat(k, 0.0f);
    slotKey(k, sizeof(k), i, "lon");
    g_slots[i].lon = g_prefs.getFloat(k, 0.0f);
  }

  int8_t a = (int8_t)g_prefs.getChar("active", -1);
  if (a >= 0 && a < (int8_t)MAX_SLOTS && g_slots[a].set) {
    g_activeSlot = a;
    g_latDeg = g_slots[a].lat;
    g_lonDeg = g_slots[a].lon;
  } else {
    g_activeSlot = -1;
    g_latDeg = 0.0;
    g_lonDeg = 0.0;
  }
}

static void saveSlotData(uint8_t i)
{
  char k[12];
  slotKey(k, sizeof(k), i, "set");
  g_prefs.putBool(k, g_slots[i].set);
  slotKey(k, sizeof(k), i, "name");
  g_prefs.putString(k, g_slots[i].name);
  slotKey(k, sizeof(k), i, "lat");
  g_prefs.putFloat(k, (float)g_slots[i].lat);
  slotKey(k, sizeof(k), i, "lon");
  g_prefs.putFloat(k, (float)g_slots[i].lon);
}

// сделать слот текущим местом
static void selectSlot(uint8_t i)
{
  g_activeSlot = (int8_t)i;
  g_latDeg = g_slots[i].lat;
  g_lonDeg = g_slots[i].lon;
  g_prefs.putChar("active", (char)g_activeSlot);
}

static void clearSlotData(uint8_t i)
{
  g_slots[i].set = false;
  g_slots[i].name[0] = 0;
  g_slots[i].lat = 0.0;
  g_slots[i].lon = 0.0;
  saveSlotData(i);
  if (g_activeSlot == (int8_t)i) {
    g_activeSlot = -1;
    g_latDeg = 0.0;
    g_lonDeg = 0.0;
    g_prefs.putChar("active", (char)g_activeSlot);
  }
}

static void loadConfig()
{
  g_azDir = (int8_t)g_prefs.getChar("azdir", 1);
  g_altDir = (int8_t)g_prefs.getChar("altdir", 1);
  g_azZeroDeg = g_prefs.getFloat("azzero", 0.0f);
  g_altZeroDeg = g_prefs.getFloat("altzero", 0.0f);
  if (g_azDir != 1 && g_azDir != -1) g_azDir = 1;
  if (g_altDir != 1 && g_altDir != -1) g_altDir = 1;
}

static void saveConfig()
{
  g_prefs.putChar("azdir", (char)g_azDir);
  g_prefs.putChar("altdir", (char)g_altDir);
  g_prefs.putFloat("azzero", (float)g_azZeroDeg);
  g_prefs.putFloat("altzero", (float)g_altZeroDeg);
}

// ----------------------------- транспорт -----------------------------

// первое слово буфера - префикс известной текстовой команды?
static bool isTextCommandPrefix(const char* s, uint8_t n)
{
  static const char* const kws[] = {
    "T", "SLOT", "SLOTUSE", "SLOTCLEAR", "AZDIR", "ALTDIR",
    "AZZERO", "ALTZERO", "CLEAR", "PTDEL", "STATUS", "HELP", "?"
  };
  uint8_t t = 0;
  while (t < n && s[t] != ' ') t++;
  for (uint8_t i = 0; i < sizeof(kws) / sizeof(kws[0]); i++) {
    if (t > strlen(kws[i])) continue;
    uint8_t j = 0;
    while (j < t && toupper((unsigned char)s[j]) == kws[i][j]) j++;
    if (j == t) return true;
  }
  return false;
}

static void serviceLinks()
{
  uint32_t now = millis();
  for (uint8_t i = 0; i < sizeof(g_links) / sizeof(g_links[0]); i++) {
    Link& l = g_links[i];
    // обрывок текстовой строки (помеха при открытии/закрытии порта, потеря байтов)
    // не должен блокировать разбор LX200 - сбрасываем по таймауту
    if (l.mode == 2 && l.len > 0 && now - l.lastMs > TEXT_CMD_TIMEOUT_MS) {
      l.mode = 0;
      l.len = 0;
    }
    while (l.io->available() > 0) {
      char c = (char)l.io->read();
      l.lastMs = now;
      if (l.mode == 0) {
        if (c == ':') {
          l.mode = 1;
          l.len = 0;
          l.buf[l.len++] = ':';
        } else if (c == '#' || c == '\n' || c == '\r') {
          // разделитель, игнорируем
        } else {
          l.mode = 2;
          l.len = 0;
          l.buf[l.len++] = c;
        }
      } else if (l.mode == 1) {
        if (c == '#') {
          l.buf[l.len] = 0;
          handleLx200(l, l.buf);
          l.mode = 0;
        } else if (c == '\n' || c == '\r' || l.len >= CMD_BUF_SIZE - 1) {
          l.mode = 0;
        } else {
          l.buf[l.len++] = c;
        }
      } else {
        if (c == '\n' || c == '\r') {
          if (l.len > 0) {
            l.buf[l.len] = 0;
            handleText(l, l.buf);
          }
          l.mode = 0;
        } else if (l.len >= CMD_BUF_SIZE - 1) {
          l.mode = 0;
        } else if (c == ':' && !isTextCommandPrefix(l.buf, l.len)) {
          // буфер не похож на текстовую команду, а пришло ':' - это начало LX200
          l.mode = 1;
          l.len = 0;
          l.buf[l.len++] = ':';
        } else {
          l.buf[l.len++] = c;
        }
      }
    }
  }
}

// ------------------------------ setup --------------------------------

void setup()
{
  Serial.begin(SERIAL_BAUD);

  g_prefs.begin("sv225", false);
  loadConfig();
  loadSlots();

  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin(18, 19);
  Wire1.setClock(400000);

  readEncoders();
  setPolarStart();
  loadCompileTime();

  g_links[0].io = &Serial;
  for (uint8_t i = 0; i < sizeof(g_links) / sizeof(g_links[0]); i++) {
    g_links[i].len = 0;
    g_links[i].mode = 0;
    g_links[i].lastMs = millis();
  }

#if WEB_UI
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 2);  // 2 клиента - меньше буферов под станции
  Serial.printf("Память до веб-сервера: %u (макс. блок %u)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  startWebServer();
#endif

  Serial.println();
  Serial.println("SV225 push-to: LX200 для Stellarium (USB). HELP - команды.");
#if WEB_UI
  Serial.print("Веб-настройка: Wi-Fi redstar01, страница ");
  Serial.println(g_https ? "https://192.168.4.1/ (сертификат самоподписанный)"
                         : "http://192.168.4.1/ (HTTPS не поднялся, GPS недоступен)");
  Serial.printf("Свободная память: %u (макс. блок %u)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
#endif
}

void loop()
{
  static uint32_t lastRead = 0;
  uint32_t now = millis();
  if (now - lastRead >= ENCODER_READ_MS) {
    lastRead = now;
    readEncoders();
  }
  serviceLinks();
}
