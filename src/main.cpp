#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_FT6206.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <TFT_eSPI.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "constants.h"
#include "interpolation.h"

// Verbose screen status messages
#define VERBOSE false

// EEPROM settings
#define EEPROM_SIZE 6 // Allocating by 2 bytes for bootCounter (uint16_t), tempRangeMin and tempRangeMax (int16_t)
#define EEPROM_BOOT_COUNTER_ADDRESS 0
#define EEPROM_TEMP_RANGE_MIN_ADDRESS 4
#define EEPROM_TEMP_RANGE_MAX_ADDRESS 8

// Touch screen I2C pins and params
#define TOUCH_SDA_PIN 6
#define TOUCH_SCL_PIN 5
#define TOUCH_I2C_ADDRESS FT62XX_DEFAULT_ADDR
#define TOUCH_I2C_FREQUENCY_KHZ 400

// MLX90640 I2C pins and params
#define SENSOR_SDA_PIN 10
#define SENSOR_SCL_PIN 11
#define SENSOR_I2C_ADDRESS 0x33
#define SENSOR_I2C_FREQUENCY_KHZ 1000

// TFT screen
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);
TFT_eSprite inf = TFT_eSprite(&tft);

// Touch controller
Adafruit_FT6206 ts = Adafruit_FT6206();

// Screens dimensions
#define SCREEN_WIDTH 480 // Y
#define SCREEN_HEIGHT 320 // X
#define IMAGE_WIDTH SCREEN_HEIGHT // 320, X
#define IMAGE_HEIGHT (SCREEN_WIDTH / 2) // 240, Y
#define INFO_WIDTH SCREEN_HEIGHT // 320, X
#define INFO_HEIGHT (SCREEN_WIDTH / 2) // 240, Y
#define LEGEND_SHIFT_Y 5
#define LEGEND_HEIGHT 42
#define LEGEND_DIVIDER_Y (LEGEND_SHIFT_Y + LEGEND_HEIGHT + 51)
#define TEXT_AREA_HEIGHT 48
#define TEXT_AREA_BORDER 2
#define TEXT_BOX_WIDTH 59
#define TEXT_BOX_WIDTH_LONG (INFO_WIDTH - (2 * (TEXT_AREA_BORDER * 2 + TEXT_BOX_WIDTH)))
#define TEXT_BOX_HEIGHT (TEXT_AREA_HEIGHT / 2 - TEXT_AREA_BORDER * 2 + 1)
#define BUTTON_Y (LEGEND_DIVIDER_Y + 31)
#define BUTTON_WIDTH 125
#define BUTTON_HEIGHT 43
#define CHECKBOX_Y (BUTTON_Y + 29)
#define CHECKBOX_WIDTH 28
#define CHECKBOX_HEIGHT 28

// Sensor: addresses and parameters
paramsMLX90640 mlx90640;
#define REFRESH_RATE 0x06 // 0x00: 0.5Hz, 0x01: 1Hz, 0x02: 2Hz, 0x03: 4Hz, 0x04: 8Hz, 0x05: 16Hz, 0x06: 32Hz, 0x07: 64Hz
#define TA_SHIFT 8 // Ambient temperature (TA) shift
#define EMMISIVITY 0.95
#define MIN_MEASURABLE_TEMP -40 // According to sensor's spec
#define MAX_MEASURABLE_TEMP 300

// Sensor params and settings
#define MATRIX_X 32 // INPUT_COLS
#define MATRIX_Y 24 // INPUT_ROWS
#define MATRIX_SIZE (MATRIX_X * MATRIX_Y)
#define MLX_MIRROR false // Set to true when the camera is facing the screen
#define SCALE_X (IMAGE_WIDTH / MATRIX_X) // 10
#define SCALE_Y (IMAGE_HEIGHT / MATRIX_Y) // 10
bool interpolation = true;
bool filtering = true;

// Mix/Max initial temperatures
int16_t tempRangeMin = 25;
int16_t tempRangeMax = 37;
int16_t tempRangeMinEEPROM = 0;
int16_t tempRangeMaxEEPROM = 0;
int tempRangeChanged = 0; // 0: no change, 11: min--, 12: min++, 13: max--, 14 max++

// Buffers for source and interpolated data & variables for other sensor data
float frame[MATRIX_SIZE];
float *frameFiltered = NULL;
uint16_t *frameInterpolated = NULL;
float ambientTemperature = tempRangeMin; // Ambient temparature calculated by sensor
float vddVoltade = 0; // Sensor's VDD (Voltage Drain Drain, plus)

// Touch screen objects and variables
TFT_eSPI_Button resetBtn; // Invoke the TFT_eSPI buttons classes
TFT_eSPI_Button saveBtn;
TFT_eSPI_Button interpolationBtn;
TFT_eSPI_Button filteringBtn;
TS_Point touch; // To store the touch coordinates
int16_t tempX = IMAGE_WIDTH / 2;
int16_t tempY = IMAGE_HEIGHT / 2;
int16_t tempXPrinted = 0;
int16_t tempYPrinted = 0;
bool touchEnabled = false;
bool sdCardEnabled = false;
bool webServerEnabled = false;

// Access point and Web server settings
const char* ssid = "thermals";
const char* password = "thermals";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
AsyncWebServer server(80);
int lastPage = 0;
int currentPage = 0;
int filesNumber = 0;
#define IMAGES_PER_PAGE 3

// Custom colors
uint16_t TFT_DARK_DARK_GREY = tft.color565(31, 31, 31);
uint16_t TFT_SUPER_DARK_GREY = tft.color565(15, 15, 15);
uint16_t TFT_DARK_DARK_CYAN = tft.color565(0, 80, 80);
uint16_t TFT_LIGHT_BLUE = tft.color565(0, 122, 204);

// Flow control and UI variables
uint16_t bootCounter = 0;
uint16_t fileCounter = 0;
float minTemp = tempRangeMin;
float maxTemp = tempRangeMax;
float minMinTemp = tempRangeMin;
float maxMaxTemp = tempRangeMax;
float minMinTempPrinted = 0;
float maxMaxTempPrinted = 0;
int lastFrameReadStatus = 0;
ulong errorsCount = 0;
ulong loopNumber = 0;
long loopDuration = 0; // ms
float fps = 0;
float fpsPrinted = 0;
bool resetRequested = false;
bool saveRequested = false;
bool saveCompleted = false;
bool saveSuccessful = true;
String statusTextPrinted = "";


// === Declarations of functions =====================================================================

void initializeAndProcessEEPROM();
void saveTemperatureRangeToEEPROM();
void textOut(String text, int32_t x = 5, int32_t y = (IMAGE_HEIGHT / 2 - 10), uint8_t font = 2, uint16_t fgcolor = TFT_WHITE, uint16_t bgcolor = TFT_BLACK);
void initializeScreen();
void initializeSDCard();
void initializeWebServer();
String listFiles(int page);
String pagination(int page);
bool isImage(String fileName);
void initializeButtons();
void initializeTouch();
bool isI2cDeviceConnected(TwoWire *wire, uint8_t i2cAddr);
void initializeThermalSensor();
void rebootThermalSensor();
bool saveScreenshot(bool thermalImageOnly);
void prepareInterpolation();
void readTempValues();
void processTempValues();
void processTouchScreen(void *arg);
void processButtonPress(TFT_eSPI_Button *btn, bool touched, int tag);
void processRequests();
void drawThermalImage();
void drawLegend(float min, float max, float center, bool numbersOnly, int position);
void drawInfo();


// === Functions =====================================================================================

// Standard Arduino framework's setup function
void setup()
{

  // Reading counter from EEPROM memory
  initializeAndProcessEEPROM();

  // Make sure that we use maximup possible CPU speed
  setCpuFrequencyMhz(240);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize SD card SPI interface
  SPI.begin(SD_CLK, SD_DO, SD_DI, -1);

  // Initialize both I2C buses and increase the clock speed
  Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN, 400000);
  Wire1.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, TOUCH_I2C_FREQUENCY_KHZ * 1000);

  // Initialize the screen
  initializeScreen();

  // Initialize touch
  initializeTouch();

  // Initialize arrays and data we use for interpolation
  prepareInterpolation();

  // Initialize MLX90640 thermal sensor
  initializeThermalSensor();

  // Initialize SD card
  initializeSDCard();

  // Initialize WebServer
  if (sdCardEnabled) initializeWebServer();

  // Draw initial thermal image
  drawThermalImage();

  // Initialize buttons
  initializeButtons();

  // Draw initial legend values
  drawLegend(tempRangeMin, tempRangeMax, (tempRangeMax - tempRangeMin) / 2, false, 1);
  drawLegend(tempRangeMin, tempRangeMax, (tempRangeMax - tempRangeMin) / 2, true, 2);

  inf.pushSprite(0, IMAGE_HEIGHT);

  // Draw screen xTtask
  xTaskCreate(processTouchScreen, "processTouchScreen", 4096, NULL, tskIDLE_PRIORITY, NULL);
}

// Standard Arduino framework's loop function
void loop()
{
  loopNumber++;
  ulong startTime = millis();

  readTempValues();
  processTempValues();
  drawThermalImage();
  if ((loopNumber % 3) == 0) drawInfo();
  processRequests();

  loopDuration = millis() - startTime;
  fps = (float)(1000.0 / loopDuration);
}


// Initializing EEPROM memory and reading initial conter values
void initializeAndProcessEEPROM()
{
  EEPROM.begin(EEPROM_SIZE);

  // Reading and updating boot counter
  bootCounter = EEPROM.readUShort(EEPROM_BOOT_COUNTER_ADDRESS);
  bootCounter++;
  EEPROM.writeUShort(EEPROM_BOOT_COUNTER_ADDRESS, bootCounter);

  // Reading the temperature range
  tempRangeMinEEPROM = EEPROM.readShort(EEPROM_TEMP_RANGE_MIN_ADDRESS);
  if (tempRangeMinEEPROM > MIN_MEASURABLE_TEMP && tempRangeMinEEPROM < MAX_MEASURABLE_TEMP && tempRangeMinEEPROM != 0 && tempRangeMinEEPROM != -1)
    tempRangeMin = tempRangeMinEEPROM;
  tempRangeMaxEEPROM = EEPROM.readShort(EEPROM_TEMP_RANGE_MAX_ADDRESS);
  if (tempRangeMaxEEPROM > MIN_MEASURABLE_TEMP && tempRangeMaxEEPROM < MAX_MEASURABLE_TEMP && tempRangeMaxEEPROM != 0 && tempRangeMaxEEPROM != -1)
    tempRangeMax = tempRangeMaxEEPROM;
  
  EEPROM.commit();
  EEPROM.end();
}

// saving 
void saveTemperatureRangeToEEPROM()
{
  if (tempRangeMinEEPROM == tempRangeMin && tempRangeMaxEEPROM == tempRangeMax)
    return;

  EEPROM.begin(EEPROM_SIZE);

  // Updating the temperature range
  if (tempRangeMin > MIN_MEASURABLE_TEMP && tempRangeMin < MAX_MEASURABLE_TEMP)
    EEPROM.writeShort(EEPROM_TEMP_RANGE_MIN_ADDRESS, tempRangeMin);
  if (tempRangeMax > MIN_MEASURABLE_TEMP && tempRangeMax < MAX_MEASURABLE_TEMP)
    EEPROM.writeShort(EEPROM_TEMP_RANGE_MAX_ADDRESS, tempRangeMax);
  
  EEPROM.commit();
  EEPROM.end();
}

// Print text out
void textOut(String text, int32_t x, int32_t y, uint8_t font, uint16_t fgcolor, uint16_t bgcolor)
{
  img.fillSprite(TFT_BLACK);
  img.setTextColor(fgcolor, bgcolor);
  img.drawString(text, x, y, font);
  img.pushSprite(0, 0);
}

// Initialize screen
void initializeScreen()
{
  // Initiate the LCD backlight LED
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Initialize ILI9488 display
  tft.init();
  tft.setRotation(0);

  // Create info sprite
  inf.createSprite(INFO_WIDTH, INFO_HEIGHT);
  inf.setSwapBytes(1);
  inf.fillSprite(TFT_BLACK);
  inf.setTextSize(2);
  inf.pushSprite(0, IMAGE_HEIGHT);

  // Create image sprite
  img.createSprite(IMAGE_WIDTH, IMAGE_HEIGHT);
  img.setSwapBytes(1);
  img.fillSprite(TFT_BLACK);
  img.pushSprite(0, 0);
}

// Initialize embedded SD card reader
void initializeSDCard()
{
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);
  delay(10);

  if (SD.begin(SD_CS)) sdCardEnabled = true;
  else sdCardEnabled = false;

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) sdCardEnabled = false;
}

// Initialize Web server
void initializeWebServer(){
  webServerEnabled = WiFi.softAP(ssid, password);
  webServerEnabled = webServerEnabled && WiFi.softAPConfig(local_IP, gateway, subnet);
  if (!webServerEnabled) return;

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    int page = 0;
    if (request->hasParam("page")) {
      page = request->getParam("page")->value().toInt();
    }
    currentPage = page;
    String content = listFiles(page);
    String paginationHtml = pagination(page);
    String html = rootPageHtml;  // Use the HTML from webpage.h
    html.replace("%CONTENT%", content);
    html.replace("%PAGINATION%", paginationHtml);
    request->send(200, "text/html", html);
  });

  server.on("/image", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("file")) {
      String fileName = request->getParam("file")->value();
      String html = imagePageHtml;
      html.replace("%IMAGE_SRC%", "/" + fileName);
      request->send(200, "text/html", html);
    } else {
      request->send(404, "text/plain", "File not found");
    }
  });

  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("file")) {
      String fileName = "/" + request->getParam("file")->value();
      if (SD.exists(fileName)) {
        SD.remove(fileName);
        filesNumber--;
        lastPage = int(filesNumber / IMAGES_PER_PAGE) + ((filesNumber % IMAGES_PER_PAGE == 0) ? 0 : 1) - 1;
        if (currentPage > lastPage) currentPage = lastPage;
      }
      request->redirect("/?page=" + String(currentPage));
    }
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("file")) {
      String fileName = "/" + request->getParam("file")->value();
      if (SD.exists(fileName)) {
        AsyncWebServerResponse *response = request->beginResponse(SD, fileName, "image/bmp", true);
        request->send(response);
      } else {
        request->send(404, "text/plain", "File not found");
      }
    }
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    String fileName = "/favicon.png";
    if (SD.exists(fileName)) {
      AsyncWebServerResponse *response = request->beginResponse(SD, fileName, "image/png");
      request->send(response);
    }
  });

  server.serveStatic("/", SD, "/");
  server.begin();
}

// List files on SD card
String listFiles(int page) {
  String html = "";
  std::vector<String> fileNames;
  int startIndex = page * IMAGES_PER_PAGE;
  int endIndex = startIndex + IMAGES_PER_PAGE;
  int count = 0;

  File root = SD.open("/");
  File file = root.openNextFile();

  // Collect file names
  while (file) {
    if (!file.isDirectory() && isImage(file.name())) {
      fileNames.push_back(String(file.name()));
      count++;
    }
    file = root.openNextFile();
  }

  // Sort file names in reverse alphabetical order
  std::sort(fileNames.begin(), fileNames.end(), std::greater<String>());

  // Generate HTML from sorted file names
  for (int i = startIndex; i < endIndex && i < count; i++) {
    html += "<div class='file-container'>";
    html += "<p><a href=\"/image?file=" + fileNames[i] + "\">" + fileNames[i] + "</a></p>";
    html += "<div class='image-container'><img src='/" + fileNames[i] + "' alt='" + fileNames[i] + "' /></div>";
    html += "<p><a href=\"/download?file=" + fileNames[i] + "\">Download</a> ";
    html += "<a href=\"/delete?file=" + fileNames[i] + "\">Delete</a></p>";
    html += "</div>";
  }

  filesNumber = count;
  lastPage = (filesNumber / IMAGES_PER_PAGE) + ((filesNumber % IMAGES_PER_PAGE == 0) ? 0 : 1) - 1;
  return html;
}

// Pagination
String pagination(int page){
  String html = "<div class='pagination'>";
  int startPage = max(0, page - 2);  // Show up to 2 pages before current
  int endPage = min(lastPage, page + 2);  // Show up to 2 pages after current

  if (page > 0) {
    html += "<a href='/?page=0'>&lt;&lt;</a> ";
    html += "<a href='/?page=" + String(page - 1) + "'>&lt;</a> ";
  }

  for (int i = startPage; i <= endPage; i++) {
    if (i == page) {
      html += "<span>" + String(i + 1) + "</span> ";  // Current page
    } else {
      html += "<a href='/?page=" + String(i) + "'>" + String(i + 1) + "</a> ";
    }
  }

  if (page < lastPage) {
    html += "<a href='/?page=" + String(page + 1) + "'>&gt;</a> ";
    html += "<a href='/?page=" + String(lastPage) + "'>&gt;&gt;</a>";
  }

  html += "</div>";
  return html;
}

// Check if the file is a BMP image
bool isImage(String fileName){
  return fileName.endsWith(".bmp");
}

// Initialize buttons
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
void initializeButtons()
{
  int x;
  int center = INFO_WIDTH / 4;
  int textSize = inf.textsize;
  int textFont = inf.textfont;
  int textDatum = inf.textdatum;

  inf.setTextFont(2);
  
  // Draw button area background
  inf.fillRect(0, LEGEND_DIVIDER_Y, INFO_WIDTH, INFO_HEIGHT - LEGEND_DIVIDER_Y - TEXT_AREA_HEIGHT, TFT_DARK_DARK_GREY);

  // Reset sensor button
  x = center;
  resetBtn.initButton(&inf, x, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT, TFT_DARKGREY, TFT_MAROON, TFT_WHITE, "Reboot", 1);
  resetBtn.setLabelDatum(0, 6, MC_DATUM);
  resetBtn.drawButton();

  // Save screenshot button
  x = INFO_WIDTH - center;
  if (sdCardEnabled) saveBtn.initButton(&inf, x, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT, TFT_DARKGREY, TFT_DARK_DARK_CYAN, TFT_WHITE, "Save", 1);
  else saveBtn.initButton(&inf, x, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT, TFT_DARKGREY, TFT_DARKGREY, TFT_SILVER, "No card", 1);
  saveBtn.setLabelDatum(0, 6, MC_DATUM);
  saveBtn.drawButton();

  // Interpolation checkbox
  x = (INFO_WIDTH / 2 - BUTTON_WIDTH) / 2 + 2;
  interpolationBtn.initButtonUL(&inf, x, CHECKBOX_Y, CHECKBOX_WIDTH, CHECKBOX_HEIGHT, TFT_DARKGREY, TFT_SUPER_DARK_GREY, TFT_WHITE, "", 1);
  interpolationBtn.setLabelDatum(0, 6, MC_DATUM);
  interpolationBtn.drawButton(false, (interpolation ? "*" : ""));
  inf.setTextColor(TFT_SILVER, TFT_DARK_DARK_GREY);
  inf.setTextSize(1);
  inf.drawString("Interpolation", x + CHECKBOX_WIDTH + 5, CHECKBOX_Y + 12, 1);

  // Filtering checkbox
  x = (INFO_WIDTH / 2) + (INFO_WIDTH / 2 - BUTTON_WIDTH) / 2 + 2;
  filteringBtn.initButtonUL(&inf, x, CHECKBOX_Y, CHECKBOX_WIDTH, CHECKBOX_HEIGHT, TFT_DARKGREY, TFT_SUPER_DARK_GREY, TFT_WHITE, "", 1);
  filteringBtn.setLabelDatum(0, 6, MC_DATUM);
  filteringBtn.drawButton(false, (filtering ? "*" : ""));
  inf.setTextColor(TFT_SILVER, TFT_DARK_DARK_GREY);
  inf.setTextSize(1);
  inf.drawString("Anti-aliasing", x + CHECKBOX_WIDTH + 6, CHECKBOX_Y + 12, 1);

  inf.setTextSize(textSize);
  inf.setTextFont(textFont);
  inf.setTextDatum(textDatum);
}

// Initialize touch
void initializeTouch()
{
  // Check if MLX90640 responds
  if (!isI2cDeviceConnected(&Wire1, TOUCH_I2C_ADDRESS))
  {
    textOut("Touch is not detected at default I2C address");
    delay(750);
    return;
  }

  // Initialize touch
  touchEnabled = ts.begin(30, &Wire1, TOUCH_I2C_ADDRESS);

  if (!touchEnabled)
  {
    textOut("Failed to initialize touch");
    delay(750);
  }
  else
  {
    if (VERBOSE)
    {
      textOut("Touch initialized");
      delay(250);
    }
  }
}

// Check if I2C device is connected
bool isI2cDeviceConnected(TwoWire *wire, uint8_t i2cAddr)
{
  wire->beginTransmission(i2cAddr);
  if (wire->endTransmission() != 0)
    return (false); // Sensor did not ACK
  return (true);
}

// Initialize MLX90640 thermal sensor making that it is properly reset and initialized
void initializeThermalSensor()
{
  // Check if MLX90640 responds
  if (!isI2cDeviceConnected(&Wire, SENSOR_I2C_ADDRESS))
  {
    textOut("MLX90640 is not detected at default I2C address, freezing");
    while (1);
  }

  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[MATRIX_SIZE + 64]; // 832
    
  for (int i = 0; i < 15; i++)
  {
    status = MLX90640_DumpEE(SENSOR_I2C_ADDRESS, eeMLX90640);
    if (status == 0) break;
    delay(10);
  }
  if (status == 0)
  {
    if (VERBOSE)
    {
      textOut("MLX90640 params loaded");
      delay(250);
    }
  }
  else
  {
    textOut("Failed to load MLX90640 params: " + String(status));
    delay(750);
  }

  for (int i = 0; i < 15; i++)
  {
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status == 0) break;
    delay(10);
  }
  if (status == 0)
  {
    if (VERBOSE)
    {
      textOut("MLX90640 params extracted");
      delay(250);
    }
  }
  else
  {
    textOut("MLX90640 params extraction failed: " + String(status));
    delay(750);
  }

  MLX90640_I2CWrite(SENSOR_I2C_ADDRESS, 0x800D, 6401); // Writes the value 1901 (HEX) = 6401 (DEC) in the register at position 0x800D to enable reading out the temperatures
  
  // Set MLX90640 device at slave i2cAddress address 0x33, refresh rate and resolution
  MLX90640_SetRefreshRate(SENSOR_I2C_ADDRESS, REFRESH_RATE);
  MLX90640_SetResolution(SENSOR_I2C_ADDRESS, 0x03); // 0x03: 19-bit (maximum) resolution

  // Once EEPROM has been read at 400kHz, we can increase to 1000kHz
  Wire.setClock(SENSOR_I2C_FREQUENCY_KHZ * 1000);
}

// General reset to I2C bus and variables
void rebootThermalSensor()
{
  saveTemperatureRangeToEEPROM();
  MLX90640_I2CGeneralReset(SENSOR_I2C_ADDRESS);
  ESP.restart();
}

// Saving the screenshot
bool saveScreenshot(bool thermalImageOnly)
{
  byte vh, vl;
  int width, height;
  String suffix;

  if (!sdCardEnabled) return false;
  
  if (thermalImageOnly)
  {
    width = IMAGE_WIDTH;
    height = IMAGE_HEIGHT;
    suffix = "T"; // [T]hermal image only
  }
  else
  {
    width = SCREEN_HEIGHT;
    height = SCREEN_WIDTH;
    suffix = "S"; // Entire [S]creen
  }

  // Forming the file name based on counters stored in EEPROM
  char buffer[5];
  sprintf(buffer, "%04d", bootCounter); // formatting the number with 4 digits, adding leading zeros
  String bootNumber = String(buffer);
  sprintf(buffer, "%04d", fileCounter + 1); // incrementing counter in advance until we are sure than the file can be saved
  String fileNumber = String(buffer);
  String fileName = "/" + bootNumber + "_" + fileNumber + "_" + suffix + ".bmp"; // 0001_0002_S.bmp

  // Creating the output file
  File outFile = SD.open(fileName, FILE_WRITE);
  if (!outFile) return false;

  unsigned char bmFlHdr[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
  // 54 = std total "old" Windows BMP file header size = 14 + 40

  unsigned char bmInHdr[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 16, 0};
  // 40 = info header size
  // 01 = num of color planes
  // 16 = bits per pixel
  // all other header info = 0, including RI_RGB (no compr), DPI resolution

  unsigned long fileSize = sizeof(uint16_t) * height * width + 54; // pix data + 54 byte hdr

  bmFlHdr[2] = (unsigned char)(fileSize); // all ints stored little-endian
  bmFlHdr[3] = (unsigned char)(fileSize >> 8); // i.e., LSB first
  bmFlHdr[4] = (unsigned char)(fileSize >> 16);
  bmFlHdr[5] = (unsigned char)(fileSize >> 24);

  bmInHdr[4] = (unsigned char)(width);
  bmInHdr[5] = (unsigned char)(width >> 8);
  bmInHdr[6] = (unsigned char)(width >> 16);
  bmInHdr[7] = (unsigned char)(width >> 24);
  bmInHdr[8] = (unsigned char)(height);
  bmInHdr[9] = (unsigned char)(height >> 8);
  bmInHdr[10] = (unsigned char)(height >> 16);
  bmInHdr[11] = (unsigned char)(height >> 24);

  outFile.write(bmFlHdr, sizeof(bmFlHdr));
  outFile.write(bmInHdr, sizeof(bmInHdr));

  for (int h = height; h > 0; h--)
  {
    for (int w = 0; w < width; w++)
    {
      uint16_t rgb;
      if (thermalImageOnly)  // getting color in rgb565 format
      {
        rgb = frameInterpolated[((h - 1) * width) + w];
      }
      else
      {
        if (h <= IMAGE_HEIGHT)
          rgb = img.readPixel(w, h - 1);
        else
          rgb = inf.readPixel(w, h - 1 - IMAGE_HEIGHT);
      }
      
      vh = (rgb & 0xFF00) >> 8; // High Byte
      vl = rgb & 0x00FF; // Low Byte

      // RGB565 to RGB555 conversion... 555 is default for uncompressed BMP
      // this conversion is from ...topic=177361.0 and has not been verified
      vl = (vh << 7) | ((vl & 0xC0) >> 1) | (vl & 0x1f);
      vh = vh >> 1;

      // Write image data to file, low byte first
      outFile.write(vl);
      outFile.write(vh);      
    }
  }

  // Close the file
  outFile.close();

  // Incrementing file counter
  fileCounter++;

  return true;
}

// Prepare interpolation arrays and data
void prepareInterpolation()
{
  // Interpolation array init
  if (interpolation)
  {
    frameInterpolated = static_cast<uint16_t*>(malloc(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(uint16_t)));
    if (frameInterpolated == NULL)
    {
      textOut("frameInterpolated malloc error, freezing");
      delay(500);
      textOut("free/need=" + String(ESP.getFreeHeap()) + "/" + String(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(uint16_t)));
      delay(500);
      while (1);
    }
    for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++)
      *(frameInterpolated + i) = colorMap[0];
  }

  // Filtered frame array init
  frameFiltered = static_cast<float *>(malloc(MATRIX_SIZE * sizeof(float)));
  if (frameFiltered == NULL)
  {
    textOut("frameFiltered malloc error, freezing");
    delay(500);
    textOut("free/need=" + String(ESP.getFreeHeap()) + "/" + String(MATRIX_SIZE * sizeof(float)));
    delay(500);
    while (1);
  }

  for (int i = 0; i < MATRIX_SIZE; i++)
    frameFiltered[i] = tempRangeMin;

  if (VERBOSE)
  {
    textOut("Interpolation initiated");
    delay(250);
  }
}

// Read temperature data from MLX90640
void readTempValues()
{
  lastFrameReadStatus = 1;
  uint16_t mlx90640Frame[MATRIX_SIZE + 64 + 2]; // 834

  for (byte x = 0; x < 2; x++)
  {
    int status = MLX90640_GetFrameData(SENSOR_I2C_ADDRESS, mlx90640Frame);
    lastFrameReadStatus = lastFrameReadStatus * status;
   
    vddVoltade = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    ambientTemperature = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = ambientTemperature - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, frame);
    MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, frame, MLX90640_GetCurMode(SENSOR_I2C_ADDRESS), &mlx90640);
  }
  
  if (lastFrameReadStatus != 0) errorsCount++;
}

// Filter and sort temperature data from MLX90640
void processTempValues()
{
  bool corruptedFrame = false;

  for (int i = 1; i < MATRIX_SIZE; i++)
  {
    if (frame[i] < MIN_MEASURABLE_TEMP || frame[i] > MAX_MEASURABLE_TEMP)
    {
      corruptedFrame = true;
      break;
    }
  }

  // Filter temperature data
  if (!corruptedFrame)
    filter(frame, frameFiltered, MATRIX_X, MATRIX_Y, MLX_MIRROR, filtering);

  // Find max and max temperature data
  minTemp = frameFiltered[0];
  maxTemp = frameFiltered[0];
  for (int i = 1; i < MATRIX_SIZE; i++)
  {
    if (frameFiltered[i] < minTemp) minTemp = frameFiltered[i];
    if (frameFiltered[i] > maxTemp) maxTemp = frameFiltered[i];
  }
  if (minTemp < minMinTemp) minMinTemp = minTemp;
  if (maxTemp > maxMaxTemp) maxMaxTemp = maxTemp;
}

// Process touch screen: will be executed as an xTask job
void processTouchScreen(void *arg)
{
  for (;;)
  {
    if (touchEnabled)
    {
      bool touched = false;

      if (ts.touched())
      {
        touched = true;
        touch = ts.getPoint(); // Retrieve a point

        // Is touch point in the thermal image area?
        if (touch.x <= IMAGE_WIDTH && touch.y <= IMAGE_HEIGHT)
        {
          tempX = touch.x; // temparature measurment point
          tempY = touch.y;
        };

        // Is touch point in the legend (raibow) area?
        const int sectionWidth = IMAGE_WIDTH / 4;
        const int centerPoint = IMAGE_WIDTH / 2;
        if (touch.y >= (IMAGE_HEIGHT + LEGEND_SHIFT_Y) && touch.y <= (IMAGE_HEIGHT + LEGEND_SHIFT_Y + LEGEND_HEIGHT))
        {
          int newMin, newMax;
          if (touch.x >= 0 && touch.x < sectionWidth) // decrease min temperature by 1 degree
          {
            newMin = tempRangeMin - 1;
            if (newMin > MIN_MEASURABLE_TEMP && newMin < MAX_MEASURABLE_TEMP)
            {
              tempRangeMin = newMin;
              tempRangeChanged = 11;
            }
          }
          if (touch.x >= sectionWidth && touch.x < centerPoint) // increase min temperature by 1 degree
          {
            newMin = tempRangeMin + 1;
            if (newMin > MIN_MEASURABLE_TEMP && newMin < MAX_MEASURABLE_TEMP)
            {
              tempRangeMin = newMin;
              tempRangeChanged = 12;
            }
          }
          if (touch.x >= centerPoint && touch.x < centerPoint + sectionWidth) // decrease max temperature by 1 degree
          {
            newMax = tempRangeMax - 1;
            if (newMax > MIN_MEASURABLE_TEMP && newMax < MAX_MEASURABLE_TEMP)
            {
              tempRangeMax = newMax;
              tempRangeChanged = 13;
            }
          }
          if (touch.x >= centerPoint + sectionWidth && touch.x <= IMAGE_WIDTH) // increase max temperature by 1 degree
          {
            newMax = tempRangeMax + 1;
            if (newMax > MIN_MEASURABLE_TEMP && newMax < MAX_MEASURABLE_TEMP)
            {
              tempRangeMax = newMax;
              tempRangeChanged = 14;
            }
          }
        };
      }

      // Button press processing
      processButtonPress(&resetBtn, touched, 1);
      processButtonPress(&saveBtn, touched, 2);
      processButtonPress(&interpolationBtn, touched, 3);
      processButtonPress(&filteringBtn, touched, 4);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Process buttons press
void processButtonPress(TFT_eSPI_Button *btn, bool touched, int tag)
{
  // Buttons
  if (touched && btn->contains(touch.x, touch.y - IMAGE_HEIGHT))
   btn->press(true);
  else
   btn->press(false);

  // Check if any key has changed state
  int textSize = inf.textsize;
  int textFont = inf.textfont;
  inf.setTextFont(2);
  if (btn->justReleased())
  {
    String label = "";
    switch (tag)
    {
      case 3: // Interpolation checkbox
          label = (interpolation ? "*" : "");
          break;
      case 4: // Filtering checkbox
          label = (filtering ? "*" : "");
          break;
      default:
          break;
    }
    btn->drawButton(false, label); // draw normal
  }
  if (btn->justPressed())
  {
    btn->drawButton(true); // draw inverted
  }
  inf.setTextFont(textSize);
  inf.setTextFont(textFont);

  if (btn->justPressed())
  {
    switch (tag)
    {
      case 1: // Reset button
          resetRequested = true;
          break;
      case 2: // Save button
          if (sdCardEnabled) saveRequested = true;
          break;
      case 3: // Interpolation checkbox
          interpolation = !interpolation;
          break;
      case 4: // Filtering checkbox
          filtering = !filtering;
          break;
      default:
          break;
    }
  }
}

// Processing requests afrer the button press
void processRequests()
{
  if (resetRequested)
  {
    rebootThermalSensor();
    resetRequested = false;
  }
  
  if (saveRequested)
  {
    saveTemperatureRangeToEEPROM();
    inf.pushSprite(0, IMAGE_HEIGHT);
    saveCompleted = false;
    saveSuccessful = saveScreenshot(true);
    saveSuccessful = saveSuccessful && saveScreenshot(false);
    saveRequested = false;
    saveCompleted = true;
  }
}

// Draw interpolated infrared image
void drawThermalImage()
{
  if (interpolation)
  {
    interpolate(frameFiltered, frameInterpolated, MATRIX_X, MATRIX_Y, IMAGE_WIDTH, IMAGE_HEIGHT, tempRangeMin, tempRangeMax);
    //interpolate(float *data, uint16_t *out, int matrixX, int matrixY, int imageW, int imageH, float tempRangeMin, float tempRangeMax)
    img.pushImage(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT, frameInterpolated);
  }
  else
  {
    for (uint8_t h = 0; h < MATRIX_Y; h++)
    {
      for (uint8_t w = 0; w < MATRIX_X; w++)
      {
        uint8_t colorIndex = mapf(frameFiltered[h * MATRIX_X + w], tempRangeMin, tempRangeMax);
        img.fillRect(SCALE_X * w, SCALE_Y * h, SCALE_X, SCALE_Y, colorMap[colorIndex]);
      }
    }
  }

  // Mark temperature measurment point
  img.drawCircle(tempX, tempY, 8, TFT_BLACK);
  img.drawCircle(tempX, tempY, 7, TFT_WHITE);
  img.drawCircle(tempX, tempY, 6, TFT_BLACK);
  img.drawCircle(tempX, tempY, 5, TFT_WHITE);
  img.fillCircle(tempX, tempY, 2, TFT_BLACK);

  img.pushSprite(0, 0);
}

// Draw a legend
void drawLegend(float min, float max, float center, bool numbersOnly, int position)
{
  if (!numbersOnly)
  {
    // Draw rainbow ruler
    for (int i = 0; i < 256; i++)
    {
      uint32_t color = colorMap[i];
      if (i % 64 == 0 || (i + 1) % 64 == 0) color = TFT_BLACK;
      inf.drawFastVLine(map(i, 0, 255, 0, IMAGE_WIDTH - 1) + 1, LEGEND_SHIFT_Y, LEGEND_HEIGHT, color);
    }

    // Draw bottom text area frames
    int y = INFO_HEIGHT - TEXT_AREA_HEIGHT;
    inf.fillRect(0, y, INFO_WIDTH, TEXT_AREA_HEIGHT, TFT_DARKGREY);
    inf.fillRect(TEXT_AREA_BORDER, y + TEXT_AREA_BORDER, TEXT_BOX_WIDTH, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    inf.fillRect(TEXT_AREA_BORDER * 2 + TEXT_BOX_WIDTH, y + TEXT_AREA_BORDER, TEXT_BOX_WIDTH_LONG, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    inf.fillRect(TEXT_AREA_BORDER * 3 + TEXT_BOX_WIDTH + TEXT_BOX_WIDTH_LONG, y + TEXT_AREA_BORDER, TEXT_BOX_WIDTH, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    inf.fillRect(TEXT_AREA_BORDER, y + TEXT_AREA_BORDER * 2 + TEXT_BOX_HEIGHT, INFO_WIDTH - TEXT_AREA_BORDER * 2, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    // Wi-Fi stats
    int textSize = inf.textsize;
    inf.setTextColor(TFT_WHITE, TFT_DARK_DARK_GREY);
    inf.setTextSize(1);
    if (webServerEnabled)
    {
       inf.drawString("ssid/pwd/url: " + String(ssid) + "/" + String(password) + "/", TEXT_AREA_BORDER + 6, y + TEXT_AREA_BORDER * 2 + TEXT_BOX_HEIGHT + 7, 1);
       inf.setTextColor(TFT_LIGHT_BLUE, TFT_DARK_DARK_GREY);
       inf.drawString("http://" + local_IP.toString(), IMAGE_WIDTH - 119, y + TEXT_AREA_BORDER * 2 + TEXT_BOX_HEIGHT + 7, 1);
       inf.setTextColor(TFT_WHITE, TFT_DARK_DARK_GREY);
    }
    else
      inf.drawString("Access point is not active", TEXT_AREA_BORDER + 20, y + TEXT_AREA_BORDER * 2 + TEXT_BOX_HEIGHT + 7, 1);

    inf.setTextSize(textSize);
    
    // Draw dividers
    inf.drawFastHLine(0, LEGEND_DIVIDER_Y, IMAGE_WIDTH, TFT_DARKGREY);
    inf.drawFastHLine(0, LEGEND_DIVIDER_Y + 1, IMAGE_WIDTH, TFT_DARKGREY);
    inf.drawFastVLine(0, LEGEND_DIVIDER_Y + 1, IMAGE_HEIGHT - LEGEND_DIVIDER_Y - 2, TFT_DARKGREY);
    inf.drawFastVLine(1, LEGEND_DIVIDER_Y + 1, IMAGE_HEIGHT - LEGEND_DIVIDER_Y - 2, TFT_DARKGREY);
    inf.drawFastVLine(IMAGE_WIDTH - 1, LEGEND_DIVIDER_Y + 1, IMAGE_HEIGHT - LEGEND_DIVIDER_Y - 2, TFT_DARKGREY);
    inf.drawFastVLine(IMAGE_WIDTH - 2, LEGEND_DIVIDER_Y + 1, IMAGE_HEIGHT - LEGEND_DIVIDER_Y - 2, TFT_DARKGREY);
  }

  // Draw min/max and center temperatures
  int textY;
  if (min < MIN_MEASURABLE_TEMP || max > MAX_MEASURABLE_TEMP || center < MIN_MEASURABLE_TEMP || center > MAX_MEASURABLE_TEMP) return;
  switch (position)
  {
    case 1: // first line: min/max and center temperatures
      textY = LEGEND_SHIFT_Y + LEGEND_HEIGHT + 8;
      inf.setTextColor(TFT_WHITE, TFT_BLACK);
      break;
    case 2: // second line: min/max and average temperature statistics
      textY = LEGEND_SHIFT_Y + LEGEND_HEIGHT + 29;
      inf.setTextColor(TFT_DARKGREY, TFT_BLACK);
      break;
    case 11: // first line: temperature range, min value decreased
      textY = LEGEND_SHIFT_Y + LEGEND_HEIGHT + 8;
      inf.setTextColor(TFT_SKYBLUE, TFT_BLACK);
      break;
    case 12: // first line: temperature range, min value increased
      textY = LEGEND_SHIFT_Y + LEGEND_HEIGHT + 8;
      inf.setTextColor(TFT_RED, TFT_BLACK);
      break;
    case 13: // first line: temperature range, max value decreased
      textY = LEGEND_SHIFT_Y + LEGEND_HEIGHT + 8;
      inf.setTextColor(TFT_SKYBLUE, TFT_BLACK);
      break;
    case 14: // first line: temperature range, max value increased
      textY = LEGEND_SHIFT_Y + LEGEND_HEIGHT + 8;
      inf.setTextColor(TFT_RED, TFT_BLACK);
      break;
    default:
      return;
  }

  int textSize = inf.textsize;
  if (position <= 2) inf.fillRect(0, textY, INFO_WIDTH - 1, 20, TFT_BLACK);
  inf.setTextSize(2);

  if (position <= 12) inf.drawString(String(min, 1).substring(0, 4), 5, textY, 1);
  if (position <= 2 || position >= 13) inf.drawString(String(max, 1).substring(0, 4), INFO_WIDTH - 50, textY, 1);
  if (position <= 2)
  {
     if (position == 1) inf.setTextColor(TFT_YELLOW, TFT_BLACK);
     inf.drawString(String(center, 2).substring(0, 5), INFO_WIDTH / 2 - 27, textY, 1);
    }
  
  inf.setTextSize(textSize);
  inf.setTextColor(TFT_WHITE, TFT_BLACK);
}

// Draw info
void drawInfo()
{
  // Draw min/max and center temperatures on legend
  float centerTemp = frameFiltered[tempY / SCALE_Y * MATRIX_X + tempX / SCALE_X];
  if (tempRangeChanged == 0)
  {
    drawLegend(minTemp, maxTemp, centerTemp, true, 1);
    if ((abs(minMinTemp - minMinTempPrinted) >= 0.01) || (abs(maxMaxTemp - maxMaxTempPrinted) >= 0.01))
    {
      drawLegend(minMinTemp, maxMaxTemp, (maxMaxTemp - minMinTemp) / 2 + minMinTemp, true, 2);
      minMinTempPrinted = minMinTemp;
      maxMaxTempPrinted = maxMaxTemp;
    }
  }
  else
  {
    if (tempRangeChanged == 11 || tempRangeChanged == 12) drawLegend(tempRangeMin, maxTemp, centerTemp, true, tempRangeChanged);
    if (tempRangeChanged == 13 || tempRangeChanged == 14) drawLegend(minTemp, tempRangeMax, centerTemp, true, tempRangeChanged);
    tempRangeChanged = 0;
  }
  
  // Text area vertical coordinate
  int y = INFO_HEIGHT - TEXT_AREA_HEIGHT;

  // Text output
  int textSize = inf.textsize;
  inf.setTextColor(TFT_WHITE, TFT_DARK_DARK_GREY);
  inf.setTextSize(1);

   // FPS
  if (abs(fps - fpsPrinted) >= 0.01)
  {
    inf.fillRect(TEXT_AREA_BORDER, y + TEXT_AREA_BORDER, TEXT_BOX_WIDTH, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    inf.drawString("fps " + String(fps, 2), TEXT_AREA_BORDER + 6, y + TEXT_AREA_BORDER + 7, 1); // fps
    fpsPrinted = fps;
  }

  // last touch coords
  if (tempX != tempXPrinted || tempY != tempYPrinted)
  {
    inf.fillRect(TEXT_AREA_BORDER * 3 + TEXT_BOX_WIDTH + TEXT_BOX_WIDTH_LONG, y + TEXT_AREA_BORDER, TEXT_BOX_WIDTH, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    inf.drawString(String(tempX) + "/" + String(tempY), INFO_WIDTH - 54, y + TEXT_AREA_BORDER + 7, 1);
    tempXPrinted = tempX;
    tempYPrinted = tempY;
  }

  // status text
  String statusText = "";
  int statusTextX, statusTextY, statusTextFont = 0;
  if (saveCompleted) // print screenshot saving status
  {
    statusText = "SAVED TO FILE: ";
    if (saveSuccessful)
    {
      statusText += "OK";
      inf.setTextColor(TFT_GREEN, TFT_DARK_DARK_GREY);
    }
    else
    {
      statusText += "ERR";
      inf.setTextColor(TFT_GREEN, TFT_DARK_DARK_GREY);
    }
    statusTextX = TEXT_AREA_BORDER * 2 + TEXT_BOX_WIDTH + 30;
    statusTextY = y + TEXT_AREA_BORDER + 3;
    statusTextFont = 2;
    saveCompleted = false;
  }
  else
  {
    statusText = "temperature range: " + String(tempRangeMin) + ".." + String(tempRangeMax);
    statusTextX = TEXT_AREA_BORDER * 2 + TEXT_BOX_WIDTH + 21;
    statusTextY = y + TEXT_AREA_BORDER + 7;
    statusTextFont = 1;
  }
  if (!statusText.equals(statusTextPrinted))
  {
    inf.fillRect(TEXT_AREA_BORDER * 2 + TEXT_BOX_WIDTH, y + TEXT_AREA_BORDER, TEXT_BOX_WIDTH_LONG, TEXT_BOX_HEIGHT, TFT_DARK_DARK_GREY);
    inf.drawString(statusText, statusTextX, statusTextY, statusTextFont);
    statusTextPrinted = statusText;
  }

  inf.setTextSize(textSize);
  inf.pushSprite(0, IMAGE_HEIGHT);
}