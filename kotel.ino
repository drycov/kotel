#define DEBUG_ENABLE

#include <EEPROM.h>
#include <OneWire.h>
#include <Wire.h>
#include <GyverNTC.h>
#include "SPI.h"
#include "Ethernet.h"
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include "GyverTimer.h"

#ifdef DEBUG_ENABLE
#define DEBUG(label, val) \
  Serial.print(label);    \
  Serial.print(": ");     \
  Serial.println(val)
#else
#define DEBUG(label, val)
#endif

void (*resetFunc)(void) = 0; // Функция перезагрузки

struct Pins
{
  static const int TEN_PINS[3];
  static const int TEMP_PINS[4];
  static const int LCD_ADDRESS = 0x27;
  static const int PUMP = 6;
  static const int KeyPadPin = A3;
};

struct Data
{
  static const int B_VALUE = 3950;
  static const int NOMINAL_TEMPERATURE = 25;
  static const int NOMINAL_RESISTANCE = 10000;
  static const int REFERENCE_RESISTANCE = 10000;
  static const int SERIAL_BAUD_RATE = 9600;
  static const int MAX_MENU_ITEMS_DISPLAYED = 4; // Максимальное количество отображаемых пунктов меню
  static const int numPoints = 13;
  static const int startAddress = 0;
  static const int MAX_HEATERS = 3;
};

struct MenuItem
{
  const char *label;
  void (*action)();
  MenuItem *subMenu; // Добавлено поле для хранения субменю
};

struct NetworkSettings
{
  String ipAddress;
  String subnetMask;
  String gateway;

  // Конструктор для инициализации значений по умолчанию
  NetworkSettings()
      : ipAddress("192.168.000.200"), subnetMask("255.255.255.000"), gateway("192.168.000.001") {}
};

NetworkSettings settings;
typedef MenuItem SubMenu;

const int Pins::TEN_PINS[3] = {3, 4, 5};
const int Pins::TEMP_PINS[4] = {A1, A2, A6, A7};

enum KeyCodes
{
  LEFT = 1,
  UP = 2,
  DOWN = 3,
  RIGHT = 4,
  OK = 5,
};

const int t_n[Data::numPoints] = {-40, -37, -35, -30, -25, -20, -15, -10, -5, 0, 5, 8, 10};
const float t_wo[Data::numPoints] = {92.9, 90.0, 88, 83, 77.9, 72.7, 67.4, 61.9, 56.2, 50.3, 44.1, 40.2, 37.5};
const float t_wi[Data::numPoints] = {71.9, 70.0, 68, 63, 57.9, 52.7, 47.4, 41.9, 36.2, 30.3, 24.1, 20.2, 17.5};

const uint8_t marker[8] = {0x10, 0x18, 0x1C, 0x1E, 0x1C, 0x18, 0x10}; // marker
const uint8_t Stop[8] = {0x00, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x00};
const uint8_t TempOut[8] = {0x04, 0x0A, 0x0A, 0x0A, 0x0A, 0x11, 0x1F, 0x0E};
const uint8_t TempIn[8] = {0x04, 0x0A, 0x0A, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E};
const uint8_t Pump[8] = {0x0A, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x0A, 0x0A};
const uint8_t EVP[8] = {0x1C, 0x1F, 0x11, 0x11, 0x15, 0x11, 0x1F, 0x1C};
unsigned int currentMenuItem = 0; // Currently selected option
unsigned long currentTime, loopTime, lastActionTime, lastmillis, lastTempActionTime;
float insideTemp, outsideTemp, waterInTemp, waterOutTemp, t_wi_output, t_wo_output;

unsigned long lastUpdateTime = 0;

long previousMillisTimeSW = 0; // счетчик прошедшего времени для мигания изменяемых значений.
long intervalTimeSW = 500;     // интервал мигания изменяемых значений.

unsigned long updateInterval = 500;             // Update every 1.5 seconds
unsigned long buttonDownStartTime = 0;          // Время начала удержания кнопки DOWN
static unsigned long interval = updateInterval; // Интервал времени для проверки температуры (10 минут)

byte readKey()
{
  static unsigned long lastPressTime = 0; // Время последнего нажатия
  int val = analogRead(Pins::KeyPadPin);
  byte currentKey = 0;
  if (val < 50)
    // 1  left
    currentKey = LEFT;
  else if (val < 160)
    // 2 160 up
    currentKey = DOWN;
  else if (val < 360)
    // 3 360 down
    currentKey = UP;
  else if (val < 560)
    // 4 560 right
    currentKey = RIGHT;
  else if (val < 860)
    // 5 860 menu
    currentKey = OK;
  unsigned long currentTime = millis();
  if (currentTime - lastPressTime >= 100)
  {
    lastPressTime = currentTime;
    return currentKey;
  }
  return 0;
}

float interpolate(float x, int numPoints, int xData[], float yData[])
{
  if (x <= xData[0])
    return yData[0];
  if (x >= xData[numPoints - 1])
    return yData[numPoints - 1];

  for (int i = 1; i < numPoints; i++)
  {
    if (x < xData[i])
    {
      float x0 = xData[i - 1];
      float x1 = xData[i];
      float y0 = yData[i - 1];
      float y1 = yData[i];
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
  }
  return 0;
}

enum MenuState
{
  NORMAL,
  MAIN_MENU,
  SUB_MENU,
  CH_SYSTEM_STATUS,
  CH_HEATERS_STATUS,
  CH_PUMP_STATUS,
  CH_WORK_MODE,
  SH_SYSTEM_SETTINGS,

};
enum SubMenuState
{
  NONE,
  SYSTEM_SETTINGS,
  NETWORK_SETTINGS,
  WOUT_SETUP,
  WIN_SETUP,
  IN_TEMP_SETUP,
};

MenuState menuState = NORMAL;
SubMenuState subMenuState = NONE;

struct SystemState
{
  int currentMenuItem;
  float setWoInTemp;
  float setRoomTemp;
  int workMode;
  int pumpOn;
  int stop;
  int heater[3];
  int tempItem;
  int selectedItem;
  int NetworkEditMode;
  int tempReached; // Флаг достижения температуры
  int tempHeater[3];
  NetworkSettings settings;
  MenuState menuState;
  SubMenuState subMenuState;
  SubMenu *subMenu;
  SubMenu *previousSubMenu;
  MenuItem *currentSubMenu;
};
SystemState systemState;

struct EEPROMData
{
  float setRoomTemp;
  float setWoInTemp;
  int workMode;
  int pumpOn;
  int stop;
  int heater[3];
  String ipAddress;
  String subnetMask;
  String gateway;

  // Функция для обновления значений переменных в структуре из данных SystemState
  void updateFromSystemState(const SystemState &systemState)
  {
    setRoomTemp = systemState.setRoomTemp;
    setWoInTemp = systemState.setWoInTemp;
    workMode = systemState.workMode;
    pumpOn = systemState.pumpOn;
    stop = systemState.stop;
    ipAddress = systemState.settings.ipAddress;
    subnetMask = systemState.settings.subnetMask;
    gateway = systemState.settings.gateway;
    for (int i = 0; i < 3; ++i)
    {
      heater[i] = systemState.heater[i];
    }
  }

  void writeToEEPROM(int startAddress)
  {
    EEPROM.put(startAddress, *this);
  }

  void readFromEEPROM(int startAddress)
  {
    EEPROM.get(startAddress, *this);
  }

  bool isEEPROMInitialized(int startAddress)
  {
    int value = 0;
    for (int i = startAddress; i < sizeof(EEPROMData); ++i)
    {
      value += EEPROM.read(i);
    }
    return value != 0; // Если значение не равно 0, значит, в EEPROM есть данные
  }
  void clearEEPROM(int startAddress)
  {
    for (unsigned int i = 0; i < sizeof(EEPROMData); ++i)
    {
      EEPROM.write(startAddress + i, 0xFF); // Записать каждый байт в EEPROM значением 0xFF
    }
  }
};

EEPROMData eepromData;

SubMenu subMenuNetworkSettings[] = {
    {"IP Setup", nullptr, nullptr},
    {"Mask Setup", nullptr, nullptr},
    {"GW Setup", nullptr, nullptr}};

SubMenu subMenuSettings[] = {
    {
        "Wout Setup",
        nullptr,
    },
    {"Network Setup", []()
     {
       systemState.menuState = SUB_MENU;
       systemState.subMenuState = NETWORK_SETTINGS;
     },
     nullptr},
    {"Reset System", []()
     {
       eepromData.clearEEPROM(Data::startAddress);
       resetFunc(); // Программная перезагрузка Arduino после стирания EEPROM
     },
     nullptr}};

// Объединение всех субменю в один массив
SubMenu allSubMenus[] = {
    {"Heaters On/Off", []()
     {
       systemState.menuState = CH_HEATERS_STATUS;
     },
     nullptr},
    {"Pump On/Off", []()
     {
       systemState.menuState = CH_PUMP_STATUS;
     },
     nullptr},
    {"Work mode", []()
     {
       systemState.menuState = CH_WORK_MODE;
     },
     nullptr},
    {"Settings", []()
     {
       systemState.menuState = SUB_MENU;
       systemState.subMenuState = SYSTEM_SETTINGS;
     },
     subMenuSettings}, // Добавьте другие подменю, если они есть
};

MenuItem menuItems[sizeof(allSubMenus) / sizeof(allSubMenus[0])] = {0}; // Создание меню на основе количества элементов массива allSubMenus

void setupMenu()
{
  for (int i = 0; i < sizeof(allSubMenus) / sizeof(allSubMenus[0]); ++i)
  {
    menuItems[i] = allSubMenus[i];
  }
}

LiquidCrystal_I2C lcd(Pins::LCD_ADDRESS, 20, 4);
GyverNTC therm;
GTimer LCDUpdater(MS, 1000);
GTimer ThermReadWindow(MS, 50000);

void setup()
{
#ifdef DEBUG_ENABLE
  Serial.begin(Data::SERIAL_BAUD_RATE);
#endif
  // EEPROM.begin(sizeof(eepromData));
  if (!eepromData.isEEPROMInitialized(Data::startAddress))
  {
    systemState.setRoomTemp = 25.0;
    systemState.setWoInTemp = 0.0;
    systemState.workMode = 1;
    systemState.pumpOn = 1;
    systemState.stop = 1;
    systemState.heater[0] = 0;
    systemState.heater[1] = 0;
    systemState.heater[2] = 0;

    systemState.settings.ipAddress = "192.168.000.200";
    systemState.settings.subnetMask = "255.255.255.000";
    systemState.settings.gateway = "192.168.000.001";
    eepromData.updateFromSystemState(systemState);
    eepromData.writeToEEPROM(Data::startAddress);
  }
  else
  {
    eepromData.readFromEEPROM(Data::startAddress);
    systemState.setRoomTemp = eepromData.setRoomTemp;
    systemState.setWoInTemp = eepromData.setWoInTemp;
    systemState.workMode = eepromData.workMode;
    systemState.pumpOn = eepromData.pumpOn;
    systemState.stop = eepromData.stop;
    systemState.settings.ipAddress = eepromData.ipAddress;
    systemState.settings.subnetMask = eepromData.subnetMask;
    systemState.settings.gateway = eepromData.gateway;
    for (int i = 0; i < Data::MAX_HEATERS; ++i)
    {
      systemState.heater[i] = eepromData.heater[i];
      systemState.tempHeater[i] = systemState.heater[i];
    }
  }
  currentTime = millis();
  loopTime = currentTime;
  therm.config(Data::REFERENCE_RESISTANCE, Data::B_VALUE);

  systemState.currentMenuItem = 0;
  systemState.menuState = NORMAL;
  systemState.subMenuState = NULL;
  systemState.tempItem = 0;

  lcd.init();
  lcd.createChar(0, marker);
  lcd.createChar(1, Stop);
  lcd.createChar(2, Pump);
  lcd.createChar(3, TempOut);
  lcd.createChar(4, TempIn);
  lcd.createChar(5, EVP);
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print("*    TEPLOTECH.    *");
  lcd.setCursor(0, 2);
  lcd.print("*    EVP-6 V1.0    *");
  lcd.setCursor(0, 3);
  lcd.print("********************");
  delay(updateInterval);

  if (currentTime - lastActionTime >= (updateInterval) * 2)
  {
    lcd.clear();
    lastActionTime = currentTime;
  }

  // pinMode(Pins::PUMP, OUTPUT);
  for (int i = 0; i < 3; i++)
  {
    pinMode(Pins::TEN_PINS[i], OUTPUT);
    digitalWrite(Pins::TEN_PINS[i], LOW);
    digitalWrite(Pins::TEN_PINS[i], HIGH);
  }
  pinMode(Pins::PUMP, OUTPUT);
  digitalWrite(Pins::PUMP, LOW);
  digitalWrite(Pins::PUMP, HIGH);
  // delay((updateInterval*2));
}

void loop()
{
  currentTime = millis();
  readTemperatures();
  byte keyPressed = readKey();

  setupMenu();

  t_wo_output = interpolate(outsideTemp, Data::numPoints, t_n, t_wo);
  t_wi_output = interpolate(outsideTemp, Data::numPoints, t_n, t_wi);
  if (LCDUpdater.isReady())
    lcd.clear();
  // if (currentTime - lastActionTime >= ((updateInterval * 2)))
  // {
  //   lcd.clear();
  //   lastActionTime = currentTime;
  // }
  if (ThermReadWindow.isReady())
    heaterController();

  if (systemState.menuState == NORMAL)
  {
    if (keyPressed == OK)
    {
      systemState.menuState = MAIN_MENU;
      lcd.clear();
      displayMenu();
    }
    else if (keyPressed == LEFT)
    {
      systemState.stop = (systemState.stop == 1) ? 0 : 1;
      eepromData.stop = systemState.stop;
      eepromData.writeToEEPROM(Data::startAddress);
    }
    displayNormalData();
    handleTemp(keyPressed);
  }
  else if (systemState.menuState == MAIN_MENU)
  {

    handleMenu(keyPressed);

    displayMenu();
  }
  else if (systemState.menuState == SUB_MENU)
  {
    if (systemState.subMenuState == NETWORK_SETTINGS)
    {
      handleNetworkSettingsMenu(keyPressed, settings);
    }
    else
    {
      handleSubMenu(keyPressed);
    }
  }
  else if (systemState.menuState == CH_HEATERS_STATUS)
  {
    handleHeaterSettings();
  }
  else if (systemState.menuState == CH_PUMP_STATUS)
  {
    handlePumpSettings();
  }
  else if (systemState.menuState == CH_WORK_MODE)
  {
    handleModeSettings();
  }

  // Если от сервера есть какие-нибудь байты,
  // считываем их и выводим на Serial Monitor:
}

void readTemperatures()
{
  float tempValues[4];
  for (int i = 0; i < 4; ++i)
  {
    therm.setPin(Pins::TEMP_PINS[i]);
    tempValues[i] = therm.getTempAverage();
  }
  insideTemp = tempValues[0];
  outsideTemp = tempValues[1];
  waterInTemp = tempValues[2];
  waterOutTemp = tempValues[3];
}

void displayNormalData()
{
  int i = 0;
  printHeaterStatus();
  displaySetWoinTemp();
  displaySystemState();
  // line1
  lcd.setCursor(0, 1);
  lcd.print(char(4));
  lcd.print(":");
  lcd.print(int(insideTemp) < -50 ? "E" : String(int(insideTemp)));
  displayWorkModeState();
  lcd.setCursor(15, 1);
  lcd.print(char(5));
  lcd.print(char(126));
  lcd.print(":");
  lcd.print(int(waterOutTemp) < -50 ? "E" : String(int(waterOutTemp)));

  // line 2
  lcd.setCursor(0, 2);
  // lcd.print(char(5));

  lcd.print(char(3));
  lcd.print(":");
  lcd.print(int(outsideTemp) < -50 ? "E" : String(int(outsideTemp)));

  displayPumpState();

  lcd.setCursor(15, 2);
  lcd.print(char(126));

  lcd.print(char(5));
  lcd.print(":");
  lcd.print(int(waterInTemp) < -50 ? "E" : String(int(waterInTemp)));
  displayHSVal();
}

void printHeaterStatus()
{
  lcd.setCursor(0, 0);

  if (systemState.stop)
  {
    lcd.print("XXX");
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(Pins::TEN_PINS[i], HIGH);
    }
  }
  else
  {
    for (int i = 0; i < 3; i++)
    {
      lcd.print(systemState.heater[i] == 1 ? "I" : "O");
      digitalWrite(Pins::TEN_PINS[i], systemState.heater[i] == 1 ? LOW : HIGH);
    }
  }
}

void displaySetWoinTemp()
{
  lcd.setCursor(8, 0);
  lcd.print("[");
  lcd.print(int(systemState.setWoInTemp));
  lcd.print("]");
}

void displaySystemState()
{
  lcd.setCursor(17, 0);
  lcd.print(char(5));
  lcd.print(":");
  lcd.print(systemState.stop == 1 ? "S" : "R");

  for (int i = 0; i < 3; i++)
  {
    if (systemState.heater[i] == 1 && !systemState.stop)
    {
      digitalWrite(Pins::TEN_PINS[i], LOW);
    }
    else
    {
      digitalWrite(Pins::TEN_PINS[i], HIGH);
    }
  }
}

void displayPumpState()
{
  lcd.setCursor(8, 2);
  lcd.print(char(2));
  lcd.print(":");
  lcd.print(systemState.pumpOn == 0 ? "off" : "on"); // Приведение к одному типу данных
  controlPump();
}

void displayWorkModeState()
{
  lcd.setCursor(8, 1);
  lcd.print(systemState.workMode == 0 ? "AUTO" : "MANU");
}

void displayHSVal()
{
  if (systemState.workMode == 0)
  {
    lcd.setCursor(0, 3);
    lcd.print(char(126));
    lcd.print(char(5));

    lcd.print("|");
    lcd.print(int(t_wi_output));
    lcd.print("|");
    lcd.setCursor(14, 3);
    lcd.print("|");
    lcd.print(int(t_wo_output));
    lcd.print("|");
    lcd.print(char(5));
    lcd.print(char(126));
  }
}

void displayMenu()
{
  int totalItems = sizeof(menuItems) / sizeof(menuItems[0]); // Получаем общее количество пунктов меню
  int startIdx = systemState.currentMenuItem;                // Индекс первого пункта для отображения
  int endIdx = startIdx + Data::MAX_MENU_ITEMS_DISPLAYED;    // Индекс последнего пункта для отображения

  if (endIdx >= totalItems)
  {
    endIdx = totalItems;                                // Установка конечного индекса на общее количество пунктов
    startIdx = endIdx - Data::MAX_MENU_ITEMS_DISPLAYED; // Пересчитываем начальный индекс, чтобы показать 4 пункта
    if (startIdx < 0)
    {
      startIdx = 0; // Обработка случая, когда менее 4 пунктов
    }
  }

  for (int i = startIdx; i < endIdx; i++)
  {
    lcd.setCursor(0, i - startIdx); // Установка позиции на экране LCD для пунктов меню
    lcd.print((i == systemState.currentMenuItem) ? char(0) : char(32));
    lcd.print(menuItems[i].label);
  }
}

void displaySubMenu()
{
  int totalSubMenuItems = 0;
  SubMenu *currentSubMenu = nullptr;

  if (systemState.subMenuState == SYSTEM_SETTINGS)
  {
    totalSubMenuItems = sizeof(subMenuSettings) / sizeof(subMenuSettings[0]);
    currentSubMenu = subMenuSettings;
  }
  else if (systemState.subMenuState == NETWORK_SETTINGS)
  {
    totalSubMenuItems = sizeof(subMenuNetworkSettings) / sizeof(subMenuNetworkSettings[0]);
    currentSubMenu = subMenuNetworkSettings;
  }

  for (int i = 0; i < totalSubMenuItems; i++)
  {
    lcd.setCursor(0, i); // Установка позиции на экране LCD для пунктов субменю
    lcd.print((i == systemState.currentMenuItem) ? char(0) : char(32));
    lcd.print(currentSubMenu[i].label);
  }
}

void handleMenu(byte keyPressed)
{
  MenuState previousMenuState = systemState.menuState;
  int totalItems = sizeof(menuItems) / sizeof(menuItems[0]);

  switch (keyPressed)
  {
  case DOWN:
    systemState.currentMenuItem = (systemState.currentMenuItem - 1 + totalItems) % totalItems;
    break;

  case UP:
    systemState.currentMenuItem = (systemState.currentMenuItem + 1) % totalItems;
    break;

  case OK:
    handleMenuOK();
    break;

  case LEFT:
    handleMenuLeft();
    break;
  }

  if (systemState.menuState != previousMenuState)
  {
    lcd.clear();
    displayMenu();
  }
}

void handleSubMenu(byte keyPressed)
{
  int totalItems = 0;
  SubMenu *currentSubMenu = nullptr;

  if (systemState.subMenuState == SYSTEM_SETTINGS)
  {
    totalItems = sizeof(subMenuSettings) / sizeof(subMenuSettings[0]);
    currentSubMenu = subMenuSettings;
  }
  else if (systemState.subMenuState == NETWORK_SETTINGS)
  {
    totalItems = sizeof(subMenuNetworkSettings) / sizeof(subMenuNetworkSettings[0]);
    currentSubMenu = subMenuNetworkSettings;
  }

  switch (keyPressed)
  {
  case DOWN:
    systemState.currentMenuItem = (systemState.currentMenuItem - 1 + totalItems) % totalItems;
    break;

  case UP:
    systemState.currentMenuItem = (systemState.currentMenuItem + 1) % totalItems;
    break;

  case OK:
    handleSubMenuOK(currentSubMenu);
    break;

  case LEFT:
    handleSubMenuLeft();
    break;
  }
  displaySubMenu();
}

// Обработка события OK в основном меню
void handleMenuOK()
{
  MenuItem *currentItem = &menuItems[systemState.currentMenuItem];
  if (currentItem->subMenu != nullptr)
  {
    systemState.menuState = SUB_MENU;
    systemState.subMenu = currentItem->subMenu;
    if (currentItem->action != nullptr)
    {
      currentItem->action();
    }
    systemState.currentMenuItem = 0;
    lcd.clear();
    displaySubMenu();
  }
  else if (currentItem->action != nullptr)
  {
    currentItem->action();
  }
}

// Обработка события LEFT в основном меню
void handleMenuLeft()
{
  eepromData.updateFromSystemState(systemState);
  eepromData.writeToEEPROM(Data::startAddress);
  systemState.menuState = NORMAL;
  lcd.clear();
}

void handleSubMenuOK(SubMenu *currentSubMenu)
{
  if (currentSubMenu != nullptr)
  {
    SubMenu *currentItem = &currentSubMenu[systemState.currentMenuItem];
    if (currentItem->subMenu != nullptr)
    {
      systemState.previousSubMenu = systemState.subMenu;
      systemState.subMenu = currentItem->subMenu;
      if (currentItem->action != nullptr)
      {
        currentItem->action();
      }
    }
    else if (currentItem->action != nullptr)
    {
      currentItem->action();
    }

    systemState.currentMenuItem = 0;
    lcd.clear();
    displaySubMenu();
  }
}

void handleSubMenuLeft()
{
  if (systemState.menuState == SUB_MENU)
  {
    systemState.menuState = MAIN_MENU;
    systemState.currentMenuItem = 0;
    lcd.clear();
    displayMenu();
  }
}

void controlPump()
{
  digitalWrite(Pins::PUMP, systemState.pumpOn ? LOW : HIGH);
}

void handleTemp(byte keyPressed)
{
  static int Temperature = systemState.setWoInTemp; // Используем статическую переменную для сохранения значения между вызовами функции

  if (keyPressed == DOWN)
  {
    if (Temperature < 85)
    { // Убедимся, что значение не превышает 85
      Temperature++;
    }
  }
  else if (keyPressed == UP)
  {
    if (Temperature > 0)
    { // Убедимся, что значение не меньше 0
      Temperature--;
    }
  }
  systemState.setWoInTemp = Temperature; // Присваиваем новое значение переменной systemState.setWoInTemp
  eepromData.setWoInTemp = systemState.setWoInTemp;
  eepromData.writeToEEPROM(Data::startAddress);
}

void handleHeaterSettings()
{
  lcd.clear();
  int heaterValues[] = {systemState.heater[0], systemState.heater[1], systemState.heater[2]};
  const char *heaterNames[] = {"Heater A: ", "Heater B: ", "Heater C: "};

  int currentField = 0;

  while (true)
  {
    if (currentTime - lastActionTime >= updateInterval)
    {
      lcd.clear();
      lastActionTime = currentTime;
    }
    for (int i = 0; i < 3; i++)
    {
      systemState.heater[i] = heaterValues[i];
      digitalWrite(Pins::TEN_PINS[i], heaterValues[i] ? LOW : HIGH);
    }
    lcd.setCursor(0, 0);
    lcd.print("Heaters On/Off");
    for (int i = 0; i < 3; i++)
    {
      lcd.setCursor(0, i + 1);
      lcd.print(i == currentField ? char(0) : char(32));
      lcd.print(i + 1);
      lcd.print(": ");
      lcd.print(heaterNames[i]);
      lcd.print(heaterValues[i] == 1 ? "on" : "off");
      lcd.print("   ");
    }

    byte keyPressed = readKey();

    if (keyPressed == DOWN)
    {
      currentField = (currentField + 1) % 3;
    }
    else if (keyPressed == UP)
    {
      currentField = (currentField + 1) % 3;
    }
    else if (keyPressed == RIGHT || keyPressed == LEFT)
    {
      heaterValues[currentField] = !heaterValues[currentField]; // Toggle heater value
      digitalWrite(Pins::TEN_PINS[currentField], heaterValues[currentField] ? LOW : HIGH);
    }
    else if (keyPressed == OK)
    {
      for (int i = 0; i < 3; i++)
      {
        systemState.heater[i] = heaterValues[i];
        digitalWrite(Pins::TEN_PINS[i], heaterValues[i] ? LOW : HIGH);
      }
      systemState.menuState = MAIN_MENU;
      break;
    }
  }
}

void handlePumpSettings()
{
  lcd.clear();
  const char *pumpName = "Pump: ";
  while (true)
  {
    if (currentTime - lastActionTime >= updateInterval)
    {
      lcd.clear();
      lastActionTime = currentTime;
    }
    controlPump();
    lcd.setCursor(0, 0);
    lcd.print("Pump On/Off");
    lcd.setCursor(0, 1);
    lcd.print(char(0));
    // lcd.print(1);
    // lcd.print(": ");
    lcd.print(pumpName);
    lcd.print(systemState.pumpOn == 1 ? "on" : "off");
    lcd.print("   ");

    byte keyPressed = readKey();

    if (keyPressed == RIGHT || keyPressed == LEFT)
    {
      systemState.pumpOn = !systemState.pumpOn; // Toggle pump value
      controlPump();
    }
    else if (keyPressed == OK)
    {
      systemState.menuState = MAIN_MENU;
      break;
    }
  }
}

void handleModeSettings()
{
  lcd.clear();
  const char *ItemName = "Mode: ";

  while (true)
  {
    if (currentTime - lastActionTime >= updateInterval)
    {
      lcd.clear();
      lastActionTime = currentTime;
    }
    lcd.setCursor(0, 0);
    lcd.print("Work mode");
    lcd.setCursor(0, 1);
    lcd.print(char(0));
    // lcd.print(1);
    // lcd.print(": ");
    lcd.print(ItemName);
    lcd.print(systemState.workMode == 0 ? "AUTO" : "MANU");

    // lcd.print(systemState.workMode == 1 ? "on" : "off");
    // lcd.print("   ");

    byte keyPressed = readKey();

    if (keyPressed == RIGHT || keyPressed == LEFT)
    {
      systemState.workMode = !systemState.workMode; // Toggle pump value
      // lcd.print(systemState.workMode == 0 ? "AUTO" : "MANU");
    }
    else if (keyPressed == OK)
    {
      systemState.menuState = MAIN_MENU;
      break;
    }
  }
}

void handleNetworkSettingsMenu(byte keyPressed, NetworkSettings &settings)
{
  int selectedItem = systemState.selectedItem;
  int mode = systemState.NetworkEditMode;
  int tempItem = systemState.tempItem;
  int totalItems = 3;
  switch (keyPressed)
  {
  case UP:
    if (mode == 0)
    {
    }
    else if (mode == 1)
    {
      if (settings.ipAddress[tempItem] < '9')
      {
        settings.ipAddress[tempItem]++;
      }
      else
      {
        settings.ipAddress[tempItem] = '0';
      }
    }
    break;
  case DOWN:
    if (mode == 0)
    {
      if (settings.ipAddress[tempItem] > '0')
      {
        settings.ipAddress[tempItem]--;
      }
      else
      {
        settings.ipAddress[tempItem] = '9';
      }
    }
    else if (mode == 1)
    {
      if (settings.ipAddress[tempItem] > '0')
      {
        settings.ipAddress[tempItem]--;
      }
      else
      {
        settings.ipAddress[tempItem] = '9';
      }
    }
    break;
  case LEFT:
    if (mode == 1)
    {
      if (selectedItem > 0)
      {
        selectedItem--;
      }
    }
    else if (mode == 3)
    {
      if (selectedItem > 0)
      {
        selectedItem--;
      }
    }
    break;
  case RIGHT:
    if (mode == 1)
    {
      if (selectedItem < 14)
      {
        selectedItem++;
      }
    }
    else if (mode == 3)
    {
      if (selectedItem < 14)
      {
        selectedItem++;
      }
    }
    break;
  case OK:
    if (mode == 0 && selectedItem == 0 || 1 || 2)
    {
    }
    else if (mode == 1 && selectedItem == 0 || 1 || 2)
    {
    }
    else if (selectedItem == 3)
    {
    }
    break;
  }
}

void heaterController()
{
  float setWoInTemp = systemState.setWoInTemp;
  if (systemState.workMode == 0)
  {
    if (currentTime - lastTempActionTime >= interval)
    {
      lastTempActionTime = currentTime;
      if (waterOutTemp < t_wo_output)
      {
        int heatersToTurnOn = 1;
        for (int i = 0; i < Data::MAX_HEATERS && heatersToTurnOn > 0; ++i)
        {
          if (systemState.heater[i] == 0)
          {
            systemState.heater[i] = 1;
            heatersToTurnOn--;
          }
        }
        if (heatersToTurnOn == 0)
        {
          systemState.tempReached = 0;
        }
      }
      else if (waterOutTemp >= t_wo_output)
      {
        for (int i = 0; i < Data::MAX_HEATERS; ++i)
        {
          systemState.heater[i] = 0;
        }
        systemState.tempReached = 1;
      }
    }
  }
  else if (systemState.workMode == 1)
  {
    if (currentTime - lastTempActionTime >= interval)
    {
      lastTempActionTime = currentTime;
      if (waterOutTemp < setWoInTemp)
      {
        for (int i = 0; i < Data::MAX_HEATERS; ++i)
        {
          systemState.heater[i] = systemState.tempHeater[i];
        }
        systemState.tempReached = 0;
      }
      else if (waterOutTemp >= setWoInTemp)
      {
        for (int i = 0; i < Data::MAX_HEATERS; ++i)
        {
          systemState.tempHeater[i] = systemState.heater[i];
          systemState.heater[i] = 0;
        }
        systemState.tempReached = 1;
      }
    }
  }
}
