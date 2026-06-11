#include <WiFi.h>
#include <HardwareSerial.h>

// ---------- WiFi configuration ----------
const char* ssid         = "mate60";
const char* wifiPassword = "13143366";

// ---------- NTRIP (China Mobile CORS) ----------
const char* ntripHost  = "ntrip.geodetic.gov.hk";
const int   ntripPort  = 2101;
const char* mountpoint = "HKCL_32";
const char* ntripUser  = "";
const char* ntripPass  = "";

// ---------- LG290P UART pins ----------
// ESP32 GPIO8 -> LG290P RXD  (ESP32 TX2)
// ESP32 GPIO9 -> LG290P TXD  (ESP32 RX2)
#define RXD2 9   // ESP32 RX2 pin, connect to LG290P TXD
#define TXD2 8   // ESP32 TX2 pin, connect to LG290P RXD

HardwareSerial gpsSerial(2);    // UART2 for LG290P
WiFiClient     ntripClient;     // TCP client to NTRIP caster

// NMEA parsing
String partsGGA[15];
String partsRMC[12];
String nmeaSentence = "";
bool   hasGGA = false, hasRMC = false;
String lastGGA = "";
unsigned long lastGGASend = 0;

// ---------- GPS data struct ----------
struct GPSData {
  bool  fix;
  int   satellites;
  float hdop;
  float latitude;
  float longitude;
  float speed;
  float heading;
  int   fixType;   // 0=no fix, 1=SPS, 2=DGPS, 4=fixed RTK, 5=float RTK
} gps;

// ---------- Base64 (for Basic Auth) ----------
String base64Encode(const String &data) {
  const char* base64_chars =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String ret;
  int val = 0, valb = -6;
  for (int i = 0; i < data.length(); i++) {
    uint8_t c = data.charAt(i);
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      ret += base64_chars[(val >> valb) & 0x3F];
      valb -= 6;
    }
  }
  if (valb > -6) ret += base64_chars[((val << 8) >> (valb + 8)) & 0x3F];
  while (ret.length() % 4) ret += '=';
  return ret;
}

// ---------- Connect to WiFi ----------
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, wifiPassword);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed.");
  }
}

// ---------- Connect to NTRIP (same format as web_rtk.py) ----------
void connectToNTRIP() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skip NTRIP.");
    return;
  }

  Serial.printf("Connecting to NTRIP %s:%d\n", ntripHost, ntripPort);
  if (!ntripClient.connect(ntripHost, ntripPort)) {
    Serial.println("NTRIP connect failed!");
    return;
  }

  // Build Basic Auth like Python
  String auth      = String(ntripUser) + ":" + String(ntripPass);
  String authBase64 = base64Encode(auth);

  // *** IMPORTANT ***
  // Request format is EXACTLY the same as web_rtk.py:
  //   GET /{mount} HTTP/1.1
  //   User-Agent: NTRIP client
  //   Accept: */*
  //   Connection: keep-alive
  //   Authorization: Basic xxx
  //   <blank line>
  String request =
    String("GET /") + mountpoint + " HTTP/1.1\r\n" +
    "User-Agent: NTRIP client\r\n" +
    "Accept: */*\r\n" +
    "Connection: keep-alive\r\n" +
    "Authorization: Basic " + authBase64 + "\r\n" +
    "\r\n";

  ntripClient.print(request);
  Serial.println("NTRIP request sent.");

  // Read response header, same as we would in Python
  String header = "";
  unsigned long t0 = millis();
  while (millis() - t0 < 3000 && ntripClient.connected()) {
    while (ntripClient.available()) {
      char c = ntripClient.read();
      header += c;
      if (header.endsWith("\r\n\r\n")) {
        break;
      }
    }
    if (header.endsWith("\r\n\r\n")) {
      break;
    }
  }

  Serial.println("NTRIP response header:");
  Serial.println("-----");
  Serial.println(header);
  Serial.println("-----");

  // Accept "200" or "ICY 200" as success
  if (header.indexOf("200") < 0 && header.indexOf("ICY 200") < 0) {
    Serial.println("NTRIP handshake failed (no 200), closing connection.");
    ntripClient.stop();
    return;
  }

  Serial.println("NTRIP connected, waiting for RTCM...");
}

// ---------- Print GPS data ----------
void printGPSData() {
  Serial.println("-------------");
  Serial.print("Positioning Status: ");
  Serial.println(gps.fix ? "Yes" : "No");

  Serial.print("RTK Status: ");
  switch (gps.fixType) {
    case 0: Serial.println("No Fix"); break;
    case 1: Serial.println("GPS SPS Mode"); break;
    case 2: Serial.println("DGNSS (SBAS/DGPS)"); break;
    case 3: Serial.println("GPS PPS Fix"); break;
    case 4: Serial.println("Fixed RTK"); break;
    case 5: Serial.println("Float RTK"); break;
    default:
      Serial.print("Unknown (");
      Serial.print(gps.fixType);
      Serial.println(")");
      break;
  }

  if (gps.fixType == 4 || gps.fixType == 5) {
    Serial.println("RTK Data: RECEIVED");
  } else {
    Serial.println("RTK Data: NOT RECEIVED");
  }

  Serial.print("Satellites Number: ");
  Serial.println(gps.satellites);
  Serial.print("HDOP: ");
  Serial.println(gps.hdop, 2);
  Serial.print("Latitude: ");
  Serial.println(gps.latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(gps.longitude, 6);
  Serial.print("Speed (knots): ");
  Serial.println(gps.speed, 3);
  Serial.print("Heading (degrees): ");
  Serial.println(gps.heading, 2);
  Serial.println("-------------");
}

// ---------- NMEA helpers ----------
void splitNMEA(const String &sentence, String *parts, int maxParts) {
  int startIdx = 0;
  int partIdx  = 0;
  while (partIdx < maxParts) {
    int commaIdx = sentence.indexOf(',', startIdx);
    if (commaIdx == -1) {
      parts[partIdx++] = sentence.substring(startIdx);
      break;
    }
    parts[partIdx++] = sentence.substring(startIdx, commaIdx);
    startIdx = commaIdx + 1;
  }
}

float convertToDecimalDegree(const String &val, const String &direction) {
  if (val.length() < 6) return 0.0;

  float deg = 0.0;
  float min = 0.0;

  if (direction == "N" || direction == "S") {
    deg = val.substring(0, 2).toFloat();
    min = val.substring(2).toFloat();
  } else if (direction == "E" || direction == "W") {
    deg = val.substring(0, 3).toFloat();
    min = val.substring(3).toFloat();
  } else {
    return 0.0;
  }

  float decDeg = deg + min / 60.0;
  if (direction == "S" || direction == "W") decDeg = -decDeg;
  return decDeg;
}

void parseGNGGA(const String &sentence) {
  splitNMEA(sentence, partsGGA, 15);
  gps.fixType    = partsGGA[6].toInt();
  gps.fix        = (gps.fixType > 0);
  gps.satellites = partsGGA[7].toInt();
  gps.hdop       = partsGGA[8].toFloat();
  gps.latitude   = convertToDecimalDegree(partsGGA[2], partsGGA[3]);
  gps.longitude  = convertToDecimalDegree(partsGGA[4], partsGGA[5]);

  lastGGA = sentence;
}

void parseGNRMC(const String &sentence) {
  splitNMEA(sentence, partsRMC, 12);
  if (partsRMC[2] == "A") {
    gps.speed   = partsRMC[7].toFloat();
    gps.heading = partsRMC[8].toFloat();
  } else {
    gps.speed   = 0;
    gps.heading = 0;
  }
}

// ---------- Arduino setup / loop ----------
void setup() {
  Serial.begin(460800);                       // USB serial (debug)
  gpsSerial.begin(460800, SERIAL_8N1, RXD2, TXD2); // LG290P UART

  Serial.println("ESP32-S3 LG290P NTRIP client (CORS).");

  connectWiFi();
  connectToNTRIP();
}

void loop() {
  // 1) Receive RTCM from NTRIP and forward to LG290P
  if (ntripClient.connected()) {
    while (ntripClient.available()) {
      uint8_t c = ntripClient.read();
      gpsSerial.write(c);
      // Debug: show that we are receiving data
      // if (c == 0xD3) Serial.print("[RTCM]");
    }
  }

  // 2) Read NMEA from LG290P, parse GGA/RMC
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      nmeaSentence.trim();
      if (nmeaSentence.startsWith("$GNGGA")) {
        parseGNGGA(nmeaSentence);
        hasGGA = true;
      } else if (nmeaSentence.startsWith("$GNRMC")) {
        parseGNRMC(nmeaSentence);
        hasRMC = true;
      }
      nmeaSentence = "";
    } else if (c != '\r') {
      nmeaSentence += c;
    }
  }

  // 3) When both GGA and RMC are updated, print GPS info
  if (hasGGA && hasRMC) {
    printGPSData();
    hasGGA = hasRMC = false;
  }

  // 4) Send GGA to NTRIP every 5 seconds, like Python script
  if (ntripClient.connected() && lastGGA.length() > 0 &&
      millis() - lastGGASend > 5000) {
    ntripClient.println(lastGGA);
    lastGGASend = millis();
    Serial.println(String("[GGA sent to NTRIP] ") + lastGGA);
  }

  // 5) Reconnect if NTRIP is disconnected
  if (!ntripClient.connected()) {
    Serial.println("NTRIP disconnected, reconnecting...");
    ntripClient.stop();
    delay(1000);
    connectToNTRIP();
  }

  delay(1);
}
