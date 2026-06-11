#include <HardwareSerial.h>

HardwareSerial gpsSerial(2);
#define RXD2 9
#define TXD2 10

String nmeaSentence = "";

bool hasGGA = false, hasRMC = false;

struct GPSData {
  bool fix;              // Position fix status (true if fix acquired)
  int satellites;        // Number of satellites in view
  float hdop;            // Horizontal Dilution of Precision (accuracy indicator)
  float latitude;        // Latitude in decimal degrees
  float longitude;       // Longitude in decimal degrees
  float speed;           // Speed over ground in knots
  float heading;         // Course over ground (heading) in degrees
} gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(460800, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS Serial Started");
  // printf("GPS Serial Started\r\n");
}

void loop() {
  // Read from GPS serial port while data is available
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      nmeaSentence.trim();
      if (nmeaSentence.startsWith("$GNGGA")) {
        parseGNGGA(nmeaSentence);  // Parse GNGGA sentence for fix, satellites, HDOP, lat, lon
        hasGGA = true;
      } else if (nmeaSentence.startsWith("$GNRMC")) {
        parseGNRMC(nmeaSentence);  // Parse GNRMC sentence for speed and heading
        hasRMC = true;
      }
      nmeaSentence = "";
    } else if (c != '\r') {
      nmeaSentence += c;  // Build the NMEA sentence string, ignoring carriage returns
    }
  }

  // When both GGA and RMC data have been parsed, print GPS data
  if (hasGGA && hasRMC) {
    printGPSData();
    hasGGA = hasRMC = false;
  }
}

void printGPSData() {
  Serial.println("-------------");
  Serial.print("Positioning Status: ");
  Serial.println(gps.fix ? "Yes" : "No");
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

  // Uncomment below to print using printf style
  /*
  printf("-------------\r\n");
  printf("Positioning Status: %s\r\n", gps.fix ? "Yes" : "No");
  printf("Satellites Number: %d\r\n", gps.satellites);
  printf("HDOP: %.2f\r\n", gps.hdop);
  printf("Latitude: %.6f\r\n", gps.latitude);
  printf("Longitude: %.6f\r\n", gps.longitude);
  printf("Speed (knots): %.3f\r\n", gps.speed);
  printf("Heading (degrees): %.2f\r\n", gps.heading);
  printf("-------------\r\n");
  */
}

// Parse GNGGA NMEA sentence for position and fix info
void parseGNGGA(String sentence) {
  String parts[15] = {""};
  splitNMEA(sentence, parts, 15);

  gps.fix = (parts[6].toInt() > 0);         // Fix status: 0 = invalid, >0 = valid
  gps.satellites = parts[7].toInt();       // Number of satellites used
  gps.hdop = parts[8].toFloat();            // Horizontal dilution of precision
  gps.latitude = convertToDecimalDegree(parts[2], parts[3]);  // Latitude in decimal degrees
  gps.longitude = convertToDecimalDegree(parts[4], parts[5]); // Longitude in decimal degrees
}

// Parse GNRMC NMEA sentence for speed and heading info
void parseGNRMC(String sentence) {
  String parts[12] = {""};
  splitNMEA(sentence, parts, 12);

  // Status field (A=valid, V=invalid)
  if (parts[2] == "A") {
    gps.speed = parts[7].toFloat();      // Speed over ground in knots
    gps.heading = parts[8].toFloat();    // Course over ground in degrees
  } else {
    gps.speed = 0;
    gps.heading = 0;
  }
}

// Helper function: split NMEA sentence into fields separated by commas
void splitNMEA(String sentence, String *parts, int maxParts) {
  int startIdx = 0;
  int partIdx = 0;
  while (partIdx < maxParts) {
    int commaIdx = sentence.indexOf(',', startIdx);
    if (commaIdx == -1) {
      // Last field or no more commas
      parts[partIdx++] = sentence.substring(startIdx);
      break;
    }
    parts[partIdx++] = sentence.substring(startIdx, commaIdx);
    startIdx = commaIdx + 1;
  }
}

// Convert NMEA latitude/longitude format (degrees and minutes) to decimal degrees
float convertToDecimalDegree(String val, String direction) {
  if (val.length() < 6) return 0.0;
  float deg = 0.0;
  float min = 0.0;
  if (direction == "N" || direction == "S") {
    deg = val.substring(0, 2).toFloat();  // Extract degrees for latitude (2 digits)
    min = val.substring(2).toFloat();     // Extract minutes for latitude
  } else if (direction == "E" || direction == "W") {
    deg = val.substring(0, 3).toFloat();  // Extract degrees for longitude (3 digits)
    min = val.substring(3).toFloat();     // Extract minutes for longitude
  } else {
    return 0.0;
  }
  float decDeg = deg + min / 60.0;
  if (direction == "S" || direction == "W") decDeg = -decDeg;  // Negative for South and West
  return decDeg;
}
