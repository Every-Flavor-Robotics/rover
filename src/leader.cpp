#include <Arduino_JSON.h>
#include <WiFi.h>
#include <esp_now.h>
#include <motorgo_group_leader.h>

WiFiUDP udp;
unsigned int localUdpPort = 8008;  // local port to listen on
char incomingPacket[255];          // buffer for incoming packets

MotorGoGroupLeader leader;

void setup()
{
  Serial.begin(115200);

  delay(3000);

  // Connect to Wi-Fi
  WiFi.mode(WIFI_AP_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Start listening for UDP messages
  Serial.println("Connected to WiFi");
  udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s on UDP port %d\n",
                WiFi.localIP().toString().c_str(), localUdpPort);

  // Create array of strings of device names
  std::vector<String> device_names;
  device_names.push_back("rear_wheels");

  leader.init(device_names);
}

void loop()
{
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize,
                  udp.remoteIP().toString().c_str(), udp.remotePort());
    int len = udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
  }

  CommandMessage message;
  message.command = 10;
  message.enabled = false;

  leader.send_message("rear_wheels", message);
  delay(10);
}
