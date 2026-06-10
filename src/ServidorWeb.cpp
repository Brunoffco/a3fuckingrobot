// ServidorWeb.cpp - DISABLED (using TelemetriaWiFi.cpp instead)
// Keep this file for reference, but all WiFi functionality is now in TelemetriaWiFi.cpp

#if 0  // Disabled

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

const char* AP_SSID = "SDL";
const char* AP_SENHA = "murdock#";

WebServer webServer(80);

void arquivoHTML() {
  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    webServer.send(500, "text/plain", "Erro ao carregar a página");
    return;
  }

  String html = file.readString();
  file.close();
  webServer.send(200, "text/html", html);
}

void arquivoCSS() {
  File file = LittleFS.open("/styles.css", "r");
  if (!file) {
    webServer.send(500, "text/plain", "Erro ao carregar o CSS");
    return;
  }

  String css = file.readString();
  file.close();
  webServer.send(200, "text/css", css);
}

void arquivoJS() {
  File file = LittleFS.open("/script.js", "r");
  if (!file) {
    webServer.send(500, "text/plain", "Erro ao carregar o JS");
    return;
  }

  String js = file.readString();
  file.close();
  webServer.send(200, "application/javascript", js);
}

void configurarServidorWeb() {
  delay(1000);

  if (!LittleFS.begin(true)) {
    Serial.println("Erro ao iniciar LittleFS");
    return;
  }

  IPAddress local_IP(172, 16, 16, 1);
  IPAddress gateway(172, 16, 16, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_SENHA);

  Serial.print("IP do AP: ");
  Serial.println(WiFi.softAPIP());

  webServer.on("/", arquivoHTML);
  webServer.on("/styles.css", arquivoCSS);
  webServer.on("/script.js", arquivoJS);

  webServer.begin();
  Serial.println("Servidor HTTP iniciado!");
}

void servidorAtivo() {
  webServer.handleClient();
}

#endif  // Disabled
