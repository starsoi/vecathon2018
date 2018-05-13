#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

const char* ssid= "Connectify-feuermann";
const char* password="feuermann";
String request = String("NO_DATA");
String instruction = String("NO_INSTRUCTION");

WiFiClient client;
//client._timeout = 10;

const int httpPort = 8080;

const char* host = "192.168.96.1";
SoftwareSerial mySerial(5, 16); //Rx,Tx
void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); 
}



  
  // We now create a URI for the request
  String url = "/get";
  String line1 = String("INIT_LINE");
  String line2 = String("INIT_LINE");
  String line3 = String("INIT_LINE");
void loop() {
  delay(500);
  // Use WiFiClient class to create TCP connections

  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
 }
  
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
 
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 500) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
   if (client.available()){
            line1 = client.readStringUntil('$');
            line2 = client.readStringUntil('$');
            line3 = client.readStringUntil('$');   
   }
line1=line1.substring(143);
Serial.println(line1);
Serial.println(line2);
Serial.println(line3);

  // Read all the lines of the reply from server and print them to Serial
  if (mySerial.available()){
   
    request = mySerial.readStringUntil('!');
    request = request.substring(request.length()-2,request.length());   
    //Serial.println(request);
  if(request.equals(String("OK"))){
            //Serial.println(request);
            mySerial.print(String("1234")+ String(";")+line2 + String(";")+line1+ String(";")+line3 + String(";"));

            request = String("NO_DATA");
   }
  }
}
