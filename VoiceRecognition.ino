#include <driver/i2s.h> 
#include <WiFiClientSecure.h> 
#include "network_param.h" 
#define I2S_WS 25 
#define I2S_SD 33 
#define I2S_SCK 32 
#define I2S_PORT I2S_NUM_0 
#define I2S_SAMPLE_RATE (16000) 
#define I2S_SAMPLE_BITS (16) // 24 
#define I2S_CHANNEL_BITS (32) 
#define RECORD_TIME (2) //Seconds 
#define I2S_CHANNEL_NUM (1) 
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME) // 1 * 16000 * 2 * 2 = 64.000 bytes 
#define HEADER_SIZE 44 
#define HEADER_SIZE_2BYTES 48 
#define HEADER_SIZE_BASE64 60 
#define RECORD_STORAGE_SIZE ((FLASH_RECORD_SIZE * 4 / 3) + HEADER_SIZE_BASE64) 

WiFiClientSecure client; 

void i2sInit() { 
  i2s_config_t i2s_config = { 
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), 
    .sample_rate = I2S_SAMPLE_RATE, 
    .bits_per_sample = i2s_bits_per_sample_t(I2S_CHANNEL_BITS), //I2S_BITS_PER_SAMPLE_24BIT,//I2S_BITS_PER_SAMPLE_16BIT, 
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //I2S_CHANNEL_FMT_ONLY_LEFT, 
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = 0, 
    .dma_buf_count = 2, 
    .dma_buf_len = 1024, 
    .use_apll = 1, 
    //.bits_per_chan = i2s_bits_per_chan_t(I2S_CHANNEL_BITS) 
    }; 
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL); 
    
    const i2s_pin_config_t pin_config = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = -1, .data_in_num = I2S_SD }; 
    
    i2s_set_pin(I2S_PORT, &pin_config);
} 

void wavHeader(byte* header, int wavSize) { 
  // header plain binary 
  // 16 bit format header[0] = 'R'; 
  header[1] = 'I'; header[2] = 'F'; 
  header[3] = 'F'; 
  unsigned int fileSize = wavSize + HEADER_SIZE - 8; // 2 detik ==> 16.000 * 2 bytes * 2 detik = 64.036 ==> 00 00 FA 00 
  header[4] = (byte)(fileSize & 0xFF); 
  // 24 header[5] = (byte)((fileSize >> 8) & 0xFF); 
  // FA header[6] = (byte)((fileSize >> 16) & 0xFF); 
  // 00 header[7] = (byte)((fileSize >> 24) & 0xFF); 
  // 00 header[8] = 'W'; 
  header[9] = 'A'; 
  header[10] = 'V'; 
  header[11] = 'E'; 
  header[12] = 'f'; 
  header[13] = 'm'; 
  header[14] = 't';
  header[15] = ' '; 
  header[16] = 0x10; // Length of format data as listed above 
  header[17] = 0x00; 
  header[18] = 0x00; 
  header[19] = 0x00; 
  header[20] = 0x01; // Type of format (1 is PCM) - 2 byte integer 
  header[21] = 0x00; 
  header[22] = 0x01; // Number of Channels => 1 
  header[23] = 0x00; 
  header[24] = 0x80; // Sample Rate - 32 byte integer 0x3E80 = 16000 
  header[25] = 0x3E; 
  header[26] = 0x00; 
  header[27] = 0x00; 
  header[28] = 0x00; //0x00; // (Sample Rate * BitsPerSample * Channels) / 8 ==> 0x7D00 = 32000 <= 16.000 * 16 * 1 /8 
  header[29] = 0x7D; //0x7D; 
  header[30] = 0x00; 
  header[31] = 0x00; 
  header[32] = 0x02; //0x02; // (BitsPerSample * Channels) / 8) => 16 * 1 / 8 = 2 ==> 16 bit mono or 8 bit stereo 
  header[33] = 0x00; 
  header[34] = 0x10; //0x10; // Bits per sample ==> 0x10 = 16 
  header[35] = 0x00; 
  header[36] = 'd'; 
  header[37] = 'a'; 
  header[38] = 't';
  header[39] = 'a'; 
  header[40] = (byte)(wavSize & 0xFF); // 00 
  header[41] = (byte)((wavSize >> 8) & 0xFF); // FA 
  header[42] = (byte)((wavSize >> 16) & 0xFF); // 00 
  header[43] = (byte)((wavSize >> 24) & 0xFF); // 00 
} 

static const char b64_table[] = { 
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 
  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 
  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 
  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 
  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 
  'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 
  'w', 'x', 'y', 'z', '0', '1', '2', '3', 
  '4', '5', '6', '7', '8', '9', '+', '/' 
}; 

// base64 encode from 3 byte src buffer to 4 byte dst buffer 
  
void bs64_enc34(unsigned char* src, unsigned char* dst, size_t len) { 
  unsigned char tmp[3]; uint8_t i = 0; 
  uint16_t spos=0, dpos = 0; 
  // size = 0, num = 0; 
  while(len) { 
    // read up to 3 bytes at a time into `tmp' 
    tmp[i] = src[spos+i]; 
    i++; // if 3 bytes read then encode into destination 
    if (3 == i) { 
      dst[dpos] = b64_table[tmp[0] >> 2]; 
      dst[dpos+1] = b64_table[((tmp[0] & 0x03) << 4) + (tmp[1] >> 4)]; 
      dst[dpos+2] = b64_table[((tmp[1] & 0x0f) << 2) + (tmp[2] >> 6)]; 
      dst[dpos+3] = b64_table[tmp[2] & 0x3f]; 
      spos += 3; 
      dpos += 4; 
      i = 0; 
      } 
      len--; 
  } 
  //Serial.printf("%d-%d ",i,len); 
  if (i > 0) { 
    dst[dpos] = b64_table[tmp[0] >> 2]; 
    if (i == 1) { 
      dst[dpos+1] = b64_table[(tmp[0] & 0x03) << 4]; 
      dst[dpos+2] = '='; 
    } 
    else { 
      // if counts == 2 
      dst[dpos+1] = b64_table[((tmp[0] & 0x03) << 4) + (tmp[1] >> 4)]; 
      dst[dpos+2] = b64_table[(tmp[1] & 0x0f) << 2];
    } 
    dst[dpos+3] = '='; 
  } 
} 

bool connstate = 0; 
void setup() { 
  // put your setup code here, to run once: const char* ssid = "PUT YOUR OWN WIFI NAME"; 
  const char* password = "PUT YOUR OWN PASSWORD"; 
  Serial.begin(115200); Serial.println(millis()); 
  Serial.println(ESP.getFreeHeap()); 
  Serial.println(F("init i2s")); 
  i2sInit(); 
  //for (int i = 0; i < 5; i++) { 
    //i2s_read(I2S_NUM_0, test_data, 2048, &i2slen, portMAX_DELAY); 
    //} Serial.printf("Connecting to %s\n", ssid); 
    if (String(WiFi.SSID()) != String(ssid)) { 
      WiFi.mode(WIFI_STA); WiFi.begin(ssid, password); 
    } 
    while (WiFi.status() != WL_CONNECTED) { 
      delay(500); Serial.print("."); 
    } Serial.println(); 
    Serial.print(F("Connected! IP address: ")); 
    Serial.println(WiFi.localIP()); 
} 

// transport len 960 become 1280 base64 data 
#define TRANSPORT_LENGTH 960 // 768 // 960 // 1056 // 1152 
#define TRANSPORT_LENGTH_B64 (TRANSPORT_LENGTH * 4 / 3) 
#define SEGMENT_LENGTH 2160 // 2048 
#define BUFFER_COUNT 48 

void print_http_gspeech(uint8_t ** audio, uint32_t conlen) { 
  String HttpBody1 = 
  "{\"config\":{\"encoding\":\"LINEAR16\",\"sampleRateHertz\":16000,\"languageCode\":\"id-ID\"},\"audio\":{\"content\":\""; 
  String HttpBody3 = "\"}}\r\n\r\n"; 
  //int httpBody2Length = (audio->wavDataSize + sizeof(audio->paddedHeader)) * 4 / 3; // 4/3 is from base64 encoding 
  String ContentLength = String(HttpBody1.length() + conlen*4/3 + HttpBody3.length());
  String HttpHeader; 
  // if (authentication == USE_APIKEY) 
  HttpHeader = String("POST /v1/speech:recognize?key=") 
              + ApiKey 
              + String(" HTTP/1.1\r\nHost: speech.googleapis.com\r\nContent-Type: application/json\r\nContent-Length: ") 
              + ContentLength 
              + String("\r\n\r\n");
  
  int idx = 0, pos = 0, lap = 0, len, iter = 0; 
  client.print(HttpHeader); 
  client.print(HttpBody1); 
  //Serial.println(conlen); 
  uint8_t httpdata[TRANSPORT_LENGTH_B64]; 
  while(conlen){ 
    if(conlen < TRANSPORT_LENGTH){ 
      len = conlen; 
    }
    else { 
      len = TRANSPORT_LENGTH; 
    } 
    
    if(pos + len <= SEGMENT_LENGTH){ 
      bs64_enc34(audio[idx]+pos, httpdata, len); 
      if(pos + len == SEGMENT_LENGTH){ 
        pos = 0; idx++; 
      } 
      else { 
        pos += len; 
      }
    } 
    
    else { 
      lap = 0; bs64_enc34(audio[idx]+pos, httpdata, SEGMENT_LENGTH - pos); // 2048 - 1536 = 512 
      idx++; 
      lap += (SEGMENT_LENGTH - pos) * 4 / 3; 
      bs64_enc34(audio[idx], httpdata+lap, len + pos - SEGMENT_LENGTH); // 768 + 1536 - 2048 = 256 
      pos += len - SEGMENT_LENGTH; 
    }
    
    client.write(httpdata, len*4/3); 
    conlen -= len; 
  } 
  
  //client.write(audio,conlen); 
  client.print(HttpBody3); 
} 

void loop() { 
  Serial.println(FLASH_RECORD_SIZE); 
  Serial.println(RECORD_STORAGE_SIZE); 
  Serial.println(millis());
  Serial.println(ESP.getFreeHeap()); 
  uint8_t i2s_data[SEGMENT_LENGTH]; 
  uint32_t i2slen, record_size, inavail = 0, noavail = 0; 
  const char* server = "speech.googleapis.com"; 
  uint16_t i, curpos, actlen; 
  uint8_t arrpos; //byte0, byte1, byte2, byte3, tmprev[2]; 
  int32_t maxd = 0, mind = 0; 
  //WiFiClientSecure client; 
  
  for (i = 0; i < 5; i++) { 
    i2s_read(I2S_NUM_0, i2s_data, SEGMENT_LENGTH, &i2slen, portMAX_DELAY); 
  } 
  
  client.setCACert(root_ca); 
  
  Serial.println(F("***Allocate Buffer***")); 
  uint8_t *recdata[BUFFER_COUNT]; 
  for(i=0;i<BUFFER_COUNT;i++){ 
    recdata[i] = (uint8_t *)calloc(SEGMENT_LENGTH,1); 
    if(recdata[i] == NULL) { 
      Serial.print(F("Failed Buffer ")); 
      Serial.println(i); 
      while(1); 
    } 
    Serial.print('.'); 
  } 
  Serial.println(); 
  Serial.println(millis()); 
  Serial.println(ESP.getFreeHeap()); 
  wavHeader(recdata[0], FLASH_RECORD_SIZE); 
  
  while (1) { 
    record_size = 0; 
    arrpos = 0; 
    curpos = 44; //48; // 56; // 60; 
    //flash_wrsize = HEADER_SIZE; 
    if (true) { 
      // button press conditional if any 
      Serial.println(F("***Start Recording***")); 
      while (record_size < FLASH_RECORD_SIZE) { 
        //read data from I2S bus of INMP441 
        i2slen = 0; i2s_read(I2S_NUM_0, i2s_data, SEGMENT_LENGTH, &i2slen, 0); 
        if (i2slen) { 
          if ((i2slen/8)*2 + record_size > FLASH_RECORD_SIZE) { 
            i2slen = ((FLASH_RECORD_SIZE - record_size)/2) * 8; 
          } 
          actlen = 0; 
          for (i = 0; i < i2slen; i = i + 8) {
            int32_t tmpd = ((((int32_t)(int8_t)i2s_data[i+3])) << 17) | (((uint32_t)i2s_data[i+2]) << 9) | (((uint32_t)i2s_data[i+1]) << 1) | (((uint32_t)i2s_data[i]) >> 7); 
            if(tmpd > maxd){ 
              maxd = tmpd; 
            } 
            else if (tmpd < mind) { 
              mind = tmpd; 
            } 
            tmpd = tmpd / 64; 
            recdata[arrpos][curpos] = ((uint8_t) (tmpd & 0xFF)); 
            recdata[arrpos][curpos+1] = (uint8_t) (tmpd >> 8); 
            curpos += 2; 
            if(curpos == SEGMENT_LENGTH){ 
              curpos = 0; arrpos++; 
            } 
            actlen += 2; // 3 6 
          } 
          Serial.print('.'); 
          record_size += actlen; 
          //Serial.printf("%d %d %d %d %d\n",i2slen,actlen,record_size,arrpos,curpos); 
          inavail++; 
        } 
        else { 
          noavail++; 
        } 
      } 
      Serial.println(); 
      Serial.println(millis()); 
      Serial.println(ESP.getFreeHeap()); 
      Serial.println(F("***Recording Finish***")); 
      Serial.printf("%d %d %d %d %d %d\n", inavail, noavail, record_size, actlen, mind, maxd); 
      if (client.connect(server, 443)) { 
        connstate = 1; 
      } 
      else { 
        connstate = 1; Serial.println("Connection failed!"); 
        while(1); 
      } 
      Serial.println(millis()); 
      Serial.println(ESP.getFreeHeap()); 
      Serial.println(F("Send Request")); 
      print_http_gspeech(recdata,FLASH_RECORD_SIZE + HEADER_SIZE); 
      Serial.println(); String text_return = ""; 
      Serial.println(millis());
      Serial.println(ESP.getFreeHeap()); 
      Serial.println(F("Wait for Reply")); 
      while (!client.available()); 
      
      while (client.available()) { 
        char temp = client.read(); 
        text_return += temp; 
      } 
      Serial.println(); 
      Serial.println(text_return); 
      Serial.println(millis()); 
      Serial.println(ESP.getFreeHeap()); 
      if ((text_return.indexOf("ikut") != -1 && text_return.indexOf("Om") != -1) || (text_return.indexOf("sini") != -1 && text_return.indexOf("ikut") != -1 || text_return.indexOf("Om") != -1)) { 
        Serial1.begin(9600, SERIAL_8N1,27,26); // open serial over TX and RX pins 
        Serial.println("start"); 
        uint8_t vol20[6] = {0x06,0xF9,0x02,0x01,0x14,0x16}; 
        //uint8_t play[5] = {0x04,0xFB,0x01,0x01,0x01}; 
        //uint8_t play[7] = {0x04,0xFB,0x03,0x06,0x00,0x01,0x09}; 
        uint8_t play[7] = {0x04,0xFB,0x03,0x06,0x00,0x02,0x0A}; 
        Serial1.write(vol20,6); 
        delay(1000); 
        Serial1.write(play,7); 
        Serial.println("end"); 
      } 
      else if ((text_return.indexOf("om") != -1 && text_return.indexOf("makanan") != -1) || (text_return.indexOf("sini") != -1 && text_return.indexOf("permen") != -1)) { 
        Serial1.begin(9600, SERIAL_8N1,27,26); // open serial over TX and RX pins 
        Serial.println("start"); uint8_t vol20[6] = {0x06,0xF9,0x02,0x01,0x14,0x16}; 
        //uint8_t play[5] = {0x04,0xFB,0x01,0x01,0x01}; 
        uint8_t play[7] = {0x04,0xFB,0x03,0x06,0x00,0x01,0x09}; 
        //uint8_t play[7] = {0x04,0xFB,0x03,0x06,0x00,0x02,0x0A}; 
        Serial1.write(vol20,6); 
        delay(1000); 
        Serial1.write(play,7); 
        Serial.println("end"); 
      } 
      while(1); 
    } 
    // if (true) 
  }
  // while (1)
}
