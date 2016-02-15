#define BT Serial1

#define BAUD_9600 0
#define BAUD_115200 1
#define BAUD_230400 2
#define BAUD_460800 3
#define BAUD_921600 4
#define BAUD_1382400 5

#define RN42 0
#define HC06 1
#define HM11 2

void btSetup(uint8_t module, int initBaud, int finalBaud){
    if (module == RN42){

    }
    else if (module == HC06){
        BT.begin(initBaud);
        delay(100);
        Serial.print("AT");
        BT.println("AT");

        delay(1000);

        switch (finalBaud){
            case BAUD_9600:    BT.print("AT+BAUD4"); delay(500); BT.begin(9600);   break;
            case BAUD_115200:  BT.print("AT+BAUD8"); delay(500); BT.begin(115200); break;
            case BAUD_230400:  BT.print("AT+BAUD9"); delay(500); BT.begin(230400); break;
            case BAUD_460800:  BT.print("AT+BAUDA"); delay(500); BT.begin(460800); break;
            case BAUD_921600:  BT.print("AT+BAUDB"); delay(500); BT.begin(921600); break;
            case BAUD_1382400: BT.print("AT+BAUDC"); delay(500); BT.begin(1382400); break;
        }
    }
    else if (module == HM11){
        BT.begin(initBaud);
        delay(100);
        Serial.print("AT");
        BT.print("AT");

        delay(1000);

        switch (finalBaud){
            case BAUD_9600:    BT.print("AT+BAUD0"); delay(500); BT.begin(9600);   break;
            case BAUD_115200:  BT.print("AT+BAUD4"); delay(500); BT.begin(115200); break;
            case BAUD_230400:  BT.print("AT+BAUD8"); delay(500); BT.begin(230400); break;
            // case BAUD_460800:  BT.print("AT+BAUDA"); delay(500); BT.begin(460800); break;
            // case BAUD_921600:  BT.print("AT+BAUDB"); delay(500); BT.begin(921600); break;
            // case BAUD_1382400: BT.print("AT+BAUDC"); delay(500); BT.begin(1382400); break;
            default: break;
        }

    }
}

bool bluetoothEcho(){
    Serial.println("hi");
    char c;
    if(BT.available()){
        Serial.print((char)BT.read());  
    }
    if(Serial.available()){
        c = Serial.read();
        if(c == '~') return false;
        BT.print(c);
    }
    return true;
}

#define CLEARBT(){                  \
    while (BT.available()){         \
        BT.read();    }}              \