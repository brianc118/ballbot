#define BT Serial1

#define BAUD_9600 0
#define BAUD_115200 1
#define BAUD_230400 2
#define BAUD_460800 3
#define BAUD_921600 4
#define BAUD_1382400 5

#define RN42 0
#define HC06 1

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
            case 0: BT.print("AT+BAUD4"); delay(500); BT.begin(9600);   break;
            case 1: BT.print("AT+BAUD8"); delay(500); BT.begin(115200); break;
            case 2: BT.print("AT+BAUD9"); delay(500); BT.begin(230400); break;
            case 3: BT.print("AT+BAUDA"); delay(500); BT.begin(460800); break;
            case 4: BT.print("AT+BAUDB"); delay(500); BT.begin(921600); break;
            case 5: BT.print("AT+BAUDC"); delay(500); BT.begin(1382400); break;
        }
    }
}

bool bluetoothEcho(){
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