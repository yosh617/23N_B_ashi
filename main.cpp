
#include "UnbufferedSerial.h"
#include "mbed.h"
#include "BNO055.h"
#include "PIDcontroller.h"
#include <cstdio>
#include <string>
bool debug_log=false;
const int MD[4]={0x26,0x54,0x56,0x50};  // MDアドレス   右前,左前,右後,左後
int speed=40;   // 全体スピード
int max_speed=0xf0; // 最大速度
int min_speed=0x10; // 最低速度
int hosei[4]={0};   // 個体値補正
int duty[4]={0};    // 最終的なduty
char di='s';
char TEMP=di;
const char BRK = 0x80;  // ブレーキ
int senkai_speed=16;    // 旋回速度
int F(int s,int i); // 前進計算
int B(int s,int i); // 後退計算
UnbufferedSerial pc(USBTX,USBRX);  // PCとのUnbuffered Serial
void input(void);   // attach用
bool received=false;    // Unbuffered data received
int index;  // いま何もじめ？
char buffer;    // 1文字
char data[128];  // コマンド文字列
I2C motor(PB_9,PB_8);   // MDとのI2C
// AnalogOut led(LED1);   // led
DigitalOut sig(PA_12);  // 非常停止ボタン   0:動く  1:止まる

//pid
double p=5;    // 補正用 pゲイン
double i=0.01;
double d=0.001;
int O=0;
int Olim=40;
int Ofast=40;
int Oslow=20;
PID pid(p,i,d,0.10);
void function_for_hosei(void);  // 補正
int chijiki_hosei[4]={0};   // 補正結果
int pid_hosei=0;
Ticker ticker_for_hosei;    // Ticker
Ticker ticker_sender;
// エアシリンダーズ     0:伸ばす    1:縮む
// DigitalOut  airF(PA_13);    // 前輪
// DigitalOut  airB(PH_1);     // 後輪
// DigitalOut  airUE(PH_0);    // パタパタエア
BNO055 CHIJIKI(PB_3,PB_10); // ちじき        SDA SCL
float CHIJIKI_=0;   // 地磁気の値 -180~0~180
float raw_CHIJIKI_=0;   // 生のデータ   0~360
double goal=0; // 目標値
void sender(char add,char dat); // モーター動かす
void debugger(void);    // 確認用関数
void show(void);    // 変数確認
void send();  // 動き         direction fbrls
void sensor_reader(void);   // センサー読む
int F(int speed,int i){return BRK+speed+hosei[i]+pid_hosei;};    // 前進計算
int B(int speed,int i){return BRK-speed-hosei[i]-pid_hosei;};    // 後退計算

int M_data[4] = {'0x80','0x80','0x80','0x80'};
Thread send_Mth;
void sender_Mdata();

// main関数
int main(){
    printf("mbed start...\n");
    sig.write(1);
    // airF.write(1);
    // airB.write(1);
    // airUE.write(0);
    CHIJIKI.reset();
    printf("CHIJIKI WAITING\n");
    while(!CHIJIKI.check());
    pc.attach(input,SerialBase::RxIrq);
    ticker_for_hosei.attach(function_for_hosei,100ms);
    ticker_sender.attach(send,30ms);
    send_Mth.start(sender_Mdata);

    printf("loop start!\n");
    while(true){
        sensor_reader();        
        if(received){
            received=false;
            switch(data[0]){
            case 'v':
                switch(data[1]){
                case 's':
                    speed=atoi(&data[2]);
                    printf("speed:%d\n",speed);
                    break;
                case 'p':
                    p=atof(&data[2]);
                    printf("p:%f\n",p);
                    pid.setGain(p,i,d);
                    break;
                case 'i':
                    i=atof(&data[2]);
                    printf("i:%f\n",i);
                    pid.setGain(p,i,d);
                    break;
                case 'd':
                    d=atof(&data[2]);
                    printf("d:%f\n",d);
                    pid.setGain(p,i,d);
                    break;
                case 'g':
                    goal=atof(&data[2]);
                    printf("goal:%f\n",goal);
                    break;
                case 'o':
                    O=atoi(&data[4]);
                    switch(data[3]){
                    case 'f':
                        Ofast=O;
                        break;
                    case 's':
                        Oslow=O;
                        break;
                    case 'l':
                        Olim=O;
                        break;
                    }
                    break;
                case 'l':
                    debug_log=!debug_log;
                    break;
                default:
                    show();
                    break;
                }
                break;
            case 'd':   //d
                if(data[1]=='d'){
                    printf("%d",pid_hosei);
                    continue;
                }
                debugger();
                break;
            case 'p':   //p
                di='s';
                sig.write(1);
                // airF.write(0);
                // airB.write(0);
                // airUE.write(0);
                // ue_power.write(0);
                printf("pause!\n");
                break;
            case 'c':   //c
                di='s';
                sig.write(0);
                // airF.write(0);
                // airB.write(0);
                // airUE.write(1); 
                // ue_power.write(1);
                printf("continue!\n");
                break;
            case 'a':   // 足回り
                speed=atoi(&data[2]);
                switch(data[1]){
                case 's':
                    Olim=Ofast;
                    di='s';
                    di='s';
                    break;
                default:    // それ以外
                    //printf("data:%c\n",data[1]);
                    Olim=Oslow;
                    di=data[1];   //  a + f,b,r,l
                    break;
                }
                break;
            case 's':   // 旋回
                switch(data[1]){
                    case 's':
                        Olim=Ofast;
                        di='s';
                        break;
                    case 'h':
                        Olim=Oslow;
                        di='s';
                        switch(data[2]){
                        case 'r':
                            goal+=90;
                            if(goal==270)goal=-90;
                            break;
                        case 'l':
                            goal-=90;
                            if(goal==-270)goal=90;
                            break;
                        }
                        break;
                }
                    break;
            }
            char data[128]="";
        }
        if(debug_log && TEMP!=di)printf("di:%c\n",di);
        TEMP=di;
    }
}

void input(){
    pc.read(&buffer,1);
    data[index]=buffer;
    index++;
    if(data[index-1]=='\n'){
        data[index]='\0';
        index=0;
        received=true;
        if(data[0]=='p'){
            sig=1;
            pc.write("!\n",1);
        }
    }
}

void sender(char add,char dat){
    motor.start();
    motor.write(add);
    if(dat<min_speed)dat=min_speed;
    else if(max_speed<dat)dat=max_speed;
    // if(debug_log&&dat!=128)printf("dat: %d\n",dat);
    motor.write(dat);
    motor.stop();
    // wait_us(100);
    ThisThread::sleep_for(1ms);
}

void send(){
    switch(di){
    case 'f':   // 前
        for(int i=0;i<4;i++){
            // sender(MD[i],F(speed,i));
            M_data[i] = F(speed,i);
        }
        break;
    case 'b':   // 後
        for(int i=0;i<4;i++){
            // sender(MD[i],B(speed,i));
            M_data[i] = B(speed,i);
        }
        break;
    case 'r':   // 右
        for(int i=0;i<4;i++){
            if(i==0||i==3){
                // sender(MD[i],B(speed,i));
                M_data[i] = B(speed,i);
            }else{
                // sender(MD[i],F(speed,i));
                M_data[i] = F(speed,i);
            }
        }
        break;
    case 'l':   // 左
        for(int i=0;i<4;i++){
            if(i==0||i==3){
                // sender(MD[i],F(speed,i));
                M_data[i] = F(speed,i);
            }else{
                // sender(MD[i],B(speed,i));
                M_data[i] = B(speed,i);
            }
        }
        break;
    case 'm':   // 右旋回
        for(int i=0;i<4;i++){
            if(i==0||i==2){
                // sender(MD[i],B(speed,i)+senkai_speed);
                M_data[i] = (B(speed,i)+senkai_speed);
            }else{
                // sender(MD[i],F(speed,i)-senkai_speed);
                M_data[i] = (F(speed,i)+senkai_speed);
            }
        }
        break;
    case 'h':   // 左旋回
        for(int i=0;i<4;i++){
            if(i==1||i==3){
                // sender(MD[i],B(speed,i)+senkai_speed);
                M_data[i] = (B(speed,i)+senkai_speed);
            }else{
                // sender(MD[i],F(speed,i)-senkai_speed);
                M_data[i] = (F(speed,i)+senkai_speed);
            }
        }
        break;
    case 's':
        for(int i=0;i<4;i++){
            // sender(MD[i],BRK+chijiki_hosei[i]);
            M_data[i] = BRK+chijiki_hosei[i];
        }
        break;
    default:
        for(int i=0;i<4;i++){
            // sender(MD[i],BRK+chijiki_hosei[i]);
            M_data[i] = BRK+chijiki_hosei[i];
        }
        break;
    }
}

void sender_Mdata(){
    while (true)
    {
        for(int i=0;i<4;i++){
            sender(MD[i],M_data[i]);
            ThisThread::sleep_for(1ms);
        }
    }
}


void function_for_hosei(){
    // sensor_reader();
    float now=CHIJIKI_;    // 実際の値
    float error=goal-now;    // あとどれくらい動かす必要があるか＝0°からの誤差(±180°)
    while (error < -180) {
        error += 360;
    }
    while (error > 180) {
        error -= 360;
    }
    pid.setSetPoint(0);    // 常に0を目指す
    pid.setInputLimits(-180,0);    // 符号を外すことで計算を楽に    
    pid.setOutputLimits(0,Olim);    // 最大補正速度はOlim
    pid.setProcessValue(-abs(error));    // errorの値= 0からどれくらい離れているか なので0を目標のPIDではerrorをいれればいい    
    pid_hosei= pid.compute();    // 計算
    if(error<0)pid_hosei*=-1;    // もし左に動く必要がある(errorがマイナス)なら計算結果（符号なし）に-1をかける
    chijiki_hosei[0]=-pid_hosei;    // 右旋回をするイメージ（もしpid_hoseiがマイナスならマイナス方向に右旋回＝左旋回になる）
    chijiki_hosei[1]=pid_hosei;
    chijiki_hosei[2]=-pid_hosei;
    chijiki_hosei[3]=pid_hosei;
}



void sensor_reader(){
    // 地磁気の相対角（初期位置からの）を取得
    CHIJIKI.setmode(OPERATION_MODE_IMUPLUS);   //魔法
    CHIJIKI.get_angles();
    raw_CHIJIKI_=CHIJIKI.euler.yaw;     //取得  0~360
    CHIJIKI_=raw_CHIJIKI_;
    while (CHIJIKI_ > 180) {    // CHIJIKI_<180
        CHIJIKI_ -= 360;
    }
    while (CHIJIKI_ < -180) {   // -180<CHIJIKI_
        CHIJIKI_ += 360;
    }
                            // -180 < CHIJIKI < 180
}

void debugger(){
    printf("+-----------------------------\n");
    printf("| sig             :   %d\n",sig.read());
    printf("| CHIJIKI_        :   %f\n",CHIJIKI_);
    // printf("| raw_CHIJIKI_    :   %f\n",raw_CHIJIKI_);
    printf("| speed           :   %d\n",speed);
    printf("| CJK hosei       :   %d %d %d %d\n",chijiki_hosei[0],chijiki_hosei[1],chijiki_hosei[2],chijiki_hosei[3]);
    printf("| direct          :   %c\n",di);
    printf("+-----------------------------\n");
    show();
    // printf("| motor           :   MM HM MU HU\n");
    // printf("| hosei           :   %d %d %d %d\n",hosei[0],hosei[1],hosei[2],hosei[3]);
    // printf("| final duty      :   %d %d %d %d\n",duty[0],duty[1],duty[2],duty[3]);
    // printf("+------------------------------------\n");
}


void show(){
    printf("+-----------------------------\n");
    // printf("| speed             (s) :%d\n",speed);
    printf("| p                 (p) :%f\n",p);
    printf("| i                 (i) :%f\n",i);
    printf("| d                 (d) :%f\n",d);
    printf("| pid_hosei         (X) :%d\n",pid_hosei);
    printf("| goal              (g) :%f\n",goal);
    printf("| Olim              (ol) :%d\n",Olim);
    printf("| Oslow             (os):%d\n",Oslow);
    printf("| Ofast             (of):%d\n",Ofast);
    float now=CHIJIKI_;    // 実際の値
    float error=goal-now;    // あとどれくらい動かす必要があるか＝0°からの誤差(±180°)
    while (error < -180) {
        error += 360;
    }
    while (error > 180) {
        error -= 360;
    }
    printf("| error                 :%f\n",error);

    // printf("| ")
    printf("+-----------------------------\n");
}