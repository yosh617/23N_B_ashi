
#include "UnbufferedSerial.h"
#include "mbed.h"
#include "BNO055.h"
#include "PIDcontroller.h"
#include <cstdio>
bool debug_log=false;
const int MD[4]={0x26,0x54,0x56,0x50};  // MDアドレス   右前,左前,右後,左後
int WOOD=150;   // [mm]
int speed=40;   // 全体スピード
int max_speed=0xf0; // 最大速度
int min_speed=0x10; // 最低速度
int hosei[4]={0};   // 個体値補正
int duty[4]={0};    // 最終的なduty
char di='s';
char TEMP=di;
int kakuzai_fast_speed=40;   // 角材超えるときのスピード
int kakuzai_slow_speed=30;
const char BRK = 0x80;  // ブレーキ
int senkai_speed=16;    // 旋回速度
int state=0;    // 0:準備中   1:main  2:auto_run
bool auto_running=false;    // auto_run予備停止用
int F(int s,int i); // 前進計算
int B(int s,int i); // 後退計算
// BufferedSerial pc(USBTX,USBRX); // PCとのシリアル(state==1)
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
// エアシリンダーズ     0:伸ばす    1:縮む
DigitalOut  airF(PA_13);    // 前輪
DigitalOut  airB(PH_1);     // 後輪
DigitalOut  airUE(PH_0);    // パタパタエア
DigitalOut ue_power(PC_8);  // 上電源確認君

// 赤外線センサーズ
AnalogIn    sensorF(PA_6);  // 前
AnalogIn    sensorB(PA_7);  // 後
float value[2]; // 生の値
double dis[2]={};   // 計算後の値
BNO055 CHIJIKI(PB_3,PB_10); // ちじき        SDA SCL
float CHIJIKI_=0;   // 地磁気の値 -180~0~180
float raw_CHIJIKI_=0;   // 生のデータ   0~360
double goal=0; // 目標値
int flag = 0;   // 角材用フラグ
bool finish = false;    // 角材終了判定
void sender(char add,char dat); // モーター動かす
void sensor_reader(void);   // センサー読む
void auto_run(void);    // 角材
// float compute_dig(float d1,float d2);   // 角度の差を計算する
void debugger(void);    // 確認用関数
void show(void);    // 変数確認
void send(char d);  // 動き         direction fbrls
int F(int speed,int i){return BRK+speed+hosei[i]+chijiki_hosei[i];};    // 前進計算
int B(int speed,int i){return BRK-speed-hosei[i]-chijiki_hosei[i];};    // 後退計算

// main関数
int main(){
    printf("mbed start...\n");
    ue_power.write(0);
    sig.write(1);
    airF.write(0);
    airB.write(0);
    airUE.write(0);
    CHIJIKI.reset();
    printf("CHIJIKI WAITING\n");
    while(!CHIJIKI.check());
    pc.attach(input,SerialBase::RxIrq);
    ticker_for_hosei.attach(function_for_hosei,100ms);
    state=1;
    printf("loop start!\n");
    while(true){
        sensor_reader();
        // debugger();
        // printf("%d\n",pid_hosei);
        if(state==0){
            send('s');
            state=1;
            received=false;
        }else if(state==1){
            // if(pc.read(&buffer,1)>0){   // PCから受信したら
                // led=!led;
            if(received){
                received=false;
                // printf("cmd:%s\n",data);
                switch(data[0]){
                case 'v':
                    switch(data[1]){
                    case 'k':   //vk
                        switch(data[2]){
                        case 'w':   //vw
                            WOOD=atoi(&data[3]);
                            printf("WOOD:%d\n",WOOD);
                            break;
                        case 's':
                            kakuzai_slow_speed=atoi(&data[3]);
                            printf("kakuzai_slow_speed%d\n",kakuzai_slow_speed);
                            break;
                        case 'f':
                            kakuzai_fast_speed=atoi(&data[3]);
                            printf("kakuzai_fast_speed%d\n",kakuzai_fast_speed);
                            break;
                        }
                        break;
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
                    send('s');
                    sig.write(1);
                    airF.write(0);
                    airB.write(0);
                    airUE.write(0);
                    ue_power.write(0);
                    printf("pause!\n");
                    break;
                case 'c':   //c
                    send('s');
                    sig.write(0);
                    airF.write(0);
                    airB.write(0);
                    airUE.write(1); 
                    ue_power.write(1);
                    printf("continue!\n");
                    break;
                case 'a':   // 足回り
                    speed=atoi(&data[2]);
                    //printf("a:%c\n",data[1]);
                    switch(data[1]){
                    case 'a':
                        switch(data[2]){
                        case 'f':
                            switch(data[3]){
                            case 'u':
                                airF.write(1);  // aafu
                                break;
                            case 'd':
                                airF.write(0);  //aafd
                                break;
                            }
                            break;
                        case 'b':
                            switch(data[3]){
                            case 'u':
                                airB.write(1);  //aabu
                                break;
                            case 'd':
                                airB.write(0);  //aabd
                                break;
                            }
                            break;
                        }
                        break;
                    case 's':
                        Olim=Ofast;
                        di='s';
                        send('s');
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
                    // case 'r':
                    //     speed=atoi(&data[2]);
                    //     send('m');
                    //     break;
                    // case 'l':
                    //     speed=atoi(&data[2]);
                    //     send('h');
                    //     break;
                    case 's':
                        Olim=Ofast;
                        di='s';
                        send('s');
                        break;
                    case 'h':
                        Olim=Oslow;
                        di='s';
                        switch(data[2]){
                        case 'r':
                            goal+=90;
                            if(goal==360)goal=0;
                            break;
                        case 'l':
                            goal-=90;
                            if(goal==-360)goal=0;
                            break;
                        }
                        break;
                    }
                    break;
                case 'k':
                    airF.write(0);  // age
                    airB.write(0);  // age
                    printf("kakuzai k\n");
                    speed=kakuzai_slow_speed;    // slow...
                    di='f';  // going
                    auto_running=true;  // allow auto run
                    auto_run();
                    break;
                }
                char data[128]="";
            }
            if(debug_log && TEMP!=di)printf("di:%c\n",di);
            TEMP=di;
            send(di);
        }else if(state==2){
            if(received){
                received=false;
                if(data[0]=='p'){
                    printf("emergency stop\n");
                    state=0;
                }
            }
            if(state==2){
                send('f');
                auto_run();
            }
        }
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
        if(state==2){
            if(data[0]=='p' ){//or data[0]=='k'){
                sig=1;
                state=0;
            }
        }
    }
}

void sender(char add,char dat){
    motor.start();
    motor.write(add);
    if(dat<min_speed)dat=min_speed;
    else if(max_speed<dat)dat=max_speed;
    if(debug_log&&dat!=128)printf("dat: %d\n",dat);
    motor.write(dat);
    motor.stop();
    wait_us(100);
}

void send(char d){
    switch(d){
    case 'f':   // 前
        for(int i=0;i<4;i++){
            sender(MD[i],F(speed,i));
        }
        break;
    case 'b':   // 後
        for(int i=0;i<4;i++){
            sender(MD[i],B(speed,i));
        }
        break;
    case 'r':   // 右
        for(int i=0;i<4;i++){
            if(i==0||i==3){
                sender(MD[i],B(speed,i));
            }else{
                sender(MD[i],F(speed,i));
            }
        }
        break;
    case 'l':   // 左
        for(int i=0;i<4;i++){
            if(i==0||i==3){
                sender(MD[i],F(speed,i));
            }else{
                sender(MD[i],B(speed,i));
            }
        }
        break;
    case 'm':   // 右旋回
        for(int i=0;i<4;i++){
            if(i==0||i==2){
                sender(MD[i],B(speed,i)+senkai_speed);
            }else{
                sender(MD[i],F(speed,i)-senkai_speed);
            }
        }
        break;
    case 'h':   // 左旋回
        for(int i=0;i<4;i++){
            if(i==1||i==3){
                sender(MD[i],B(speed,i)+senkai_speed);
            }else{
                sender(MD[i],F(speed,i)-senkai_speed);
            }
        }
        break;
    case 's':
        for(int i=0;i<4;i++){
            sender(MD[i],BRK+chijiki_hosei[i]);
        }
        break;
    default:
        for(int i=0;i<4;i++){
            sender(MD[i],BRK+chijiki_hosei[i]);
        }
        break;
    }
    wait_us(10000);
}

void function_for_hosei(){
    // sensor_reader();
    float now=CHIJIKI_;    // 実際の値
    float error=goal-now;    // あとどれくらい動かす必要があるか
    pid.setSetPoint(0);    // 常に0を目指す
    pid.setInputLimits(0,180);    // 符号を外すことで計算を楽に    
    pid.setOutputLimits(0,Olim);    // 最大補正速度はOlim
    pid.setProcessValue(abs(error));    // errorの値=0からどれくらい離れているかなので0を目標のPIDではerrorをいれればいい    
    pid_hosei= pid.compute();    // 計算
    if(error<0)pid_hosei*=-1;    // もし左に動く必要がある(errorがマイナス)なら計算結果（符号なし）に-1をかける
    chijiki_hosei[0]=pid_hosei;    // 右旋回をするイメージ（もしpid_hoseiがマイナスならマイナス方向に右旋回＝左旋回になる）
    chijiki_hosei[1]=-pid_hosei;
    chijiki_hosei[2]=pid_hosei;
    chijiki_hosei[3]=-pid_hosei;
}



void sensor_reader(){
    // 赤外線センサーを読む
    value[0] = sensorF.read();
    value[1] = sensorB.read();

    // それをもとに計算する
    dis[0] = 71.463 * pow(value[0],-1.084);
    dis[1] = 71.463 * pow(value[1],-1.084);

    // 地磁気の相対角（初期位置からの）を取得
    CHIJIKI.setmode(OPERATION_MODE_IMUPLUS);   //魔法
    CHIJIKI.get_angles();
    raw_CHIJIKI_=CHIJIKI.euler.yaw;     //取得  0~360
    CHIJIKI_=raw_CHIJIKI_-goal;
    while (CHIJIKI_ < -180) {
        CHIJIKI_ += 360;
    }
    while (CHIJIKI_ > 180) {
        CHIJIKI_ -= 360;
    }
}

void debugger(){
    printf("+-----------------------------\n");
    printf("| sig             :   %d\n",sig.read());
    printf("| CHIJIKI_        :   %f\n",CHIJIKI_);
    // printf("| raw_CHIJIKI_    :   %f\n",raw_CHIJIKI_);
    printf("| distance        :   %f , %f\n",dis[0],dis[1]);
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
    printf("| WOOD              (kw):%d\n",WOOD);
    printf("| kakuzai_slow_speed(ks):%d\n",kakuzai_slow_speed);
    printf("| kakuza_fast_speed (kf):%d\n",kakuzai_fast_speed);
    // printf("| speed             (s) :%d\n",speed);
    printf("| p                 (p) :%f\n",p);
    printf("| i                 (i) :%f\n",i);
    printf("| d                 (d) :%f\n",d);
    printf("| pid_hosei         (X) :%d\n",pid_hosei);
    printf("| goal              (g) :%f\n",goal);
    printf("| Olim              (o) :%d\n",Olim);
    printf("| Oslow             (os):%d\n",Oslow);
    printf("| Ofast             (of):%d\n",Ofast);
    printf("| state                 :%d\n",state);
    // printf("| ")
    printf("+-----------------------------\n");
}

/*
   \|state | flag | finish | action
----+------+------+--------+-----------------------------
  1 |  1   |   0  |  false | 変数初期化、準備
  2 |  2   |   0  |  false | 角材が来るまでまつ→前あげ→flag 0->1
  3 |  2   |   1  |  false | 角材を前が超えるのを待つ→前下げ、後ろ上げ→10s待つ（k,pコマンド受付→auto_run停止）
  4 |  2   |   2  |  false | 10s待ち終わった→後ろ下げ→初期化＆mainに戻す: state 2->0
  5 |  1   | 2->0 |  true  | main loop -> 1
*/
void auto_run(void){
    if(auto_running){
        if(state==1){
            state=2;
            flag = 0;
            bool finish = false;
            printf("state:2\n");
            speed=kakuzai_slow_speed;
        }else if(state==2){
            sensor_reader();
            // debugger();
            if(dis[0] <= WOOD && flag == 0){
                airF.write(1); // 前あげ
                flag=1;
                printf("----------mae!----------\n");
                printf("dis0:%f\n",dis[0]);
                speed=kakuzai_fast_speed;
            }else if(dis[1] <= WOOD && flag == 1){
                flag=2;
                printf("----------go!----------\n");
                printf("dis1:%f\n",dis[1]);
                airF.write(0);
                ThisThread::sleep_for(100ms);
                printf("air change\n");
                airB.write(1);
                char buffer;
                for(int i = 0; i < 5; i++){ 
                    if(received){
                       received=false;
                       if(buffer=='k' || buffer=='p'){
                           printf("stop kakuzai!\n");
                           finish=true;
                           break;
                       }
                    }
                    printf("sleep\n");
                    ThisThread::sleep_for(100ms);
                }
                printf("--fin--\n");
                printf("test\n");
                airB.write(0);
                if(finish){
                    printf("finished\n");
                    finish = false;
                    flag = 0;
                }
                printf("state:0 switched\n");
                auto_running=false;
                state=0;
                di='s';
                send('s');
            // }else{
                // printf("flag:%d     dis0:%f     dis1:%f\n",flag,dis[0],dis[1]);
            }
        }
    }
}

