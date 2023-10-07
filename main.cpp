#include "UnbufferedSerial.h"
#include "mbed.h"
#include "BNO055.h"
#include "PIDcontroller.h"
const int MD[4]={0x26,0x54,0x56,0x50};  // MDアドレス   右前,左前,右後,左後
int WOOD=150;   // [mm]
int speed=40;   // 全体スピード
int max_speed=0xf0; // 最大速度
int min_speed=0x10; // 最低速度
int hosei[4]={0};   // 個体値補正
int duty[4]={0};    // 最終的なduty
int kakuzai_speed=20;   // 角材超えるときのスピード
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
double p=0.1;    // 補正用 pゲイン
double i=0.001;
double d=0.0;
PID pid(p,i,d,0.050);
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
float old_CHIJIKI=0;    // 前回の地磁気の値
float raw_CHIJIKI_=0;   // 生のデータ   0~360
float raw_old_CHIJIKI=0;    // 生のデータ   0~360
float max_warp=75;  // 許される瞬間の地磁気の変化量
int warp=0; // 地磁気の飛び
double yaw_Q=0; // 地磁気のすごいやつ！
int goal=0; // 目標値
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
    while(!CHIJIKI.check());
    pc.attach(input,UnbufferedSerial::RxIrq);
    pid.setInputLimits(-180.0,180.0);
    pid.setOutputLimits(-20, 20);
    pid.setSetPoint(0);
    ticker_for_hosei.attach(function_for_hosei,50ms);
    state=1;
    printf("loop start!\n");
    while(true){
        sensor_reader();
        // debugger();
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
                    case 'w':   //vw
                        WOOD=atoi(&data[2]);
                        printf("WOOD:%d\n",WOOD);
                        break;
                    case 'k':   //vk
                        kakuzai_speed=atoi(&data[2]);
                        printf("kakuzai_speed%d\n",kakuzai_speed);
                        break;
                    case 's':
                        speed=atoi(&data[2]);
                        printf("speed:%d\n",speed);
                    default:
                        show();
                        break;
                    }
                    break;
                case 'd':   //d
                    debugger();
                    break;
                case 'p':   //p
                    send('s');
                    sig.write(1);
                    airF.write(0);
                    airB.write(0);
                    airUE.write(1);
                    ue_power.write(0);
                    printf("pause!\n");
                    break;
                case 'c':   //c
                    send('s');
                    sig.write(0);
                    airF.write(0);
                    airB.write(0);
                    airUE.write(0);
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
                    default:    // それ以外
                        //printf("data:%c\n",data[1]);
                        send(data[1]);   //  a + f,b,r,l,s
                        break;
                    }
                    break;
                case 's':   // 旋回
                    switch(data[1]){
                    case 'r':
                        speed=atoi(&data[2]);
                        send('m');
                        break;
                    case 'l':
                        speed=atoi(&data[2]);
                        send('h');
                        break;
                    case 's':
                        speed=atoi(&data[2]);
                        send('s');
                        break;
                    }
                    break;
                case 'k':
                    airF.write(0);  // age
                    airB.write(0);  // age
                    printf("kakuzai k\n");
                    speed=kakuzai_speed;    // slow...
                    send('f');  // going
                    auto_running=true;  // allow auto run
                    break;
                }
                char data[128]="";
            }
        }else if(state==2){
            auto_run();
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
    // //printf("dat: %d\n",dat);
    motor.write(dat);
    motor.stop();
    wait_us(32);
}

void send(char d){
    //printf("switch:%c\n",d);
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
            sender(MD[i],BRK);
        }
        break;
    default:
        for(int i=0;i<4;i++){
            sender(MD[i],BRK);
        }
        break;
    }
}

void function_for_hosei(){
    // sensor_reader();
    pid.setProcessValue(CHIJIKI_);
    pid_hosei= pid.compute();
    chijiki_hosei[0]=pid_hosei;
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
    old_CHIJIKI=CHIJIKI_;   //入れ替え
    if(old_CHIJIKI<0)raw_old_CHIJIKI=old_CHIJIKI-360;   //元の形に戻す 0~360
    else raw_old_CHIJIKI=old_CHIJIKI;
    raw_CHIJIKI_=CHIJIKI.euler.yaw;     //取得  0~360
    if(180<raw_CHIJIKI_ && raw_CHIJIKI_<360)CHIJIKI_=raw_CHIJIKI_-360;  //-180~180に変換
    else CHIJIKI_=raw_CHIJIKI_;
}

void debugger(){
    printf("+-----------------------------\n");
    printf("| sig             :   %d\n",sig.read());
    printf("| CHIJIKI_        :   %f\n",CHIJIKI_);
    printf("| distance        :   %f , %f\n",dis[0],dis[1]);
    printf("| speed           :   %d\n",speed);
    printf("+-----------------------------\n");
    // printf("| motor           :   MM HM MU HU\n");
    // printf("| hosei           :   %d %d %d %d\n",hosei[0],hosei[1],hosei[2],hosei[3]);
    // printf("| CJK hosei       :   %d %d %d %d\n",chijiki_hosei[0],chijiki_hosei[1],chijiki_hosei[2],chijiki_hosei[3]);
    // printf("| final duty      :   %d %d %d %d\n",duty[0],duty[1],duty[2],duty[3]);
    // printf("+------------------------------------\n");
}

void show(){
    printf("+-----------------------------\n");
    printf("| WOOD            (w):%d\n",WOOD);
    printf("| kakuzai_speed   (k):%d\n",kakuzai_speed);
    printf("| speed           (s):%d\n",speed);
    printf("| p               (p):%f\n",p);
    printf("| i               (i):%f\n",i);
    printf("| d               (d):%f\n",d);
    printf("| pid_hosei       (X):%d\n",chijiki_hosei);
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
            int flag = 0;
            bool finish = false;
            printf("state:2\n");
        }else if(state==2){
            sensor_reader();
            // debugger();
            if(dis[0] <= WOOD && flag == 0){
                airF.write(1); // 前あげ
                flag=1;
                printf("----------mae!----------\n");
                printf("dis0:%f\n",dis[0]);
            }else if(dis[1] <= WOOD && flag == 1){
                flag=2;
                printf("----------go!----------\n");
                printf("dis1:%f\n",dis[1]);
                airF.write(0);
                ThisThread::sleep_for(100ms);
                printf("air change\n");
                airB.write(1);
                char buffer;
                for(int i = 0; i < 20; i++){
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
                printf("fin\n");
                auto_running=false;
                airB.write(0);
                if(finish){
                    printf("finished\n");
                    finish = false;
                    flag = 0;
                state=0;
                send('s');
                printf("state:1 switched\n");
                }
            }else{
                printf("flag:%d     dis0:%f     dis1:%f\n",flag,dis[0],dis[1]);
            }
        }
    }
}

