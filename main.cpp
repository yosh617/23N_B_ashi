#include "mbed.h"
#include "BNO055.h"

// MDアドレス   右前,左前,右後,左後
const int MD[4]={0x26,0x54,0x56,0x50};
// [mm]
int WOOD=100;
// 全体スピード
int speed=20;
// 最大速度
int max_speed=0xf0;
// 最低速度
int min_speed=0x10;
// 個体値補正
int hosei[4]={0};
// 最終的なduty
int duty[4]={0};
// ブレーキ
const char BRK = 0x80;
/*
状態
0:準備中   1:main
*/
int state=0;
// 前進計算
int F(int s,int i);
// 後退計算
int B(int s,int i);
// PCとのシリアル
BufferedSerial pc(USBTX,USBRX);
// MDとのI2C
I2C motor(PB_9,PB_8);
// 非常停止ボタン   0:動く  1:止まる
DigitalOut sig(PA_12);
// エアシリンダーズ
DigitalOut  airF(PA_13); // 前輪
DigitalOut  airB(PH_1); // 後輪
// 上電源確認君
DigitalOut ue_power(PC_8);
// 赤外線センサーズ
AnalogIn    sensorF(PA_6); // 前
AnalogIn    sensorB(PA_7); // 後
// 生の値
float value[2];
// 計算後の値
double dis[2]={};
// ちじき        SDA SCL
BNO055 CHIJIKI(PB_3,PB_10);
// 地磁気の値 -180~0~180
float CHIJIKI_=0;
// 前回の地磁気の値
float old_CHIJIKI=0;
// 生のデータ   0~360
float raw_CHIJIKI_=0;
// 生のデータ   0~360
float raw_old_CHIJIKI=0;
// 許される瞬間の地磁気の変化量
float max_warp=75;
// 地磁気の飛び (x90)
int warp=0;
// 目標値
int goal=0;
// モーター動かす
void sender(char add,char dat);
// センサー読む
void sensor_reader(void);
// 角度の差を計算する
float compute_dig(float d1,float d2);
// 確認用関数
void debugger(void);
// 動き         direction fbrls
void send(char d);
// 補正
void This_is_function_for_hosei(void);
// 補正用 kゲイン
long double p=1;
//補正結果
int chijiki_hosei[4]={0};
// Ticker
Ticker This_is_ticker_for_hosei;

// 前進計算
int F(int speed,int i){return BRK+speed+hosei[i]+chijiki_hosei[i];};
// 後退計算
int B(int speed,int i){return BRK-speed-hosei[i]-chijiki_hosei[i];};

// main関数
int main(){
    printf("mbed start...");
    ue_power.write(0);
    sig.write(1);
    airF.write(0);
    airB.write(1);
    CHIJIKI.reset();
    while(!CHIJIKI.check());
    This_is_ticker_for_hosei.attach(This_is_function_for_hosei,50ms);
    char buffer;
    int index;
    char cmd[128];
    state=1;
    printf("loop start!\n");
    while(true){
        if(pc.read(&buffer,1)>0){
            if(buffer=='\n'){
                cmd[index]='\0';
                if(cmd[0]=='v'){
                    switch(cmd[1]){
                        printf("...\n");
                    }
                }else if(state==1){
                    switch(cmd[0]){
                    case 'd':
                        debugger();
                        break;
                    case 'p':
                        sig.write(1);
                        ue_power.write(0);
                        printf("pause!\n");
                        break;
                    case 'c':
                        sig.write(0);
                        ue_power.write(1);
                        printf("continue!\n");
                        break;
                    case 'a':
                        speed=atoi(&cmd[2]);
                        switch(cmd[1]){
                        case 'a':
                            switch(cmd[2]){
                            case 'f':
                                switch(cmd[3]){
                                case 'u':
                                    airF.write(1);
                                    break;
                                case 'd':
                                    airF.write(0);
                                    break;
                                }
                                break;
                            case 'b':
                                switch(cmd[3]){
                                case 'u':
                                    airB.write(1);
                                    break;
                                case 'd':
                                    airB.write(0);
                                    break;
                                }
                                break;
                            }
                            break;
                        default:
                            send(cmd[1]);
                            break;
                        }
                    }
                }
                char cmd[128]="";
                index=0;
            }else{
                cmd[index]=buffer;
                index++;
            }
        }
    }
}

void sender(char add,char dat){
    motor.start();
    motor.write(add);
    if(dat<min_speed)dat=min_speed;
    else if(max_speed<dat)dat=max_speed;
    motor.write(dat);
    motor.stop();
    wait_us(100);
}

void send(char d){
    switch(d){
    case 'f':
        for(int i=0;i<4;i++){
            sender(MD[i],F(speed,i));
        }
        break;
    case 'b':
        for(int i=0;i<4;i++){
            sender(MD[i],B(speed,i));
        }
        break;
    case 'r':
        for(int i=0;i<4;i++){
            if(i==0||i==3){
                sender(MD[i],B(speed,i));
            }else{
                sender(MD[i],F(speed,i));
            }
        }
        break;
    case 'l':
        for(int i=0;i<4;i++){
            if(i==0||i==3){
                sender(MD[i],F(speed,i));
            }else{
                sender(MD[i],B(speed,i));
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

void This_is_function_for_hosei(){
    sensor_reader();
    float now = goal-CHIJIKI_;
    if(0<now){  //左向き過ぎてる
        chijiki_hosei[0]=0;
        chijiki_hosei[1]=abs(now*p);
        chijiki_hosei[2]=0;
        chijiki_hosei[3]=abs(now*p);
    }else{      //右向きすぎてる
        chijiki_hosei[0]=abs(now*p);
        chijiki_hosei[1]=0;
        chijiki_hosei[2]=abs(now*p);
        chijiki_hosei[3]=0;        
    }
}

float compute_dig(float d1,float d2){

}

void sensor_reader(){
    // 赤外線センサーを読む
    value[0] = sensorF.read();
    value[1] = sensorB.read();

    //それをもとに計算する
    dis[0] = 71.463 * pow(value[0],-1.084);
    dis[1] = 71.463 * pow(value[1],-1.084);

    // 地磁気の相対角（初期位置からの）を取得
    CHIJIKI.setmode(OPERATION_MODE_NDOF);   //魔法
    CHIJIKI.get_angles();
    old_CHIJIKI=CHIJIKI_;   //入れ替え
    if(old_CHIJIKI<0)raw_old_CHIJIKI=180-old_CHIJIKI;   //元の形に戻す 0~360
    else raw_old_CHIJIKI=old_CHIJIKI;
    raw_CHIJIKI_=CHIJIKI.euler.yaw;     //取得  0~360
    if(180<raw_CHIJIKI_ && raw_CHIJIKI_<360)CHIJIKI_=180-raw_CHIJIKI_;  //-180~180に変換
    else CHIJIKI_=raw_CHIJIKI_;

    //飛びすぎてたら...
}

