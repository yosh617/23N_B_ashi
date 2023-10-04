#include "mbed.h"
#include "BNO055.h"

const int MD[4]={0x26,0x54,0x56,0x50};     // MDアドレス   右前,左前,右後,左後
int WOOD=100;                              // [mm]
int speed=20;                              // 全体スピード
int max_speed=0xf0;                        // 最大速度
int min_speed=0x10;                        // 最低速度
int hosei[4]={0};                          // 個体値補正
int duty[4]={0};                           // 最終的なduty
const char BRK = 0x80;                     // ブレーキ
int senkai_speed=16;                       // 旋回速度
int state=0;                               //0:準備中   1:main
int F(int s,int i);                        // 前進計算
int B(int s,int i);                        // 後退計算
BufferedSerial pc(USBTX,USBRX);            // PCとのシリアル
I2C motor(PB_9,PB_8);                      // MDとのI2C
DigitalOut sig(PA_12);                     // 非常停止ボタン   0:動く  1:止まる

// エアシリンダーズ     0:伸ばす    1:縮む
DigitalOut  airF(PA_13);                 // 前輪
DigitalOut  airB(PH_1);                  // 後輪
DigitalOut ue_power(PC_8);               // 上電源確認君

// 赤外線センサーズ
AnalogIn    sensorF(PA_6);               // 前
AnalogIn    sensorB(PA_7);               // 後
float value[2];                          // 生の値
double dis[2]={};                        // 計算後の値
BNO055 CHIJIKI(PB_3,PB_10);              // ちじき        SDA SCL
float CHIJIKI_=0;                        // 地磁気の値 -180~0~180
float old_CHIJIKI=0;                     // 前回の地磁気の値
float raw_CHIJIKI_=0;                    // 生のデータ   0~360
float raw_old_CHIJIKI=0;                 // 生のデータ   0~360
float max_warp=75;                       // 許される瞬間の地磁気の変化量
int warp=0;                              // 地磁気の飛び (x90)
double yaw_Q=0;                          // 地磁気のすごいやつ！
int goal=0;                              // 目標値
void sender(char add,char dat);          // モーター動かす
void sensor_reader(void);                // センサー読む
float compute_dig(float d1,float d2);    // 角度の差を計算する
void debugger(void);                     // 確認用関数
void send(char d);                       // 動き         direction fbrls
void This_is_function_for_hosei(void);   // 補正
long double p=1;                         // 補正用 kゲイン
int chijiki_hosei[4]={0};                // 補正結果
Ticker This_is_ticker_for_hosei;         // Ticker
int F(int speed,int i){return BRK+speed+hosei[i]+chijiki_hosei[i];};    // 前進計算
int B(int speed,int i){return BRK-speed-hosei[i]-chijiki_hosei[i];};    // 後退計算

// main関数
int main(){
    printf("mbed start...");
    ue_power.write(0);
    sig.write(1);
    airF.write(0);
    airB.write(0);
    CHIJIKI.reset();
    while(!CHIJIKI.check());
    This_is_ticker_for_hosei.attach(This_is_function_for_hosei,50ms);
    char buffer;
    int index;
    char cmd[128];
    state=1;
    printf("loop start!\n");
    while(true){
        sensor_reader();
        // debugger();
        if(pc.read(&buffer,1)>0){   // PCから受信したら
            if(buffer=='\n'){       // 改行だったら
                cmd[index]='\0';    // \0 : 文字列の最後の意味
                if(cmd[0]=='v'){
                    switch(cmd[1]){
                        printf("...\n");    // 未実装
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
                    case 'a':   // 足回り
                        speed=atoi(&cmd[2]);
                        switch(cmd[1]){
                        case 'a':
                            switch(cmd[2]){
                            case 'f':
                                switch(cmd[3]){
                                case 'u':
                                    airF.write(1);  // aafu
                                    break;
                                case 'd':
                                    airF.write(0);  //aafd
                                    break;
                                }
                                break;
                            case 'b':
                                switch(cmd[3]){
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
                            printf("cmd:%c\n",cmd[1]);
                            send(cmd[1]);   //  a + f,b,r,l,s
                            break;
                        }
                    case 's':   // 旋回
                        switch(cmd[1]){
                        case 'r':
                            speed=atoi(&cmd[2]);
                            send('R');
                            break;
                        case 'l':
                            speed=atoi(&cmd[2]);
                            send('L');
                            break;
                        case 's':
                            speed=atoi(&cmd[2]);
                            send('s');
                            break;
                        }
                        break;
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
    printf("dat: %d\n",dat);
    motor.write(dat);
    motor.stop();
    wait_us(100);
}

void send(char d){
    // printf("%c",d);
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
    case 'R':   // 右旋回
        for(int i=0;i<4;i++){
            if(i==0||i==2){
                sender(MD[i],B(speed,i)+senkai_speed);
            }else{
                sender(MD[i],F(speed,i)-senkai_speed);
            }
        }
        break;
    case 'L':   // 左旋回
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

void This_is_function_for_hosei(){
    // sensor_reader();
    float now = goal-CHIJIKI_;
    float h=now*p;
    if(h<0)h*=-1;
    if(0<now){  //左向き過ぎてる
        chijiki_hosei[0]=0;
        chijiki_hosei[1]=h;
        chijiki_hosei[2]=0;
        chijiki_hosei[3]=h;
    }else{      //右向きすぎてる
        chijiki_hosei[0]=h;
        chijiki_hosei[1]=0;
        chijiki_hosei[2]=h;
        chijiki_hosei[3]=0;        
    }
}

float compute_dig(float d1,float d2){
    float d=d1-d2;
    while(d<-180){
        d+=360;
    }
    while(d>180){
        d-=360;
    }
    return abs(d);
}

void sensor_reader(){
    // 赤外線センサーを読む
    value[0] = sensorF.read();
    value[1] = sensorB.read();

    // それをもとに計算する
    dis[0] = 71.463 * pow(value[0],-1.084);
    dis[1] = 71.463 * pow(value[1],-1.084);

    // 地磁気の相対角（初期位置からの）を取得
    CHIJIKI.setmode(OPERATION_MODE_NDOF);   //魔法
    CHIJIKI.get_angles();
    old_CHIJIKI=CHIJIKI_;   //入れ替え
    if(old_CHIJIKI<0)raw_old_CHIJIKI=180-old_CHIJIKI;   //元の形に戻す 0~360
    else raw_old_CHIJIKI=old_CHIJIKI;
    raw_CHIJIKI_=CHIJIKI.euler.yaw;     //取得  0~360
    if(180<raw_CHIJIKI_ && raw_CHIJIKI_<360)CHIJIKI_=raw_CHIJIKI_-360;  //-180~180に変換
    else CHIJIKI_=raw_CHIJIKI_;
    CHIJIKI.getEulerFromQ(yaw_Q);
    yaw_Q*=-1;
    // 飛びすぎてたら...
    if(compute_dig(raw_old_CHIJIKI, raw_CHIJIKI_)>max_warp){
        printf("warping!!\n");
    }
}

void debugger(){
    printf("+------------------------------------\n");
    printf("| sig             :   %d\n",sig.read());
    printf("| CHIJIKI_        :   %f\n",CHIJIKI_);
    printf("| CHIJIKI_Q       :   %f\n",yaw_Q);
    printf("| distance        :   %f , %f\n",dis[0],dis[1]);
    printf("| speed           :   %d\n",speed);
    printf("+------------------------------------\n");
    printf("| motor           :   MM HM MU HU\n");
    printf("| hosei           :   %d %d %d %d\n",hosei[0],hosei[1],hosei[2],hosei[3]);
    printf("| CJK hosei       :   %d %d %d %d\n",chijiki_hosei[0],chijiki_hosei[1],chijiki_hosei[2],chijiki_hosei[3]);
    printf("| final duty      :   %d %d %d %d\n",duty[0],duty[1],duty[2],duty[3]);
    printf("+------------------------------------\n");
}