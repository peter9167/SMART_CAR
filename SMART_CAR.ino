///////////////////////////////////////////////////////////////////
// 4WD JEEP Smart Car coding for RMC03 V1.0, 2017.01.04
///////////////////////////////////////////////////////////////////

#include <SoftwareSerial.h> //소프트웨어시리얼 관련 헤더파일
#include <U8glib.h> //OLED 관련 헤더파일
#include <string.h> //memory 관련 헤더파일
#include "Thread.h" //쓰레드 관련 헤더파일

void LcdCallBack(); //쓰레드 처리를 위한 콜백함수 정의

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0); // I2C / TWI
void draw(int x, int y, String c_str); //디스플레이 드로우 함수

#define MR_ENABLE1 6 //Enable pin - Motor1
#define MR_ENABLE2 5 //Enable pin - Motor2

#define MR_INT1 4 //Control pin 1 - Motor1
#define MR_INT2 7 //Control pin 2 - Motor1
#define MR_INT3 8 //Control pin 1 - Motor2
#define MR_INT4 9 //Control pin 2 - Motor2
#define M_Speed A0

//61SMART CAR 만들기 Ver 1.0
#define BUZZER 11
#define Sonar_T 10 //Sonar Trigger
#define Sonar_E 12 //Sonar Echo
#define LED 13
#define MAX_DISTANCE 200 //유효거리 지정

#define InfraR A1 //Right Infrared sensor
#define InfraL A2 //Left Infrared sensor
//#define InfraM A3 //Center Infrared Sensor for Break.

int cmU = 0; //HC-SR04 의 거리 측정값
int cmU_old = 0; //cmU 의 이전 측정값
int cmU_avg = 0; //cmU 의 평균값
long duration; //시속시간
int i_InfraL = 20; //왼쪽 적외선 센서의 거리 측정값
int i_InfraR = 20; //오른쪽 적외선 센서의 거리 측정값
int i_InfraL_Old = 20; //왼쪽 적외선 센서의 이전 거리 측정값
int i_InfraR_Old = 20; //오른쪽 적외선 센서의 이전 거리 측정값
//int i_InfraM = 0;
float f_InfraL = 20.0; //왼쪽 적외선 센서의 float 형 거리 측정값
float f_InfraR = 20.0; //오른쪽 적외선 센서의 float 형 거리 측정값

//Bzuzzer 음계
int iNote[] = {2093, 2349, 2637, 2793, 3136, 3520, 3951, 4186}; //도레미파솔라시도
int iFreq = 150; //가변주파수 저장 변수

//62SMART CAR 만들기 Ver 1.0

//주행모드의 정의.
#define CAR_DIR_FW 0 //전진
#define CAR_DIR_BK 1 //후진
#define CAR_DIR_LF 2 //좌선회
#define CAR_DIR_RF 3 //우선회
#define CAR_DIR_ST 4 //정지

int pot_value = 0;
int motor_speed_l = 0;
int motor_speed_r = 0;
bool dir_stat = false;
int g_carDirection = CAR_DIR_ST; //주행모드 변수 및 정지를 기본값으로 함

//HC-12 433MHz Serial Wireless Communication Module.(for receiver pasing)
int iActPos = 80; //Analog 출력값이 80 이상이 되여야만 모터로 출력.
int first_idx = 99; //수신된 문자열중 "$RTx"이 몇번째 인덱스에 있는지 확인
int second_idx = 99; //수신된 문자열중 "$END"이 몇번째 인텍스에 있는지 확인
bool rcv_state = 0; //수신된 문자열이 있다면...

int X1v, Y1v, Z1v; //Left Joystick value - X, Y, Z
int Btn_Up, Btn_Dn, Btn_Left, Btn_Right; //Left Potentiometer, Rright Potentiometer.
String Rcv_Str; //HC-12 로부터 수신문자열을 저장
int get_val[20] = {0}; //HC-12 로부터 수신문자열의 정보를 분리하여 숫자정수로 저장

int set_val; //strotok 에서으 토큰의 갯수를 세기위한 변수
char rx_buf[64] = {0}; //Rcv_Str 의 문자열을 문자열형 변수로 저장

//63SMART CAR 만들기 Ver 1.0
char ms[64] = {0}; //rx_buf 배열의 값으 메모리로 출력하기 위한 변수
char cs[16] = {0}; //strtok()에 의해 추출된 값을 저장
char *p; //strtok()에 의해 추출된 값의 포인터 변수
char str[64] = {0}; //디버깅을 위한 문자열형 변수

char c_lcd[4][20];
char c_char[20];

int i, j; //for general loop
int loop_cnt = 0; //작업중인 상태를 저장.
int drive_mode = 0; //0-Smart Drive, 1-Remote Drive, 2- Car Stop.
bool b_drive_state = false;
bool b_light = false;
bool b_sound = false;
bool b_state = false;

SoftwareSerial HC12Serial(2, 3); // RX, TX

/*클래스의 함수들을 멤버 함수 또는 메쏘드(Method)라 하고
  객체(Object)는 클래스로부터 만들어 내는 구체적인 존재로
  객체를 클래스의 인스턴스화라고 합니다.*/
Thread LcdThread = Thread(); //Thrad 클래스의 인스턴 생성

void setup() {

  Serial.begin(9600); //H/W 시리얼 포트 오픈
  HC12Serial.begin(9600); delay(100); //S/W 시리얼 포트 오픈


  pinMode(MR_ENABLE1, OUTPUT); //PWM 핀, 출력모드
  pinMode(MR_ENABLE2, OUTPUT);

  pinMode(MR_INT1, OUTPUT); //디지털 핀, 출력모드
  pinMode(MR_INT2, OUTPUT);
  pinMode(MR_INT3, OUTPUT);
  pinMode(MR_INT4, OUTPUT);

  pinMode(Sonar_T, OUTPUT); //트리거는 출력으로 설정
  pinMode(Sonar_E, INPUT); //에코는 입력으로 설정
  pinMode(InfraR, INPUT); //적외선 센서 입력모드
  pinMode(InfraL, INPUT);
  //pinMode(InfraM,INPUT);
  pinMode(BUZZER, OUTPUT); //부저 출력모드
  pinMode(LED, OUTPUT); //LED 핀 출력모드

  vAlram(1);

  // flip screen, if required
  // u8g.setRot180();

  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {

    //65SMART CAR 만들기 Ver 1.0

    u8g.setColorIndex(255); // white
  } else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3); // max intensity
  } else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1); // pixel on
  } else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255, 255, 255);
  }

  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawLine(0, 0, 128, 0);
    u8g.drawStr(0, 15, "Arduino UNO R3");
    u8g.drawStr(0, 30, "4WD JEEP Smart Car");
    u8g.drawStr(0, 45, "Creat&Bulding");
    u8g.drawStr(0, 60, "www.firstbot.co.kr");
    u8g.drawLine(0, 63, 128, 63);
  } while (u8g.nextPage());

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  LcdThread.onRun(LcdCallBack); //The target callback function to be called.
  LcdThread.setInterval(500); //Setts the desired interval for the Thread (in Ms).

  //66SMART CAR 만들기 Ver 1.0

}

void loop() {

  rcv_state = 0; //수신상태를 초기화.
  Rcv_Str = ""; //수신될 문자열 변수의 초기화.

  if ( HC12Serial.available() ) {
    HC12Serial.setTimeout(100); //Timeout 설정.
    Rcv_Str = HC12Serial.readStringUntil('\n'); //문자열중 "\n"이 올 때까지 기다린다.
    Serial.println(Rcv_Str); //디버깅 시리얼 포트로 출력한다.
  } else {
  }

  /*
    수신문자열을 분리하여 저장할 변수 :
    X1v, Y1v, Z1v, Btn_Up, Btn_Right, Btn_Dn, Btn_Left
    수신문자열 예 : $RTx,512,520,1,0,0,0,0,$END - 27 자리 문자열
    수신문자열 예 : $RTx,5,6,1,0,0,0,0,$END - 23 자리 최소 문자열 길이
    수신문자열 예 : $RTx,1023,1023,1,0,0,0,0,$END - 29 자리 문자열
  */
  first_idx = 99; //"$RTx"가 문자열중에서 몇번째에 있는지...
  second_idx = 99; //"$END"가 문자열중에서 몇번째에 있는지...
  first_idx = Rcv_Str.indexOf("$RTx"); //수신된 문자열중 "$RTx"가 있는지 확인
  second_idx = Rcv_Str.lastIndexOf("$END"); //수신된 문자열중 "$END"가 있는지 확인

  //67SMART CAR 만들기 Ver 1.0
  //Serial.print(first_idx);
  //Serial.print(" : ");
  //Serial.println(second_idx);

  set_val = 0; //추출된 문자의 수를 세고 배열의 인덱스로 사용
  /*
    수신 문자열중 "$RTx"가 첫번째에, "$END"가 문자열중 33 번보다 크다면 정상 수신으로 판단
    콤마를 기준으로 문자를 분리 하기위해 문자열을 char 형 변수에 저장
  */

  if ( (first_idx == 0) && (second_idx >= 19) ) {

    rcv_state = 1; //문자열중 "$RTx"와 "$END"가 있다면 수신 성공으로 판단

    for ( i = 0; i < second_idx; i++) {
      rx_buf[i] = Rcv_Str[i]; //Rcv_Str 의 문자열을 문자열형 변수로 저장
    }
    sprintf(ms, rx_buf); //rx_buf 의 문자열을 메모리에 저장

    /*
      strtok 함수 원형 : char *strtok(char *_Str, const char *_Delim);
      strtok() 함수는 _Str 과 _Delim 을 쪼개어 토큰으로 만든 후, 비교하여 같은 문자를
      만났을 때, 그 문자를 NULL 값으로 바꾸는 함수이다.
    */
    //첫 호출시에는 대상문자열(_Str)과 자르기를 위한 문자(_Delim)를 인수로 전달
    p = strtok(ms, ","); //First Token 의 값을 포인트 p 에 저장
    sprintf(cs, "%s", p); //포인트 p 의 값을 cs 에 저장

    //68SMART CAR 만들기 Ver 1.0

    get_val[set_val] = atoi(cs); //$RTx 는 숫자가 아니므로 0 으로 반환됨

    //이후에는 _Str 을 NULL 으로, Delim 은 자르기를 위한 문자를 인수로 전달한다.
    //더이상 반환활 값이 없을 때까지 구분문자를 추출한다.
    while ((p = strtok(NULL, ",")) != NULL) { //another token 추출
      set_val++; //추출된 구문문자(token) 수 카운트
      sprintf(cs, "%s", p); //포인트 p 의 값을 문자열형 변수 cs 에 저장
      get_val[set_val] = atoi(cs); //문자열형 cs 의 값을 정수로 변환하여 저장
      if ( set_val == 7 ) {
        break; //받을 구문문자가 7 이면 빠져나감
      }
    }
    set_val = 0;

    //for debug
    sprintf(str, "OK %d, %d, %d, %d, %d, %d, %d",

            get_val[1], get_val[2], get_val[3], get_val[4], get_val[5], get_val[6], get_val[7]);

    //Serial.println(str);

    X1v = get_val[1]; //Joystick X - Analog value
    Y1v = get_val[2]; //Joystick Y - Analog value
    Z1v = get_val[3]; //Joystick Z - Digital value
    Btn_Up = get_val[4]; //Up Button - Analog value
    Btn_Right = get_val[5]; //Right Button - Analog value
    Btn_Dn = get_val[6]; //Down Button - Digital value
    Btn_Left = get_val[7]; //Left Button - Analog value

  } else {

    //69SMART CAR 만들기 Ver 1.0
  }

  //Light On/Off
  int loop_sen = 2;
  if ( ( rcv_state == 1 ) && ( Btn_Right == 0 ) && ( loop_cnt >= loop_sen ) ) {
    loop_cnt = 0;
    if ( b_light == false ) {
      b_light = true;
      digitalWrite(13, HIGH);
    } else {
      b_light = false;
      digitalWrite(13, LOW);
    }
  }

  //Buzzer On/Off
  if ( ( rcv_state == 1 ) && ( Btn_Left == 0 ) && ( loop_cnt >= loop_sen ) ) {
    loop_cnt = 0;
    vAlram(4);
  }

  //Smart Drive or Remote Control
  if ( ( rcv_state == 1 ) && ( Btn_Up == 0 ) && ( loop_cnt >= loop_sen ) ) {

    loop_cnt = 0;
    if ( b_drive_state == false ) {
      drive_mode = 0; //Smart Drive

      //70SMART CAR 만들기 Ver 1.0

      b_drive_state = true;
      Serial.println("Drive 0");
    } else if ( b_drive_state == true ) {
      drive_mode = 1; //Remote control
      b_drive_state = false;
      Serial.println("Drive 1");
    }
  }

  //Car STOP
  if ( ( rcv_state == 1 ) && ( Btn_Dn == 0 ) && ( loop_cnt >= loop_sen ) ) {
    loop_cnt = 0;
    drive_mode = 2;
    vAlram(0);
  }

  //조종기가 없을시 스마트 주행 및 정지 상태를 가변저항의 값으로 결정한다.
  //가변저항의 값이 200 이상이면 자동 주행모드.
  //가변저항의 값이 10~200 미만이면 무선 주행모드.
  //가변저항의 값이 50 미만이만 모터 정지.
  //무선조종모드에서는 조종기 버튼에 의한 자동 주행 모드 추가.

  pot_value = analogRead(M_Speed); delay(1); //아날로그 입력
  pot_value = map(pot_value, 0, 1023, 0, 255); //pot_value 를 0~255 로 비례 변환함.
  pot_value = constrain(pot_value, 0, 255); //pot_value 를 0~255 로 제한함.

  //71SMART CAR 만들기 Ver 1.0

  if ( pot_value >= 200 ) {
    drive_mode = 0; //Smart drive_mode
  } else if ( (pot_value >= 50) && (pot_value < 200) ) {
    //drive_mode = 1; //Remote drive mode
  } else if ( pot_value < 50 ) {
    drive_mode = 2; //Car Stop
  }

  //Serial.print("potentiometer value : ");
  //Serial.println(pot_value);

  //무선 송신기에서 Data 를 수신받고, 가변저항의 값이 200 보다 작다면...무선조정으로.
  if ( (rcv_state == 1) && ( drive_mode == 1 ) ) {
    //X1v(Joystick X 값 범위) : Left(1023)-Cent(512)-Right(0)
    //Y1v(Joystick X 값 범위) : Up(10230)-Cent(512)-Down(0)
    //아날로그 출력 범위(0~255)로 계산
    X1v = -round((511 - X1v) / 2); //변환값 : 255 ~ 0, 0 ~ -255
    Y1v = -round((511 - Y1v) / 2); //변환값 : 255 ~ 0, 0 ~ -255
    //Serial.print(X1v);
    //Serial.print(" : ");
    //Serial.println(Y1v);

    ////////////////////////////////////////////////////////////
    //1.Forward(255~0)
    if ( Y1v >= iActPos ) { //iActPos = 80 이상 되어야 모터 동작

      g_carDirection = CAR_DIR_FW;

      //1.1.turn left forward - 우측모터가 빨리 회전

      //72SMART CAR 만들기 Ver 1.0

      if ( X1v >= iActPos ) {

        motor_speed_r = Y1v - int(X1v);
        motor_speed_l = Y1v + int(X1v);
        //1.2.turn rigit forward - 좌측모터가 빨리 회전

      } else if (X1v <= -iActPos ) {

        X1v = int(abs(X1v)); //convert plus
        motor_speed_r = Y1v + int(X1v);
        motor_speed_l = Y1v - int(X1v);

        //1.3.Direct forward
      } else {

        motor_speed_l = Y1v;
        motor_speed_r = Y1v;

      }
      ////////////////////////////////////////////////////////////
      //2.Backward(0~-255)
    } else if ( Y1v <= -iActPos ) {
      g_carDirection = CAR_DIR_BK;

      Y1v = int(abs(Y1v)); //convert plus
      //2.1.turn left backward
      if ( X1v >= iActPos ) {

        //g_carSpeed_l = Y1v + int(X1v);
        motor_speed_l = Y1v + int(X1v);
        motor_speed_r = Y1v - int(X1v);
        //2.2.furn rigit backward
      } else if (X1v <= -iActPos ) {

        X1v = int(abs(X1v)); //convert plus
        motor_speed_l = Y1v - int(X1v);

        //73SMART CAR 만들기 Ver 1.0

        motor_speed_r = Y1v + int(X1v);

        //2.3.Direct backward
      } else {

        motor_speed_l = Y1v;
        motor_speed_r = Y1v;

      }
      ////////////////////////////////////////////////////////////
      //3.turn left or right and stop, Y1v : 80~255, -80~255 에서만 동작.
    } else if ( (Y1v < iActPos) && (Y1v > (0 - iActPos)) ) {
      //3.1.turn Right
      if ( X1v >= iActPos ) {

        g_carDirection = CAR_DIR_RF;
        motor_speed_l = int(X1v);
        motor_speed_r = int(X1v);

        //3.1.turn Left
      } else if ( X1v <= (-iActPos) ) {
        g_carDirection = CAR_DIR_LF;
        motor_speed_l = -int(X1v);
        motor_speed_r = -int(X1v);

        //3.2.stop
      } else {

        g_carDirection = CAR_DIR_ST;
        motor_speed_l = 0;
        motor_speed_r = 0;

      }
      ////////////////////////////////////////////////////////////
      //4. Stop

      //74SMART CAR 만들기 Ver 1.0

    } else { //Stop
      g_carDirection = CAR_DIR_ST;
      motor_speed_l = 0;
      motor_speed_r = 0;
    }

    if ( motor_speed_l >= 255) {
      motor_speed_l = 255;
    };
    if ( motor_speed_r >= 255) {
      motor_speed_r = 255;
    };
    if ( motor_speed_l <= 0) {
      motor_speed_l = 0;
    };
    if ( motor_speed_r <= 0) {
      motor_speed_r = 0;
    };

    car_update(); //설정값 반영

    //////////////////////////////////////////////////////////////
    //무선 신호 수신여부와 관계없이 가변저항의 값이 200 보다 크다면 자율주행모드로 동작
    //////////////////////////////////////////////////////////////
  } else if ( drive_mode == 0 ) {

    //좌우 방향은 스마트카를 뒤에서 보았을 때를 기준으로 한다.

    GetDistance_UltraSonic();
    GetDistance_InfraRed();
    if ( cmU > 50 ) {
      cmU = 50;
    };
    //Serial.print("Ultrasonic : ");
    //Serial.println(cmU);
    //Serial.print(i_InfraL);
    //Serial.print(" -- Infrared -- ");

    //75SMART CAR 만들기 Ver 1.0

    //Serial.println(i_InfraR);

    motor_speed_l = 255;
    motor_speed_r = 255;

    //장애물에 근접하면 주행 속도를 늦춘다.
    motor_speed_l -= 50 - cmU;
    motor_speed_r -= 50 - cmU;

    int sen_val = 30;
    if ( cmU >= 50 ) { //초음파센서에 50cm 이내에 장애물이 없다면 전진.

      sen_val = 40; //센서 감지거리 설정
      g_carDirection = CAR_DIR_FW;
      //좌우 적외선 센서에 20cm 이내에 장애물이 없다면 전진
      if ( (i_InfraL >= sen_val ) && (i_InfraR >= sen_val) ) {

        car_update();

        //우측 적외선 센서에 장애물이 감지되면 좌측으로 선회
      } else if ( (i_InfraL >= sen_val ) && (i_InfraR < sen_val) ) {
        motor_speed_l = 0; //좌측 모터의 회전 속도 감소
        car_update();
        //좌측 적외선 센서에 장애물이 감지되면 우측으로 선회
      } else if ( (i_InfraL < sen_val ) && (i_InfraR >= sen_val) ) {
        motor_speed_r = 0; //우측 모터의 회전 속도 감소

        car_update();
      }
      delay(10);

      //76SMART CAR 만들기 Ver 1.0

    } else if ( (cmU >= 20) && (cmU < 50) ) {
      sen_val = 20;
      //장애물에 근접하면 주행 속도를 늦춘다.
      //motor_speed_l -= 50 - cmU;
      //motor_speed_r -= 50 - cmU;
      //좌우 적외선 센서에 20cm 이내에 장애물이 없다면 전진
      if ( (i_InfraL >= sen_val ) && (i_InfraR >= sen_val) ) {
        g_carDirection = CAR_DIR_FW;
        car_update(); delay(10);
        //우측 적외선 센서에 장애물이 감지되면 좌측으로 급선회
      } else if ( (i_InfraL >= sen_val ) && (i_InfraR < sen_val) ) {
        g_carDirection = CAR_DIR_LF;
        car_update(); delay(10);
        //좌측 적외선 센서에 장애물이 감지되면 우측으로 급선회
      } else if ( (i_InfraL < sen_val ) && (i_InfraR >= sen_val) ) {
        g_carDirection = CAR_DIR_RF;
        car_update(); delay(10);
        //좌우 적외선 센서에 장애물이 감지되면 후진
      } if ( (i_InfraL < sen_val ) || (i_InfraR < sen_val) ) {
        g_carDirection = CAR_DIR_BK;
        car_update(); delay(30);
        if ( i_InfraL >= i_InfraR ) {
          g_carDirection = CAR_DIR_LF;
          car_update(); delay(50);
        } else {
          g_carDirection = CAR_DIR_RF;
          car_update(); delay(20);

          //77SMART CAR 만들기 Ver 1.0

        }
        //vAlram(3);
      }
    } else if ( cmU < 20 ) { //초음파센서에 50cm 이내에 장애물이 있다면 후진...
      sen_val = 15;
      g_carDirection = CAR_DIR_BK;
      car_update(); delay(30);

      if ( i_InfraL >= i_InfraR ) {
        g_carDirection = CAR_DIR_LF;
        car_update(); delay(50);
      } else {
        g_carDirection = CAR_DIR_RF;
        car_update(); delay(20);
      }
    }
  } else if ( drive_mode == 2 ) {
    //Serial.println("STOP");
    motor_speed_l = 0;
    motor_speed_r = 0;
    g_carDirection = CAR_DIR_ST;
    car_update();
    delay(5);
  }

  memset(c_lcd[0], 0, 20);
  memset(c_lcd[1], 0, 20);

  //78SMART CAR 만들기 Ver 1.0

  memset(c_lcd[2], 0, 20);
  memset(c_lcd[3], 0, 20);
  sprintf(c_lcd[0], "%c", "");
  sprintf(c_lcd[1], "%c", "");
  sprintf(c_lcd[2], "%c", "");
  sprintf(c_lcd[3], "%c", "");

  sprintf(c_lcd[0], "%s %d", "Pot_v : ", pot_value);
  sprintf(c_lcd[1], "%s %d cm", "Ultra : ", cmU);
  sprintf(c_lcd[2], "%s %dL %dR", "Infra : ", i_InfraL, i_InfraR);
  sprintf(c_lcd[3], "%s", "no receive data");
  if ( drive_mode == 1 ) {
    sprintf(c_lcd[0], "Pot:%d Ultra:%d", pot_value, cmU);
    sprintf(c_lcd[1], "IRL:%d IRR:%d", i_InfraL, i_InfraR);
    sprintf(c_lcd[2], "%d %d %d", get_val[1], get_val[2], get_val[3]);
    sprintf(c_lcd[3], "%d %d %d %d", get_val[4], get_val[5], get_val[6], get_val[7] );

  } else {
    //
  }
  /*
    u8g.firstPage();
    do {
    draw(0, 15, c_lcd[0]); //Potentiometer Value
    draw(0, 30, c_lcd[1]); //Ultrasonic HC-SR04 Value
    draw(0, 45, c_lcd[2]); //Left/Right Infrared sensor value
    draw(0, 60, c_lcd[3]); //Button Value

    79SMART CAR 만들기 Ver 1.0

    } while( u8g.nextPage() );
  */

  //OLED 출력을 Thread 로 처리, 쓰레드로 처리하지 않을시 동작 완료 때까지 지연됨.
  //Returns true, if the Thread should be runned.
  //(Basicaly,the logic is: (reached time AND is enabled?).
  if (LcdThread.shouldRun()) {
    LcdThread.run(); //This will run the Thread (call the callback function).
  }

  loop_cnt += 1;
  rcv_state = 0;

}

void GetDistance_UltraSonic() {

  //HC-SR04 Ultrasonic sensor(0~400cm)
  digitalWrite(Sonar_T, HIGH);
  delayMicroseconds(10);
  digitalWrite(Sonar_T, LOW);
  duration = pulseIn(Sonar_E, HIGH);
  cmU = microsecondsToCentimeters(duration);
  if ( cmU > MAX_DISTANCE ) {
    cmU = MAX_DISTANCE;
  }
  cmU_avg = (cmU_old + cmU) / 2;
  cmU_old = cmU;
  cmU = cmU_avg;

  //80SMART CAR 만들기 Ver 1.0

}

void GetDistance_InfraRed()
{
  //Sharp GP2Y0A21YK Infrared sensor(10~80cm)
  //좌우 측면에 장애물이 있는지 확인
  i_InfraL = constrain(analogRead(InfraL), 70, 800); delay(5);
  f_InfraL = 12343.85 * pow(i_InfraL, -1.15);
  i_InfraL = (int)((f_InfraL + i_InfraL_Old) / 2);

  i_InfraR = constrain(analogRead(InfraR), 70, 800); delay(5);
  f_InfraR = 12343.85 * pow(i_InfraR, -1.15);
  i_InfraR = (int)((f_InfraR + i_InfraR_Old) / 2);

  i_InfraL_Old = i_InfraL;
  i_InfraR_Old = i_InfraR;

}

void car_update()
{
  if ( g_carDirection == CAR_DIR_FW) {
    car_forward();
  } else if ( g_carDirection == CAR_DIR_BK) {
    car_backward();
  } else if ( g_carDirection == CAR_DIR_LF) {
    car_left();

    //81SMART CAR 만들기 Ver 1.0
  } else if ( g_carDirection == CAR_DIR_RF) {
    car_right();
  } else if ( g_carDirection == CAR_DIR_ST) {
    car_stop();
  }
}

void car_forward() {
  analogWrite(MR_ENABLE1, motor_speed_l);
  analogWrite(MR_ENABLE2, motor_speed_r);
  digitalWrite(MR_INT1, HIGH);
  digitalWrite(MR_INT2, LOW);
  digitalWrite(MR_INT3, HIGH);
  digitalWrite(MR_INT4, LOW);
}

void car_backward() {
  analogWrite(MR_ENABLE1, motor_speed_l);
  analogWrite(MR_ENABLE2, motor_speed_r);
  digitalWrite(MR_INT1, LOW);
  digitalWrite(MR_INT2, HIGH);
  digitalWrite(MR_INT3, LOW);
  digitalWrite(MR_INT4, HIGH);
}

void car_left() {
  analogWrite(MR_ENABLE1, motor_speed_l);

  //82SMART CAR 만들기 Ver 1.0

  analogWrite(MR_ENABLE2, motor_speed_r);
  digitalWrite(MR_INT1, HIGH);
  digitalWrite(MR_INT2, LOW);
  digitalWrite(MR_INT3, LOW);
  digitalWrite(MR_INT4, HIGH);
}

void car_right() {
  analogWrite(MR_ENABLE1, motor_speed_l);
  analogWrite(MR_ENABLE2, motor_speed_r);
  digitalWrite(MR_INT1, LOW);
  digitalWrite(MR_INT2, HIGH);
  digitalWrite(MR_INT3, HIGH);
  digitalWrite(MR_INT4, LOW);
}

void car_stop() {
  analogWrite(MR_ENABLE1, 0);
  analogWrite(MR_ENABLE2, 0);
}

void vAlram(int kind) {

  if ( kind == 0 ) { //Emergency Alram
    for ( iFreq = 150; iFreq < 1800; iFreq++ ) {
      tone(BUZZER, iFreq, 5);
    }

    //83SMART CAR 만들기 Ver 1.0

    for ( iFreq = 1800; iFreq >= 150; iFreq-- ) {
      tone(BUZZER, iFreq, 5);
    }
  } else if ( kind == 1 ) { //a note
    int elementCount = sizeof(iNote) / sizeof(int);
    for ( i = 0; i < elementCount; i++) //note 를 play
    {
      tone(BUZZER, iNote[i], 500);
      delay(50);
    }
  } else if ( kind == 2 ) { //50Hz
    tone(BUZZER, 100, 10);
  } else if ( kind == 3 ) { //400Hz
    tone(BUZZER, 400, 500);
  } else if ( kind == 4 ) { //1000Hz
    tone(BUZZER, 1000, 500);
  }

}

long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{

  //84SMART CAR 만들기 Ver 1.0

  return microseconds / 29 / 2;
}

void LcdCallBack() {
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);

  u8g.firstPage();
  do {
    u8g.drawStr(0, 15, c_lcd[0]); //Potentiometer Value
    u8g.drawStr(0, 30, c_lcd[1]); //Ultrasonic HC-SR04 Value
    u8g.drawStr(0, 45, c_lcd[2]); //Left/Right Infrared sensor value
    u8g.drawStr(0, 60, c_lcd[3]); //Button Value
  } while ( u8g.nextPage() );

}

void draw(int x, int y, String c_str) {
  memset(c_char, 0, c_str.length());
  for ( int ix = 0; ix < c_str.length() - 1; ix++ ) {
    c_char[ix] = c_str[ix];
  }
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( x, y, c_char);
}
