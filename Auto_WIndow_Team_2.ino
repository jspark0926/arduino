#define TIMER_MAX_COUNT           (0xffffU)
#define MOTOR_0                   2        // IN1을 2번핀에 연결
#define MOTOR_1                   3        // IN2을 3번핀에 연결
#define MOTOR_2                   4        // IN3을 4번핀에 연결
#define MOTOR_3                   5        // IN4을 5번핀에 연결
#define MOTOR_HIGH                HIGH
#define MOTOR_LOW                 LOW
#define MOTOR_POSITION_MIN        0
#define MOTOR_POSITION_MAX        15000
#define MOTOR_CONTROL(a,b,c,d)    {digitalWrite(MOTOR_0, a);digitalWrite(MOTOR_1, b);digitalWrite(MOTOR_2, c);digitalWrite(MOTOR_3, d);}

#define DHTPIN 10     // 온습도 센서가 10번에 연결
#define DHTTYPE DHT22   // DHT22 온습도 센서 사용
#include "DHT.h" //DHT 라이브러리 호출
DHT dht(10, DHT22); //DHT 설정(10,DHT22)
#define no_dust 0.60 //최초 전원 인가 후 깨끗한 환경에서의 지속적으로 측정되는 전압값 넣어주세요 

int dustout=A4; // 센서의 5번핀 먼지를 측정 아두이노 A4번에 연결

float vo_value=0; // 센서로 읽은 값 변수 선언
float sensor_voltage=0; // 센서로 읽은 값을 전압으로 측정한 값
float dust_density=0; // 실제 미세먼지 밀도




enum {
   TIMER_STOP,
   TIMER_START,
   TIMER_TIMEOUT
};

enum {
  MOTOR_MOVE_BACKWARD,
  MOTOR_MOVE_FORWARD
};

typedef struct
{
   unsigned char state;
   unsigned int time;
   unsigned int pastTime;
}TimerConfigType;

typedef struct
{
  unsigned char gpio_0;
  unsigned char gpio_1;
  unsigned char gpio_2;
  unsigned char gpio_3;
}MotorContorlTable;

unsigned char motorStep = 0;
unsigned char motorMode = MOTOR_MOVE_BACKWARD;
unsigned int motorPosition = MOTOR_POSITION_MIN;

void Timer_Start(TimerConfigType* TimerConfig, unsigned int time)
{
   TimerConfig->state = TIMER_START;
   TimerConfig->time = time;
   TimerConfig->pastTime = millis();
}

void Timer_Stop(TimerConfigType* TimerConfig)
{
   TimerConfig->state = TIMER_STOP;
}

unsigned char Timer_Check(TimerConfigType* TimerConfig)
{
   unsigned int time;

   if(TIMER_START == TimerConfig->state)
   {
      time = millis();
      
      if(time < TimerConfig->pastTime)
      {
         time = (TIMER_MAX_COUNT - TimerConfig->pastTime) + time + 1;
      }
      else
      {
         time = time - TimerConfig->pastTime;
      }

      if(time >= TimerConfig->time)
      {
         TimerConfig->state = TIMER_TIMEOUT;
      }
   }

   return TimerConfig->state;
}

TimerConfigType WindowCtrl_SensorTimer;
TimerConfigType WindowCtrl_MotorCtrlTimer;
MotorContorlTable WindowCtrl_MotorCtrlTable[9] = { // 모터제어 테이블
  {
    MOTOR_HIGH,
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_LOW,
  },
  {
    MOTOR_HIGH,
    MOTOR_HIGH,
    MOTOR_LOW,
    MOTOR_LOW,
  },
  {
    MOTOR_LOW,
    MOTOR_HIGH,
    MOTOR_LOW,
    MOTOR_LOW,
  },
  {
    MOTOR_LOW,
    MOTOR_HIGH,
    MOTOR_HIGH,
    MOTOR_LOW,
  },
  {
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_HIGH,
    MOTOR_LOW,
  },
  {
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_HIGH,
    MOTOR_HIGH,
  },
  {
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_HIGH,
  },
  {
    MOTOR_HIGH,
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_HIGH,
  },
  {
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_LOW,
    MOTOR_LOW,
  }
};

float get_voltage(float value)
{
 float V= value * 5.0 / 1024; // 아날로그값을 전압값으로 바꿈
 return V;
}

float get_dust_density(float voltage)
{
 float dust=(voltage-no_dust) / 0.005; // 데이터 시트에 있는 미세먼지 농도(ug) 공식 기준
 return dust;
}


void WindowCtrl_SensorTask(void)
{
  Serial.print("Motor Mode: ");
  Serial.println(motorMode);
  Serial.print("Motor Position: ");
  Serial.println(motorPosition);

  digitalWrite(v_led,LOW); 
  delayMicroseconds(280); // 280us동안 딜레이
  vo_value=analogRead(dustout); // 데이터를 읽음
  delayMicroseconds(40); // 320us - 280us
  delayMicroseconds(9680); // 10us(주기) -320us(펄스폭) 한 값
  sensor_voltage=get_voltage(vo_value); // get_voltage 함수 실행
  dust_density=get_dust_density(sensor_voltage);
  Serial.print("Dust Density = ");
  Serial.print(dust_density);
  Serial.println(" [ug/m^3]");
  float h = dht.readHumidity(); //습도값을 h에 저장
  Serial.print("Humidity: "); //문자열 출력
  Serial.print(h); //습도값 출력
  Serial.println("%");
  if ((dust_density > 80 || h > 80) && motorPosition == MOTOR_POSITION_MAX) {
    motorMode = 0;
  
   }
   else if((dust_density < 60 && h < 70) && motorPosition == MOTOR_POSITION_MIN){
    motorMode = 1;
   }
}

void WindowCtrl_MotorCtrlTask(void)
{
  unsigned char g0, g1, g2, g3;
  char direction;

  if(motorPosition <= MOTOR_POSITION_MIN && MOTOR_MOVE_FORWARD == motorMode) // 문이 0에서 양의방향 이동할때
  {
    direction = 1;
  }
  else if(motorPosition >= MOTOR_POSITION_MAX && MOTOR_MOVE_BACKWARD == motorMode) // 문이 0에서 음의방향 이동할때
  {
    direction = -1;
  }
  else if(motorPosition > MOTOR_POSITION_MIN && motorPosition < MOTOR_POSITION_MAX) // 기본 동작(문 열고 닫기)
  {
    if(MOTOR_MOVE_FORWARD == motorMode)
    {
      direction = 1;
    }
    else
    {
      direction = -1;
    }
  }
  else
  {
    direction = 0;
  }

  if(direction == 0) // 모든 출력 off
  {
    g0 = WindowCtrl_MotorCtrlTable[8].gpio_0;
    g1 = WindowCtrl_MotorCtrlTable[8].gpio_1;
    g2 = WindowCtrl_MotorCtrlTable[8].gpio_2;
    g3 = WindowCtrl_MotorCtrlTable[8].gpio_3;
  }
  else
  {
    motorPosition += direction;

    if(direction > 0 && motorStep >= 7)
    {
      motorStep  = 0;
    }
    else if(direction < 0 && motorStep <= 0)
    {
      motorStep = 7;
    }
    else
    {
      motorStep += direction;
    }
    
    g0 = WindowCtrl_MotorCtrlTable[motorStep].gpio_0;
    g1 = WindowCtrl_MotorCtrlTable[motorStep].gpio_1;
    g2 = WindowCtrl_MotorCtrlTable[motorStep].gpio_2;
    g3 = WindowCtrl_MotorCtrlTable[motorStep].gpio_3;
  }

  MOTOR_CONTROL(g0, g1, g2, g3)
}


void setup() {
  // put your setup code here, to run once:
  /*sensor 관련한 부분만 추가해서 초기화해주는 부분*/
  Serial.begin(9600);

  Timer_Start(&WindowCtrl_SensorTimer, 1000);
  Timer_Start(&WindowCtrl_MotorCtrlTimer, 1);

  pinMode(MOTOR_0, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);

  dht.begin();
}

void loop() {
  unsigned char sensorTimerState;
  unsigned char motorCtrlTimerState;

  sensorTimerState = Timer_Check(&WindowCtrl_SensorTimer);

  if(TIMER_TIMEOUT == sensorTimerState)
  {
    Timer_Start(&WindowCtrl_SensorTimer, 1000);
    WindowCtrl_SensorTask();
  }

  motorCtrlTimerState = Timer_Check(&WindowCtrl_MotorCtrlTimer);

  if(TIMER_TIMEOUT == motorCtrlTimerState)
  {
    Timer_Start(&WindowCtrl_MotorCtrlTimer, 1);
    WindowCtrl_MotorCtrlTask();
  }

}