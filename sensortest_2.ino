#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL313_U.h>
#include <SimpleKalmanFilter.h>
/*********************************************************************************************************************************************/
/*   Тестовая программа для акселерометра ADXL313                                                                                            */
/*  Программа считывает данные осей XYZ акселерометра и передает их Фильтру Калмана в виде simpleKalmanFilter_x(a, b, c).                    */
/*  Наиболее важным параметров в работе фильтра является третий параметр С. При значении 1 фильтра  фильтр пропускает  сигнал без фильтрации */
/*  При значениях меньше 0.1  0.05 итд..фильтрации сильнее но выходные данные реагирую медленне. Неровности лучше сглаживаются. Оптимальное  */
/*  значение 0.01. Далее массив deltas каждый его элемент накапливает количество  значений отклонения сигнала. если отклонений нет - все время*/
/*  идут одинаковые значение то увеличивается 0 элемент. Если отклонение 1 то увеличивается первый элемент..если отклонение больше 4 то увели-*/
/*  чивается 4-ый элемент. Далее адаптивное значение берется как максимальное значение среди всех элементов кроме 0 и 4-го.                   */
/*  Параметр  stability_samples задает количество выборок больше которого, если входные значение не выходили за delta то считается что        */
/*  система стабильна и производим замер угла.  При запросе хостом угла выдаем признак angle_stable. Если он 0 - то тряски нет и угол актуален*/
/*  в противном случае он равен значению delta -вычисленному адаптивному порогу срабатывания.                                                 */
/*  BLOCK1 блок опроса датчика. Запускать его в коде как минимум 20 раз в секунду (раз в 50 милисек) n раз                                          */
/*  BLOCK2 - анализ массива оклонений и вычисление нового отклонения.  Запуск его не реже  чем через 65535/n секунд. Где n кол-во запуска болка 1 в секунду */
/*  Требовние не жесткое его идея что бы не переполнилось значение элемента массива (65535) 




/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL313_Unified accel = Adafruit_ADXL313_Unified(12345);
// 3 object for filtered axis data
SimpleKalmanFilter simpleKalmanFilter_x(2, 4, 0.01);
SimpleKalmanFilter simpleKalmanFilter_y(2, 4, 0.01);
SimpleKalmanFilter simpleKalmanFilter_z(2, 4, 0.01);

int16_t estimated_value_x,estimated_value_y,estimated_value_z;                //current axis filtered value
int16_t estimated_value_x_prev,estimated_value_y_prev,estimated_value_z_prev; // previous axis filtered value
uint8_t stable=0;   //  stable filtered data counter
uint8_t stability_samples=50; 
uint8_t delta=2;   // special parameter difference between current and previous filtered axis acceleration !!!!!!!SENSITIVITY!!!!!
float angle;      //common angle
uint8_t  angle_stable; 
uint16_t  deltas[5]; //array delta times
uint16_t maximum,minimum;
uint8_t j,index;
void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.println  ("Data Rate:    "); 
    Serial.println   (accel.getDataRate()); 
  switch(accel.getDataRate())
  {
    case ADXL313_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL313_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL313_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL313_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL313_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL313_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL313_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL313_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL313_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL313_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL313_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL313_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL313_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL313_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL313_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL313_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL313_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL313_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    case ADXL313_RANGE_1_G:
      Serial.print  ("1"); 
      break;
    case ADXL313_RANGE_0_5_G:
      Serial.print  ("0.5 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}

void setup(void) 
{
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(115200);
  Serial.println("Accelerometer Test"); Serial.println("");
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);
  digitalWrite(13,LOW);
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL313 ... check your connections */
    Serial.println("Ooops, no ADXL313 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL313_RANGE_1_G);
  // accel.setRange(ADXL313_RANGE_8_G);
  // accel.setRange(ADXL313_RANGE_4_G);
  // accel.setRange(ADXL313_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");
  angle_stable=delta;
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
  //measure every 50ms
  /* BLOCK 1  */
  if (millis()%50==0) {
  /* Display the results (acceleration is measured in m/s^2) */

 digitalWrite(11,HIGH);
  // calculate the estimated value with Kalman Filter
  estimated_value_x = simpleKalmanFilter_x.updateEstimate(accel.getX());
  estimated_value_y = simpleKalmanFilter_y.updateEstimate(accel.getY());
  estimated_value_z = simpleKalmanFilter_z.updateEstimate(accel.getZ());
 digitalWrite(11,LOW);
 // if filtered values stable incremen counter
 if ( (abs((estimated_value_x-estimated_value_x_prev))<delta) && (abs((estimated_value_y-estimated_value_y_prev))<delta) && (abs((estimated_value_z-estimated_value_z_prev))<delta)  ) 
 
 {stable++;
 }
 else 
 
 {stable=0;
  angle_stable=delta;
 }

// if more than 100 times we have stable values measure angle
if (stable>stability_samples) 
{
  //measure angle
  stable=0;
  angle_stable=0;
 digitalWrite(12,HIGH);
   angle= 90- atan(abs(estimated_value_z)/(sqrt(pow(estimated_value_y,2)+pow(estimated_value_x,2)) ) )*57,296;
 digitalWrite(12,LOW);  
  
  } 
// adaptive delta counter
 index = abs(estimated_value_x-estimated_value_x_prev);
 index=(index>=4)?4:index;
 deltas[index]++;

 index = abs(estimated_value_y-estimated_value_y_prev);
 index=(index>=4)?4:index;
 deltas[index]++;


 index = abs(estimated_value_z-estimated_value_z_prev);
 index=(index>=4)?4:index;
 deltas[index]++;
  
  estimated_value_x_prev=estimated_value_x;
  estimated_value_y_prev=estimated_value_y;
  estimated_value_z_prev=estimated_value_z;

  /* END BLOCK 1 */

  
  Serial.print("X: "); Serial.print(accel.getX()); Serial.print("  ");
  Serial.print("Stable? "); Serial.print(angle_stable); Serial.print("  ");
  Serial.print("angle: "); Serial.print(angle); Serial.print("  ");
  
  Serial.print("Y: "); Serial.print(accel.getY()); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel.getZ()); Serial.print(" ");Serial.println(j);  
  
  }
/* BLOCK 2*/
// adaptive minimum calculation test alhorythm. run it every xx min
if (millis()%100000<2) {
// ищем максимум среди элементов массива кроме 0го и последнего элемента. Индекс этого элемента и есть порог статика/динамика
maximum=0;
j=0;
for (int i=1;i<4;i++) 
{
  if (deltas[i]>maximum) 
  {
    maximum=deltas[i];
    j=i;
    }
}
// 
delta=(j>=3)?3:j+1;

//clear deltas massive
memset(deltas,0,sizeof(deltas));
}

/* END BLOCK 2*/

}

