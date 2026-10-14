/*
project_ 4-Wheel Drive Electric Vehicle
//ai google gemini GPS M9N, IMU MPU9250 , Pure Pursuit , ExtendedKalmanFilter , 
//5-State Extended Kalman Filter (GPS M9N + SPI IMU + 4 Wheel Odom L298N)
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

UBLOX    - U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt)
*/
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.h> // 📌 เรียกใช้งานไลบรารี EspSoftwareSerial สำหรับช่องทางที่ 4
#include <math.h>
#include <SPI.h>
#include "MPU9250.h"

#define RX1_B1PIN 16     
#define TX1_B1PIN 17     
#define RX2_LIDAR 18     
#define TX2_LIDAR 19     
#define RPLIDAR_MOTOR 5  

// 📌 กำหนดขา GPIO ว่างสำหรับต่อเซนเซอร์ A01NYUB (สามารถเปลี่ยนเบอร์ขาได้ตามสะดวกครับ)
#define RX3_A01NYUB 15   // ต่อเข้าขา TX ของเซนเซอร์ A01NYUB
#define TX3_A01NYUB 4   // ต่อเข้าขา RX ของเซนเซอร์ (ถ้าไม่ใช้ส่งคำสั่ง ปล่อยลอยได้ครับ)

////SoftwareSerial Serial3;  // 🚀 สถาปนาออบเจกต์ช่องซีเรียลช่องที่ 4 ขึ้นมาใช้งาน
SoftwareSerial Serial3(RX3_A01NYUB, TX3_A01NYUB, false, 256); 

MPU9250 IMU(SPI, 2); 
SoftwareSerial gpsSerial(5, 4); 
WiFiUDP Udp;

const char* ssid     = "ชื่อ_WiFi_ของคุณ";
const char* password = "รหัส_WiFi_ของคุณ";
const char* targetIP = "111.111.111.111"; 
const int udpPort    = 4210;         
       
const int pinCS_IMU = 10; // พิน CS สำหรับ MPU9250 (SD3 บน NodeMCU)

// โครงสร้างข้อมูลไบนารีจาก GPS M9N
struct UBX_NAV_POSLLH_Payload {
    uint32_t iTOW; int32_t lon; int32_t lat; int32_t height; int32_t hMSL; uint32_t hAcc; uint32_t vAcc;
} __attribute__((packed));

struct UBX_NAV_VELNED_Payload {
    uint32_t iTOW; int32_t velN; int32_t velE; int32_t velD; uint32_t speed; uint32_t gSpeed; int32_t heading; uint32_t sAcc; uint32_t cAcc;
} __attribute__((packed));

UBX_NAV_POSLLH_Payload navPosLLH;
UBX_NAV_VELNED_Payload navVelNed;

const uint8_t CFG_VALSET_POSLLH[] = {0xB5, 0x62, 0x06, 0x8A, 0x08, 0x00, 0x00, 0x07, 0x00, 0x00, 0x23, 0x00, 0x91, 0x20, 0x01, 0x65, 0xCA};
const uint8_t CFG_VALSET_VELNED[] = {0xB5, 0x62, 0x06, 0x8A, 0x08, 0x00, 0x00, 0x07, 0x00, 0x00, 0x43, 0x00, 0x91, 0x20, 0x01, 0x85, 0x8D};
const uint8_t CFG_VALSET_10HZ[]   = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x4C, 0x8D};
enum UBX_State { WAIT_SYNC1, WAIT_SYNC2, WAIT_CLASS, WAIT_ID, WAIT_LEN1, WAIT_LEN2, READ_PAYLOAD, WAIT_CKA, WAIT_CKB };
enum UBX_State gpsState = WAIT_SYNC1;
uint8_t currentID = 0;

uint8_t gpsPayloadBuffer[128]; // มิติเดี่ยวระบุขนาดชัดเจน
uint16_t payloadLength = 0;
uint16_t payloadIndex = 0;
uint8_t calcCK_A = 0, calcCK_B = 0;
uint8_t receivedCK_A = 0, receivedCK_B = 0;

void updateGpsChecksum(uint8_t data) {
    calcCK_A += data;
    calcCK_B += calcCK_A;
}

// คอนฟิกูเรชันพิกัด
bool isHomeSet = false;
double homeLat = 0.0, homeLon = 0.0;
const double EARTH_RADIUS = 6378137.0;

// State Vector: [0:Pos_N, 1:Pos_E, 2:Vel_N, 3:Vel_E, 4:Heading]
double x_est[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double x_heading = 0.0;
double P_cov[5][5] = { {1,0,0,0,0}, {0,1,0,0,0}, {0,0,1,0,0}, {0,0,0,1,0}, {0,0,0,0,0.1} };

double filtered_lat = 0.0, filtered_lon = 0.0;
unsigned long lastTime = 0;
double imu_ax = 0.0, imu_ay = 0.0, imu_gz = 0.0;

 // --- 1. ตั้งค่าระยะเยื้องจริงที่วัดได้จากตัวรถ (หน่วย: เมตร) ---
const float OFFSET_GPS_X   = -0.095;  // GPS อยู่ข้างหน้าจุดหมุน 12 cm ระยะจากจุดหมุนรถ ไปยัง เสา GPS
const float OFFSET_GPS_Y   = 0.000; // GPS ติดตั้งเยื้องไปทางซ้ายของกลางรถ 5 cm (ค่าจึงเป็นลบ)
const float OFFSET_GPS_Z   0.65f  // ⚠️ ความสูงของเสา GPS จากจุดหมุนรถ (สูง 65 ซม.)
const float OFFSET_LIDAR_X = 0.245;  // LiDAR อยู่ข้างหน้าจุดหมุน 45 cm ระยะจากจุดหมุนรถ ไปยัง LiDAR
const float OFFSET_LIDAR_Y = 0.000;  // LiDAR อยู่ตรงกลางรถพอดีเป๊ะ ไม่เยื้องซ้ายขวา
const float OFFSET_LIDAR_Z   0.35f  // ⚠️ ความสูงของตัว LiDAR วัดจากระนาบจุดหมุน CG ขึ้นมา (เช่น สูง 35 ซม.)

// พารามิเตอร์ Pure Pursuit
struct Waypoint { double lat; double lon; double targetN; double targetE; };
Waypoint waypoints[] = { {13.847123, 100.493456, 0, 0}, {13.847550, 100.493900, 0, 0} };
const int TOTAL_WAYPOINTS = sizeof(waypoints) / sizeof(waypoints[0]);
int currentWaypointIdx = 0;
bool isNavigationComplete = false;
const double LOOK_AHEAD_DIST = 3.5;   
const double WHEELBASE = 0.35;        
const double ARRIVAL_RADIUS = 1.5;    
double steering_angle_deg = 0.0;     
// ==========================================
// 🛠️ ส่วนที่เพิ่มเข้ามา: Differential Drive & Speed Mixer
// ==========================================
// พารามิเตอร์ระยะห่างระหว่างล้อซ้ายและล้อขวา (Track Width / Drive Base)
const double TRACK_WIDTH = 0.30;       // ระยะห่างล้อซ้าย-ขวา (เมตร) เช่น 30 เซนติเมตร
const double MAX_BASE_SPEED = 1.5;     // ความเร็วสูงสุดตอนวิ่งทางตรง (m/s) เช่น 1.5 เมตรต่อวินาที
const double MIN_BASE_SPEED = 0.4;     // ความเร็วต่ำสุดตอนเข้าโค้งศอก (m/s) เพื่อไม่ให้รถหยุดนิ่ง

// ตัวแปรเอาต์พุตความเร็วของมอเตอร์แต่ละล้อ (หน่วย: m/s)
double motorSpeedFL = 0.0; // Front Left
double motorSpeedRL = 0.0; // Rear Left
double motorSpeedFR = 0.0; // Front Right
double motorSpeedRR = 0.0; // Rear Right

// ==================================================================
// 📌 ประกาศตัวแปรสถานะและเซนเซอร์ระดับ Global (สำหรับเชื่อมโยงระบบ)  ถาม AI Google Gemini
//โครงสร้างฟังก์ชัน Main_Kalman_Update_Loop() แบบรวมร่างสมบูรณ์แบบเสร็จสรรพในฟังก์ชันเดียว โดยถอดบทเรียนจาก คณิตศาสตร์ฟิสิกส์ 12 สถานะ, 
//ระบบล้างแรงเหวี่ยงหนีศูนย์, Adaptive Gain ตามรอบมอเตอร์ และระบบแปลงพิกัดมุมส่าย (Yaw Transformation) ร่วมกับ LiDAR และ Dual Ultrasonic ครับ

// ==================================================================
// ตัวแปรสถานะ (States X_1 ถึง X_12)
float X_1 = 0.0f;  // Position North (m)
float X_2 = 0.0f;  // Position East (m)
float X_3 = 0.0f;  // Yaw Angle (rad)
float X_4 = 0.0f;  // Velocity X Body (m/s)
float X_5 = 0.0f;  // Velocity Y Body (m/s)
float X_6 = 0.0f;  // Gyro Z / Yaw Rate (rad/s)
float X_7 = 0.0f;  // Motor 1 Speed (rad/s)
float X_8 = 0.0f;  // Motor 2 Speed (rad/s)
float X_9 = 0.0f;  // Motor 3 Speed (rad/s)
float X_10 = 0.0f; // Motor 4 Speed (rad/s)
float X_11 = 0.0f; // Accel X Filtered
float X_12 = 0.0f; // Accel Y Filtered

// ระยะติดตั้งเซนเซอร์ GPS/LiDAR ห่างจากจุดศูนย์กลางเครื่อง (CG) หน่วย: เมตร
#define OFFSET_GPS_X  0.10f  
#define OFFSET_GPS_Y  0.00f  

// อาร์เรย์และค่าแชร์จากหน่วยประมวลผลฉากหลัง (Background RTOS Tasks)
extern float global_lidar_distance[360];
extern volatile float dist_front_meters;
extern volatile float dist_back_meters;
extern float GyroZ, AccXf, AccYf, angle_diff;
extern float true_velN, true_velE;
extern SemaphoreHandle_t lidarMutex;

// ==================================================================
// 🛠 ฟังก์ชันย่อยสำหรับคัดกรองกรวยมุมเลเซอร์ LiDAR (Angle Window Max Filter)
// ==================================================================
float get_filtered_distance(float* lidar_array, int target_angle, int window_size) {
    float valid_samples[61]; // รองรับ window สูงสุด 30 องศา (±30)
    int sample_count = 0;
    
    for (int i = -window_size; i <= window_size; i++) {
        int current_angle = target_angle + i;
        if (current_angle < 0) current_angle += 360;
        if (current_angle >= 360) current_angle -= 360;
        
        float raw_dist = lidar_array[current_angle];
        if (raw_dist > 0.15f && raw_dist < 6.0f) {
            valid_samples[sample_count] = raw_dist;
            sample_count++;
        }
    }
    if (sample_count == 0) return -1.0f; 
    
    // คืนค่าที่ไหลที่สุดในกรวยเพื่อทะลุสิ่งกีดขวางไปหาผนังห้องจริง
    float max_dist = 0.0f;
    for (int j = 0; j < sample_count; j++) {
        if (valid_samples[j] > max_dist) max_dist = valid_samples[j];
    }
    return max_dist;
}

// ฟังก์ชันควบคุมมุมให้อยู่ในกรอบ 0 ถึง 2*PI
float normalizeAngle2Pi(float angle) {
    angle = fmod(angle, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;
    return angle;
}

// ==================================================================
// 🚀 ฟังก์ชันหลัก: ลูปทำนายและอัปเดตสถานะตัวกรอง (จบในหนึ่งเดียว)
// ==================================================================
void Main_Kalman_Update_Loop(float dtt, float FL_pwm2, float FR_pwm2, float BL_pwm2, float BR_pwm2) {
  
  // ------------------------------------------------------------------
  // PHASE 1: PREDICTION (คำนวณสถานะมอเตอร์และแรงขับล่วงหน้าด้วยคณิตศาสตร์ฟิสิกส์)
  // ------------------------------------------------------------------
  X_10 = X_10 + ((BR_pwm2 * 6.765f) - (X_10 * 37.47f)) * dtt; // Motor M4 ω₄
  X_9  = X_9  + ((BL_pwm2 * 6.765f) - (X_9  * 37.47f)) * dtt; // Motor M3 ω₃
  X_8  = X_8  + ((FR_pwm2 * 6.765f) - (X_8  * 37.47f)) * dtt; // Motor M2 ω₂
  X_7  = X_7  + ((FL_pwm2 * 6.765f) - (X_7  * 37.47f)) * dtt; // Motor M1 ω₁

  float Fxx = X_7 + X_8 + X_9 + X_10; // Total Thrust
  float Fyy = X_8 - X_7 + X_10 - X_9; // Pitch Dynamic
  float Tzz = X_8 - X_7 + X_9 - X_10; // Yaw Torque

  // ------------------------------------------------------------------
  // PHASE 2: ADAPTIVE GAIN CALCULATION (เปลี่ยน Noise แบบเรียลไทม์ตามรอบมอเตอร์)
  // ------------------------------------------------------------------
  float motor_speed_avg = (X_7 + X_8 + X_9 + X_10) * 0.25f;
  float motor_min_speed = 100.0f; 
  float motor_max_speed = 800.0f; 
  float v_factor = (motor_speed_avg - motor_min_speed) / (motor_max_speed - motor_min_speed);
  if(v_factor < 0.0f) v_factor = 0.0f;
  if(v_factor > 1.0f) v_factor = 1.0f;

  float K_vel = 0.954f - (v_factor * 0.700f); // บินเร็วมอเตอร์สั่น -> ลดการเชื่อเซนเซอร์ความเร็ว
  float K_acc = 13.254f - (v_factor * 10.000f); // บินเร็วมอเตอร์สั่น -> ลดการเชื่อเซนเซอร์ IMU ความเร่ง

  // ------------------------------------------------------------------
  // PHASE 3: LEVER ARM COMPENSATION (คาถาล้างแรงเหวี่ยงหนีศูนย์ของเซนเซอร์ความเร็ว)
  // ------------------------------------------------------------------
  float wz_rad = GyroZ; 
  float v_gyro_x = -wz_rad * OFFSET_GPS_Y;
  float v_gyro_y =  wz_rad * OFFSET_GPS_X;

  // แปลงความเร็วเหวี่ยงเข้าสู่พิกัดโลกก่อนทำความสะอาด
  float v_gyro_North = (v_gyro_x * cos(X_3)) - (v_gyro_y * sin(X_3));
  float v_gyro_East  = (v_gyro_x * sin(X_3)) + (v_gyro_y * cos(X_3));

  float clean_velN = true_velN - v_gyro_North;
  float clean_velE = true_velE - v_gyro_East;

  // 📌 Step 1: คำนวณระยะทางที่เสา GPS โยกเยกเบี่ยงเบนไป บนตัวบอดี้รถ (Body Frame)
    // เมื่อหน้ารถทิ่ม (Pitch) หรือรถตะแคง (Roll) ความสูง Z จะส่งผลให้เสาเหวี่ยงออกด้านข้าง
    float delta_x_body = (OFFSET_GPS_X * cos(pitch_rad)) + (OFFSET_GPS_Z * sin(pitch_rad));
    float delta_y_body = (OFFSET_GPS_Y * cos(roll_rad))  - (OFFSET_GPS_Z * sin(roll_rad));

    // 📌 Step 2: แตกแรงระยะที่โยกเยกเข้าสู่ทิศทางพิกัดโลก (Inertial Frame) โดยใช้มุม Yaw
    float delta_North = (delta_x_body * cos(yaw_rad)) - (delta_y_body * sin(yaw_rad));
    float delta_East  = (delta_x_body * sin(yaw_rad)) + (delta_y_body * cos(yaw_rad));
       
// 📌 3. คำนวณระยะที่หัวเลเซอร์ "โยกเยก" บนตัวบอดี้รถ (ชดเชยเครื่องหมายทิศทางฟิสิกส์ขวา)
  float delta_lidar_x_body = (OFFSET_LIDAR_X * cos(pitch_rad)) - (OFFSET_LIDAR_Z * sin(pitch_rad));
  float delta_lidar_y_body = (OFFSET_LIDAR_Y * cos(roll_rad))  + (OFFSET_LIDAR_Z * sin(roll_rad));

  // 📌 4. แปลงระยะโยกเยก 3 มิติเข้าสู่พิกัดโลก (Inertial Frame) ด้วยมุม Yaw
  float lidar_offset_N = (delta_lidar_x_body * cos(yaw_rad)) - (delta_lidar_y_body * sin(yaw_rad));
  float lidar_offset_E = (delta_lidar_x_body * sin(yaw_rad)) + (delta_lidar_y_body * cos(yaw_rad));

       
  // ------------------------------------------------------------------
  // PHASE 4: STATE UPDATE FOR VELOCITY & GYRO (ความเร็วเชิงเส้น-เชิงมุม)
  // ------------------------------------------------------------------
  X_6  = X_6  + ((Tzz * -0.018889f) - (X_6 * 10.1556111f) + (GyroZ - X_6) * 55.8291f) * dtt;
  X_5  = X_5  + ((Fyy * 0.000150f) - (X_5 * 10.4494100f) + (clean_velE - X_5) * K_vel) * dtt;
  X_4  = X_4  + ((Fxx * 0.297040f) - (X_4 * 8.8235290f) + (clean_velN - X_4) * K_vel) * dtt;
  X_11 = X_11 + ((Fxx * 0.1692081f) - (X_4 * 9.4845100f) + (AccXf - X_11) * K_acc) * dtt;
  X_12 = X_12 + ((Fxx * -0.119521f) - (X_4 * -7.0445000f) + (AccYf - X_12) * K_acc) * dtt;

  // อัปเดตมุม Yaw (สถานะ X_3)
  X_3 = X_3 + (X_6 + angle_diff * 0.917f) * dtt;
  X_3 = normalizeAngle2Pi(X_3);

  // ------------------------------------------------------------------
  // PHASE 5: COORDINATE ROTATION FOR LIDAR 360 (หมุนพิกัดเลเซอร์ตามมุมส่ายตัวลำ)
  // ------------------------------------------------------------------
  float yaw_deg_raw = fmod(X_3 * 180.0f / M_PI, 360.0f);
  int drone_yaw_deg = (int)yaw_deg_raw;
  if (drone_yaw_deg < 0) drone_yaw_deg += 360;

  int target_N = 0   - drone_yaw_deg; if (target_N < 0) target_N += 360;
  int target_E = 90  - drone_yaw_deg; if (target_E < 0) target_E += 360;
  int target_S = 180 - drone_yaw_deg; if (target_S < 0) target_S += 360;

  float r_lidar_North = -1.0f;
  float r_lidar_East  = -1.0f;
  float r_lidar_South = -1.0f;

  // ดึงข้อมูลอาเรย์จากเซนเซอร์ฉากหลังอย่างปลอดภัยด้วย Mutex
  if (xSemaphoreTake(lidarMutex, (TickType_t)5) == pdTRUE) {
      r_lidar_North = get_filtered_distance(global_lidar_distance, target_N, 15);
      r_lidar_East  = get_filtered_distance(global_lidar_distance, target_E, 15);
      r_lidar_South = get_filtered_distance(global_lidar_distance, target_S, 15);
      xSemaphoreGive(lidarMutex);
  }

  // ------------------------------------------------------------------
  // PHASE 6: DYNAMIC WALL & DUAL ULTRASONIC FUSION (คำนวณและผสมสัญญาณตำแหน่ง)
  // ------------------------------------------------------------------
  float final_posN = X_1;
  float final_posE = X_2;
  float K_pos_N = 0.0154f;
  float K_pos_E = 0.0124f;

  // --- จัดการแกนทิศเหนือ-ใต้ (แกน X โลก หรือ North) ---
  if (r_lidar_North > 0.0f && r_lidar_South > 0.0f) {
      // สร้างพิกัดกำแพงโลกปัจจุบันแบบแปรผันตามแผนที่จริง
      float dynamic_wall_north = X_1 + r_lidar_North;
      float dynamic_wall_south = X_1 - r_lidar_South;

      // แตกเวกเตอร์ระยะทางแนวเอียงของ Ultrasonic เข้าสู่แกนตรงพิกัดโลก
      float front_dist_world_N = dist_front_meters * cos(X_3);
      float back_dist_world_N  = -dist_back_meters * cos(X_3);

      // แปลงเป็นค่าพิกัดตัวลำอ้างอิงกำแพงแปรผัน
      float pos_N_from_front = dynamic_wall_north - front_dist_world_N;
      float pos_N_from_back  = dynamic_wall_south - back_dist_world_N;

      float sum_posN = 0.0f;
      float weight_N = 0.0f;

      // ดักสัญญาณกระชากหลบสิ่งกีดขวาง (Innovation Gate 40 เซนติเมตร)
      if (abs(pos_N_from_front - X_1) < 0.40f && dist_front_meters > 0.28f) {
          sum_posN += pos_N_from_front; weight_N += 1.0f;
      }
      if (abs(pos_N_from_back - X_1) < 0.40f && dist_back_meters > 0.28f) {
          sum_posN += pos_N_from_back; weight_N += 1.0f;
      }

      // ตกผลึกหาข้อสรุปตำแหน่งร่วมกันด่านสุดท้าย
      float lidar_base_posN = dynamic_wall_north - r_lidar_North;
      if (weight_N > 0.0f) {
          final_posN = (lidar_base_posN * 0.5f) + ((sum_posN / weight_N) * 0.5f);
          K_pos_N = 0.035f; // เพิ่มแรงดึง Gain เมื่อข้อมูลเซนเซอร์ผ่านเกณฑ์ยืนยันร่วมกัน
      } else {
          final_posN = lidar_base_posN; // ถ้าโดนบังกระชั้นชิด พึ่งพาเฉพาะข้อมูลเสถียรของ LiDAR ล่วงหน้า
          K_pos_N = 0.020f;
      }
  }

  // --- จัดการแกนทิศตะวันออก-ตะวันตก (แกน Y โลก หรือ East) ---
  if (r_lidar_East > 0.0f) {
      float dynamic_wall_east = X_2 + r_lidar_East;
      final_posE = dynamic_wall_east - r_lidar_East;
      K_pos_E = 0.025f;
  }

  // ------------------------------------------------------------------
  // PHASE 7: POSITION FINAL INTEGRATION (แปลงความเร็วประกอบพิกัดอัปเดตเข้า X_1, X_2)
  // ------------------------------------------------------------------
  float vel_world_N = (X_4 * cos(X_3)) - (X_5 * sin(X_3));
  float vel_world_E = (X_4 * sin(X_3)) + (X_5 * cos(X_3));

  X_1 = X_1 + (vel_world_N + (final_posN - X_1) * K_pos_N) * dtt;
  X_2 = X_2 + (vel_world_E + (final_posE - X_2) * K_pos_E) * dtt;
}
////////////////////////////////////////////////////////////////////////////
// ==================================================================
// 📌 กำหนดค่าคงที่เกน PID (Tuning Parameters - ปรับแต่งตามน้ำหนักรถจริง)
// ==================================================================
// เกนสำหรับควบคุมตำแหน่งและทิศทาง (Outer Loop)
float Kp_pos = 1.20f;   // เกนดึงรถเข้าหาจุดเป้าหมายพิกัด
float Kp_yaw = 2.50f;   // เกนหมุนหัวรถให้ตรงทางเป้าหมาย
float Kd_yaw = 0.15f;

// เกนสำหรับควบคุมความเร็วรอบล้อ (Inner Loop มอเตอร์)
float Kp_wheel = 3.50f;
float Ki_wheel = 0.12f;
float Kd_wheel = 0.05f;

// ขีดจำกัดทางฮาร์ดแวร์รถ
#define MAX_PWM 255.0f
#define TRACK_WIDTH 0.5f // ระยะห่างระว่างล้อซ้าย-ขวา (เมตร)

// ==================================================================
// 🚀 ฟังก์ชันหลัก: ระบบควบคุม PID ตำแหน่งและความเร็วล้อรถ 4 ล้อแยกอิสระ
// ==================================================================
void Main_PID_Control_Loop(float dtt, float target_X, float target_Y) {
  
  // ------------------------------------------------------------------
  // STEP 1: OUTER LOOP - POSITION CONTROL (คำนวณหาจุดต่างและมุมหันของรถ)
  // ------------------------------------------------------------------
  // 1. คำนวณระยะห่างพิกัดปัจจุบัน (X_1, X_2) เทียบกับจุดเป้าหมาย (target_X, target_Y)
  float error_X = target_X - X_1;
  float error_Y = target_Y - X_2;
  float distance_error = sqrt((error_X * error_X) + (error_Y * error_Y));

  // 2. คำนวณมุมเป้าหมายที่รถต้องหันหัวไป (Target Heading Angle)
  float target_yaw = atan2(error_Y, error_X);
  
  // 3. คำนวณส่วนต่างมุมหน้ารถปัจจุบัน (X_3 คือมุม Yaw ที่คาลมานคำนวณชดเชยแล้ว)
  float yaw_error = target_yaw - X_3;
  
  // บังคับส่วนต่างมุมให้อยู่ในช่วง -PI ถึง +PI เพื่อให้รถเลือกเลี้ยวในมุมที่สั้นที่สุดเสมอ
  if (yaw_error > M_PI)  yaw_error -= 2.0f * M_PI;
  if (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

  // 4. คำนวณคำสั่งควบคุมหลักประจำตัวรถ (Vx: ความเร็วตรง, Wz: ความเร็วหมุนเลี้ยว)
  float cmd_Vx = distance_error * Kp_pos;
  
  static float last_yaw_error = 0.0f;
  float cmd_Wz = (yaw_error * Kp_yaw) + ((yaw_error - last_yaw_error) / dtt * Kd_yaw);
  last_yaw_error = yaw_error;

  // จำกัดความเร็วตรงสูงสุดไม่ให้ล้อฟรีทิ้งรุนแรง
  if (cmd_Vx > 1.5f) cmd_Vx = 1.5f; // จำกัดที่ 1.5 m/s
  
  // ถ้าใกล้ถึงจุดหมายแล้ว (ระยะ < 10 ซม.) ให้หยุดเดินหน้า สั่งหมุนคุมตำแหน่งนิ่งๆ
  if (distance_error < 0.10f) cmd_Vx = 0.0f;

  // ------------------------------------------------------------------
  // STEP 2: DYNAMIC MIXER (แปลงคำสั่งความเร็วตัวรถ แยกสเกลเป็นรายล้อซ้าย-ขวา)
  // ------------------------------------------------------------------
  // สูตรคณิตศาสตร์จลนศาสตร์ผกผัน (Inverse Kinematics) ของรถถัง Skid-Steer
  float target_vel_Left  = cmd_Vx - (cmd_Wz * TRACK_WIDTH / 2.0f);
  float target_vel_Right = cmd_Vx + (cmd_Wz * TRACK_WIDTH / 2.0f);

  // แปลงหน่วย m/s เป็นความเร็วรอบเรเดียนต่อวินาที (rad/s) เพื่อให้ตรงกับสถานะคาลมาน X_7 - X_10
  // สมมติล้อรถมีเส้นผ่านศูนย์กลางขนาดเท่าในรูปภาพ (รัศมีล้อ R ประมาณ 0.13 เมตร)
  float wheel_radius = 0.13f;
  float target_w_Left  = target_vel_Left  / wheel_radius;
  float target_w_Right = target_vel_Right / wheel_radius;

  // ------------------------------------------------------------------
  // STEP 3: INNER LOOP - CLOSED-LOOP SPEED PID (คุมรอบล้อมอเตอร์รายล้อ)
  // ------------------------------------------------------------------
  // ประกาศตัวแปรสะสมค่าความผิดพลาดสะสม (Integral Errors) แบบ Static
  static float err_sum_FL = 0.0f, err_sum_BL = 0.0f;
  static float err_sum_FR = 0.0f, err_sum_BR = 0.0f;
  static float last_err_FL = 0.0f, last_err_BL = 0.0f;
  static float last_err_FR = 0.0f, last_err_BR = 0.0f;

  // คำนวณหา Error ของล้อแต่ละล้อ เทียบกับสถานะความเร็วรอบบริสุทธิ์จากคาลมานฟิลเตอร์
  float err_FL = target_w_Left  - X_7;   // ล้อหน้าซ้าย
  float err_BL = target_w_Left  - X_9;   // ล้อหลังซ้าย
  float err_FR = target_w_Right - X_8;   // ล้อหน้าขวา
  float err_BR = target_w_Right - X_10;  // ล้อหลังขวา

  // สะสมค่าความเพี้ยนเพื่อล้างภาระโหลดตอนเจอเนินหญ้าขรุขระ (Integral Term)
  err_sum_FL += err_FL * dtt; err_sum_BL += err_BL * dtt;
  err_sum_FR += err_FR * dtt; err_sum_BR += err_BR * dtt;

  // สมการ PID ขับจ่ายสัญญาณกำลัง PWM ออกสู่ชุดมอเตอร์ไดรฟ์ทักล้อ
  float FL_out = (err_FL * Kp_wheel) + (err_sum_FL * Ki_wheel) + ((err_FL - last_err_FL) / dtt * Kd_wheel);
  float BL_out = (err_BL * Kp_wheel) + (err_sum_BL * Ki_wheel) + ((err_BL - last_err_BL) / dtt * Kd_wheel);
  float FR_out = (err_FR * Kp_wheel) + (err_sum_FR * Ki_wheel) + ((err_FR - last_err_FR) / dtt * Kd_wheel);
  float BR_out = (err_BR * Kp_wheel) + (err_sum_BR * Ki_wheel) + ((err_BR - last_err_BR) / dtt * Kd_wheel);

  // บันทึกค่าประวัติความผิดพลาดรอบนี้ไว้ใช้รอบถัดไป
  last_err_FL = err_FL; last_err_BL = err_BL;
  last_err_FR = err_FR; last_err_BR = err_BR;

  // ------------------------------------------------------------------
  // STEP 4: SATURATION & HARDWARE OUTPUT (กรองสเกลสัญญาณจำกัดเพดานไฟ)
  // ------------------------------------------------------------------
  // บังคับจำกัดไฟ PWM ไม่ให้ทะลุระดับบอร์ดพัง (ช่วง -255.0 ถึง +255.0 โวลต์สมมติ)
  if (FL_out > MAX_PWM) FL_out = MAX_PWM; else if (FL_out < -MAX_PWM) FL_out = -MAX_PWM;
  if (BL_out > MAX_PWM) BL_out = MAX_PWM; else if (BL_out < -MAX_PWM) BL_out = -MAX_PWM;
  if (FR_out > MAX_PWM) FR_out = MAX_PWM; else if (FR_out < -MAX_PWM) FR_out = -MAX_PWM;
  if (BR_out > MAX_PWM) BR_out = MAX_PWM; else if (BR_out < -MAX_PWM) BR_out = -MAX_PWM;

  // 📌 นำค่าออกส่งคำสั่งต่อไปยังภาคควบคุมมอเตอร์ฮาร์ดแวร์รถของคุณ (เช่น บอร์ดไดรฟ์ IBTI / Sabertooth)
  analogWrite(pin_motor_FL_PWM, abs((int)FL_out));
  analogWrite(pin_motor_BL_PWM, abs((int)BL_out));
  analogWrite(pin_motor_FR_PWM, abs((int)FR_out));
  analogWrite(pin_motor_BR_PWM, abs((int)BR_out));
  
  // ทิศทางขั้วมอเตอร์ ดักตามเครื่องหมาย +/- ของค่าลัพธ์ PID
  digitalWrite(pin_motor_FL_DIR, (FL_out >= 0) ? HIGH : LOW);
  digitalWrite(pin_motor_BL_DIR, (BL_out >= 0) ? HIGH : LOW);
  digitalWrite(pin_motor_FR_DIR, (FR_out >= 0) ? HIGH : LOW);
  digitalWrite(pin_motor_BR_DIR, (BR_out >= 0) ? HIGH : LOW);
}

///////////////////////////////////////////////////////////////////////////
void calculateDifferentialDrive() {
    if (isNavigationComplete) {
        // หากถึงเป้าหมาย Waypoint ครบถ้วนแล้ว สั่งหยุดมอเตอร์ทั้งหมดทันที
        motorSpeedFL = 0.0; motorSpeedRL = 0.0;
        motorSpeedFR = 0.0; motorSpeedRR = 0.0;
        return;
    }

    // 1. ระบบ Adaptive Speed: คำนวณหาความเร็วทางตรงที่เหมาะสม (Base Speed)
    // ถ้ารถต้องหักเลี้ยวมาก (มุม steering_angle_deg สูง) ให้ชะลอความเร็วหลักลงเพื่อความปลอดภัย
    double abs_steer = abs(steering_angle_deg);
    double target_base_speed = MAX_BASE_SPEED * (1.0 - (abs_steer / 45.0)); // ปรับลดสัดส่วนตามมุมเลี้ยว
    target_base_speed = constrain(target_base_speed, MIN_BASE_SPEED, MAX_BASE_SPEED);

    // 2. แปลงมุมพวงมาลัยพยากรณ์จาก Pure Pursuit ให้กลายเป็น "ความเร็วเชิงมุมในการหมุนตัวรถ" (Yaw Rate 'omega')
    // สัมพันธ์ตามสูตรจลนศาสตร์: omega = (v * tan(delta)) / Wheelbase
    double steering_rad = steering_angle_deg * DEG_TO_RAD;
    double target_yaw_rate = (target_base_speed * tan(steering_rad)) / WHEELBASE;

    // 3. สมการ Differential Drive Kinematics: แยกคำนวณความเร็วฝั่งซ้ายและฝั่งขวา
    // v_left  = v - (omega * Track_Width / 2)
    // v_right = v + (omega * Track_Width / 2)
    double speed_left  = target_base_speed - (target_yaw_rate * TRACK_WIDTH / 2.0);
    double speed_right = target_base_speed + (target_yaw_rate * TRACK_WIDTH / 2.0);

    // 4. กระจายความเร็วส่งตรงให้มอเตอร์ทั้ง 4 ตัวแยกอิสระ
    motorSpeedFL = speed_left;
    motorSpeedRL = speed_left;  // มอเตอร์ฝั่งซ้ายทำงานความเร็วเท่ากัน
    
    motorSpeedFR = speed_right;
    motorSpeedRR = speed_right; // มอเตอร์ฝั่งขวาทำงานความเร็วเท่ากัน
}

// ==========================================
// 🛠️ ส่วนที่เพิ่มเข้ามา: พารามิเตอร์มอเตอร์ 4 ตัว และ Wheel Odometry
// ==========================================
const double WHEEL_RADIUS = 0.05; // รัศมีล้อรถ (เมตร) เช่น 5 เซนติเมตร
// สมมติฟังก์ชันอ่านค่าความเร็วเชิงมุมจาก Encoder ของแต่ละล้อ (หน่วย: rad/s)
double getWheelSpeedFL() { return 10.0; } // ล้อหน้าซ้าย (Front Left) - แทนที่ด้วยค่าจริงจากฮาร์ดแวร์ของคุณ
double getWheelSpeedFR() { return 10.0; } // ล้อหน้าขวา (Front Right)
double getWheelSpeedRL() { return 10.0; } // ล้อหลังซ้าย (Rear Left)
double getWheelSpeedRR() { return 10.0; } // ล้อหลังขวา (Rear Right)

// ฟังก์ชันคำนวณความเร็วเฉลี่ยของตัวรถจากล้อทั้ง 4 (Forward Kinematics)
double getVehicleOdomSpeed() {
    double v_fl = getWheelSpeedFL() * WHEEL_RADIUS;
    double v_fr = getWheelSpeedFR() * WHEEL_RADIUS;
    double v_rl = getWheelSpeedRL() * WHEEL_RADIUS;
    double v_rr = getWheelSpeedRR() * WHEEL_RADIUS;
    // นำความเร็วของล้อทั้ง 4 มารวมกันเพื่อหาค่าเฉลี่ยความเร็วเดินหน้าของตัวรถ (m/s)
    return (v_fl + v_fr + v_rl + v_rr) / 4.0;
}

// ==========================================
// 5. ฟังก์ชันย่อยสำหรับเข้าถึง MPU9250 แบบไม่มีพอยน์เตอร์
// ==========================================
void writeRegisterSPI(uint8_t subAddress, uint8_t data) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS_IMU, LOW);
    SPI.transfer(subAddress);
    SPI.transfer(data);
    digitalWrite(pinCS_IMU, HIGH);
    SPI.endTransaction();
}

void setupNativeMPU9250() {
    pinMode(pinCS_IMU, OUTPUT);
    digitalWrite(pinCS_IMU, HIGH);
    SPI.begin();
    writeRegisterSPI(27, 0x08); 
    writeRegisterSPI(28, 0x08); 
}

void readNativeIMU() {
    uint8_t imuBuffer[14]; 
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS_IMU, LOW);
    SPI.transfer(59 | 0x80); 
    for (int i = 0; i < 14; i++) {
        imuBuffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(pinCS_IMU, HIGH);
    SPI.endTransaction();
    
    int16_t raw_ax = (((int16_t)imuBuffer[0]) << 8) | imuBuffer[1];
    int16_t raw_ay = (((int16_t)imuBuffer[2]) << 8) | imuBuffer[3];
    int16_t raw_gz = (((int16_t)imuBuffer[12]) << 8) | imuBuffer[13];
    
    imu_ax = (double)raw_ax / 8192.0 * 9.80665;
    imu_ay = (double)raw_ay / 8192.0 * 9.80665;
    imu_gz = (double)raw_gz / 65.5 * DEG_TO_RAD;
}


// ฟังก์ชันแปลงพิกัดและการกรองมุม
void convertLatLonToMeters(double lat, double lon, double &posN, double &posE) {
    if (!isHomeSet) { homeLat = lat; homeLon = lon; isHomeSet = true; }
    posN = (lat - homeLat) * DEG_TO_RAD * EARTH_RADIUS;
    posE = (lon - homeLon) * DEG_TO_RAD * EARTH_RADIUS * cos(homeLat * DEG_TO_RAD);
}
void convertMetersToLatLon(double posN, double posE, double &lat, double &lon) {
    if (!isHomeSet) return;
    lat = homeLat + (posN / (EARTH_RADIUS * DEG_TO_RAD));
    lon = homeLon + (posE / (EARTH_RADIUS * DEG_TO_RAD * cos(homeLat * DEG_TO_RAD)));
}
double normalizeAngle(double angle) {
    while (angle > PI)  angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}
bool parseGpsBinary() {
    bool newGPSData = false;
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        switch (gpsState) {
            case WAIT_SYNC1: if (c == 0xB5) gpsState = WAIT_SYNC2; break;
            case WAIT_SYNC2: if (c == 0x62) gpsState = WAIT_CLASS; else gpsState = WAIT_SYNC1; break;
            case WAIT_CLASS: calcCK_A = 0; calcCK_B = 0; updateGpsChecksum(c); if (c == 0x01) gpsState = WAIT_ID; else gpsState = WAIT_SYNC1; break;
            case WAIT_ID: updateGpsChecksum(c); if (c == 0x02 || c == 0x12) { currentID = c; gpsState = WAIT_LEN1; } else gpsState = WAIT_SYNC1; break;
            case WAIT_LEN1: updateGpsChecksum(c); payloadLength = c; gpsState = WAIT_LEN2; break;
            case WAIT_LEN2: updateGpsChecksum(c); payloadLength |= (c << 8);
                if ((currentID == 0x02 && payloadLength == 28) || (currentID == 0x12 && payloadLength == 36)) { payloadIndex = 0; gpsState = READ_PAYLOAD; } 
                else gpsState = WAIT_SYNC1; break;
            case READ_PAYLOAD: updateGpsChecksum(c); if (payloadIndex < sizeof(gpsPayloadBuffer)) gpsPayloadBuffer[payloadIndex++] = c;
                if (payloadIndex >= payloadLength) gpsState = WAIT_CKA; break;
            case WAIT_CKA: receivedCK_A = c; gpsState = WAIT_CKB; break;
            case WAIT_CKB: receivedCK_B = c;
                if (calcCK_A == receivedCK_A && calcCK_B == receivedCK_B) {
                    if (currentID == 0x02) memcpy(&navPosLLH, gpsPayloadBuffer, sizeof(navPosLLH));
                    if (currentID == 0x12) memcpy(&navVelNed, gpsPayloadBuffer, sizeof(navVelNed));
                    newGPSData = true;
                }
                gpsState = WAIT_SYNC1; break;
        }
    }
    return newGPSData;
}
// ==========================================
// 🏎️ EXTENDED KALMAN FILTER WITH MOTOR FUSION
// ==========================================
void runExtendedKalmanFilter(double dt, bool hasNewGps) {
    readNativeIMU(); 

    // อ่านความเร็วจากมอเตอร์ 4 ตัวผ่านฟังก์ชัน Odometry
    double v_odom = getVehicleOdomSpeed();
    
    // --- 1. PREDICTION PHASE (ทำนายล่วงหน้าผ่านตัวแปรเดี่ยว) ---
    double a_North = imu_ax * cos(x_heading) - imu_ay * sin(x_heading);
    double a_East  = imu_ax * sin(x_heading) + imu_ay * cos(x_heading);


    // ดึงค่าทิศทางปัจจุบันจากตัวกรอง
    double theta = x_est[4]; 

    // --- [1. PREDICTION PHASE: ขับเคลื่อนโมเดลด้วย IMU + Gyro] ---

    x_est[0] += (x_est[2] * dt) + (0.5 * a_North * dt * dt); // Pos_N
    x_est[1] += (x_est[3] * dt) + (0.5 * a_East * dt * dt);  // Pos_E
    x_est[2] += a_North * dt;                               // Vel_N
    x_est[3] += a_East * dt;                                // Vel_E
    x_est[4] += imu_gz * dt;                                    // Headingจาก Gyro
    x_est[4] = normalizeAngle(x_est[4]);
    x_heading = x_est[4];

    double q_imu = 0.05; double q_gyro = 0.01;
    for(int i=0; i<4; i++) P_cov[i][i] += q_imu * dt;
    P_cov[4][4] += q_gyro * dt;

    // --- [2. UPDATE PHASE 1: ปรับแก้ด้วยความเร็วของมอเตอร์ 4 ตัว (Wheel Odometry)] ---
    // แปลงความเร็วจากตัวรถเข้าสู่แกนโลก (ทิศเหนือ/ทิศตะวันออก)
    double z_odom_velN = v_odom * cos(x_est[4]);
    double z_odom_velE = v_odom * sin(x_est[4]);
    double R_odom = 0.04; // ค่าความแปรปรวนของมอเตอร์ (ระบุความคลาดเคลื่อนล้อฟรี/สลิป)

    double K_odomN = P_cov[2][2] / (P_cov[2][2] + R_odom);
    double K_odomE = P_cov[3][3] / (P_cov[3][3] + R_odom);

    x_est[2] += K_odomN * (z_odom_velN - x_est[2]);
    x_est[3] += K_odomE * (z_odom_velE - x_est[3]);
    P_cov[2][2] *= (1.0 - K_odomN);
    P_cov[3][3] *= (1.0 - K_odomE);

    // --- [3. UPDATE PHASE 2: ปรับแก้ด้วยสัญญาณไบนารีจาก GPS M9N] ---
    if (hasNewGps) {
        // (ระบบพาร์สเซอร์จะดักจับข้อมูลเข้าโครงสร้างอัตโนมัติ)
        double z_posN, z_posE;
        convertLatLonToMeters(navPosLLH.lat/10000000.0, navPosLLH.lon/10000000.0, z_posN, z_posE);
        
        static bool waypointInitialized = false;
        if (isHomeSet && !waypointInitialized) {
            for (int i = 0; i < TOTAL_WAYPOINTS; i++) {
                convertLatLonToMeters(waypoints[i].lat, waypoints[i].lon, waypoints[i].targetN, waypoints[i].targetE);
            }
            waypointInitialized = true;
        }

        double z_gps_velN = navVelNed.velN / 100.0;
        double z_gps_velE = navVelNed.velE / 100.0;
        double z_heading = normalizeAngle((navVelNed.heading / 100000.0) * DEG_TO_RAD);

        double R_pos = sq((double)navPosLLH.hAcc / 1000.0);
        double R_vel = sq((double)navVelNed.sAcc / 1000.0);
        double R_hdg = sq((double)navVelNed.cAcc / 100000.0 * DEG_TO_RAD);

        double K[5];
        K[0] = P_cov[0][0] / (P_cov[0][0] + R_pos);
        K[1] = P_cov[1][1] / (P_cov[1][1] + R_pos);
        K[2] = P_cov[2][2] / (P_cov[2][2] + R_vel);
        K[3] = P_cov[3][3] / (P_cov[3][3] + R_vel);
        K[4] = P_cov[4][4] / (P_cov[4][4] + R_hdg);

        x_est[0] += K[0] * (z_posN - x_est[0]);
        x_est[1] += K[1] * (z_posE - x_est[1]);
        x_est[2] += K[2] * (z_gps_velN - x_est[2]);
        x_est[3] += K[3] * (z_gps_velE - x_est[3]);
        x_est[4] += K[4] * normalizeAngle(z_heading - x_est[4]);
        x_est[4] = normalizeAngle(x_est[4]);

        for(int i=0; i<5; i++) P_cov[i][i] *= (1.0 - K[i]);
    }

    convertMetersToLatLon(x_est[0], x_est[1], filtered_lat, filtered_lon);
}

// อัลกอริทึม Pure Pursuit 
void updatePurePursuit() {
    if (isNavigationComplete || !isHomeSet) return;
    double carN = x_est[0]; double carE = x_est[1]; double carHeading = x_est[4];
    double wpN = waypoints[currentWaypointIdx].targetN; double wpE = waypoints[currentWaypointIdx].targetE;
    double distance_to_wp = sqrt(sq(wpN - carN) + sq(wpE - carE));

    if (distance_to_wp < ARRIVAL_RADIUS) {
        currentWaypointIdx++;
        if (currentWaypointIdx >= TOTAL_WAYPOINTS) { isNavigationComplete = true; steering_angle_deg = 0.0; return; }
        wpN = waypoints[currentWaypointIdx].targetN; wpE = waypoints[currentWaypointIdx].targetE;
    }
    double dx = wpE - carE; double dy = wpN - carN;
    double target_local_x = dx * cos(-carHeading) - dy * sin(-carHeading);
    double curvature = (2.0 * target_local_x) / sq(LOOK_AHEAD_DIST);
    steering_angle_deg = atan(curvature * WHEELBASE) * RAD_TO_DEG;
    steering_angle_deg = constrain(steering_angle_deg, -35.0, 35.0);
}
const int pinENA = 14; // D5
const int pinIN1 = 15; // D8
const int pinIN2 = 16; // D0
const int pinIN3 = 12; // D6
const int pinIN4 = 13; // D7
const int pinENB = 0;  // D3

void setupL298N() {
    pinMode(pinENA, OUTPUT);
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);
    pinMode(pinIN3, OUTPUT);
    pinMode(pinIN4, OUTPUT);
    pinMode(pinENB, OUTPUT);
}
void driveL298N() {
  double speedLeft = 0.0, speedRight = 0.0;
  if (!isNavigationComplete && isHomeSet) {
double abs_steer = abs(steering_angle_deg);
double target_base_speed = MAX_BASE_SPEED * (1.0 - (abs_steer / 45.0));
target_base_speed = constrain(target_base_speed, MIN_BASE_SPEED, MAX_BASE_SPEED);
double target_yaw_rate = (target_base_speed * tan(steering_angle_deg * DEG_TO_RAD)) / WHEELBASE;
speedLeft  = target_base_speed - (target_yaw_rate * TRACK_WIDTH / 2.0);
speedRight = target_base_speed + (target_yaw_rate * TRACK_WIDTH / 2.0);
}

    // 1. แปลงความเร็ว m/s เป็นค่า PWM (0-255)
    // สมมติความเร็วสูงสุด 1.5 m/s เทียบเท่า PWM 255
    int pwmLeft = map(constrain(abs(speedLeft), 0.0, 1.5), 0.0, 1.5, 0, 255);
    int pwmRight = map(constrain(abs(speedRight), 0.0, 1.5), 0.0, 1.5, 0, 255);

    // 2. ควบคุมทิศทางและส่งความเร็วฝั่งซ้าย (Channel A)
    if (speedLeft >= 0) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
    } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
    }
    analogWrite(pinENA, pwmLeft);

    // 3. ควบคุมทิศทางและส่งความเร็วฝั่งขวา (Channel B)
    if (speedRight >= 0) {
        digitalWrite(pinIN3, HIGH);
        digitalWrite(pinIN4, LOW);
    } else {
        digitalWrite(pinIN3, LOW);
        digitalWrite(pinIN4, HIGH);
    }
    analogWrite(pinENB, pwmRight);
}
// ==========================================
// 9. ฟังก์ชันตั้งค่าระบบและลูปหลัก
// ==========================================
void setup() {
    Serial.begin(115200); 
    //gpsSerial.begin(38400);//gps 115200
    //gpsSerial.print("$PUBX,41,1,0003,0001,115200,0*1E\r\n");
    //gpsSerial.begin(115200);
       
Serial1.begin(230400, SERIAL_8N1, RX1_B1PIN, TX1_B1PIN);
Serial2.begin(115200, SERIAL_8N1, RX2_LIDAR, TX2_LIDAR); 
  // 📌 🚀 เปิดพอร์ตจำลองช่องที่ 4 สำหรับคุยกับ A01NYUB ที่ความเร็วมาตรฐาน 9600 bps
  //Serial3.begin(9600, SWSERIAL_8N1, RX3_A01NYUB, TX3_A01NYUB, false, 256);
Serial3.begin(9600);
delay(1500);     
xDataMutex = xSemaphoreCreateMutex();
xTaskCreatePinnedToCore(BackgroundReaderTask, "CommsReader", 8192, NULL, 3, NULL, 0);
delay(1500);
  analogWrite(RPLIDAR_MOTOR, 250);
  uint8_t start_scan_cmd[] = {0xA5, 0x20};
  Serial2.write(start_scan_cmd, 2);
  delay(1000);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    Serial.write(CFG_VALSET_POSLLH, sizeof(CFG_VALSET_POSLLH)); delay(100);
    Serial.write(CFG_VALSET_VELNED, sizeof(CFG_VALSET_VELNED)); delay(100);
    Serial.write(CFG_VALSET_10HZ,   sizeof(CFG_VALSET_10HZ));   delay(100);
    IMU.begin();
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    //setupNativeMPU9250();
    lastTime = micros();
}

void loop() {
    unsigned long now = micros();
    double dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if (dt <= 0.0 || dt > 0.1) dt = 0.01;
    bool hasGpsUpdate = parseGpsBinary();

    // 1. ประมวลผลฟิวชันพิกัดตำแหน่งตัวรถระดับ 100Hz
    runExtendedKalmanFilter(dt, hasGpsUpdate);
    //runExtendedKalmanFilter(dt);
    
    // 2. คำนวณหามุมหักเลี้ยวเป้าหมายจาก Pure Pursuit
    updatePurePursuit();

    // 3. ใหม่: คำนวณความเร็วแยกแต่ละมอเตอร์ด้วย Differential Drive
    calculateDifferentialDrive();
    driveL298N();
    
    // 4. แพ็กข้อมูลมอนิเตอร์ รวมถึงความเร็วมอเตอร์ฝั่งซ้าย-ขวา ส่งออกไร้สาย
    double current_speed = sqrt(sq(x_est[2]) + sq(x_est[3])) * 3.6;
    String telemetryData = String(filtered_lat, 7) + "," + String(filtered_lon, 7) + "," + 
                           String(current_speed, 1) + "," + String(x_est[4] * RAD_TO_DEG, 1) + "," +
                           String(steering_angle_deg, 1) + "," +
                           String(motorSpeedFL, 2) + "," + String(motorSpeedFR, 2);

    Udp.beginPacket(targetIP, udpPort); 
    Udp.write(telemetryData.c_str()); 
    Udp.endPacket(); 

    // ----------------------------------------------------------------------
    // 🛠️ หน้างาน: ส่วนแปลงหน่วยความเร็วเป็นสัญญาณควบคุมไดรเวอร์มอเตอร์ของคุณจริง
    // คุณสามารถนำค่า `motorSpeedFL` และ `motorSpeedFR` (หน่วย m/s) ไปเขียนฟังก์ชัน 
    // ตัวอย่าง: แปลงเมตรต่อวินาทีเป็นค่า PWM (0-255) หรือส่งเข้า PID Controller เพื่อคุมรอบล้อ (RPM)
    // ----------------------------------------------------------------------

    delay(10); 
}

// ------------------------------------------------------------------
// 🛠 ฟังก์ชันย่อย: State Machine ถอดรหัสบิตข้อมูล 4 ไบต์จากเซนเซอร์ A01NYUB ดิบ
// ------------------------------------------------------------------
void parseA01NYUB() {
    static int state = 0;
    static uint8_t ultra_buf[4];
    static int idx = 0;

    while (Serial3.available() > 0) {
        uint8_t c = Serial3.read();

        switch (state) {
            case 0: // ค้นหาหัวข้อความตัวเริ่ม 0xFF
                if (c == 0xFF) {
                    ultra_buf[0] = c;
                    idx = 1;
                    state = 1;
                }
                break;

            case 1: // บันทึกไบต์ที่เหลือจนครบ 4 ไบต์
                ultra_buf[idx++] = c;
                if (idx >= 4) {
                    // คำนวณหาค่าตรวจสอบ Checksum ความถูกต้องของข้อมูลสายบิต
                    uint8_t sum = (ultra_buf[0] + ultra_buf[1] + ultra_buf[2]) & 0xFF;
                    
                    if (sum == ultra_buf[3]) { // ถ้ารหัส Checksum ตรงแปลว่าข้อมูลถูกต้อง
                        // ประกอบบิตสูงและบิตต่ำรวมร่างเป็นระยะทางจริง (หน่วย: มิลลิเมตร)
                        uint16_t raw_mm = (ultra_buf[1] << 8) | ultra_buf[2];
                        
                        // กรองค่าผิดพลาด (A01NYUB จะวัดระยะจริงได้ตั้งแต่ 280 mm ถึง 7500 mm)
                        if (raw_mm >= 250 && raw_mm <= 7500) {
                            if (xSemaphoreTake(xDataMutex, uint32_t(2))) {
                                distance_A01NYUB = (float)raw_mm;
                                
                                // 💡 แถมคาถา Fail-Safe: ถ้าพี่เอาตัวนี้หันหน้าตรงยื่นไปด้านหน้า แล้วเจอคนตัดหน้าระยะประชิดต่ำกว่า 50 ซม.
                                // ให้บีบปรับตัวแปรดักเบรกฉุกเฉินร่วมกับเรดาร์หลักได้ทันทีเพื่อความชัวร์ดับเบิ้ลล็อกครับ
                                if (distance_A01NYUB < min_distance_recorded) {
                                    min_distance_recorded = distance_A01NYUB;
                                }
                                xSemaphoreGive(xDataMutex);
                            }
                        }
                    }
                    state = 0; // รีเซ็ตสเตทเพื่อรอรับแพ็กเกจก้อนถัดไป
                }
                break;
        }
    }
}

void BackgroundReaderTask(void *pvParameters) {
  for (;;) {
    while (Serial1.available() > 0) { parseBoard1Packet(Serial1.read()); }
    while (Serial2.available() > 0) { processOldLidarProtocol(Serial2.read()); } 
    parseA01NYUB();
    vTaskDelay(1 / portTICK_PERIOD_MS);//สิ่งที่ถูกต้อง: ให้เปลี่ยนไปใช้ฟังก์ชันมาตรฐาน pdMS_TO_TICKS(1) แทนครับ
  }
}

// (ฟังก์ชัน parseBoard1Packet, processOldLidarProtocol, accumulate_obstacle_force คงเดิมสมบูรณ์...)
void parseBoard1Packet(uint8_t c) {
    static int state = 0; static int rx_index = 0;
    switch (state) {
        case 0: if (c == 0x6C) { decoder.raw_bytes[0] = c; rx_index = 1; state = 1; } break;
        case 1:
            decoder.raw_bytes[rx_index++] = c;
            if (rx_index >= 38) { 
                if (decoder.data.tail == 0xAA) {
                    if (xSemaphoreTake(xDataMutex, uint32_t(2))) {
                        b1_telemetry.float_yaw   = decoder.data.yaw;
                        b1_telemetry.float_lat   = decoder.data.lat;  b1_telemetry.float_lon   = decoder.data.lon;  
                        b1_telemetry.float_roll  = decoder.data.roll; b1_telemetry.float_pitch = decoder.data.pitch;
                        b1_telemetry.grN_speed   = decoder.data.velN; b1_telemetry.grE_speed   = decoder.data.velE;
                        b1_telemetry.ground_speed = sqrt(pow(decoder.data.velN, 2) + pow(decoder.data.velE, 2)) / 100.0;
                        xSemaphoreGive(xDataMutex);
                    }
                }
                state = 0;
            }
            break;
    }
}
void processOldLidarProtocol(uint8_t rawByte) {
    static int bufferIndex = 0; static uint8_t packetBuffer[5];
    if (bufferIndex == 0) {
        uint8_t startBit = rawByte & 0x01; uint8_t invertedStartBit = (rawByte >> 1) & 0x01;
        if (startBit != invertedStartBit) packetBuffer[bufferIndex++] = rawByte;
    } 
    else if (bufferIndex == 1) {
        if ((rawByte & 0x01) == 1) packetBuffer[bufferIndex++] = rawByte;
        else bufferIndex = 0;
    } 
    else { packetBuffer[bufferIndex++] = rawByte; }
    if (bufferIndex >= 5) {
        uint8_t quality = packetBuffer[0] >> 2;
        uint16_t rawAngle = (packetBuffer[1] >> 1) | (packetBuffer[2] << 7);
        float angle = ((float)rawAngle) / 64.0; 
        uint16_t rawDistance = packetBuffer[3] | (packetBuffer[4] << 8);
        float distance = ((float)rawDistance) / 4.0; 
        if (distance > 150.0 && distance <= 8000.0 && quality > 0) {
            if (xSemaphoreTake(xDataMutex, uint32_t(2))) {
                accumulate_obstacle_force(angle, distance, b1_telemetry.float_yaw);
                if (angle >= 355.0 || angle <= 5.0) {
                    if (distance < min_distance_recorded) { min_distance_recorded = distance; }
                }
        // 📌 จุดเพิ่มเติม: ถ้ารอบสแกนกวาดหน้าตรงจุดนี้ วัดระยะออกมาแล้วไกลทะลุทางโล่ง (เกิน 4 เมตรขึ้นไป)
        // ให้สมองกลบน Core 0 สั่งปลดปล่อยเคลียร์ค่ากลับสู่ระยะเริ่มต้นออโต้ทันทีครับ
        if ((angle >= 355.0 || angle <= 5.0) && distance > 1000.0) {
            min_distance_recorded = distance;//8000.0
        }
                xSemaphoreGive(xDataMutex);
            }
        }
        bufferIndex = 0;
    }
}
void accumulate_obstacle_force(float angle_lidar, float distance_mm, float b1_yaw) {
if (distance_mm > SAFE_DISTANCE) return;
float angle_world_deg = b1_yaw + angle_lidar;
if (angle_world_deg >= 360.0) angle_world_deg -= 360.0;
float angle_rad = angle_world_deg * M_PI / 180.0;
float force = 1000.0 / (distance_mm + 1.0);
total_fx -= force * sin(angle_rad); total_fy -= force * cos(angle_rad);
}
