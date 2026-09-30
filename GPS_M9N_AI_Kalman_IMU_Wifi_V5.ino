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
#include <SPI.h>
#include "MPU9250.h"

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
    gpsSerial.begin(38400);//gps 115200
    gpsSerial.print("$PUBX,41,1,0003,0001,115200,0*1E\r\n");
    gpsSerial.begin(115200);
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
