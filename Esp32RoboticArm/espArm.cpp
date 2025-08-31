#include <math.h>
#include <SCServo.h>
SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// Konfiguracja manipulatora
const float L1 = 120.0;  
const float L2 = 120.0;  
const float L3 = 110.0;  
const float DELTA_THETA = 0.0174533; // 1 stopień w radianach
const float PI = 3.14159265358979323846;

// Struktura dla kątów przegubów
struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
};

// Tryby orientacji końcówki
enum OrientationMode {
    ORIENTATION_DOWN,  // Końcówka skierowana w dół
    ORIENTATION_FLAT   // Końcówka poziomo
};

// =============================================================================
// FUNKCJA 1: Pełne rozwiązanie IK z przeszukiwaniem
// =============================================================================
JointAngles solve_ik_full(float x_target, float y_target, float z_target) {
    float r_target = sqrt(x_target * x_target + y_target * y_target);
    float px = r_target;
    float py = z_target;

    float d = sqrt(px * px + py * py);
    if (d > (L1 + L2 + L3)) {
        throw "Punkt poza zasięgiem manipulatora";
    }

    float theta1 = atan2(y_target, x_target);

    float min_cost = INFINITY;
    JointAngles best_angles;
    bool solution_found = false;

    // Przeszukiwanie kandydatów dla theta4
    for (float theta4_c = -PI; theta4_c < PI; theta4_c += DELTA_THETA) {
        float wrist_r = r_target - L3 * cos(theta4_c);
        float wrist_z = z_target - L3 * sin(theta4_c);

        float D = sqrt(wrist_r * wrist_r + wrist_z * wrist_z);
        if (D > (L1 + L2) || D < abs(L1 - L2)) {
            continue;
        }

        float cos_theta3 = (wrist_r * wrist_r + wrist_z * wrist_z - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
            continue;
        }

        float theta3 = -acos(cos_theta3);  // Elbow down
        float k1 = L1 + L2 * cos(theta3);
        float k2 = L2 * sin(theta3);
        float theta2 = atan2(wrist_z, wrist_r) - atan2(k2, k1);
        float theta4 = theta4_c - (theta2 + theta3);

        // Obliczanie kosztu
        float cost = theta3 * theta3 + theta4 * theta4;

        if (cost < min_cost) {
            min_cost = cost;
            best_angles = {theta1, theta2, theta3, theta4};
            solution_found = true;
        }
    }

    if (!solution_found) {
        throw "Brak rozwiązania IK dla tej pozycji";
    }

    return best_angles;
}

// =============================================================================
// FUNKCJA 2: Szybkie rozwiązanie IK z określoną orientacją
// =============================================================================
JointAngles solve_ik_wrist(float x_target, float y_target, float z_target, OrientationMode orientation_mode) {
    float r_target = sqrt(x_target * x_target + y_target * y_target);
    float px = r_target;
    float py = z_target;

    float d = sqrt(px * px + py * py);
    if (d > (L1 + L2)) {
        throw "Punkt poza zasięgiem manipulatora";
    }

    float theta1 = atan2(y_target, x_target);

    float cos_theta3 = (r_target * r_target + z_target * z_target - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    
    if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
        throw "Brak rozwiązania IK dla tej pozycji";
    }

    float theta3 = -acos(cos_theta3);  // Elbow down
    
    float k1 = L1 + L2 * cos(theta3);
    float k2 = L2 * sin(theta3);
    float theta2 = atan2(z_target, r_target) - atan2(k2, k1);

    float theta4;
    if (orientation_mode == ORIENTATION_DOWN) {
        theta4 = -PI/2.0 - (theta2 + theta3);  // Końcówka skierowana w dół
    } else {
        theta4 = 0 - (theta2 + theta3);  // Końcówka poziomo
    }

    JointAngles angles = {theta1, theta2, theta3, theta4};
    return angles;
}

void moveServosSyncEx(uint8_t ids[4], int targetPos[4], u16 baseSpeed, u16 acc)
{
    s16 currentPos[4];
    u16 speed[4];
    s16 position[4];
    
    int delta[4];
    int maxDelta = 0;

    // 1. Odczyt aktualnych pozycji
    for (int i = 0; i < 4; i++) {
        currentPos[i] = st.ReadPos(ids[i]);
        delta[i] = abs(targetPos[i] - currentPos[i]);
        if (delta[i] > maxDelta) maxDelta = delta[i];
    }

    // 2. Oblicz prędkości proporcjonalnie
    for (int i = 0; i < 4; i++) {
        if (delta[i] == 0) speed[i] = 0;
        else speed[i] = (u16)((float)delta[i] / maxDelta * baseSpeed);
        if (speed[i] < 1) speed[i] speed[i] = 1; // minimalna prędkość
        position[i] = targetPos[i];
    }

    // 3. Wywołanie SyncWritePosEx producenta
    st.SyncWritePosEx(ids, 4, position, speed, acc);
}

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
}

