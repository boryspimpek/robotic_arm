#include <math.h>
#include <vector>

const float L1 = 120.0;  
const float L2 = 120.0;    
const float L3 = 120.0;  
const float DELTA_THETA = 0.05236;

struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
};

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
    for (float theta4_c = -M_PI; theta4_c < M_PI; theta4_c += DELTA_THETA) {
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

        float theta3 = -acos(cos_theta3);
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