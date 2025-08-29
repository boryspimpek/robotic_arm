#include <math.h>

const float L1 = 120.0;  
const float L2 = 120.0;    
const float PI = 3.14159265358979323846;

struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
};

enum OrientationMode {
    ORIENTATION_DOWN,
    ORIENTATION_FLAT
};

JointAngles solve_ik_wrist(float x_target, float y_target, float z_target, OrientationMode orientation_mode) {
    float l1 = L1;
    float l2 = L2;
    // l3 jest ignorowane w tej funkcji

    float r_target = sqrt(x_target * x_target + y_target * y_target);
    float px = r_target;
    float py = z_target;

    float d = sqrt(px * px + py * py);
    if (d > (l1 + l2)) {
        throw "Punkt poza zasięgiem manipulatora";
    }

    float theta1 = atan2(y_target, x_target);

    float cos_theta3 = (r_target * r_target + z_target * z_target - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    
    if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
        throw "Brak rozwiązania IK dla tej pozycji";
    }

    float theta3 = -acos(cos_theta3);  // Elbow down
    
    float k1 = l1 + l2 * cos(theta3);
    float k2 = l2 * sin(theta3);
    float theta2 = atan2(z_target, r_target) - atan2(k2, k1);

    float theta4;
    if (orientation_mode == ORIENTATION_DOWN) {
        theta4 = -PI/2.0 - (theta2 + theta3);  // Końcówka skierowana w dół
    } else if (orientation_mode == ORIENTATION_FLAT) {
        theta4 = 0 - (theta2 + theta3);  // Końcówka poziomo
    } else {
        theta4 = 0;  // Domyślnie
    }

    JointAngles angles = {theta1, theta2, theta3, theta4};
    return angles;
}