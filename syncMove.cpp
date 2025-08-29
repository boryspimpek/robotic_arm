#include <SCServo.h>

SMS_STS sm;  // sterownik SCServo (dla STS, jeśli masz SC to użyj SMS_SC)


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
