import numpy as np
from math import sin, cos

def calculate_jacobian(theta1, theta2, theta3, theta4):
    """
    Oblicza macierz Jakobiana dla manipulatora z 4 stopniami swobody.
    
    Args:
        theta1: kąt obrotu podstawy [rad]
        theta2: kąt pierwszego stawu [rad]
        theta3: kąt drugiego stawu [rad]
        theta4: kąt trzeciego stawu (efektor) [rad]
        
    Returns:
        J: macierz Jakobiana 6x4
    """
    # Długości ogniw
    l1, l2, l3 = 120, 120, 110  # [mm] - wartości z Twojego kodu
    
    # Pozycje jointów (kinematyka prosta)
    # Joint 1: obracająca się podstwa (0, 0, 0)
    p0 = np.array([0.0, 0.0, 0.0])
    
    # Joint 2: koniec pierwszego ramienia
    x1 = 0.0
    y1 = 0.0
    z1 = l1
    p1 = np.array([x1, y1, z1])
    
    # Joint 3: koniec drugiego ramienia
    x2 = l2 * cos(theta2) * cos(theta1)
    y2 = l2 * cos(theta2) * sin(theta1)
    z2 = l1 + l2 * sin(theta2)
    p2 = np.array([x2, y2, z2])
    
    # Joint 4 (efektor): koniec trzeciego ramienia
    x3 = x2 + l3 * cos(theta2 + theta3) * cos(theta1)
    y3 = y2 + l3 * cos(theta2 + theta3) * sin(theta1)
    z3 = z2 + l3 * sin(theta2 + theta3)
    p3 = np.array([x3, y3, z3])
    
    # Efektor końcowy (z uwzględnieniem orientacji theta4)
    x_e = x3  # Tutaj możesz dodać offset jeśli efektor ma długość
    y_e = y3
    z_e = z3
    p_e = np.array([x_e, y_e, z_e])
    
    # Wektory osi obrotu dla każdego jointa (w przestrzeni światowej)
    # Dla jointa 1 (podstawa) - oś Z
    z0 = np.array([0.0, 0.0, 1.0])
    
    # Dla jointa 2 - oś Y (lub X w zależności od konfiguracji)
    # Zakładam, że joint 2 obraca się wokół osi Y (przód/tył)
    z1 = np.array([-sin(theta1), cos(theta1), 0.0])
    
    # Dla jointa 3 - oś Y
    z2 = np.array([-sin(theta1), cos(theta1), 0.0])
    
    # Dla jointa 4 (efektor) - oś Y
    z3 = np.array([-sin(theta1), cos(theta1), 0.0])
    
    # Jakobian - kolumny odpowiadają kolejnym jointom
    J = np.zeros((6, 4))
    
    # Dla jointa 1 (obrotowy)
    J[0:3, 0] = np.cross(z0, (p_e - p0))  # Część translacyjna
    J[3:6, 0] = z0                         # Część rotacyjna
    
    # Dla jointa 2 (obrotowy)
    J[0:3, 1] = np.cross(z1, (p_e - p1))  # Część translacyjna
    J[3:6, 1] = z1                         # Część rotacyjna
    
    # Dla jointa 3 (obrotowy)
    J[0:3, 2] = np.cross(z2, (p_e - p2))  # Część translacyjna
    J[3:6, 2] = z2                         # Część rotacyjna
    
    # Dla jointa 4 (obrotowy - efektor)
    J[0:3, 3] = np.cross(z3, (p_e - p3))  # Część translacyjna
    J[3:6, 3] = z3                         # Część rotacyjna
    
    return J

def calculate_manipulability(theta1, theta2, theta3, theta4):
    """
    Oblicza wskaźnik manipulowalności dla podanych kątów jointów.
    """
    try:
        J = calculate_jacobian(theta1, theta2, theta3, theta4)
        # Bierzemy tylko część translacyjną Jakobiana (pierwsze 3 wiersze)
        J_translational = J[0:3, :]
        JJT = np.dot(J_translational, J_translational.T)
        manipulability = np.sqrt(np.linalg.det(JJT))
        return max(manipulability, 1e-10)  # Zabezpieczenie przed wartościami ujemnymi/zerowymi
    except:
        return 1e-10  # Minimalna wartość w przypadku błędu
    

# manipulat = calculate_manipulability(0.3, 0.4, 1.2, 0.8)
# print(manipulat)