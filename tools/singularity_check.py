import numpy as np
from math import sin, cos, sqrt, atan2, acos, pi, atan, degrees, radians, hypot

def inverse_kinematics(x_target, y_target, z_target, orientation_mode="neutral", 
                      l1=120, l2=120, l3=110):
    """
    Pełna odwrotna kinematyka z uwzględnieniem orientacji końcówki
    
    Args:
        x_target, y_target, z_target: Pozycja docelowa końcówki
        orientation_mode: "neutral", "horizontal", "vertical_down", "vertical_up"
        l1, l2, l3: Długości ramion
    
    Returns:
        tuple: (theta1, theta2, theta3, theta4) - kąty jointów w radianach
    """
    
    delta_theta = radians(1)
    theta4_candidates = np.arange(-pi, pi, delta_theta)
    
    r_target = hypot(x_target, y_target)
    px = r_target
    py = z_target
    
    d = hypot(px, py)
    if d > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem manipulatora")
    
    theta1 = atan2(y_target, x_target)
    
    min_cost = float('inf')
    best_angles = None
    best_orientation_error = float('inf')
    tol = radians(2)
    
    for theta4_c in theta4_candidates:
        # Oblicz pozycję nadgarstka
        wrist_r = r_target - l3 * cos(theta4_c)
        wrist_z = z_target - l3 * sin(theta4_c)
        
        # Sprawdź osiągalność dla pierwszych dwóch ramion
        D = hypot(wrist_r, wrist_z)
        if D > (l1 + l2) or D < abs(l1 - l2):
            continue
        
        # Oblicz theta3
        cos_theta3 = (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if not (-1 <= cos_theta3 <= 1):
            continue
        
        theta3 = -acos(cos_theta3)  # Konfiguracja "łokieć w dół"
        
        # Oblicz theta2
        k1 = l1 + l2 * cos(theta3)
        k2 = l2 * sin(theta3)
        theta2 = atan2(wrist_z, wrist_r) - atan2(k2, k1)
        
        # Oblicz theta4 (dostosuj do orientacji)
        theta4 = theta4_c - (theta2 + theta3)
        
        # Oblicz orientację końcówki
        end_orientation = theta2 + theta3 + theta4
        
        # Funkcja kosztu w zależności od trybu orientacji
        if orientation_mode == "vertical_down":
            orientation_error = abs(end_orientation + pi/2)  # Pionowo w dół: -90°
            cost = orientation_error**2
        elif orientation_mode == "vertical_up":
            orientation_error = abs(end_orientation - pi/2)  # Pionowo w górę: +90°
            cost = orientation_error**2
        elif orientation_mode == "horizontal":
            orientation_error = abs(end_orientation)  # Poziomo: 0°
            cost = orientation_error**2
        else:  # neutral
            cost = theta3**2 + theta4**2  # Minimalizacja zgięcia
            orientation_error = 0.0
        
        if cost < min_cost:
            min_cost = cost
            best_angles = (theta1, theta2, theta3, theta4)
            best_orientation_error = orientation_error
    
    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")
    
    if orientation_mode != "neutral" and best_orientation_error > tol:
        raise ValueError(f"Orientacja '{orientation_mode}' poza zakresem ({degrees(best_orientation_error):.2f}°)")
    
    return best_angles

def check_max_reach_with_orientation(x, y, z, orientation_mode="neutral", 
                                   l1=120, l2=120, l3=110):
    """
    Sprawdza singularność maksymalnego zasięgu z pełną odwrotną kinematyką
    
    Args:
        x, y, z: Pozycja docelowa końcówki
        orientation_mode: "neutral", "horizontal", "vertical_up", "vertical_down"
        l1, l2, l3: Długości ramion
    
    Returns:
        dict: Informacje o singularności, osiągalności i konfiguracji
    """
    
    result = {
        'is_reachable': True,
        'is_singular': False,
        'risk_level': 'LOW',
        'orientation_mode': orientation_mode,
        'target_position': (x, y, z),
        'joint_angles': None,
        'wrist_position': None,
        'details': []
    }
    
    try:
        # Oblicz pełną kinematykę odwrotną
        theta1, theta2, theta3, theta4 = inverse_kinematics(x, y, z, orientation_mode, l1, l2, l3)
        result['joint_angles'] = {
            'theta1': degrees(theta1),
            'theta2': degrees(theta2),
            'theta3': degrees(theta3),
            'theta4': degrees(theta4)
        }
        
        # Oblicz pozycję nadgarstka
        wrist_x = l1 * cos(theta2) * cos(theta1) + l2 * cos(theta2 + theta3) * cos(theta1)
        wrist_y = l1 * cos(theta2) * sin(theta1) + l2 * cos(theta2 + theta3) * sin(theta1)
        wrist_z = l1 * sin(theta2) + l2 * sin(theta2 + theta3)
        
        result['wrist_position'] = (wrist_x, wrist_y, wrist_z)
        wrist_distance = hypot(wrist_x, wrist_y, wrist_z)
        
        # Sprawdź singularności
        max_reach_12 = l1 + l2
        min_reach_12 = abs(l1 - l2)
        distance_to_max = max_reach_12 - wrist_distance
        distance_to_min = wrist_distance - min_reach_12
        
        tolerance = 10.0  # mm
        
        # Singularność wyprostowania
        if distance_to_max < tolerance:
            result['is_singular'] = True
            result['risk_level'] = 'CRITICAL'
            result['details'].append(f"Ramiona prawie wyprostowane: brakuje {distance_to_max:.1f}mm do max zasięgu")
        
        # Singularność złożenia
        elif distance_to_min < tolerance:
            result['is_singular'] = True
            result['risk_level'] = 'HIGH'
            result['details'].append(f"Ramiona prawie złożone: {distance_to_min:.1f}mm zapasu")
        
        # Sprawdź kąty bliskie singularnościom
        if abs(theta3) < radians(10):  # Łokieć prawie prosty
            result['risk_level'] = 'MEDIUM'
            result['details'].append(f"Kąt theta3 bliski 0°: {degrees(theta3):.1f}°")
        
        result['details'].append(f"Rozwiązanie IK znalezione: θ1={degrees(theta1):.1f}°, θ2={degrees(theta2):.1f}°, θ3={degrees(theta3):.1f}°, θ4={degrees(theta4):.1f}°")
        
    except ValueError as e:
        result['is_reachable'] = False
        result['is_singular'] = True
        result['risk_level'] = 'IMPOSSIBLE'
        result['details'].append(str(e))
    
    return result

def compare_orientations(x, y, z, l1=120, l2=120, l3=110):
    """
    Porównuje osiągalność i konfiguracje dla różnych orientacji w tym samym punkcie
    """
    orientations = ["neutral", "horizontal", "vertical_up", "vertical_down"]
    results = {}
    
    print(f"\n=== ANALIZA PUNKTU ({x}, {y}, {z}) ===")
    print(f"Odległość od podstawy: {hypot(x, y, z):.1f}mm")
    print(f"Maksymalny teoretyczny zasięg: {l1 + l2 + l3}mm")
    
    for orientation in orientations:
        try:
            result = check_max_reach_with_orientation(x, y, z, orientation, l1, l2, l3)
            results[orientation] = result
            
            print(f"\n{orientation.upper()}:")
            print(f"  Osiągalne: {result['is_reachable']}")
            print(f"  Singularność: {result['is_singular']}")
            print(f"  Ryzyko: {result['risk_level']}")
            
            if result['joint_angles']:
                angles = result['joint_angles']
                print(f"  Kąty: θ1={angles['theta1']:.1f}°, θ2={angles['theta2']:.1f}°, θ3={angles['theta3']:.1f}°, θ4={angles['theta4']:.1f}°")
            
            for detail in result['details']:
                print(f"  - {detail}")
                
        except Exception as e:
            print(f"  BŁĄD: {e}")
            results[orientation] = None
    
    return results

# Przykład użycia
if __name__ == "__main__":
    # Test różnych punktów i orientacji
    test_points = [
        (200, 0, 100),   # Punkt w zasięgu
        (300, 0, 0),     # Daleko, ale osiągalne
        (230, 0, 0),     # Blisko granicy
        (0, 0, 250),     # Wysoko nad robotem
        (150, 150, 50),  # Punkt w przestrzeni 3D
    ]
    
    for point in test_points:
        compare_orientations(*point)
        print("\n" + "="*50)
