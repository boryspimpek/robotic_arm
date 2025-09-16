import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math

class RobotArm:
    def __init__(self):
        # Długości ramion
        self.l1 = 115  # mm
        self.l2 = 115  # mm
        self.l3 = 110  # mm
        
        # Początkowe kąty (w stopniach)
        self.theta1 = 45
        self.theta2 = -30
        self.theta3 = -15
        
        # Inicjalizacja wykresu
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.subplots_adjust(bottom=0.25)
        
        # Ustawienia wykresu
        self.ax.set_xlim(-350, 350)
        self.ax.set_ylim(-50, 350)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('3-Segmentowe Robotyczne Ramię w Płaszczyźnie XZ')
        self.ax.set_xlabel('X [mm]')
        self.ax.set_ylabel('Z [mm]')
        
        # Pozycja końcówki
        self.end_effector_x, self.end_effector_z = self.forward_kinematics()
        
        # Tekst z kątami
        self.angle_text = self.ax.text(-320, 300, '', fontsize=12, 
                                      bbox=dict(facecolor='white', alpha=0.8))
        
        # Rysuj ramię
        self.line, = self.ax.plot([], [], 'o-', linewidth=3, markersize=8, color='blue')
        
        # Przyciski
        self.create_buttons()
        
        self.update_plot()
    
    def forward_kinematics(self):
        """Oblicza pozycję końcówki na podstawie kątów"""
        theta1_rad = np.radians(self.theta1)
        theta2_rad = np.radians(self.theta2)
        theta3_rad = np.radians(self.theta3)
        
        # Pozycje stawów
        x1 = self.l1 * np.sin(theta1_rad)
        z1 = self.l1 * np.cos(theta1_rad)
        
        x2 = x1 + self.l2 * np.sin(theta1_rad + theta2_rad)
        z2 = z1 + self.l2 * np.cos(theta1_rad + theta2_rad)
        
        x3 = x2 + self.l3 * np.sin(theta1_rad + theta2_rad + theta3_rad)
        z3 = z2 + self.l3 * np.cos(theta1_rad + theta2_rad + theta3_rad)
        
        return x3, z3
    
    def update_plot(self):
        """Aktualizuje wykres"""
        # Oblicz pozycje stawów
        theta1_rad = np.radians(self.theta1)
        theta2_rad = np.radians(self.theta2)
        theta3_rad = np.radians(self.theta3)
        
        x0, z0 = 0, 0
        x1 = self.l1 * np.sin(theta1_rad)
        z1 = self.l1 * np.cos(theta1_rad)
        
        x2 = x1 + self.l2 * np.sin(theta1_rad + theta2_rad)
        z2 = z1 + self.l2 * np.cos(theta1_rad + theta2_rad)
        
        x3 = x2 + self.l3 * np.sin(theta1_rad + theta2_rad + theta3_rad)
        z3 = z2 + self.l3 * np.cos(theta1_rad + theta2_rad + theta3_rad)
        
        # Aktualizuj linię ramienia
        self.line.set_data([x0, x1, x2, x3], [z0, z1, z2, z3])
        
        # Aktualizuj tekst z kątami
        angle_info = f'Kąty:\nθ1 = {self.theta1:.1f}°\nθ2 = {self.theta2:.1f}°\nθ3 = {self.theta3:.1f}°\n'
        angle_info += f'Pozycja końcówki: ({x3:.1f}, {z3:.1f})'
        self.angle_text.set_text(angle_info)
        
        self.fig.canvas.draw_idle()
    
    def create_buttons(self):
        """Tworzy przyciski sterujące dla wszystkich kątów"""
        # Przyciski dla θ1
        ax_theta1_plus = plt.axes([0.1, 0.05, 0.08, 0.04])
        ax_theta1_minus = plt.axes([0.1, 0.10, 0.08, 0.04])
        ax_theta1_label = plt.axes([0.1, 0.15, 0.08, 0.04])
        
        self.btn_theta1_plus = Button(ax_theta1_plus, 'θ1 +')
        self.btn_theta1_minus = Button(ax_theta1_minus, 'θ1 -')
        self.btn_theta1_label = Button(ax_theta1_label, 'θ1', color='lightgray')
        self.btn_theta1_label.label.set_color('black')
        
        self.btn_theta1_plus.on_clicked(self.increase_theta1)
        self.btn_theta1_minus.on_clicked(self.decrease_theta1)
        
        # Przyciski dla θ2
        ax_theta2_plus = plt.axes([0.25, 0.05, 0.08, 0.04])
        ax_theta2_minus = plt.axes([0.25, 0.10, 0.08, 0.04])
        ax_theta2_label = plt.axes([0.25, 0.15, 0.08, 0.04])
        
        self.btn_theta2_plus = Button(ax_theta2_plus, 'θ2 +')
        self.btn_theta2_minus = Button(ax_theta2_minus, 'θ2 -')
        self.btn_theta2_label = Button(ax_theta2_label, 'θ2', color='lightgray')
        self.btn_theta2_label.label.set_color('black')
        
        self.btn_theta2_plus.on_clicked(self.increase_theta2)
        self.btn_theta2_minus.on_clicked(self.decrease_theta2)
        
        # Przyciski dla θ3
        ax_theta3_plus = plt.axes([0.4, 0.05, 0.08, 0.04])
        ax_theta3_minus = plt.axes([0.4, 0.10, 0.08, 0.04])
        ax_theta3_label = plt.axes([0.4, 0.15, 0.08, 0.04])
        
        self.btn_theta3_plus = Button(ax_theta3_plus, 'θ3 +')
        self.btn_theta3_minus = Button(ax_theta3_minus, 'θ3 -')
        self.btn_theta3_label = Button(ax_theta3_label, 'θ3', color='lightgray')
        self.btn_theta3_label.label.set_color('black')
        
        self.btn_theta3_plus.on_clicked(self.increase_theta3)
        self.btn_theta3_minus.on_clicked(self.decrease_theta3)
        
        # Przyciski resetu i dużych kroków
        ax_reset = plt.axes([0.65, 0.10, 0.1, 0.04])
        ax_big_step = plt.axes([0.65, 0.05, 0.1, 0.04])
        ax_small_step = plt.axes([0.65, 0.15, 0.1, 0.04])
        
        self.btn_reset = Button(ax_reset, 'Reset')
        self.btn_big_step = Button(ax_big_step, 'Duże kroki')
        self.btn_small_step = Button(ax_small_step, 'Małe kroki')
        
        self.btn_reset.on_clicked(self.reset_angles)
        self.btn_big_step.on_clicked(self.set_big_step)
        self.btn_small_step.on_clicked(self.set_small_step)
        
        # Domyślny krok
        self.step_size = 5
    
    def increase_theta1(self, event):
        """Zwiększa kąt theta1"""
        self.theta1 += self.step_size
        if self.theta1 > 180:
            self.theta1 = 180
        self.update_plot()
    
    def decrease_theta1(self, event):
        """Zmniejsza kąt theta1"""
        self.theta1 -= self.step_size
        if self.theta1 < -180:
            self.theta1 = -180
        self.update_plot()
    
    def increase_theta2(self, event):
        """Zwiększa kąt theta2"""
        self.theta2 += self.step_size
        if self.theta2 > 180:
            self.theta2 = 180
        self.update_plot()
    
    def decrease_theta2(self, event):
        """Zmniejsza kąt theta2"""
        self.theta2 -= self.step_size
        if self.theta2 < -180:
            self.theta2 = -180
        self.update_plot()
    
    def increase_theta3(self, event):
        """Zwiększa kąt theta3"""
        self.theta3 += self.step_size
        if self.theta3 > 180:
            self.theta3 = 180
        self.update_plot()
    
    def decrease_theta3(self, event):
        """Zmniejsza kąt theta3"""
        self.theta3 -= self.step_size
        if self.theta3 < -180:
            self.theta3 = -180
        self.update_plot()
    
    def reset_angles(self, event):
        """Resetuje kąty do pozycji początkowej"""
        self.theta1 = 45
        self.theta2 = -30
        self.theta3 = -15
        self.update_plot()
    
    def set_big_step(self, event):
        """Ustawia duże kroki (10 stopni)"""
        self.step_size = 10
        self.btn_big_step.color = 'lightblue'
        self.btn_small_step.color = '0.85'  # Domyślny kolor
    
    def set_small_step(self, event):
        """Ustawia małe kroki (1 stopień)"""
        self.step_size = 1
        self.btn_small_step.color = 'lightblue'
        self.btn_big_step.color = '0.85'  # Domyślny kolor

# Uruchom program
if __name__ == "__main__":
    robot_arm = RobotArm()
    plt.show()