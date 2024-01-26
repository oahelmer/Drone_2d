import numpy as np
from scipy.integrate import odeint
import pygame
import os

verdi = 1 #er høyden til dronen
referanse = 0.5 #ønsket høyde til drone
verdi_x = 1 # er x posisjon
referanse_x = 1.5

t = 0
FPS = 60

kp = 100
ki = 10
kd = 60

skalar = 1
kp_pitch = 1 * skalar
ki_pitch = 0 * skalar
kd_pitch =  (4*kp_pitch+1/40) * skalar

h = 1/FPS
g = 9.81

HEIGHT, WIDTH = 1000, 1800
WHITE = (255, 255, 255)
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("DRONE")

def runge_kutta4(y_n, t_n, h, f):
    # Konstanter
    k1 = f(t_n, y_n)
    k2 = f(t_n + h/2, y_n + (h/2)*k1)
    k3 = f(t_n + h/2, y_n + (h/2)*k2)
    k4 = f(t_n + h,   y_n + h * k3)
    
    # Regn ut og returner neste verdi
    return y_n + (h/6)*(k1 + 2*k2 + 2*k3 + k4)



class PID:
    def __init__(self, kp, ki, kd) -> None:
        self.e = 0
        self.e_prev = 0
        self.integral = 0
        self.u = 0

        # Verdier brukt i funksjonen
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def pid(self, time, y_ref, y):
        # Kontroll på e_prev
        self.e_prev = self.e

        # Kontroll på p ledd
        self.e = y_ref - y  # Error in position

        # Kontroll på integral ledd
        self.integral += (self.e - self.e_prev)*h/2 + self.e_prev

        if self.integral <= -50:
            self.integral = -50
        elif self.integral >= 50:
            self.integral = 50

        # Kontroll på derivert
        e_derivert = (self.e - self.e_prev)/h

        # Setter pådraget med hensyn på PID leddene
        self.u = self.kp * self.e + self.ki * self.integral + self.kd * e_derivert



class System:
    def __init__(self, initial_y, initial_x, initial_velocity_y, initial_velocity_x, tyngde) -> None:
        self.y = initial_y
        self.x = initial_x
        self.velocity_y = initial_velocity_y
        self.velocity_x = initial_velocity_x
        self.acceleration_y = 0
        self.acceleration_x = 0
        self.pitch = 0
        self.velocity_pitch = 0
        self.acceleration_pitch = 0
        self.tyngde = tyngde

        self.IMAGE = pygame.image.load(os.path.join('Images', 'drone_image_fixed.xcf'))
 

    def update(self, u_y, u_pitch, h):
        Fl = (u_y + self.tyngde*9.81 + u_pitch)/2
        Fr = (u_y + self.tyngde*9.81 - u_pitch)/2

        self.acceleration_pitch = (Fl - Fr)/self.tyngde
        self.acceleration_y = (Fl + Fr)*np.cos(self.pitch)/self.tyngde
        self.acceleration_x = (Fl + Fr)*np.sin(self.pitch)/self.tyngde


#_________FORWARD_EULER_____________________________________

        self.velocity_pitch += self.acceleration_pitch * h
        self.pitch += self.velocity_pitch * h

        self.velocity_x += self.acceleration_x * h
        self.x += self.velocity_x * h

        self.velocity_y += self.acceleration_y * h
        self.y += self.velocity_y * h

#        self.IMAGE = pygame.transform.rotate(self.IMAGE, self.velocity_pitch*h)




def draw_window(Player):
    WIN.fill(WHITE)
    WIN.blit(Player.IMAGE, (Player.x * 500, Player.y * 500))
    pygame.display.update()


def main():
    global verdi, t, tyngde

    run = True
    clock = pygame.time.Clock() 

    pid = PID(kp, ki, kd)
    pid2 = PID(kp_pitch,ki_pitch,kd_pitch)
    system = System(initial_y=verdi,initial_x= verdi_x, initial_velocity_y=0, initial_velocity_x=0, tyngde=2)


    while(run):
        clock.tick(FPS)   # Controls Frames per seconds
        
        pid.pid(t, referanse, system.y)
        pid2.pid(t, referanse_x, system.x)

        system.update(pid.u, pid2.u, h)
        print("y:", system.y)
        print("x:", system.x)
        print("pitch:", system.pitch)



        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        draw_window(system)
        
        t = t + h
        
    pygame.quit()

if __name__ == "__main__":
    main()