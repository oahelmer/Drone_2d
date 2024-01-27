# Drone_2d
2d dronesimulasjon regulert av pid regulatorer.
Dronen har to propeller som påfører hver sin kraft på høyere og venstre side av dronen. 
disse kreftene blir styrt av regulatorene for å regulere høyde og posisjon.
        Fl = (u_y + self.tyngde*9.81 + u_pitch)/2
        Fr = (u_y + self.tyngde*9.81 - u_pitch)/2
        self.acceleration_pitch = (Fl - Fr)/self.tyngde
        self.acceleration_y = (Fl + Fr)*np.cos(self.pitch)/self.tyngde
        self.acceleration_x = (Fl + Fr + g*self.tyngde)*np.sin(self.pitch)/self.tyngde
kreftene blir simulert av forward euler numerisk metode fordi den er enkelt implementerbar, selv om dette kan føre til mindre stabilitetsmarginer enn feks backwards euler eller RK4.

Høyde regulatoren er en vanlig pid regulator med kp, ki og kd.
posisjonsregulatoren er en kaskaderegulator av en PID for posisjon som setter referansen til en PD-regulator som kontrollerer pitchen.
PD-regulatoren tar kun referanse input i intervallet [-pitch_grense , pitch_grense] for å forhindre for stor pitch. 
PD-regulatoren er designet med hensyn på en linearisert modell av det ulinjære systemet som tar utgangspunkt i at pitchen skal være lav.
Så grensen medfører stabilitet.
