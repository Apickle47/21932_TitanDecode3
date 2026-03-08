package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShotSolution {
    public final double rpm;
    public final double hoodPosition;

    public ShotSolution(double rpm, double hoodPosition) {
        this.rpm = rpm;
        this.hoodPosition = hoodPosition;
    }

    public double getRpm() {
        return rpm;
    }
    public double getHoodP() {
        return hoodPosition;
    }
}
