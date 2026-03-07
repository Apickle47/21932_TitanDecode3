package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShotPoint {
    public final double distance;
    public final double rpm;
    public final double hoodPosition;
    public final double time;

    public ShotPoint(double distance, double rpm, double hoodPosition) {
        this.distance = distance;
        this.rpm = rpm;
        this.hoodPosition = hoodPosition;
        this.time = 0;
    }
    public ShotPoint(double distance, double rpm, double hoodPosition, double time) {
        this.distance = distance;
        this.rpm = rpm;
        this.hoodPosition = hoodPosition;
        this.time = time;
    }
}