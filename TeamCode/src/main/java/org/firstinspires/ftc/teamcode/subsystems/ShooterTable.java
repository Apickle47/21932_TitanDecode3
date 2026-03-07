package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTable {

    private static final ShotPoint[] SHOT_TABLE = {
            new ShotPoint(24.0, 2200, 0.18),
            new ShotPoint(30.0, 2325, 0.22),
            new ShotPoint(36.0, 2450, 0.27),
            new ShotPoint(42.0, 2580, 0.31),
            new ShotPoint(48.0, 2720, 0.36)
    };

    public static ShotSolution getShotSolution(double distance) {
        // Clamp below minimum
        if (distance <= SHOT_TABLE[0].distance) {
            return new ShotSolution(SHOT_TABLE[0].rpm, SHOT_TABLE[0].hoodPosition);
        }

        // Clamp above maximum
        if (distance >= SHOT_TABLE[SHOT_TABLE.length - 1].distance) {
            ShotPoint last = SHOT_TABLE[SHOT_TABLE.length - 1];
            return new ShotSolution(last.rpm, last.hoodPosition);
        }

        // Find the two points the distance is between
        for (int i = 0; i < SHOT_TABLE.length - 1; i++) {
            ShotPoint p1 = SHOT_TABLE[i];
            ShotPoint p2 = SHOT_TABLE[i + 1];

            if (distance >= p1.distance && distance <= p2.distance) {
                double t = (distance - p1.distance) / (p2.distance - p1.distance);

                double rpm = p1.rpm + t * (p2.rpm - p1.rpm);
                double hood = p1.hoodPosition + t * (p2.hoodPosition - p1.hoodPosition);

                return new ShotSolution(rpm, hood);
            }
        }

        // Should never happen if table is valid
        ShotPoint last = SHOT_TABLE[SHOT_TABLE.length - 1];
        return new ShotSolution(last.rpm, last.hoodPosition);
    }
}
