package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTable {

    private static final ShotPoint[] SHOT_TABLE = {
            new ShotPoint(37.923,1210, 1),
            new ShotPoint(59.83,1250, 0.8),
            new ShotPoint(71.06, 1330, 0.75),
            new ShotPoint(80.57, 1380, 0.7),
            new ShotPoint(91.65, 1460, 0.65),
            new ShotPoint(103.30, 1550, 0.6),
            new ShotPoint(113.61, 1640, 0.575),
            new ShotPoint(117.13, 1680, 0.55),
            new ShotPoint(132.19, 1800, 0.55),
            new ShotPoint(141.63, 1845, 0.55),
            new ShotPoint(150.0, 1905, 0.5),
            new ShotPoint(170.80, 2020, 0.5),

            /*
            new ShotPoint(55.425, 1250, 0.75),
            new ShotPoint(64.39, 1300, 0.65),
            new ShotPoint(70.36, 1270, 0.7),
            new ShotPoint(90.1, 1490, 0.6),
            new ShotPoint(101.81, 1550, 0.55),
            new ShotPoint(101.81, 1550, 0.55),
             */


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
