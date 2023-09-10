// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public final class Constants {

    public static final class DriveTrain {
        public static final class MotorIDs {
            public static final int frontLeft = 2;
            public static final int frontRight = 3;
            public static final int rearLeft = 4;
            public static final int rearRight = 5;

        }

        public static final class PhysicalDimensions {
            public static final Translation2d wheelPosFrontLeft = new Translation2d(-0.75, 1.25);
            public static final Translation2d wheelPosFrontRight = new Translation2d(0.75, 1.25);
            public static final Translation2d wheelPosRearLeft = new Translation2d(-0.75, -1.25);
            public static final Translation2d wheelPosRearRight = new Translation2d(0.75, -1.25);
        }
    }


}
