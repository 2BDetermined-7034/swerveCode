/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve {

        public static final double length = 0.6096;
        public static final double width = 0.762;

        public static final int frontLeftDrive = 7;
        public static final int frontRightDrive = 1;
        public static final int backLeftDrive = 5;
        public static final int backRightDrive = 3;

        public static final int frontLeftSpin = 8;
        public static final int frontRightSpin = 2;
        public static final int backLeftSpin = 6;
        public static final int backRightSpin = 4;

        //170 RPM at max speed
        // 0 - 1 range
        public static final double maxSpeed = 0.3;

        //which direction (degrees) a module has to rotate to spin
        public static final double frSpin = -127.65;
        public static final double brSpin = 127.65;
        public static final double blSpin = 52.35;
        public static final double flSpin = -52.35;

    }

}
