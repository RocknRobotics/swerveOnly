package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    //The diamatre (in metres) of the wheels for driving
    public static final double driveWheelDiameter = 0.1016d;
    
    //Update rate of the accelerometer, in Hz (range 4-200)
    public static final byte accelerometerUpdateFrequency = 50;

    //The physical max speed the robot can move in metres/second
    public static final double maxTranslationalSpeed = 1d; //TODO
    //The physical max speed the robot can rotate in radians/second
    public static final double maxAngularSpeed = 1d; //TODO

    //The port the drive controller is connected to
    public static final int driveControllerPort = 0;

    //The value of the driveController below which we will treat it as a zero instead
    public static final double driveControllerStopBelowThis = 0.1;

    public static final class motorConstants {
        public static final class turnConstants {
            //PID controller position constant
            public static final double kP = 0.5; //Change this?
            //Gear ratio between the turn motors and the module?
            //New info: between encoder and output, is maybe one since encoder is on output shaft?
            public static final double gearRatio = 1d / 1d;
            //The amount of radians per rotation of the turn motor (Size of "wheel" being rotated doesn't matter---One full rotation
            //of any size wheel equals 2 radians)
            public static final double radsPerRotation = gearRatio * 2 * Math.PI;

            public static final int leftUpID = 5;
            public static final int leftDownID = 6;
            public static final int rightUpID = 7;
            public static final int rightDownID = 8;

            //TODO
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = false; //Done

            public static final int leftUpEncoderID = 0;
            public static final int leftDownEncoderID = 1;
            public static final int rightUpEncoderID = 2;
            public static final int rightDownEncoderID = 3;

            //TODO
            //Encoder offsets---the values of the encoders when the module is in the "0" position (facing forward or whatever)
            public static final double leftUpOffset = 0.0;
            public static final double leftDownOffset = 0.0;
            public static final double rightUpOffset = 0.0;
            public static final double rightDownOffset = 0.0;

            //TODO
            public static final boolean leftUpEncoderInvert = true;
            public static final boolean leftDownEncoderInvert = true;
            public static final boolean rightUpEncoderInvert = true;
            public static final boolean rightDownEncoderInvert = true;
        }

        public static final class driveConstants {
            //The gear ratio between the drive motors and the drive wheel---the amount of drive wheel rotations per motor rotation
            public static final double gearRatio = 1d / 6.12d;
            //The amount of metres traveled per rotation of the drive motor (Circumference of wheel * wheel rotation per motor rotation)
            public static final double metresPerRotation = gearRatio * driveWheelDiameter * Math.PI;
            //Units in metres/second, the velocity below which the swerve module motors will stop instead of going to a desired state
            public static final double stopBelowThisVelocity = 0.001d; //Make it higher? Or is it good?
            //The physical max speed in rotations/second of the drive motors
            public static final double maxSpeed = 5700d; //5714.28369140625

            public static final int leftUpID = 1;
            public static final int leftDownID = 2;
            public static final int rightUpID = 3;
            public static final int rightDownID = 4;

            //TODO
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = true; //Done

            //The distance in metres between the left wheels and the right wheels
            public static final double leftToRightDistanceMetres = 0.685;
            //Distance in metres between the "up" wheels and the "down" wheels
            public static final double upToDownDistanceMetres = 0.685;

            //Order--- leftUp, leftDown, rightUp, rightDown
            public static final SwerveDriveKinematics drivemotorKinematics = new SwerveDriveKinematics(
                new Translation2d(upToDownDistanceMetres / 2d, -leftToRightDistanceMetres / 2d), 
                new Translation2d(-upToDownDistanceMetres / 2d, -leftToRightDistanceMetres / 2d), 
                new Translation2d(upToDownDistanceMetres / 2d, leftToRightDistanceMetres / 2d), 
                new Translation2d(-upToDownDistanceMetres / 2d, leftToRightDistanceMetres / 2d));
        }
    }
}
