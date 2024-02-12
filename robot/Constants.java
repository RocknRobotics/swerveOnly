package frc.robot;

public final class Constants {
    //The diamatre (in metres) of the wheels for driving
    public static final double driveWheelDiameter = 0.1016d;
    
    //Update rate of the accelerometer, in Hz (range 4-200)
    public static final byte accelerometerUpdateFrequency = 50;

    //The physical max speed the robot can move in metres/second
    public static final double maxTranslationalSpeed = 3.420296449459403d;

    //The physical max speed the robot can rotate in radians/second
    public static final double maxAngularSpeed = Math.PI * 2;

    //The port the drive controller is connected to
    public static final int driveControllerPort = 0;

    //The value of the driveController below which we will treat it as a zero instead
    public static final double driveControllerStopBelowThis = 0.15;

    //Starting positions of the drive modules relative to the center of the robot in meters as [x, y]
    public static final double[] leftUpStartPos = new double[]{-0.3425d, 0.3425d};
    public static final double[] leftDownStartPos = new double[]{-0.3425d, -0.3425d};
    public static final double[] rightUpStartPos = new double[]{0.3425d, 0.3425d};
    public static final double[] rightDownStartPos = new double[]{0.3425d, -0.3425d};

    public static final class motorConstants {
        public static final class turnConstants {
            //PID controller position constant
            public static final double kP = 1.0;

            //Gear ratio between the turn motors and the module
            public static final double gearRatio = 1d / 1d;

            //The amount of radians per rotation of the turn motor (Size of "wheel" being rotated doesn't matter---One full rotation
            //of any size wheel equals 2 radians)
            public static final double radsPerRotation = gearRatio * 2 * Math.PI;
            public static final double degreesPerRotation = 360 * radsPerRotation / (2 * Math.PI);

            //IDs of turn motors
            public static final int leftUpID = 5;
            public static final int leftDownID = 6;
            public static final int rightUpID = 7;
            public static final int rightDownID = 8;

            //Invert turn wheels
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = false;

            //IDs of turn encoders
            public static final int leftUpEncoderID = 0;
            public static final int leftDownEncoderID = 1;
            public static final int rightUpEncoderID = 2;
            public static final int rightDownEncoderID = 3;

            //Encoder offsets---the values of the encoders when the module is in the "0" position (facing forward)
            public static final double leftUpOffset = 0.037448048774097;
            public static final double leftDownOffset = 0.8234566592540471;
            public static final double rightUpOffset = 0.022887491889667152;
            public static final double rightDownOffset = 0.012182849316736015;

            //Invert turn encoders
            public static final boolean leftUpEncoderInvert = false;
            public static final boolean leftDownEncoderInvert = false;
            public static final boolean rightUpEncoderInvert = false;
            public static final boolean rightDownEncoderInvert = false;
        }

        public static final class driveConstants {
            //The gear ratio between the drive motors and the drive wheel---the amount of drive wheel rotations per motor rotation
            public static final double gearRatio = 1d / 6.12d;

            //The amount of metres traveled per rotation of the drive motor (Circumference of wheel * wheel rotation per motor rotation)
            public static final double metresPerRotation = gearRatio * driveWheelDiameter * Math.PI;

            //Units in metres/second, the velocity below which the swerve module motors will stop instead of going to a desired state
            public static final double stopBelowThisVelocity = 0.015d;
            
            //The physical max speed in rotations/second of the drive motors
            public static final double maxSpeed = 5700d; //5714.28369140625

            //IDs of drive motors
            public static final int leftUpID = 1;
            public static final int leftDownID = 2;
            public static final int rightUpID = 3;
            public static final int rightDownID = 4;

            //INvert drive wheels
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = true;
            public static final boolean rightDownInvert = true;

            //The distance in metres between the left wheels and the right wheels
            public static final double leftToRightDistanceMetres = 0.685d;

            //Distance in metres between the front wheels and the back wheels
            public static final double upToDownDistanceMetres = 0.685d;

            //Wheel won't drive if the angle difference between target and current is too big
            public static final double angleTolerance = 60;
        }
    }
}
