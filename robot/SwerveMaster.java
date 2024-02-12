package frc.robot;

import frc.robot.Constants.motorConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;

public class SwerveMaster {
    //Variables for each swerve module
    public SwerveModule leftUpModule;
    public SwerveModule leftDownModule;
    public SwerveModule rightUpModule;
    public SwerveModule rightDownModule;

    //Accelerometer object
    public AHRS accelerometer;

    //Enables continious input
    private PIDController turnPIDController;

    //Odometry - robot's position
    private double[] robotPosition;

    //Constructor
    public SwerveMaster() {
        //Create swerve module objects
        leftUpModule = new SwerveModule(driveConstants.leftUpID, turnConstants.leftUpID, turnConstants.leftUpEncoderID, driveConstants.leftUpInvert, 
        turnConstants.leftUpInvert, turnConstants.leftUpEncoderInvert, turnConstants.leftUpOffset);
        leftDownModule = new SwerveModule(driveConstants.leftDownID, turnConstants.leftDownID, turnConstants.leftDownEncoderID, driveConstants.leftDownInvert, 
        turnConstants.leftDownInvert, turnConstants.leftDownEncoderInvert, turnConstants.leftDownOffset);
        rightUpModule = new SwerveModule(driveConstants.rightUpID, turnConstants.rightUpID, turnConstants.rightUpEncoderID, driveConstants.rightUpInvert, 
        turnConstants.rightUpInvert, turnConstants.rightUpEncoderInvert, turnConstants.rightUpOffset);
        rightDownModule = new SwerveModule(driveConstants.rightDownID, turnConstants.rightDownID, turnConstants.rightDownEncoderID, driveConstants.rightDownInvert, 
        turnConstants.rightDownInvert, turnConstants.rightDownEncoderInvert, turnConstants.rightDownOffset);

        //Set their position in relation to the center of the robot
        leftUpModule.resetPosition(-0.3425d, 0.3425d);
        leftDownModule.resetPosition(-0.3425d, -0.3425d);
        rightUpModule.resetPosition(0.3425d, 0.3425d);
        rightDownModule.resetPosition(0.3425d, -0.3425d);

        //Create accelerometer object
        accelerometer = new AHRS(Port.kMXP, Constants.accelerometerUpdateFrequency);
        accelerometer.reset();

        //Create PID Controller object
        turnPIDController = new PIDController(Constants.motorConstants.turnConstants.kP, 0d, 0);

        //Set PID values
        turnPIDController.enableContinuousInput(0, 360);

        //Create robot position array as [x, y] starting as [0, 0]
        robotPosition = new double[2];

        //Drift constant to counteract drift
        SmartDashboard.putNumber("Drift Constant", -2d);
    }

    //Method called every 20ms 
    public void update(PS4Controller controller, double driveFactor, double turnFactor) {
        //If driveFactor and turnFactor are 0, do nothing and stop the motors
        if(driveFactor == 0 && turnFactor == 0) {
            this.set(new double[]{0, 0, 0, 0}, new double[]{0, 0, 0, 0});
            return;
        }

        //Puts X an Y axis inputs of controller in an array as [leftX, leftY, rightX]
        double[] inputs = new double[]{Math.abs(controller.getLeftX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftX(), 
            Math.abs(controller.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftY(), 
            Math.abs(controller.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightX()};
        
        //Apply drive factor
        inputs[0] *= driveFactor;
        inputs[1] *= driveFactor;
        inputs[2] *= turnFactor; 

        //Call the main method for teleop
        teleopUpdate(inputs, 
        new double[]{leftUpModule.getDriveVelocity(), leftDownModule.getDriveVelocity(), rightUpModule.getDriveVelocity(), rightDownModule.getDriveVelocity()}, 
        new double[]{leftUpModule.getAbsoluteTurnPosition(), leftDownModule.getAbsoluteTurnPosition(), rightUpModule.getAbsoluteTurnPosition(), rightDownModule.getAbsoluteTurnPosition()}, 
        this.getReducedAngle(), driveFactor, turnFactor);
    }

    //Sets motor speeds given the drive and turn motor speeds
    public void set(double[] driveSets, double[] turnSets) {
        leftUpModule.set(driveSets[0], turnSets[0]);
        leftDownModule.set(driveSets[1], turnSets[1]);
        rightUpModule.set(driveSets[2], turnSets[2]);
        rightDownModule.set(driveSets[3], turnSets[3]);
    }

    public void resetAccelerometer() {
        accelerometer.reset();
    }

    public boolean accelerometerIsCalibrating() {
        return accelerometer.isCalibrating();
    }

    //Gets the angle the robot is facing from 0 - 360 degrees
    public double getReducedAngle() {
        //You've got to be joking the darn navx is cw positive when we need ccw positive readings
        double temp = -accelerometer.getAngle();
        
        //Just making sure the range is 0 - 360
        while(temp <= 0) {
            temp += 360;
        }
        while(temp > 360) {
            temp -= 360;
        }

        return temp;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getReducedAngle());
    }

    //Main method for teleop driving
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle, double driveFactor, double turnFactor) {
        //If inputs are 0, do nothing and turn off the motors
        if(inputs[0] == 0 && inputs[1] == 0 && inputs[2] == 0) {
            set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
            return;
        }

        //Smart Dashboard stuff to track variables
        SmartDashboard.putNumber("Yaw: ", accelerometer.getYaw());
        SmartDashboard.putNumber("Reduced Yaw: ", reducedAngle);

        SmartDashboard.putNumber("Current 0 Angle: ", leftUpModule.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Current 1 Angle: ", leftDownModule.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Current 2 Angle: ", rightUpModule.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Current 3 Angle: ", rightDownModule.getAbsoluteTurnPosition());
        
        //Create the arrays that will hold the drive and turn motor speeds
        double driveSets[] = new double[]{0d, 0d, 0d, 0d};
        double turnSets[] = new double[]{0d, 0d, 0d, 0d};

        //Adjust inputs to account for drift while driving and turning
        double adjustedX = inputs[0] / (Math.abs(inputs[1]) + Math.abs(inputs[2]) + 1);
        double adjustedY = -inputs[1] / (Math.abs(inputs[0]) + Math.abs(inputs[2]) + 1);
        double adjustedOmega = inputs[2] / (Math.abs(inputs[1]) + Math.abs(inputs[0]) + 1);

        SmartDashboard.putNumber("Adjusted X: ", adjustedX);
        SmartDashboard.putNumber("Adjusted Y: ", adjustedY);
        SmartDashboard.putNumber("Adjusted Omega: ", adjustedOmega);
        

        //The desired global translational vector of each wheel as described by angle and magnitude
        double[] translationalAngles = new double[4];
        double[] translationalMagnitude = new double[4];
        //Desired turning vector of each wheel
        double[] rotationalAngles =  new double[4];
        double[] rotationalMagnitude = new double[4];
        //Combined desired relative vector of each wheel as determined by the translational and turn vectors
        double[] desiredAngle = new double[4];
        double[] desiredMagnitude = new double[4];

        //The list of the desired relative angle of each motor if turning right
        double[] angleListOne = new double[]{135d, 225d, 45d, 315d};
        //Turning left
        double[] angleListTwo = new double[]{315d, 45d, 225d, 135d};

        //Iterate through each motor
        for(int i = 0; i < 4; i++) {
            //Calculating the angle of the global desired angle of the translational vector
            translationalAngles[i] = Math.atan2(adjustedY, adjustedX) + Math.PI * 2 - Math.PI / 2;
            translationalAngles[i] -= (1d / (0.75 + turnFactor)) * (0.75 + driveFactor) * adjustedOmega * SmartDashboard.getNumber("Drift Constant", -2d);

            //Making sure range is 0 to 2pi
            while(translationalAngles[i] > Math.PI * 2) {
                translationalAngles[i] -= Math.PI * 2;
            }
            while(translationalAngles[i] <= 0) {
                translationalAngles[i] += Math.PI * 2;
            }

            //Converting to degrees (0 to 360)
            translationalAngles[i] *= 180 / Math.PI;

            //Subtracting the gyro angle to make the angle field relative
            //Don't need to check range since cos and sin functions don't care about that
            translationalAngles[i] -= reducedAngle;

            //Calculate the magnitude of the wheel's desired translation using Pythagorean Theorem
            translationalMagnitude[i] = Math.sqrt(Math.pow(adjustedX, 2) + Math.pow(adjustedY, 2));

            //Set the rotational magnitude to the adjustedOmega
            rotationalMagnitude[i] = Math.abs(adjustedOmega);

            //Setting the rotational angle based on whether turning left/right
            //If no rotation, then the magnitude is zero, so it doesn't matter what the angle is
            if(adjustedOmega < 0) {
                rotationalAngles[i] = angleListOne[i]; //Right turn
            } else {
                rotationalAngles[i] = angleListTwo[i]; //Left turn
            }

            //Formula for converting from two vectors with a given magnitude and angle
            // into an x and y value
            double tempX = translationalMagnitude[i] * Math.cos(translationalAngles[i] * Math.PI / 180) 
                + rotationalMagnitude[i] * Math.cos(rotationalAngles[i] * Math.PI / 180);
            double tempY = translationalMagnitude[i] * Math.sin(translationalAngles[i] * Math.PI / 180) 
                + rotationalMagnitude[i] * Math.sin(rotationalAngles[i] * Math.PI / 180);

            //Need to check if no translational or rotational input since that would make x/y 0,
            // and then atan2 might have undefined behaviour
            if(adjustedX == 0 && adjustedY == 0) {
                tempX = rotationalMagnitude[i] * Math.cos(rotationalAngles[i] * Math.PI / 180);
                tempY = rotationalMagnitude[i] * Math.sin(rotationalAngles[i] * Math.PI / 180);
            } else if(adjustedOmega == 0) {
                tempX = translationalMagnitude[i] * Math.cos(translationalAngles[i] * Math.PI / 180);
                tempY = translationalMagnitude[i] * Math.sin(translationalAngles[i] * Math.PI / 180);
            }

            //Getting the relative desired angle of the vector for this wheel
            desiredAngle[i] = Math.atan2(tempX, tempY) * 180 / Math.PI;

            //Ensures it's in the right range for the purposes of setting the turn motor
            if(desiredAngle[i] <= 0) {
                desiredAngle[i] += 360;
            }

            //Rotate right 90 degrees to align correctly
            desiredAngle[i] -= 90;
            if(desiredAngle[i] <= 0) {
                desiredAngle[i] += 360;
            }

            //Getting the magnitude via the pythagorean theorem
            desiredMagnitude[i] = Math.sqrt(Math.pow(tempX, 2) + Math.pow(tempY, 2));

            //Optimization - invert drive of wheel rather than turn a full 180 degrees
            int driveSetInvert = 1;

            //turnPIDController takes into account the wrap from 0 to 360, so using it when calculating whether
            // it's optimal to add/subtract 180 for reversing actually results in it at most turning 90 
            if(Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) >= Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] + 180))) {
                driveSetInvert = -1;
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i] + 180) / 360d;

                if (Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] + 180)) > driveConstants.angleTolerance) {
                    driveSetInvert = 0;
                }
            } else if(Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) >= Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] - 180))) {
                driveSetInvert = -1;
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i] - 180) / 360d;

                if (Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] - 180)) > driveConstants.angleTolerance) {
                    driveSetInvert = 0;
                }
            } else {
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i]) / 360d;

                if (Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) > driveConstants.angleTolerance) {
                    driveSetInvert = 0;
                }
            }
            //Invert wheel if optimized
            driveSets[i] = driveSetInvert * -desiredMagnitude[i];
            
            
    
            //Debugging purposes
            SmartDashboard.putNumber("Trans Ang " + i + ": ", translationalAngles[i]);
            SmartDashboard.putNumber("Trans Mag " + i + ": ", translationalMagnitude[i]);
            SmartDashboard.putNumber("Rot Ang " + i + ": ", rotationalAngles[i]);
            SmartDashboard.putNumber("Rot Mag " + i + ": ", rotationalMagnitude[i]);
            SmartDashboard.putNumber("Combined X " + i + ": ", tempX);
            SmartDashboard.putNumber("Combined Y " + i + ": ", tempY);
            SmartDashboard.putNumber("Desired Ang " + i + ": ", desiredAngle[i]);
            SmartDashboard.putNumber("Desired Mag " + i + ": ", desiredMagnitude[i]);
            SmartDashboard.putNumber("Turn Set " + i + ": ", turnSets[i]);
            SmartDashboard.putNumber("Drive Set " + i + ": ", driveSets[i]);
        }

        //Odometry - Update the position of the robot using the angle the robot is facing
        // and the velocity of the wheels
        updateRobotPosition(reducedAngle);

        //Odometry - Checking positions of wheels and center of robot
        SmartDashboard.putNumber("Robot X", robotPosition[0]);
        SmartDashboard.putNumber("Robot Y", robotPosition[1]);
        SmartDashboard.putNumber("M0 X", leftUpModule.getModulePosition()[0]);
        SmartDashboard.putNumber("M0 Y", leftUpModule.getModulePosition()[1]);
        SmartDashboard.putNumber("M1 X", leftDownModule.getModulePosition()[0]);
        SmartDashboard.putNumber("M1 Y", leftDownModule.getModulePosition()[1]);
        SmartDashboard.putNumber("M2 X", rightUpModule.getModulePosition()[0]);
        SmartDashboard.putNumber("M2 Y", rightUpModule.getModulePosition()[1]);
        SmartDashboard.putNumber("M3 X", rightDownModule.getModulePosition()[0]);
        SmartDashboard.putNumber("M3 Y", rightDownModule.getModulePosition()[1]);

        //Set the wheel speeds
        set(driveSets, turnSets);

        //Uncomment to check values without moving robot
        //set(new double[]{0, 0, 0, 0}, new double[]{0, 0, 0, 0});
    }

    //Odometry - Reset origin with new origin at center of robot
    //Also needs gyro to be reset
    public void resetOrigin() {
        resetRobotPosition(0, 0);
        resetAccelerometer();
        leftUpModule.resetPosition(-0.3425d, 0.3425d);
        leftDownModule.resetPosition(-0.3425d, -0.3425d);
        rightUpModule.resetPosition(0.3425d, 0.3425d);
        rightDownModule.resetPosition(0.3425d, -0.3425d);
    }

    //Odometry - Set robot position with x and y in meters from origin
    public void resetRobotPosition(double x, double y) {
        robotPosition[0] = x;
        robotPosition[1] = y;
        alignModules(getReducedAngle());
    }

    //Odometry - Get the robots position x and y in meters relative to origin
    public double[] getRobotPosition() {
        return robotPosition;
    }

    //Odometry - Update robot position. 
    //Should be called as often as possible for less error
    public void updateRobotPosition(double reducedAngle) {
        double[] leftUpPos = leftUpModule.getPosition(reducedAngle);
        double[] leftDownPos = leftDownModule.getPosition(reducedAngle);
        double[] rightUpPos = rightUpModule.getPosition(reducedAngle);
        double[] rightDownPos = rightDownModule.getPosition(reducedAngle);

        //Calculate 2 centers of robot using mid-point formula
        double[] center1 = {(leftUpPos[0] + rightDownPos[0]) / 2, (leftUpPos[1] + rightDownPos[1]) / 2};
        double[] center2 = {(leftDownPos[0] + rightUpPos[0]) / 2, (leftDownPos[1] + rightUpPos[1]) / 2};

        //Calculate mid-point of the two centers to get "true center" (supposedly)
        robotPosition[0] = (center1[0] + center2[0]) / 2;
        robotPosition[1] = (center1[1] + center2[1]) / 2;

        //Reset position of wheels with new center
        //Have to account for which direction the robot is facing
        alignModules(reducedAngle);
    }

    //Odometry - Reset position of wheels with new center
    public void alignModules(double reducedAngle) {
        leftUpModule.alignPosition(robotPosition, reducedAngle, Constants.leftUpStartPos);
        leftDownModule.alignPosition(robotPosition, reducedAngle, Constants.leftDownStartPos);
        rightUpModule.alignPosition(robotPosition, reducedAngle, Constants.rightUpStartPos);
        rightDownModule.alignPosition(robotPosition, reducedAngle, Constants.rightDownStartPos);
    }
}