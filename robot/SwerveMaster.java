package frc.robot;

import frc.robot.Constants.motorConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;

public class SwerveMaster {
    public SwerveModule leftUpModule;
    public SwerveModule leftDownModule;
    public SwerveModule rightUpModule;
    public SwerveModule rightDownModule;

    public AHRS accelerometer;

    //Enables continious input I think? I'm writing this using another person's code as a guide---I'll mess around with changing
    //this once we have the swerve drive built
    private PIDController turnPIDController;
    private PIDController turnSetController;

    private double currentX;
    private double currentY;
    private double currentHeading;

    private double desiredX;
    private double desiredY;
    private double desiredHeading;

    public SwerveMaster() {
        leftUpModule = new SwerveModule(driveConstants.leftUpID, turnConstants.leftUpID, turnConstants.leftUpEncoderID, driveConstants.leftUpInvert, 
        turnConstants.leftUpInvert, turnConstants.leftUpEncoderInvert, turnConstants.leftUpOffset);
        leftDownModule = new SwerveModule(driveConstants.leftDownID, turnConstants.leftDownID, turnConstants.leftDownEncoderID, driveConstants.leftDownInvert, 
        turnConstants.leftDownInvert, turnConstants.leftDownEncoderInvert, turnConstants.leftDownOffset);
        rightUpModule = new SwerveModule(driveConstants.rightUpID, turnConstants.rightUpID, turnConstants.rightUpEncoderID, driveConstants.rightUpInvert, 
        turnConstants.rightUpInvert, turnConstants.rightUpEncoderInvert, turnConstants.rightUpOffset);
        rightDownModule = new SwerveModule(driveConstants.rightDownID, turnConstants.rightDownID, turnConstants.rightDownEncoderID, driveConstants.rightDownInvert, 
        turnConstants.rightDownInvert, turnConstants.rightDownEncoderInvert, turnConstants.rightDownOffset);

        accelerometer = new AHRS(Port.kMXP, Constants.accelerometerUpdateFrequency);
        accelerometer.reset();

        turnPIDController = new PIDController(Constants.motorConstants.turnConstants.kP, 0d, 0);
        //NEW, changed max/min input and changed kp to 0.8 (it's what worked when testing---probably don't change it for now)
        turnPIDController.enableContinuousInput(0, 360);
        turnSetController = new PIDController(0.8, 0.0, 0.0);

        currentX = 0d;
        currentY = 0d;
        currentHeading = 0d;

        desiredX = 0d;
        desiredY = 0d;
        desiredHeading = 0d;
    }

    public void update(PS4Controller controller, double factor) {
        if(factor == 0) {
            this.set(new double[]{0, 0, 0, 0}, new double[]{0, 0, 0, 0});
            return;
        }

        //NEW, ternaries before calculating factor allows lower speeds when the drive factor is low
        teleopUpdate(new double[]{Math.abs(controller.getLeftX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftX() * factor, 
            Math.abs(controller.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftY() * factor, 
            Math.abs(controller.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightX() * factor}, 
        new double[]{leftUpModule.getDriveVelocity(), leftDownModule.getDriveVelocity(), rightUpModule.getDriveVelocity(), rightDownModule.getDriveVelocity()}, 
        new double[]{leftUpModule.getAbsoluteTurnPosition(), leftDownModule.getAbsoluteTurnPosition(), rightUpModule.getAbsoluteTurnPosition(), rightDownModule.getAbsoluteTurnPosition()}, 
        this.getReducedAngle());
    }

    public void set(double[] driveSets, double[] turnSets) {
        leftUpModule.set(driveSets[0], turnSets[0]);
        leftDownModule.set(driveSets[1], turnSets[1]);
        rightUpModule.set(driveSets[2], turnSets[2]);
        rightDownModule.set(driveSets[3], turnSets[3]);
    }

    public void resetAccelerometer() {
        accelerometer.reset();

        currentX = 0d;
        currentY = 0d;
        currentHeading = 0d;

        desiredX = 0d;
        desiredY = 0d;
        desiredHeading = 0d;
    }

    public boolean accelerometerIsCalibrating() {
        return accelerometer.isCalibrating();
    }

    public double getReducedAngle() {
        //You've got to be joking the darn navx is cw positive when we need ccw positive readings
        double temp = -accelerometer.getAngle();
        
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

    //Does the heavy lifting
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle) {
        //NEW
        SmartDashboard.putNumber("Yaw: ", accelerometer.getYaw());
        SmartDashboard.putNumber("Reduced Yaw: ", reducedAngle);

        turnSetController.setP(SmartDashboard.getNumber("kP: ", turnSetController.getP()));
        turnSetController.setI(SmartDashboard.getNumber("kI: ", turnSetController.getI()));
        turnSetController.setD(SmartDashboard.getNumber("kD: ", turnSetController.getD()));
        turnSetController.setTolerance(SmartDashboard.getNumber("Position Tolerance: ", turnSetController.getPositionTolerance()), SmartDashboard.getNumber("Velocity Tolerance: ", turnSetController.getVelocityTolerance()));

        SmartDashboard.putNumber("Current 0 Angle: ", leftUpModule.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Current 1 Angle: ", leftDownModule.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Current 2 Angle: ", rightUpModule.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Current 3 Angle: ", rightDownModule.getAbsoluteTurnPosition());

        //NEW, deleted this for loop. Needs to be put back if you add an i component to the turnSetController (ensures integral doesn't accumulate forever)
        
        //Arrays to be published later
        double driveSets[] = new double[]{0d, 0d, 0d, 0d};
        double turnSets[] = new double[]{0d, 0d, 0d, 0d};
        //Converts PS4 joystick inputs (field relative) into robot-relative speeds
        //First is x m/s where fwd is positive
        //Second is y m/s where left is positive
        //Third is rad/s where counter clockwise is positive
        //Fourth is robot angle
        //Input[0] == left/right, input[1] == up/down, input[2] == left/right (turn)
        //So first == input[1], second == -input[0], and third == -input[2]?
        /*double controllerAngle = Math.atan2(-inputs[1], inputs[0]) - Math.PI / 2;
        if(controllerAngle < 0) {
            controllerAngle += Math.PI * 2;
        }
        double angleDifference = controllerAngle - this.getReducedAngle() * Math.PI / 180;
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds((this.getReducedAngle() < 180 ? 1 : -1) * (Math.cos(angleDifference) < 0 ? -1 : 1) * Math.abs(inputs[1]) * Constants.maxTranslationalSpeed, 
        (this.getReducedAngle() < 90 || this.getReducedAngle() > 270 ? -1 : 1) * (Math.sin(angleDifference) < 0 ? -1 : 1) * Math.abs(inputs[0]) * Constants.maxTranslationalSpeed, inputs[2] * Constants.maxAngularSpeed, getRotation2d());*/
        double desiredVx = -inputs[1] * Constants.maxTranslationalSpeed * Robot.driveControllerFactor;
        double desiredVy = inputs[0] * Constants.maxTranslationalSpeed * Robot.driveControllerFactor;
        double desiredOmega = -inputs[2] * Constants.maxAngularSpeed * Robot.driveControllerFactor;
        double[] translationalAngles = new double[4];
        double[] translationalMagnitude = new double[4];
        double[] rotationalAngles =  new double[4];
        double[] rotationalMagnitude = new double[4];
        double[] desiredAngle = new double[4];
        double[] desiredMagnitude = new double[4];

        if(inputs[0] == 0 && inputs[1] == 0 && inputs[2] == 0) {
            set(driveSets, turnSets);
            return;
        }

        SwerveModule[] moduleList = new SwerveModule[]{leftUpModule, leftDownModule, rightUpModule, rightDownModule};
        double[] angleListOne = new double[]{135d, 225d, 45d, 315d};
        double[] angleListTwo = new double[]{315d, 45d, 225d, 135d};

        for(int i = 0; i < 4; i++) {
            translationalAngles[i] = Math.atan2(-inputs[0], -inputs[1]) + Math.PI * 2;
            if(translationalAngles[i] > Math.PI * 2) {
                translationalAngles[i] -= Math.PI * 2;
            }
            translationalAngles[i] *= 180 / Math.PI;
            translationalAngles[i] -= reducedAngle;
            translationalMagnitude[i] = Constants.maxTranslationalSpeed * Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2)) / Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2) + Math.pow(inputs[2], 2));
            rotationalMagnitude[i] = Constants.motorConstants.driveConstants.leftToRightDistanceMetres * Math.PI * Math.abs(inputs[2]) / Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2) + Math.pow(inputs[2], 2));
            if(inputs[2] < 0) {
                rotationalAngles[i] = angleListOne[i];
            } else {
                rotationalAngles[i] = angleListTwo[i];
            }

            double tempX = translationalMagnitude[i] * Math.cos(translationalAngles[i] * Math.PI / 180) 
                + rotationalMagnitude[i] * Math.cos(rotationalAngles[i] * Math.PI / 180);
            double tempY = translationalMagnitude[i] * Math.sin(translationalAngles[i] * Math.PI / 180) 
                + rotationalMagnitude[i] * Math.sin(rotationalAngles[i] * Math.PI / 180);

            if(inputs[0] == 0 && inputs[1] == 0) {
                tempX = rotationalMagnitude[i] * Math.cos(rotationalAngles[i] * Math.PI / 180);
                tempY = rotationalMagnitude[i] * Math.sin(rotationalAngles[i] * Math.PI / 180);
            } else if(inputs[2] == 0) {
                tempX = translationalMagnitude[i] * Math.cos(translationalAngles[i] * Math.PI / 180);
                tempY = translationalMagnitude[i] * Math.sin(translationalAngles[i] * Math.PI / 180);
            }

            desiredAngle[i] = Math.atan2(tempX, tempY) * 180 / Math.PI;
            if(desiredAngle[i] <= 0) {
                desiredAngle[i] += 360;
            }
            desiredAngle[i] -= 90;
            if(desiredAngle[i] <= 0) {
                desiredAngle[i] += 360;
            }
            desiredMagnitude[i] = Math.sqrt(Math.pow(tempX, 2) + Math.pow(tempY, 2));

            int driveSetInvert = 1;

            if(Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) >= Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] + 180))) {
                driveSetInvert = -1;
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i] + 180) / 360d;
            } else if(Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) >= Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] - 180))) {
                driveSetInvert = -1;
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i] - 180) / 360d;
            } else {
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i]) / 360d;
            }

            driveSets[i] = driveSetInvert * -desiredMagnitude[i] * (Math.min(1, Math.abs(inputs[2]) + Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2))));

            SmartDashboard.putNumber("Translational Angle " + i + ": ", translationalAngles[i]);
            SmartDashboard.putNumber("Translational Magnitude " + i + ": ", translationalMagnitude[i]);
            SmartDashboard.putNumber("Rotational Angle " + i + ": ", rotationalAngles[i]);
            SmartDashboard.putNumber("Rotational Magnitude " + i + ": ", rotationalMagnitude[i]);
            SmartDashboard.putNumber("Combined X " + i + ": ", tempX);
            SmartDashboard.putNumber("Combined Y " + i + ": ", tempY);
            SmartDashboard.putNumber("Desired Angle " + i + ": ", desiredAngle[i]);
            SmartDashboard.putNumber("Desired Magnitude " + i + ": ", desiredMagnitude[i]);
            SmartDashboard.putNumber("Turn Set " + i + ": ", turnSets[i]);
            SmartDashboard.putNumber("Drive Set " + i + ": ", driveSets[i]);
        }

        set(driveSets, turnSets);
    }
}