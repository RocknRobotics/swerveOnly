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

public class SwerveMaster {
    public SwerveModule leftUpModule;
    public SwerveModule leftDownModule;
    public SwerveModule rightUpModule;
    public SwerveModule rightDownModule;

    public AHRS accelerometer;

    private SwerveDriveOdometry odometer;

    //Enables continious input I think? I'm writing this using another person's code as a guide---I'll mess around with changing
    //this once we have the swerve drive built
    private PIDController turnPIDController;
    private PIDController turnSetController;

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

        odometer = new SwerveDriveOdometry(driveConstants.drivemotorKinematics, getRotation2d(), new SwerveModulePosition[]{
            new SwerveModulePosition(Math.sqrt(2) * Constants.motorConstants.driveConstants.leftToRightDistanceMetres, Rotation2d.fromDegrees(135)), 
            new SwerveModulePosition(Math.sqrt(2) * Constants.motorConstants.driveConstants.leftToRightDistanceMetres, Rotation2d.fromDegrees(225)), 
            new SwerveModulePosition(Math.sqrt(2) * Constants.motorConstants.driveConstants.leftToRightDistanceMetres, Rotation2d.fromDegrees(45)), 
            new SwerveModulePosition(Math.sqrt(2) * Constants.motorConstants.driveConstants.leftToRightDistanceMetres, Rotation2d.fromDegrees(315))});
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
    }

    public boolean accelerometerIsCalibrating() {
        return accelerometer.isCalibrating();
    }

    public double getReducedAngle() {
        //You've got to be joking the darn navx is cw positive when we need ccw positive readings
        double temp = -accelerometer.getAngle() % 360;
        //NEW
        //Ranges from 180 to -180
        //Make it 0 to 360
        if(temp == 180) {
            return temp;
        }else if(temp < 0) {
            return 360 + temp;
        } else {
            //Greater than add 180 leave it as is
            return temp;
        }
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getReducedAngle());
    }

    //Does the heavy lifting
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle) {
        //Update odometry
        odometer.update(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
            new SwerveModulePosition(velocities[0], Rotation2d.fromDegrees(positions[0])), 
            new SwerveModulePosition(velocities[1], Rotation2d.fromDegrees(positions[1])), 
            new SwerveModulePosition(velocities[2], Rotation2d.fromDegrees(positions[2])), 
            new SwerveModulePosition(velocities[3], Rotation2d.fromDegrees(positions[3]))});

        //NEW
        SmartDashboard.putNumber("Yaw: ", accelerometer.getYaw());
        SmartDashboard.putNumber("Reduced Yaw: ", reducedAngle);

        turnSetController.setP(SmartDashboard.getNumber("kP: ", turnSetController.getP()));
        turnSetController.setI(SmartDashboard.getNumber("kI: ", turnSetController.getI()));
        turnSetController.setD(SmartDashboard.getNumber("kD: ", turnSetController.getD()));
        turnSetController.setTolerance(SmartDashboard.getNumber("Position Tolerance: ", turnSetController.getPositionTolerance()), SmartDashboard.getNumber("Velocity Tolerance: ", turnSetController.getVelocityTolerance()));

        SmartDashboard.putNumber("Current 0 Angle: ", leftUpModule.getState().angle.getDegrees());
        SmartDashboard.putNumber("Current 1 Angle: ", leftDownModule.getState().angle.getDegrees());
        SmartDashboard.putNumber("Current 2 Angle: ", rightUpModule.getState().angle.getDegrees());
        SmartDashboard.putNumber("Current 3 Angle: ", rightDownModule.getState().angle.getDegrees());

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
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-inputs[1] * Constants.maxTranslationalSpeed, 
        inputs[0] * Constants.maxTranslationalSpeed, inputs[2] * Constants.maxAngularSpeed, odometer.getPoseMeters().getRotation());
        //Converts those speeds to targetStates since I'm not a monster who puts everything on one line (I had to resist the urge to)
        SwerveModuleState[] targetStates = Constants.motorConstants.driveConstants.drivemotorKinematics.toSwerveModuleStates(desiredSpeeds);

        SmartDashboard.putNumber("Desired X Speed: ", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Desired Y Speed: ", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Desired Omega Speed: ", desiredSpeeds.omegaRadiansPerSecond);

        //Scales the targetStates in case it's not physically possible (for example it's impossible to go the true full move velocity 
        //and true full rotational velocity at the same time)
        /*Constants.motorConstants.driveConstants.drivemotorKinematics.toChassisSpeeds(
            new SwerveModuleState[]{new SwerveModuleState(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModuleState(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModuleState(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModuleState(velocities[3], Rotation2d.fromRadians(positions[3]))})*/

        //NEW, commented this out for now (if you comment it back in use the whiles in the for loop below to make degree range 0 to 360)
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, desiredSpeeds, Constants.motorConstants.driveConstants.maxSpeed / 60.0d * Constants.motorConstants.driveConstants.metresPerRotation, 
        Constants.maxTranslationalSpeed, Constants.maxAngularSpeed);

        double proportion2 = Math.abs(inputs[2]) * inputs[2] / Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2) + Math.pow(inputs[2], 2));
        double proportionXY = Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2)) / Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2) + Math.pow(inputs[2], 2));

        targetStates[0].angle = Rotation2d.fromDegrees(targetStates[0].angle.getDegrees() + ((inputs[2] < 0 ? 1 : -1) * proportion2 * (1 / Robot.driveControllerFactor) * 90));
        targetStates[1].angle = Rotation2d.fromDegrees(targetStates[1].angle.getDegrees() - ((inputs[2] < 0 ? 1 : -1) * proportion2 * (1 / Robot.driveControllerFactor) * 90));
        targetStates[2].angle = Rotation2d.fromDegrees(targetStates[2].angle.getDegrees() - ((inputs[2] < 0 ? 1 : -1) * proportion2 * (1 / Robot.driveControllerFactor) * 90));
        targetStates[3].angle = Rotation2d.fromDegrees(targetStates[3].angle.getDegrees() + ((inputs[2] < 0 ? 1 : -1) * proportion2 * (1 / Robot.driveControllerFactor) * 90));
        targetStates[0].angle = Rotation2d.fromDegrees(targetStates[0].angle.getDegrees() + odometer.getPoseMeters().getRotation().getDegrees() * 2 * (proportionXY));
        targetStates[1].angle = Rotation2d.fromDegrees(targetStates[1].angle.getDegrees() + odometer.getPoseMeters().getRotation().getDegrees() * 2 * (proportionXY));
        targetStates[2].angle = Rotation2d.fromDegrees(targetStates[2].angle.getDegrees() + odometer.getPoseMeters().getRotation().getDegrees() * 2 * (proportionXY));
        targetStates[3].angle = Rotation2d.fromDegrees(targetStates[3].angle.getDegrees() + odometer.getPoseMeters().getRotation().getDegrees() * 2 * (proportionXY));
        //NEW
        targetStates[0].angle = Rotation2d.fromDegrees(targetStates[0].angle.getDegrees() - odometer.getPoseMeters().getRotation().getDegrees() * proportion2);
        targetStates[1].angle = Rotation2d.fromDegrees(targetStates[1].angle.getDegrees() - odometer.getPoseMeters().getRotation().getDegrees() * proportion2);
        targetStates[2].angle = Rotation2d.fromDegrees(targetStates[2].angle.getDegrees() - odometer.getPoseMeters().getRotation().getDegrees() * proportion2);
        targetStates[3].angle = Rotation2d.fromDegrees(targetStates[3].angle.getDegrees() - odometer.getPoseMeters().getRotation().getDegrees() * proportion2);

        SmartDashboard.putNumber("Target State 0 Speed: ", targetStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Target State 1 Speed: ", targetStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Target State 2 Speed: ", targetStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Target State 3 Speed: ", targetStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("Target State 0 Angle: ", targetStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Target State 1 Angle: ", targetStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Target State 2 Angle: ", targetStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Target State 3 Angle: ", targetStates[3].angle.getDegrees());

        //Optimize the states
        for(int i = 0; i < targetStates.length; i++) {
            if(Math.abs(targetStates[i].speedMetersPerSecond) < Constants.motorConstants.driveConstants.stopBelowThisVelocity) {
                driveSets[i] = 0d;
                turnSets[i] = 0d;
            } else {
                //NEW
                while(targetStates[i].angle.getDegrees() < 0) {
                    targetStates[i].angle = Rotation2d.fromDegrees(targetStates[i].angle.getDegrees() + 360);
                }
                while(targetStates[i].angle.getDegrees() > 360) {
                    targetStates[i].angle = Rotation2d.fromDegrees(targetStates[i].angle.getDegrees() - 360);
                }

                //NEW---manual optimisation
                if(Math.abs(positions[i] - targetStates[i].angle.getDegrees()) >= 180) {
                    targetStates[i].speedMetersPerSecond = -targetStates[i].speedMetersPerSecond;
                    targetStates[i].angle = Rotation2d.fromDegrees(targetStates[i].angle.getDegrees() - 180);
                }

                //NEW
                while(targetStates[i].angle.getDegrees() < 0) {
                    targetStates[i].angle = Rotation2d.fromDegrees(targetStates[i].angle.getDegrees() + 360);
                }
                while(targetStates[i].angle.getDegrees() > 360) {
                    targetStates[i].angle = Rotation2d.fromDegrees(targetStates[i].angle.getDegrees() - 360);
                }

                SmartDashboard.putNumber("Optimized State " + i + " Speed: ", targetStates[i].speedMetersPerSecond);
                SmartDashboard.putNumber("Optimized State " + i + " Angle: ", targetStates[i].angle.getDegrees());
                driveSets[i] = targetStates[i].speedMetersPerSecond / (Constants.motorConstants.driveConstants.maxSpeed * Constants.motorConstants.driveConstants.metresPerRotation / 60.0d);
                SmartDashboard.putNumber("Drive Set " + i + ": ", driveSets[i]);
                turnSets[i] = turnPIDController.calculate(positions[i], targetStates[i].angle.getDegrees());
                SmartDashboard.putNumber("RawTurn Set " + i + ": ", turnSets[i]);
            }
        }

        turnSets[0] = turnSetController.calculate(leftUpModule.turnMotor.get(), turnSets[0]);
        turnSets[1] = turnSetController.calculate(leftDownModule.turnMotor.get(), turnSets[1]);
        turnSets[2] = turnSetController.calculate(rightUpModule.turnMotor.get(), turnSets[2]);
        turnSets[3] = turnSetController.calculate(rightDownModule.turnMotor.get(), turnSets[3]);

        //Actually turn the angles into a speed
        //Thanks Kevin
        for (int i = 0; i < 4; i++) {
            turnSets[i] /= 180;
        }

        SmartDashboard.putNumber("Turn Set 0: ", turnSets[0]);
        SmartDashboard.putNumber("Turn Set 1: ", turnSets[1]);
        SmartDashboard.putNumber("Turn Set 2: ", turnSets[2]);
        SmartDashboard.putNumber("Turn Set 3: ", turnSets[3]);

        
        this.set(driveSets, turnSets);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometer(double[] velocities, double[] positions, double reducedAngle, Pose2d currPose) {
        odometer.resetPosition(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
            new SwerveModulePosition(velocities[0], Rotation2d.fromDegrees(positions[0])), 
            new SwerveModulePosition(velocities[1], Rotation2d.fromDegrees(positions[1])), 
            new SwerveModulePosition(velocities[2], Rotation2d.fromDegrees(positions[2])), 
            new SwerveModulePosition(velocities[3], Rotation2d.fromDegrees(positions[3]))}, currPose);
    }
}
