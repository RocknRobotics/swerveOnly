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

        turnPIDController = new PIDController(Constants.motorConstants.turnConstants.kP, 0d, 0d);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        odometer = new SwerveDriveOdometry(driveConstants.drivemotorKinematics, getRotation2d(), new SwerveModulePosition[]{new SwerveModulePosition(0.0, Rotation2d.fromRadians(leftUpModule.getAbsoluteTurnPosition())), new SwerveModulePosition(0.0, Rotation2d.fromRadians(leftDownModule.getAbsoluteTurnPosition())), new SwerveModulePosition(0.0, Rotation2d.fromRadians(rightUpModule.getAbsoluteTurnPosition())), new SwerveModulePosition(0.0, Rotation2d.fromRadians(rightDownModule.getAbsoluteTurnPosition()))});
    }

    public void update(PS4Controller controller, double factor) {
        teleopUpdate(new double[]{Math.abs(controller.getLeftX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftX() * factor, Math.abs(controller.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftY() * factor, Math.abs(controller.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightX() * factor}, 
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
        return Math.IEEEremainder(-accelerometer.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getReducedAngle());
    }

    //Does the heavy lifting
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle) {
        for(int i = 0; i < inputs.length; i++) {
            if(Math.abs(inputs[i]) < Constants.driveControllerStopBelowThis) {
                inputs[i] = 0d;
            }

            System.out.println(inputs[i]);
        }
        
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
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(inputs[1] * Constants.maxTranslationalSpeed, 
        -inputs[0] * Constants.maxTranslationalSpeed, -inputs[2] * Constants.maxAngularSpeed, Rotation2d.fromDegrees(reducedAngle));
        //Converts those speeds to targetStates since I'm not a monster who puts everything on one line (I had to resist the urge to)
        SwerveModuleState[] targetStates = Constants.motorConstants.driveConstants.drivemotorKinematics.toSwerveModuleStates(speeds);

        //Scales the targetStates in case it's not physically possible (for example it's impossible to go the true full move velocity 
        //and true full rotational velocity at the same time)
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.motorConstants.driveConstants.drivemotorKinematics.toChassisSpeeds(
            new SwerveModuleState[]{new SwerveModuleState(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModuleState(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModuleState(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModuleState(velocities[3], Rotation2d.fromRadians(positions[3]))}), 
        Constants.motorConstants.driveConstants.maxSpeed, Constants.maxTranslationalSpeed, Constants.maxAngularSpeed);

        //Optimize the states
        for(int i = 0; i < targetStates.length; i++) {
            //targetStates[i].speedMetersPerSecond *= 10;
            System.out.println("I: "+ i + "\tSpeed: " + targetStates[i].speedMetersPerSecond);
            if(Math.abs(targetStates[i].speedMetersPerSecond) < Constants.motorConstants.driveConstants.stopBelowThisVelocity) {
                driveSets[i] = 0d;
                turnSets[i] = 0d;
            } else {
                SwerveModuleState.optimize(targetStates[i], Rotation2d.fromRadians(positions[i]));
                driveSets[i] = targetStates[i].speedMetersPerSecond / (Constants.motorConstants.driveConstants.maxSpeed * Constants.motorConstants.driveConstants.metresPerRotation);
                turnSets[i] = turnPIDController.calculate(positions[i], targetStates[i].angle.getRadians());
            }

            driveSets[i] *= 10.0;
            turnSets[i] /= 50.0;
            System.out.println("Drive: " + driveSets[i]);
            System.out.println("Turn: " + turnSets[i]);
        }

        //Update odometry
        odometer.update(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
            new SwerveModulePosition(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModulePosition(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModulePosition(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModulePosition(velocities[3], Rotation2d.fromRadians(positions[3]))});

        this.set(driveSets, turnSets);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometer(double[] velocities, double[] positions, double reducedAngle, Pose2d currPose) {
        odometer.resetPosition(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
            new SwerveModulePosition(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModulePosition(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModulePosition(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModulePosition(velocities[3], Rotation2d.fromRadians(positions[3]))}, currPose);
    }
}
