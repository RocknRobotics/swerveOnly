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
    SwerveModule[] swerveModules;

    public AHRS accelerometer;

    private SwerveDriveOdometry odometer;

    //Enables continious input I think? I'm writing this using another person's code as a guide---I'll mess around with changing
    //this once we have the swerve drive built
    private PIDController turnPIDController;

    private double[] turns;
    private double[] drives;

    boolean driveReady;

    public SwerveMaster() {
        swerveModules = new SwerveModule[] {
        new SwerveModule(driveConstants.leftUpID, turnConstants.leftUpID, turnConstants.leftUpEncoderID, driveConstants.leftUpInvert, 
        turnConstants.leftUpInvert, turnConstants.leftUpEncoderInvert, turnConstants.leftUpOffset),
        new SwerveModule(driveConstants.leftDownID, turnConstants.leftDownID, turnConstants.leftDownEncoderID, driveConstants.leftDownInvert, 
        turnConstants.leftDownInvert, turnConstants.leftDownEncoderInvert, turnConstants.leftDownOffset),
        new SwerveModule(driveConstants.rightUpID, turnConstants.rightUpID, turnConstants.rightUpEncoderID, driveConstants.rightUpInvert, 
        turnConstants.rightUpInvert, turnConstants.rightUpEncoderInvert, turnConstants.rightUpOffset),
        new SwerveModule(driveConstants.rightDownID, turnConstants.rightDownID, turnConstants.rightDownEncoderID, driveConstants.rightDownInvert, 
        turnConstants.rightDownInvert, turnConstants.rightDownEncoderInvert, turnConstants.rightDownOffset)};

        accelerometer = new AHRS(Port.kMXP, Constants.accelerometerUpdateFrequency);
        accelerometer.reset();

        turnPIDController = new PIDController(Constants.motorConstants.turnConstants.kP, 0d, 0d);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        odometer = new SwerveDriveOdometry(driveConstants.drivemotorKinematics, getRotation2d(), new SwerveModulePosition[]{new SwerveModulePosition(0.0, Rotation2d.fromRadians(leftUpModule.getAbsoluteTurnPosition())), new SwerveModulePosition(0.0, Rotation2d.fromRadians(leftDownModule.getAbsoluteTurnPosition())), new SwerveModulePosition(0.0, Rotation2d.fromRadians(rightUpModule.getAbsoluteTurnPosition())), new SwerveModulePosition(0.0, Rotation2d.fromRadians(rightDownModule.getAbsoluteTurnPosition()))});
            
        turns = new double[4];
        drives = new double[4];

        driveReady = false;
    }

    public void update(PS4Controller controller, double factor) {
        double[] axisInputs = new double[]{Math.abs(controller.getLeftX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftX() * factor,
            Math.abs(controller.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftY() * factor,
            Math.abs(controller.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightX() * factor,
            Math.abs(controller.getRightY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightY() * factor};

        double[] angles = new double[]{leftUpModule.getAbsoluteTurnPosition(),
            leftDownModule.getAbsoluteTurnPosition(),
            rightUpModule.getAbsoluteTurnPosition(),
            rightDownModule.getAbsoluteTurnPosition()};

        driveReady = drive(axisInputs, (controller.getR2Axis() < Constants.driveControllerStopBelowThis ? 0.0 : controller.getR2Axis()) / 4);//adjust drive speed. Also change to left 2d input after testing
        set();
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

    public void set() {
        leftUpModule.set(drives[0], turns[0]);
        leftDownModule.set(drives[1], turns[1]);
        rightUpModule.set(drives[2], turns[2]);
        rightDownModule.set(drives[3], turns[3]);
    }

    //If this works, we need to somehow integrate turning 
    public boolean drive(double[] axisInput, double drivePower) {
        //get radian of left input axis
        double targetRadian = Math.atan(axisInput[1] / axisInput[0]);

        //adjust for quadrants II, III, and IV
        if (axisInput[0] < 0) {
            targetRadian += Math.PI;
        } else if (axisInput[1] < 0) {
            targetRadian += Math.PI;
        }
        
        //run through each module and give them the correct turn speed
        //CCW positive
        //CW negative
        for (int i = 0; i < 4; i++) {
            double currRad = swerveModules[i].getAbsoluteTurnPosition();
            
            //calculate position of current rad relative to target rad
            double radDifference = targetRadian - currRad;

            //account for targetRad and currRad being on opposite sides of wrap
            if (Math.abs(radDifference) > Math.abs(targetRadian - (currRad + Math.PI * 2))) {
                radDifference = targetRadian - (currRad + Math.PI * 2);
            } else if (Math.abs(radDifference) > Math.abs((targetRadian + Math.PI * 2) - currRad)) {
                radDifference = (targetRadian + Math.PI * 2) - currRad;
            } 

            //allows better efficiency by turning smaller angle then inverting drive
            if (Math.abs(radDifference) > Math.PI / 2) {
                if (targetRadian >= Math.PI) {
                    targetRadian -= Math.PI;
                } else {
                    targetRadian += Math.PI;
                }

                if (radDifference > 0) {
                    radDifference -= Math.PI;
                } else {
                    radDifference += Math.PI;
                }
                swerveModules[i].invertDrive();
            }

            //actually turn to the target angle using percentage speed depending on radDifference
            //check how many are not ready for drive
            int notReady = 0;
            //adjust tolerance             V
            if (Math.abs(radDifference) > 0.05) {
                turns[i] = 0.5 * (radDifference / (Math.PI / 4)); //Max turn speed of 0.5 

                //Min turn speed of 0.1
                /* Maybe don't need this
                if (radDifference < 0 && Math.abs(turns[i]) < 0.1) {
                    turns[i] = -0.1;
                } else if (radDifference > 0 && Math.abs(turns[i]) < 0.1) {
                    turns[i] = 0.1;
                } */
                notReady++;
                drives[i] = drives[i] * 0.95; //decelerate if not on right angle
                if (Math.abs(drives[i]) < 0.05) {  
                    drives[i] = 0;
                }
            } else {
                turns[i] = 0; 
                if (driveReady) { //accelerate if on right angle and ready to drive
                    drives[i] += drivePower * (1 - (drives[i] / drivePower)) / 4;
                } else {
                    drives[i] = drives[i] * 0.95; //decelerate if not ready to drive
                }
            }

            return (notReady == 0) || (driveReady && notReady != 4);
        }
    }
}
