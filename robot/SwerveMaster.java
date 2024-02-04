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

    //Odometry
    private double[] robotPosition;


    public SwerveMaster() {
        leftUpModule = new SwerveModule(driveConstants.leftUpID, turnConstants.leftUpID, turnConstants.leftUpEncoderID, driveConstants.leftUpInvert, 
        turnConstants.leftUpInvert, turnConstants.leftUpEncoderInvert, turnConstants.leftUpOffset);
        leftDownModule = new SwerveModule(driveConstants.leftDownID, turnConstants.leftDownID, turnConstants.leftDownEncoderID, driveConstants.leftDownInvert, 
        turnConstants.leftDownInvert, turnConstants.leftDownEncoderInvert, turnConstants.leftDownOffset);
        rightUpModule = new SwerveModule(driveConstants.rightUpID, turnConstants.rightUpID, turnConstants.rightUpEncoderID, driveConstants.rightUpInvert, 
        turnConstants.rightUpInvert, turnConstants.rightUpEncoderInvert, turnConstants.rightUpOffset);
        rightDownModule = new SwerveModule(driveConstants.rightDownID, turnConstants.rightDownID, turnConstants.rightDownEncoderID, driveConstants.rightDownInvert, 
        turnConstants.rightDownInvert, turnConstants.rightDownEncoderInvert, turnConstants.rightDownOffset);

        leftUpModule.resetPosition(-0.3425d, 0.3425d);
        leftDownModule.resetPosition(-0.3425d, -0.3425d);
        rightUpModule.resetPosition(0.3425d, 0.3425d);
        rightDownModule.resetPosition(0.3425d, -0.3425d);

        accelerometer = new AHRS(Port.kMXP, Constants.accelerometerUpdateFrequency);
        accelerometer.reset();

        turnPIDController = new PIDController(Constants.motorConstants.turnConstants.kP, 0d, 0);
        //NEW, changed max/min input and changed kp to 0.8 (it's what worked when testing---probably don't change it for now)
        turnPIDController.enableContinuousInput(0, 360);

        robotPosition = new double[2];

        SmartDashboard.putNumber("Drift Constant", -2d);
    }

    public void update(PS4Controller controller, double driveFactor, double turnFactor) {
        if(driveFactor == 0 && turnFactor == 0) {
            this.set(new double[]{0, 0, 0, 0}, new double[]{0, 0, 0, 0});
            return;
        }

        double[] inputs = new double[]{Math.abs(controller.getLeftX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftX(), 
            Math.abs(controller.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftY(), 
            Math.abs(controller.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightX()};
        
        inputs[0] *= driveFactor;
        inputs[1] *= driveFactor;
        inputs[2] *= turnFactor; 
        /*if (!(inputs[0] == 0 && inputs[1] == 0)) {
            inputs = this.offsetInput(inputs);
        }*/

        //NEW, ternaries before calculating factor allows lower speeds when the drive factor is low
        teleopUpdate(inputs, 
        new double[]{leftUpModule.getDriveVelocity(), leftDownModule.getDriveVelocity(), rightUpModule.getDriveVelocity(), rightDownModule.getDriveVelocity()}, 
        new double[]{leftUpModule.getAbsoluteTurnPosition(), leftDownModule.getAbsoluteTurnPosition(), rightUpModule.getAbsoluteTurnPosition(), rightDownModule.getAbsoluteTurnPosition()}, 
        this.getReducedAngle(), driveFactor, turnFactor);
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
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle, double driveFactor, double turnFactor) {
        if(inputs[0] == 0 && inputs[1] == 0 && inputs[2] == 0) {
            set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
            return;
        }

        //NEW
        SmartDashboard.putNumber("Yaw: ", accelerometer.getYaw());
        SmartDashboard.putNumber("Reduced Yaw: ", reducedAngle);

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


        //Convert inputs to desired values---doesn't apply translation/rotation limit yet
        //Doesn't seem right... why multiply by controllerFactor?
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
            //Subtracting the gyro angle to make the angle robot relative
            //Don't need to check range since cos and sin functions don't care about that
            //Does that mean the first range check isn't necessary?
            translationalAngles[i] -= reducedAngle;
            //Calculating the translational magnitude as the proportion of the translational
            //inputs multiplied by the max speed... definitely not the right way to get the magnitude
            //since it doesn't take into account the actual input values---only the proportion
            translationalMagnitude[i] = Math.sqrt(Math.pow(adjustedX, 2) + Math.pow(adjustedY, 2));
            //Above but for rotational. Since translational magnitude is in metres, I converted
            //a max speed of 2pi to metres, which is equivalent to traveling the circumference
            //of the circle the wheels lie on, so thus I multiplied by the robot circumference
            //Again, not correct, but it at least worked
            rotationalMagnitude[i] = Math.abs(adjustedOmega);
            //Setting the rotational angle based on whether turning left/right
            //If no rotation, then the magnitude is zero, so it doesn't matter what the angle is
            if(adjustedOmega < 0) {
                rotationalAngles[i] = angleListOne[i];
            } else {
                rotationalAngles[i] = angleListTwo[i];
            }

            //Formula for converting from two vectors with a given magnitude and angle
            //Into an x and y value. Guess I got lucky that the x and y weren't switched?
            double tempX = translationalMagnitude[i] * Math.cos(translationalAngles[i] * Math.PI / 180) 
                + rotationalMagnitude[i] * Math.cos(rotationalAngles[i] * Math.PI / 180);
            double tempY = translationalMagnitude[i] * Math.sin(translationalAngles[i] * Math.PI / 180) 
                + rotationalMagnitude[i] * Math.sin(rotationalAngles[i] * Math.PI / 180);

            //Need to check if no translational or rotational input since that would make x/y 0 possibly,
            //And then atan2 might have undefined behaviour?
            //Wtf that just doesn't seem right at all
            //Isn't x/y being 0 possible if the angle between the two vectors results in 0/90/180/etc?
            //Also I think atan2 handles 0s anyways?
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
            //?
            //I don't know why this is here
            //Clearly necessary though since the robot drove correctly though
            desiredAngle[i] -= 90;
            if(desiredAngle[i] <= 0) {
                desiredAngle[i] += 360;
            }

            //Getting the magnitude via the pythagorean theorem
            desiredMagnitude[i] = Math.sqrt(Math.pow(tempX, 2) + Math.pow(tempY, 2));

            //Oh yeah forgot to mention I added optimisation while I was at it
            int driveSetInvert = 1;

            //turnPIDController takes into account the wrap from 0 to 360, so using it when calculating whether
            //it's optimal to add/subtract 180 for reversing actually results in it at most turning 90 probably
            //Not certain that 90 is the most it turns
            //Also should probably divide by 90 then? (Or 180 if I was wrong in assuming 90 is the most it can turn)
            if(Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) >= Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] + 180))) {
                driveSetInvert = -1;
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i] + 180) / 360d;
            } else if(Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i])) >= Math.abs(turnPIDController.calculate(positions[i], desiredAngle[i] - 180))) {
                driveSetInvert = -1;
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i] - 180) / 360d;
            } else {
                turnSets[i] = turnPIDController.calculate(positions[i], desiredAngle[i]) / 360d;
            }

            //So, if figured it's fine if the turnSets are always allowed to operate at max power, since the module will
            //turn to a certain angle and then the motor turns off---it's not like driving is affected

            //Drive though, needs to be variable based on motor inputs and the controller factor
            //This attempts to convert all three inputs into a range from 0 to 1 (remember invert takes care
            //of making it negative if needed).
            //However, this does not work as desired, since anything above about a 0.5 controller factor results in the speed
            //not changing since the motors are already going as fast as they can.
            //As a temporary fix, I've made the controller factor range 0 to 0.5
            //Please actually make the driveSet stuff correct so the controller range can be 0 to 1 again
            
            //Kevin - Divided desired magnitude by max speed
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

        updateRobotPosition(reducedAngle);
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


        set(driveSets, turnSets);
        //set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
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
        leftUpModule.alignPosition(robotPosition, reducedAngle, new double[]{-0.3425d, 0.3425d});
        leftDownModule.alignPosition(robotPosition, reducedAngle, new double[]{-0.3425d, -0.3425d});
        rightUpModule.alignPosition(robotPosition, reducedAngle, new double[]{0.3425d, 0.3425d});
        rightDownModule.alignPosition(robotPosition, reducedAngle, new double[]{0.3425d, -0.3425d});
    }
}