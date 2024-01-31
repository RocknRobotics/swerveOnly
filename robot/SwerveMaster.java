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

        //Ideally, this is the actual x/y/heading of the robot, as calculated from wheel/motor kinematics stuff
        currentX = 0d;
        currentY = 0d;
        currentHeading = 0d;

        //Somehow, this needs to be the desired x/y/heading of the robot as determined from the controller
        //Hard part is taking into account friction and stuff?
        //Overall, the goal of this was to prevent the case where moving it back and forth causes it to turn slightly
        //(sometimes majorly). Worse though, is driving and turning causes drift in the direction of the turn.
        //I don't care how the fix to this is implemented, literally whatever you think will work, go for it
        //A small amount of error of x/y and heading is expected to accumulate over time due to discrepancies between wheel kinematics and reality as well as error accumulation on the gyro itself,
        //what I want fixed is the rapid accumulation caused by driving/turning and the slight turning caused during translational movement
        desiredX = 0d;
        desiredY = 0d;
        desiredHeading = 0d;
    }

    public void update(PS4Controller controller, double driveFactor, double turnFactor) {
        if(driveFactor == 0) {
            this.set(new double[]{0, 0, 0, 0}, new double[]{0, 0, 0, 0});
            return;
        }

        //NEW, ternaries before calculating factor allows lower speeds when the drive factor is low
        teleopUpdate(new double[]{Math.abs(controller.getLeftX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftX() * driveFactor, 
            Math.abs(controller.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getLeftY() * driveFactor, 
            Math.abs(controller.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : controller.getRightX() * turnFactor}, 
        new double[]{leftUpModule.getDriveVelocity(), leftDownModule.getDriveVelocity(), rightUpModule.getDriveVelocity(), rightDownModule.getDriveVelocity()}, 
        new double[]{leftUpModule.getAbsoluteTurnPosition(), leftDownModule.getAbsoluteTurnPosition(), rightUpModule.getAbsoluteTurnPosition(), rightDownModule.getAbsoluteTurnPosition()}, 
        this.getReducedAngle(), driveFactor);
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
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle, double driveFactor) {
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
        double desiredVx = -inputs[1] * Constants.maxTranslationalSpeed * Robot.driveControllerFactor;
        double desiredVy = inputs[0] * Constants.maxTranslationalSpeed * Robot.driveControllerFactor;
        double desiredOmega = -inputs[2] * Constants.maxAngularSpeed * Robot.driveControllerFactor;
        //The desired global translational vector of each wheel as described by angle and magnitude
        double[] translationalAngles = new double[4];
        double[] translationalMagnitude = new double[4];
        //Desired turning vector of each wheel
        double[] rotationalAngles =  new double[4];
        double[] rotationalMagnitude = new double[4];
        //Combined desired relative vector of each wheel as determined by the translational and turn vectors
        double[] desiredAngle = new double[4];
        double[] desiredMagnitude = new double[4];

        //Necessary since when converting inputs to proportions, division by 0 occurs if all inputs are 0
        //Then the result will be NaN, causing undefined behaviour
        if(inputs[0] == 0 && inputs[1] == 0 && inputs[2] == 0) {
            set(driveSets, turnSets);
            return;
        }

        //Offset angle to counteract drift. Go to method for more info.
        if (inputs[2] != 0 && (inputs[0] != 0 || inputs[1] != 0)) {
            inputs = offsetInput(inputs);

            //Recalculate inputs using factor
            inputs[0] *= driveFactor;
            inputs[1] *= driveFactor;
        }

        //For ease of access
        SwerveModule[] moduleList = new SwerveModule[]{leftUpModule, leftDownModule, rightUpModule, rightDownModule};
        //The list of the desired relative angle of each motor if turning right
        double[] angleListOne = new double[]{135d, 225d, 45d, 315d};
        //Turning left
        double[] angleListTwo = new double[]{315d, 45d, 225d, 135d};

        for(int i = 0; i < 4; i++) {
            //Calculating the angle of the global desired angle of the translational vector
            translationalAngles[i] = Math.atan2(-inputs[0], -inputs[1]) + Math.PI * 2;
            //Making sure range is 0 to 2pi
            if(translationalAngles[i] > Math.PI * 2) {
                translationalAngles[i] -= Math.PI * 2;
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
            translationalMagnitude[i] = Constants.maxTranslationalSpeed * Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2)) / Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2) + Math.pow(inputs[2], 2));            //Above but for rotational. Since translational magnitude is in metres, I converted
            //a max speed of 2pi to metres, which is equivalent to traveling the circumference
            //of the circle the wheels lie on, so thus I multiplied by the robot circumference
            //Again, not correct, but it at least worked
            rotationalMagnitude[i] = Constants.motorConstants.driveConstants.leftToRightDistanceMetres * Math.PI * Math.abs(inputs[2]) / Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2) + Math.pow(inputs[2], 2));
            //Setting the rotational angle based on whether turning left/right
            //If no rotation, then the magnitude is zero, so it doesn't matter what the angle is
            if(inputs[2] < 0) {
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
            if(inputs[0] == 0 && inputs[1] == 0) {
                tempX = rotationalMagnitude[i] * Math.cos(rotationalAngles[i] * Math.PI / 180);
                tempY = rotationalMagnitude[i] * Math.sin(rotationalAngles[i] * Math.PI / 180);
            } else if(inputs[2] == 0) {
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

            if (desiredMagnitude[i] > Constants.maxTranslationalSpeed) {
                desiredMagnitude[i] = Constants.maxTranslationalSpeed;
            }

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
            
            //Kevin - Reduce this temp thing down to controller factor
            double temp = (Math.min(1, Math.abs(inputs[2]) + Math.sqrt(Math.pow(inputs[0], 2) + Math.pow(inputs[1], 2))));
            if (temp > driveFactor) {
                temp = driveFactor;
            }
            //Kevin - Divided desired magnitude by max speed
            driveSets[i] = driveSetInvert * -desiredMagnitude[i] / Constants.maxTranslationalSpeed * temp;
    
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

        set(driveSets, turnSets);
    }

    //This method is meant to counteract the drift from swerve drive while
    // turning and moving at the same time. This is accomplished by offsetting
    // the left axis input in relation to the right axis input.
    public double[] offsetInput(double[] inputs) {
        //Calculate the angle of the left axis
        double leftAngle = Math.atan2(-inputs[0], -inputs[1]) + Math.PI * 2;
        
        //Making sure range is 0 to 2pi
        if(leftAngle > Math.PI * 2) {
            leftAngle -= Math.PI * 2;
        }

        //Converting to degrees (0 to 360)
        leftAngle *= 180 / Math.PI;

        //ADD 90?!!??!
        leftAngle += 90;

        //Increase or decrease angle in proportion to right input X.
        leftAngle += inputs[2] * 45; //Adjust angle until no drift.
    
        //Check for wrapping 
        if (leftAngle >= 360) {
            leftAngle -= 360;
        } else if (leftAngle < 0) {
            leftAngle += 360;
        }

        //Reconvert back to radians. 
        leftAngle *= Math.PI / 180;

        //Get new inputs from the adjusted angle
        inputs[0] = Math.cos(leftAngle);
        inputs[1] = -1 * Math.sin(leftAngle);
        
        //Return inputs
        return inputs;
    }
}