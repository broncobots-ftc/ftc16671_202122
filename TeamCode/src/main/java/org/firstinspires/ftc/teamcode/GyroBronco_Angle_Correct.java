//Autonomous mode Guga 12/2/2021

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "GyroBronco_AngleCorrect")
public class GyroBronco_Angle_Correct extends LinearOpMode {

    private DcMotor front_left_motor;
    private DcMotor back_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_right_motor;
    private BNO055IMU imu;


    float xOrientation;
    float targetOrientationAngle;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.3, correction;
    //float encoderResolution= =((((1+(46/17))) * (1+(46/11))) * 28);
    //float wheelRevolution = (2*3.14159265359*95)/25.4 ;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");


        // Reveser direction of left motor
        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Place into braking mode so robot stops abruptly
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Init the IMU
        // Current X orientation to zero- X is upward.  The robot direction will be zero until we do something to change it.
        telemetry.addData("X axis angle ", getXAxisOrientation());
        telemetry.update();
        init_IMU();
        // Robots direction will be zero until we do something
        // Wait for driver to press start button
        telemetry.addData("X axis angle ", getXAxisOrientation());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
//            telemetry.addData("X axis angle ", getXAxisOrientation());
//            telemetry.addData("Y axis angle ", getYAxisOrientation());
//            telemetry.addData("Z axis angle ", getZAxisOrientation());
//            telemetry.update();
            // To travel 24 in, we need = 2.02 x 537.7 pulses
            Move_to_Position(2.02 * 537.7);
            sleep(250);
            Turn_left_to_Position(1.5 * 537.7);
            sleep(250);
            Move_to_Position(2.02 * 537.7);
            sleep(250);
            Turn_right_to_Position(1.5 * 537.7);
            telemetry.addData("X axis angle ", getXAxisOrientation());
            telemetry.update();
//            // turn CW unti you reach 90
            //rotate(90, power);
//            telemetry.addData("X axis angle ", getXAxisOrientation());
//            telemetry.update();


        }
    }

    /**
     * initialize imu...
     */
    private void init_IMU() {
        BNO055IMU.Parameters IMUparameters;

        // Create a new IMU parameter object
        IMUparameters = new BNO055IMU.Parameters();
        // Set the imu mode to imu so automatically calibrate itself
        IMUparameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as angle unit
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // use meters per seconds^2 for units of acceleration
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver it may take several seconds
        telemetry.addData("Status", "Init IMU... Please Wait");
        telemetry.update();
        // Initialize IMU using these parameters
        imu.initialize(IMUparameters);
        // Tell Drive the init is done
        telemetry.addData("Status", "IMU initialized");
        telemetry.update();
    }

    /**
     * Move to a position.  We need to get the correct pulse and wheel diameter.  ...
     */
    private void Move_to_Position(double TargetPosition) {
        // Reset the encoders
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put the motor in encoder mode
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Turn on moters using moderate power
//        front_left_motor.setPower(0.3);
//        front_right_motor.setPower(0.3);
//        back_left_motor.setPower(0.3);
//        back_right_motor.setPower(0.3);
        front_left_motor.setPower(power-correction);
        back_left_motor.setPower(power-correction);
        front_right_motor.setPower(power+correction);
        back_right_motor.setPower(power+correction);

        // Loop until the motor reaches its target position
        while (front_left_motor.getCurrentPosition() < TargetPosition) {
            // Nothing while the robot moves forward
        }
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        // sleep quarter of seconds to let the robot sleep
        sleep(250);
    }

    private void Turn_left_to_Position(double TargetPosition) {
        // Reset the encoders
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put the motor in encoder mode
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Turn on moters using moderate power
        front_left_motor.setPower(0.3);
        front_right_motor.setPower(-0.3);
        back_left_motor.setPower(0.3);
        back_right_motor.setPower(-0.3);
//        front_left_motor.setPower(power-correction);
//        back_left_motor.setPower(power-correction);
//        front_right_motor.setPower(power+correction);
//        back_right_motor.setPower(power+correction);

        // Loop until the motor reaches its target position
        while (front_left_motor.getCurrentPosition() < TargetPosition) {
            // Nothing while the robot moves forward
        }
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        // sleep quarter of seconds to let the robot sleep
        sleep(250);
    }
    private void Turn_right_to_Position(double TargetPosition) {
        // Reset the encoders
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put the motor in encoder mode
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Turn on moters using moderate power
        front_left_motor.setPower(-0.3);
        front_right_motor.setPower(+0.3);
        back_left_motor.setPower(-0.3);
        back_right_motor.setPower(+0.3);
//        front_left_motor.setPower(power-correction);
//        back_left_motor.setPower(power-correction);
//        front_right_motor.setPower(power+correction);
//        back_right_motor.setPower(power+correction);

        // Loop until the motor reaches its target position
        while (front_right_motor.getCurrentPosition() < TargetPosition) {
            // Nothing while the robot moves forward
        }
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        // sleep quarter of seconds to let the robot sleep
        sleep(250);
    }


    /**
     * Get the orientation of the axis.  Here X is up vertical.
     */
    private float getXAxisOrientation() {
        Orientation Angles;

        // Get X axis orientation of the IMU.  Typically Z is the Yaw, but here in our robot, we have X axis is vertically Upward
        // we stand behind the robot, the rev stay flat, first at the bottom, right sensor, left motor, ZYX, but here for us it is XYZ
        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return Angles.firstAngle;
    }

    private float getYAxisOrientation() {
        Orientation Angles;

        // Get Y axis orientation of the IMU.  Typically Z is the Yaw, but here in our robot, we have X axis is vertically Upward
        // we stand behind the robot, the rev stay flat, first at the bottom, right sensor, left motor, ZYX, but here for us it is XYZ
        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Angles.secondAngle;
    }

    private float getZAxisOrientation() {
        Orientation Angles;

        // Get Z axis orientation of the IMU.  Typically Z is the Yaw, but here in our robot, we have X axis is vertically Upward
        // we stand behind the robot, the rev stay flat, first at the bottom, right sensor, left motor, ZYX, but here for us it is XYZ
        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Angles.thirdAngle;
    }

//    private void checkOrientation() {
//        Orientation Angles;
//        float curHeading;
//        // read the orientation of the robot
//        Angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        this.imu.getPosition();
//        // and save the heading
//        curHeading = Angles.firstAngle;
//    }

    private double previousHeading = 0; //Outside of method
    private double integratedHeading = 0;

    /**
     * This method returns a value of the X axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     * @return The integrated heading on the interval (-inf, inf).
     */
    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        return deltaHeading ;
    }

    /**
     * This function is to rotate clockwise..
     */
    private void rotateCCW(float targetOrientationAngle) {
        // Rotate in CCW direction
        // Assume we have not turned more than 180 degree
        // Get initial orientation about the X axis in our robot which is verticle upward
        xOrientation = getXAxisOrientation();
        // Set power to robot rotate in CCW direction
        front_left_motor.setPower(-0.4);
        back_left_motor.setPower(-0.4);
        front_right_motor.setPower(0.4);
        back_right_motor.setPower(0.4);
        // loop until we reach the target orientation

            while (xOrientation < targetOrientationAngle) {
            // Update current orientation about X
            xOrientation = getXAxisOrientation();
        }
        // stop the motors
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
        // wait a moment for the robot to stop
        sleep(250);
    }

    /**
     * Counterclockwise rotation.
     */
    private void rotateCW(float targetOrientationAngle) {
        // Rotate in CW direction
        // Assume we have not turned more than 180 degree
        // Get initial orientation about the X axis in our robot which is verticle upward
        xOrientation = getXAxisOrientation();
        // Set power to robot rotate in CCW direction
        front_left_motor.setPower(0.4);
        back_left_motor.setPower(0.4);
        front_right_motor.setPower(-0.4);
        back_right_motor.setPower(-0.4);
        // loop until we reach the target orientation
        while (Math.abs(xOrientation) < 90) {
            // Update current orientation about X
            xOrientation = getXAxisOrientation();
        }
        // stop the motors
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
        // wait a moment for the robot to stop
        sleep(250);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the X axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the X axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        front_left_motor.setPower(leftPower);
        front_right_motor.setPower(rightPower);
        back_left_motor.setPower(leftPower);
        back_right_motor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


}
