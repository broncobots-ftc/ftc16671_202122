package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive Via Gamepad", group = "ftc16671")
public class MecanumDriveOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;
    double pusherPosition, pusherMinPosition, pusherMaxPosition,  lifterPosition, grabberPosition, armPosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    double SERVO_OFFSET = 0.005;
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initCarousel_and_lift(hardwareMap);
        //mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntake(hardwareMap);

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //setup gamepads
        //getting gamepad1 controls

        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * 1;
        double rotate = gamepad1.right_stick_x * 1;
        /*
        double gamepadrt = gamepad1.right_trigger;
        double gamepadlt = gamepad1.left_trigger;
        boolean gamepadLeftBumper = gamepad1.left_bumper;
        boolean gamepadRightBumper = gamepad1.right_bumper;
        //getting gamepad 2 controls
        boolean gamepad2X = gamepad2.x;
        boolean gamepad2Y = gamepad2.y;
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        double gamepad2rt = gamepad2.right_trigger;
        double gamepad2lt = gamepad2.left_trigger;
        boolean gamepad2LeftBumper = gamepad2.left_bumper;
        boolean gamepad2RightBumper = gamepad2.right_bumper;

         */
        //Setting gamepad2B***************************


        //Setting gamepad2Y***************************
        if(gamepad2.y){
            if(lifterPosition > 0.2) {
                lifterPosition -= SERVO_OFFSET;
            }
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());
        }
        //setting power for carousel and lifter
        /*
        if(gamepad2.b){
            mecanumDrive.carousel.setPower(-1.0);
        }
        if(gamepad2.x){
            mecanumDrive.carousel.setPower(1.0);
        }
        */
        //left trigger is intake forward, right is intake backward
        if (gamepad1.left_trigger > 0){
            //mecanumDrive.intake.setTargetPosition(4000000);
            mecanumDrive.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.runIntake(gamepad1.left_trigger);//take it in
            telemetry.addData("intake", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        if (gamepad1.right_trigger > 0){
            //mecanumDrive.intake.setTargetPosition(-4000000);
            mecanumDrive.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.runIntake(-gamepad1.right_trigger);//take it in
            telemetry.addData("intake", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        // Ww will move carousel colckwise and anitclockwise using gamepad 2 triggers
        if (gamepad2.left_trigger > 0){
            //mecanumDrive.intake.setTargetPosition(4000000);
            mecanumDrive.carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.carousel.setPower(gamepad2.left_trigger);//take it in
            telemetry.addData("intake", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        if (gamepad2.right_trigger > 0){
            //mecanumDrive.intake.setTargetPosition(-4000000);
            mecanumDrive.carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.carousel.setPower(-gamepad2.right_trigger);//take it in
            telemetry.addData("intake", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        //left bumper is 0 to 1, right bumper is 1 to 0






        //carousel is right joystick up and down, lift is left joystick up and down
        //mecanumDrive.carousel.setPower(gamepad2.right_stick_y);

        //mecanumDrive.lift.setPower(gamepad2.left_stick_y);

        //supply gamepad values to run motors, servo and other parts of robots
        mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.update();



    }

    /**
     *
     * @param miliseconds
     */
    private void justWait(int miliseconds){

        double currTime = getRuntime();
        double waitUntil = currTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
            //do nothing
        }

    }


}