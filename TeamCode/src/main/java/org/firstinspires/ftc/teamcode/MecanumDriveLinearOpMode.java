package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MechanumDrive Linear OpMode", group = "ftc16671")

public class MecanumDriveLinearOpMode extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;



    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap);
        //set gamepad1
        double forward = gamepad1.left_stick_y *-1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * -1;
        double rotate = gamepad1.right_stick_x * -1;
        //set gamepad2

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                mecanumDrive.driveMecanum(forward, strafe, rotate);
                distances = mecanumDrive.getDistanceCm();
                telemetry.addData("distance fwd", distances[0]);
                telemetry.addData("distance right", distances[1]);
            }
        }
    }
}