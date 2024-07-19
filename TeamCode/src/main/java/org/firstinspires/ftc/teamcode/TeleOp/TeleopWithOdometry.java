package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Autonomous.poseStorage;
import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleopWithOdometry", group = "AAAAAAAA")
public class TeleopWithOdometry extends DriveMethods {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(poseStorage.currentPose);

        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        double speedDiv = 2.0;

        telemetry.addLine("Status");
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            motorFL.setPower((leftY + leftX + rightX) / speedDiv);
            motorBL.setPower((leftY - leftX + rightX) / speedDiv);
            motorBR.setPower((leftY + leftX - rightX) / speedDiv);
            motorFR.setPower((leftY - leftX - rightX) / speedDiv);

            drive.update();
            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.update();
        }
    }
}
