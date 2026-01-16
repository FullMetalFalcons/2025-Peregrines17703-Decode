package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@Autonomous
public class CloseBlue extends LinearOpMode {

    public void runOpMode()
    {
        PeregrineShooter shooter = new PeregrineShooter(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-47, 55, Math.toRadians(-45)));

        Action preloaded, collectCloseStrip, wait2, leaveLaunch;

        preloaded = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(24.03)
                .build();
        wait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(2)
                .build();
        collectCloseStrip = drive.actionBuilder(drive.localizer.getPose())
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-38.03, 12))
                .build();

        leaveLaunch = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(0)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                //collectCloseStrip
                new SequentialAction(
                        new SequentialAction(
                                preloaded,
                                shooter.launcherToPower(.6 , 2.5, 0),
                                //shooter.intakeToPower(1, .25, 0),
                                //launch first ball
                                new ParallelAction(
                                        shooter.launcherToPower(.6, 1, 0),
                                        shooter.beltToPower(1, 1, 0),
                                        //shooter.intakeToPower(1, 1, 0),
                                        shooter.pusherToPosition(0)
                                ),
                                //reset
                                new ParallelAction(
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower(.6, 1, 0),
                                        shooter.beltToPower(1, 1, 0)
                                ),
                                //shooter.pusherToPosition(.75),

                                //wind launcher
                                new ParallelAction(
                                        shooter.launcherToPower(.6, 1, 0)
                                        //shooter.beltToPower(1, 1, 0)
                                        //shooter.intakeToPower(1, 1, 0),
                                ),
                                //shoot ball 2
                                new ParallelAction(
                                        shooter.beltToPower(1, 1, 0),
                                        shooter.pusherToPosition(0),
                                        shooter.launcherToPower(.6, 1, 0)
                                ),
                                //shooter.pusherToPosition(0),

                                //reset
                                new ParallelAction(
                                        //shooter.beltToPower(1, 1, 0),
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower(.6, 1, 0)
                                ),

                                //launch ball 3
                                new ParallelAction(
                                        shooter.launcherToPower(.6, 2, 0),
                                        shooter.beltToPower(1, 1, 0),
                                        shooter.intakeToPower(-1, 1, 0)

                                ),
                                new ParallelAction(
                                        shooter.pusherToPosition(0),
                                        shooter.beltToPower(1, 1, 0),
                                        shooter.launcherToPower(.6, 1, 0),
                                        shooter.intakeToPower(-1, 1, 0)
                                ),
                                new ParallelAction(
                                        shooter.intakeToPower(-1, 1, 0),
                                        shooter.beltToPower(1, 1, 0),
                                        shooter.pusherToPosition(.75)
                                ),
                                //shooter.pusherToPosition(.75)
                                leaveLaunch
                        )
                )
        );
    }

}
