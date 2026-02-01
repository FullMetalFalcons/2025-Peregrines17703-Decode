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
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
@Autonomous
public class CloseBlue6 extends LinearOpMode {

    VoltageSensor voltageSensor;

    public void runOpMode()
    {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        PeregrineShooter shooter = new PeregrineShooter(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-47, 55, Math.toRadians(-45)));

        Action preloaded, collectCloseStrip, wait2, moveToCloseStrip, preloaded2, leave;

        preloaded = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(24.03)
                //.turn(Math.toRadians(-5))
                .build();
        wait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(.5)
                .build();
        collectCloseStrip = drive.actionBuilder(new Pose2d(-16.2,24.01, Math.toRadians(-45)))
                .strafeTo(new Vector2d(-20, 14))
                .turn(Math.toRadians(-179+45))
                //.strafeTo(new Vector2d(-16.2, 10))
                .build();

        moveToCloseStrip = drive.actionBuilder(new Pose2d(-20,14, Math.toRadians(-179))) //-31, 39
                .strafeTo(new Vector2d(-65, 16))
                //.waitSeconds(1)
                .build();
        preloaded2 = drive.actionBuilder(new Pose2d(-65, 16, Math.toRadians(-179)))
                        .turn(Math.toRadians(132))
                        .strafeTo(new Vector2d(-16.2, 23.8))
                .build();
        leave = drive.actionBuilder(new Pose2d(-16.2, 23.8, Math.toRadians(-179+132)))
                    .strafeTo(new Vector2d(-30, 0))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                //collectCloseStrip
                new SequentialAction(
                                new ParallelAction(
                                        preloaded,
                                        shooter.launcherToPower(1450, 2.5, 0)
                                ),
                                //shooter.intakeToPower(1, .25, 0),
                                //launch first ball
                                new ParallelAction(
                                        shooter.launcherToPower(1450, 1, 0),
                                        //shooter.beltToPower(1, 1, 0),
                                        //shooter.intakeToPower(1, 1, 0),
                                        shooter.pusherToPosition(0)
                                ),
                                //reset
                                shooter.pusherToPosition(.75),
                                new ParallelAction(
                                        wait2,
                                        shooter.launcherToPower(1450, 1, 0)
                                ),
                                new ParallelAction(
                                        //shooter.pusherToPosition(.75),
                                        shooter.launcherToPower(1450, 1, 0),
                                        shooter.beltToPower(.5, 1, 0),
                                        wait2
                                ),
                                //shooter.pusherToPosition(.75),

                                //wind launcher
                                new ParallelAction(
                                        shooter.launcherToPower(1450, 1, 0),
                                        shooter.beltToPower(.5, 1, 0)
                                        //shooter.intakeToPower(1, 1, 0),
                                ),
                                //shoot ball 2
                                new ParallelAction(
                                        shooter.beltToPower(.5, .4, 0),
                                        //shooter.pusherToPosition(0),
                                        shooter.launcherToPower(1450, 1, 0)
                                ),
                                //shooter.pusherToPosition(0),

                                //reset
                                /*new ParallelAction(
                                        //shooter.beltToPower(1, 1, 0),
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())), 1, 0)
                                ),*/

                                //launch ball 3
                                new ParallelAction(
                                        shooter.launcherToPower(1450, 2, 0),
                                        shooter.beltToPower(1, 1, 0),
                                        shooter.intakeToPower(-1, 1, 0)
                                ),
                                new ParallelAction(
                                        shooter.launcherToPower(1450, .3 ,0),
                                        shooter.pusherToPosition(0)
                                ),
                                /*new ParallelAction(
                                        //shooter.pusherToPosition(0),
                                        shooter.beltToPower(.5, 1, 0),
                                        shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())), 1, 0),
                                        shooter.intakeToPower(-1, 1, 0)
                                ),
                                new ParallelAction(
                                        shooter.intakeToPower(-1, 1, 0),
                                        shooter.beltToPower(1, 1, 0)
                                        //shooter.pusherToPosition(.75)
                                ),
                                new ParallelAction(
                                        //shooter.pusherToPosition(   .75),
                                        shooter.beltToPower(1, 1, 0)
                                ),*/
                                //leaveLaunch
                                new ParallelAction(
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower(0, .2, 0),
                                        collectCloseStrip
                                        //shooter.intakeToPower(-1, 1, 0)
                                ),
                                new ParallelAction(
                                        moveToCloseStrip,
                                        shooter.intakeToPower(-1, 2.5, 0),
                                        shooter.beltToPower(1, 2.5, 0)
                                ),
                                /*new ParallelAction(
                                        shooter.beltToPower(-.2, .5, 0),
                                        shooter.intakeToPower(.2, .25, 0)
                                ),*/
                                //shooter.beltToPower(-.5, .5, 0),
                                new ParallelAction(
                                        shooter.beltToPower(-.5, .5, 0),
                                        preloaded2,
                                        shooter.launcherToPower(1450, 5, 0)
                                ),
                                //preloaded2,
                                //shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())) , 2.5, 0),
                                //shooter.intakeToPower(1, .25, 0),
                                //launch first ball
                                new ParallelAction(
                                        shooter.launcherToPower(1450, .5, 0),
                                        //shooter.beltToPower(1, 1, 0),
                                        //shooter.intakeToPower(1, 1, 0),
                                        shooter.pusherToPosition(0)
                                ),
                                //reset
                                new ParallelAction(
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower(1400, 1, 0)
                                ),
                                //shooter.pusherToPosition(.75),
                                new ParallelAction(
                                        //wait2,
                                        shooter.launcherToPower(1450, 1, 0)
                                ),
                                new ParallelAction(
                                        //shooter.pusherToPosition(.75),
                                        shooter.launcherToPower(1450, 1, 0),
                                        shooter.beltToPower(.5, 1, 0)
                                        //wait2
                                ),
                                //shooter.pusherToPosition(.75),

                                //wind launcher
                                new ParallelAction(
                                        shooter.launcherToPower(1450, 1, 0),
                                        shooter.beltToPower(.5, 1, 0)
                                        //shooter.intakeToPower(1, 1, 0),
                                ),
                                //shoot ball 2
                                new ParallelAction(
                                        shooter.beltToPower(.5, .6, 0),
                                        //shooter.pusherToPosition(0),
                                        shooter.launcherToPower(1450, 1, 0)
                                ),
                                //shooter.pusherToPosition(0),

                                //reset
                                /*new ParallelAction(
                                        //shooter.beltToPower(1, 1, 0),
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())), 1, 0)
                                ),*/
                                //launch ball 3
                                new ParallelAction(
                                        shooter.launcherToPower(1450, 1, 0),
                                        shooter.beltToPower(1, 1, 0),
                                        shooter.intakeToPower(-1, 1, 0)

                                ),
                                new ParallelAction(
                                        shooter.launcherToPower(1450, .5, 0),
                                        shooter.pusherToPosition(0)
                                ),
                                /*new ParallelAction(
                                        shooter.pusherToPosition(0),
                                        shooter.beltToPower(.5, 1, 0),
                                        shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())), 1, 0),
                                        shooter.intakeToPower(-1, 1, 0)
                                ),*/
                        leave
                        )
        );
    }

}
