package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
@Autonomous
public class CloseRed6 extends LinearOpMode {

    VoltageSensor voltageSensor;

    public void runOpMode()
    {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        PeregrineShooter shooter = new PeregrineShooter(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(47, 55, Math.toRadians(45+180)));

        TrajectoryActionBuilder preloaded, collectCloseStrip, wait2, moveToCloseStrip, preloaded2, leave, moveToMiddleStrip;

        preloaded = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(24.03);
                //.turn(Math.toRadians(-5))

        wait2 = preloaded.endTrajectory().fresh()
                .waitSeconds(.5);

        collectCloseStrip = preloaded.endTrajectory().fresh()
                .strafeTo(new Vector2d(20, 14))
                .turn(Math.toRadians(135));
                //.strafeTo(new Vector2d(-16.2, 10))

        moveToCloseStrip = collectCloseStrip.endTrajectory().fresh() //-31, 39
                .strafeTo(new Vector2d(65, 16));
                //.waitSeconds(1)
        preloaded2 = moveToCloseStrip.endTrajectory().fresh()
                .strafeTo(new Vector2d(16.2, 23.8))
                .turn(Math.toRadians(-135));
        moveToMiddleStrip = preloaded2.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, -8))
                .turn(Math.toRadians(135));
        leave = preloaded2.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, 0));


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                //collectCloseStrip
                new SequentialAction(
                        new ParallelAction(
                                preloaded.build(),
                                shooter.launcherToPower(1400, 2, 0)
                        ),
                        //shooter.intakeToPower(1, .25, 0),
                        //launch first ball
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0),
                                //shooter.beltToPower(1, 1, 0),
                                //shooter.intakeToPower(1, 1, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //reset
                        shooter.pusherToPosition(.75),
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0)
                        ),
                        new ParallelAction(
                                //shooter.pusherToPosition(.75),
                                shooter.launcherToPower(1400, .5, 0),
                                shooter.beltToPower(.5, .5, 0)
                                //new SleepAction(.5)
                        ),
                        //shooter.pusherToPosition(.75),

                        //wind launcher
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.beltToPower(.5, .25, 0)
                                //shooter.intakeToPower(1, 1, 0),
                        ),
                        //shoot ball 2
                        new ParallelAction(
                                shooter.beltToPower(.5, .4, 0),
                                //shooter.pusherToPosition(0),
                                shooter.launcherToPower(1400, .6, 0)
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
                                shooter.launcherToPower(1400, 2, 0),
                                shooter.beltToPower(1, 1, 0),
                                shooter.intakeToPower(-1, 1, 0)
                        ),
                        new ParallelAction(
                                shooter.launcherToPower(1400, .3 ,0),
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
                                shooter.launcherToPower(1400, .2, 0),
                                shooter.pusherToPosition(.75),
                                collectCloseStrip.build()
                                //shooter.intakeToPower(-1, 1, 0)
                        ),
                        new ParallelAction(
                                moveToCloseStrip.build(),
                                shooter.intakeToPower(-1, 2.5, 0),
                                shooter.beltToPower(1, 3, 0),
                                shooter.launcherToPower(-600, 3, 0)
                        ),
                                /*new ParallelAction(
                                        shooter.beltToPower(-.2, .5, 0),
                                        shooter.intakeToPower(.2, .25, 0)
                                ),*/
                        //shooter.beltToPower(-.5, .5, 0),
                        new ParallelAction(
                                preloaded2.build(),
                                shooter.launcherToPower(1400, 5, 0)
                        ),
                        //preloaded2,
                        //shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())) , 2.5, 0),
                        //shooter.intakeToPower(1, .25, 0),
                        //launch first ball
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0),
                                //shooter.beltToPower(1, 1, 0),
                                //shooter.intakeToPower(1, 1, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //reset
                        shooter.pusherToPosition(.75),
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0)
                        ),
                        new ParallelAction(
                                //shooter.pusherToPosition(.75),
                                shooter.launcherToPower(1400, .5, 0),
                                shooter.beltToPower(.5, .5, 0)
                                //new SleepAction(.5)
                        ),
                        //shooter.pusherToPosition(.75),

                        //wind launcher
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.beltToPower(.5, .25, 0)
                                //shooter.intakeToPower(1, 1, 0),
                        ),
                        //shoot ball 2
                        new ParallelAction(
                                shooter.beltToPower(.5, .4, 0),
                                //shooter.pusherToPosition(0),
                                shooter.launcherToPower(1400, .6, 0)
                        ),
                        //shooter.pusherToPosition(0),

                        //reset
                                /*new ParallelAction(
                                        //shooter.beltToPower(1, 1, 0),
                                        shooter.pusherToPosition(.75),
                                        shooter.launcherToPower((.57 * (13/voltageSensor.getVoltage())), 1, 0)
                                ),*/

                        //launch ball 3
                        /*new ParallelAction(
                                shooter.launcherToPower(1400, 2, 0),
                                shooter.beltToPower(1, 1, 0),
                                shooter.intakeToPower(-1, 1, 0)
                        ),
                        new ParallelAction(
                                shooter.launcherToPower(1400, .3 ,0),
                                shooter.pusherToPosition(0)
                        ),
                        //leave.build()*/
                        moveToMiddleStrip.build()
                )
        );
    }

}
