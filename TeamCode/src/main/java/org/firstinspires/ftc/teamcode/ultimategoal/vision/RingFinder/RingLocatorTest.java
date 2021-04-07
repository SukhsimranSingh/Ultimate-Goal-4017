package org.firstinspires.ftc.teamcode.ultimategoal.vision.RingFinder;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.util.DashboardUtil.addPacket;
import static org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRing;
import static org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot;
import static org.firstinspires.ftc.teamcode.util.DashboardUtil.sendPacket;;

@TeleOp(name = "Ring Locator Pipeline Test")
public class RingLocatorTest extends LinearOpMode {

    private RingLocator detector;
    private ArrayList<Ring> rings;
    public static double x = 87;
    public static double y = 63;
    public static double theta = PI/2;

    @Override
    public void runOpMode() {
        detector = new RingLocator(this);
        detector.start();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();


        waitForStart();

        while (opModeIsActive()) {
            rings = detector.getRings(x, y, theta);

            for (int i = 0; i < rings.size(); i++) {
                if (i == 0) {
                    drawRing(rings.get(i), "green");
                } else if (i == 1) {
                    drawRing(rings.get(i), "yellow");
                } else if (i == 2) {
                    drawRing(rings.get(i), "red");
                }
            }

            fieldOverlay.setStroke("#3F51B5");
            drawRobot(fieldOverlay, new Pose2d(x, y, theta));

            addPacket("Rings", rings);
            sendPacket();
        }

        detector.stop();
    }
}