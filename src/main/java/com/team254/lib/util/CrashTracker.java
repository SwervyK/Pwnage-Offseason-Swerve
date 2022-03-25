package com.team254.lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

import edu.wpi.first.wpilibj.Timer;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    public static void logRobotConstruction() {
        logMarker("robot startup");
    }

    public static void logRobotInit() {
        logMarker("robot init");
    }

    public static void logTeleopInit() {
        logMarker("teleop init");
    }

    public static void logAutoInit() {
        logMarker("auto init");
    }

    public static void logDisabledInit() {
        logMarker("disabled init");
    }

    public static void logTestInit() {
        logMarker("test init");
    }

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    public static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {

        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/logs/crashes_and_markers.txt", true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(Timer.getFPGATimestamp());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}