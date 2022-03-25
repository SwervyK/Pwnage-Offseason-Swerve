package com.team254.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import com.team254.lib.util.ReflectingCSVWriter;
import java.io.File;

/* Manage log files */
public class Logger<T> {
    // Log file location
    public static final String kBaseDirectory = "/home/lvuser/";
    public static final String kLogDirectory = kBaseDirectory + "logs/";
    
    private String mFileName = null;

    // Assign this to log the IO to CSV
    private ReflectingCSVWriter<T> mCSVWriter = null;

    private DriverStation mDriverStation = null;

   public Logger (String fileName, Class<T> typeClass) {
        mFileName = fileName;
        mCSVWriter = new ReflectingCSVWriter<>(kLogDirectory + fileName + ".csv", typeClass);
    }

    public synchronized void startLogging() {
    }

    public synchronized void stopLogging() {
        System.out.println("Stopping logging");
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }

        mDriverStation = DriverStation.getInstance();
        if (mDriverStation != null && mDriverStation.isOperatorControl()) {
            int matchNumber = mDriverStation.getMatchNumber();
            if (matchNumber != 0) {
                String oldName = kLogDirectory + mFileName + ".csv";
                String newName = kLogDirectory + mFileName + "-" + matchNumber + ".csv";
                File oldFile = new File(oldName);
                File newFile = new File(newName);
                if(oldFile.renameTo(newFile)){
                    System.out.println("Renamed file from " + oldName + " -> " + newName);
                }else{
                    System.out.println("Rename failed: " + oldName + " -> " + newName);
                }
            }
        }

    }

    public void add(T value) {
        if (mCSVWriter != null)
            mCSVWriter.add(value);
    }

    public void flush() {
        if (mCSVWriter != null)
            mCSVWriter.flush();
    }

    public void write() {
        if (mCSVWriter != null)
            mCSVWriter.write();
    }
}