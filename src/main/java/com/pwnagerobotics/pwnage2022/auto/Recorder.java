package com.pwnagerobotics.pwnage2022.auto;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Scanner;

import com.pwnagerobotics.pwnage2022.auto.Action.ControllerState;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Recorder {
    
    private File mAutoFile;
    private File mAutoDir = new File("home/lvuser/autoRecordings");
    private static SendableChooser<File> mAutoChooser = new SendableChooser<File>();
    private ArrayList<String> mActions = new ArrayList<String>();
    private boolean mIsRecording = false;
    
    public Recorder() {
        try {
            String[] fileNames = mAutoDir.list();
            if (fileNames.length > 0) {
                for (int i = 0; i < fileNames.length; i++) {
                    mAutoChooser.addOption(fileNames[i], new File(mAutoDir.getPath(), fileNames[i]));
                }
                mAutoFile = mAutoChooser.getSelected();
            }
            SmartDashboard.putData("Auto Chooser", mAutoChooser);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error in getting files: Recorder.java");
        }
        
    }
    
    public void newAuto(String filename) {
        mAutoFile = new File(mAutoDir.getPath(), filename);
        try {
            if (!mAutoFile.exists()) 
                mAutoFile.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("New auto created");
        mIsRecording = true;
    }
    
    public void recordInputs(double throttle, double strafe, double rotationX, double rotationY, boolean fieldCentricRotation, double timestamp) {
        mActions.add(timestamp+":[t="+throttle+"]"+"[s="+strafe+"]"+"[rx="+rotationX+"]"+"[ry="+rotationY+"]"+"[fc="+fieldCentricRotation+"] ");
    }
    
    public void stopRecording() {
        try {
            FileWriter writer = new FileWriter(mAutoFile);
            for (String s : mActions) {
                writer.write(s + "\n");
            }
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error while saving file: Recorder.java");
        }
        System.out.print("Auto saved as: " + mAutoFile.getName());
        mIsRecording = false;
    }

    public boolean isRecording() {
        return mIsRecording;
    }
    
    private Action[] autoActions;
    public Action getSwerveStateAtTimestamp(double timestamp) {
        if (autoActions == null) loadAuto();
        int closestIndex = 0;
        double difference = Double.POSITIVE_INFINITY;
        for (int i = 0; i < autoActions.length; i++) {
            if (timestamp - autoActions[i].getTimestamp() < difference) {
                difference = timestamp - autoActions[i].getTimestamp();
                if (difference < 0) {
                    break;
                }
                closestIndex = i;
            }
        }
        return autoActions[closestIndex];
    }
    
    private void loadAuto() {
        String[] data = readData(mAutoChooser.getSelected());
        autoActions = new Action[data.length];
        for (int i = 0; i < data.length; i++) {
            double throttle = Double.parseDouble(data[i].substring(data[i].indexOf("t=")+2,data[i].indexOf("][s")));
            double strafe = Double.parseDouble(data[i].substring(data[i].indexOf("s=")+2,data[i].indexOf("][rx")));
            double rotationX = Double.parseDouble(data[i].substring(data[i].indexOf("rx=")+3,data[i].indexOf("][ry")));
            double rotationY = Double.parseDouble(data[i].substring(data[i].indexOf("ry=")+3,data[i].indexOf("][fc")));
            double timestamp = Double.parseDouble(data[i].substring(0, data[i].indexOf(":")));
            Boolean fieldCentricRotation = Boolean.parseBoolean(data[i].substring(data[i].indexOf("fc=")+3,data[i].indexOf("] ")));
            autoActions[i] = new Action(new ControllerState(throttle, strafe, rotationX, rotationY), fieldCentricRotation);
            autoActions[i].setTimestamp(timestamp);
        }
    }
    
    private String[] readData(File file) {
        String[] result = new String[0];
        try {
            result = new String[(int)Files.lines(file.toPath()).count()];
            Scanner scanner = new Scanner(file);
            int index = 0;
            while (scanner.hasNextLine()) {
                result[index] = scanner.nextLine();
                index++;
            }
            scanner.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return result;
    }
}