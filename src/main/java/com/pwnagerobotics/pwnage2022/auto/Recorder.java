package com.pwnagerobotics.pwnage2022.auto;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Recorder {
    
    private File mAutoFile;
    private File[] mAutoFiles;
    private File mAutoDir = new File("home/lvuser/autoRecordings");
    private static SendableChooser<File> mAutoChooser = new SendableChooser<File>();
    private ArrayList<String> mAutoPath = new ArrayList<String>();
    
    public Recorder(){
        try {
            String[] fileNames = mAutoDir.list();
            mAutoFiles = new File[fileNames.length];
            if (fileNames.length > 0) {
                for (int i = 0; i < fileNames.length; i++) {
                    mAutoFiles[i] = new File(mAutoDir.getPath(), fileNames[i]);
                    mAutoChooser.addOption(fileNames[i], mAutoFiles[i]);
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
    }
    
    public void recordInputs(double throttle, double strafe, double rotationX, double rotationY, boolean fieldCentricRotation, double timestamp) {
        mAutoPath.add(timestamp+":[t="+throttle+"]"+"[s="+strafe+"]"+"[rx="+rotationX+"]"+"[ry="+rotationY+"]"+"fc="+fieldCentricRotation+"] ");
    }
    
    public void stopRecording() {
        String[] data = new String[mAutoPath.size()];
        data = mAutoPath.toArray(data);
        writeData(data, mAutoFile);
    }
    
    private Action[] autoActions;
    
    public Action getSwerveStateAtTimestamp(double timestamp) {
        if (autoActions == null) loadFromFile();
        int closestIndex = 0;
        double difference = 999999999; //INFINITY
        for(int i = 0; i < autoActions.length; i++) {
            if(timestamp - autoActions[i].getTimestamp() < difference){
                difference = timestamp - autoActions[i].getTimestamp();
                if(difference < 0){
                    break;
                }
                closestIndex = i;
            }
        }
        return autoActions[closestIndex];
    }
    
    private void loadFromFile() {
        String[] data = readData(mAutoChooser.getSelected());
        autoActions = new Action[data.length];
        for(int i = 0; i < data.length; i++){
            double throttle = Double.parseDouble(data[i].substring(data[i].indexOf("t=")+2,data[i].indexOf("][s")));
            double strafe = Double.parseDouble(data[i].substring(data[i].indexOf("s=")+2,data[i].indexOf("][rx")));
            double rotationX = Double.parseDouble(data[i].substring(data[i].indexOf("rx=")+3,data[i].indexOf("][ry")));
            double rotationY = Double.parseDouble(data[i].substring(data[i].indexOf("ry=")+3,data[i].indexOf("][fc")));
            double timestamp = Double.parseDouble(data[i].substring(0, data[i].indexOf(":")));
            Boolean fieldCentricRotation = Boolean.parseBoolean(data[i].substring(data[i].indexOf("fc=")+3,data[i].indexOf("] ")));
            autoActions[i] = new Action(throttle, strafe, rotationX, rotationY, fieldCentricRotation);
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
    
    private void writeData(String data, File file) {
        try  {
            FileWriter writer = new FileWriter(file);
            writer.write(data);
            writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    private void writeData(String[] dataArr, File file) {
        String data = "";
        for (int i = 0; i < dataArr.length; i++) {
            data += (dataArr[i] + "\n");
        }
        writeData(data, file);
    }
}