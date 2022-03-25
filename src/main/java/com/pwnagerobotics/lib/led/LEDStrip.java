package com.pwnagerobotics.lib.led;

import java.util.ArrayList;

import com.pwnagerobotics.lib.led.jobs.Job;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.pwnagerobotics.lib.util.Range;

public class LEDStrip
{

    public static class LEDStripConstants {
        public int kLedPwmPort = 0;
        public int kLedStripLength = 0;
    }

    //public static AddressableLED mLed = new AddressableLED(Constants.kLedPwmPort);
    private static AddressableLED mLed;
    //public static AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.kLedStripLength);
    private static AddressableLEDBuffer buffer;
    protected final LEDStripConstants mConstants;

    private Job mJob;
    private final Range[] mRanges;

    public LEDStrip(final LEDStripConstants constants, Range[] ranges) {
        mConstants = constants;
        if (mLed == null && buffer == null) {
            mLed =  new AddressableLED(mConstants.kLedPwmPort);
            buffer = new AddressableLEDBuffer(mConstants.kLedStripLength);
        }
        mLed.setLength(buffer.getLength());
        mLed.start();
        mRanges = ranges;
    }

    public LEDStrip(final LEDStripConstants constants, int start, int end) {
        this(constants, new Range[] {new Range(start, end)});
    }

    private ArrayList<ArrayList<Color>> getBuffer() {
        ArrayList<ArrayList<Color>> buffer = new ArrayList<>(mRanges.length);
        ArrayList<Integer> sizes = new ArrayList<>(mRanges.length);
        //Set the default state of the buffer. 
        for(Range r : mRanges) {
            if(!sizes.contains(r.size())) {
                ArrayList<Color> newBuffer = new ArrayList<Color>(r.size());
                for (int i = 0; i < r.size(); i++)
                    newBuffer.add(null);

                buffer.add(newBuffer);
                sizes.add(r.size());
            }
        }
        return buffer;
    }

    public synchronized static void flush() {
        mLed.setData(buffer);
    }

    public synchronized void exec() {
        if(mJob == null || mJob.isExpired()) return;

        ArrayList<ArrayList<Color>> newBuffer = getBuffer();
        double timestamp = Timer.getFPGATimestamp();

        for(int i = 0; i < newBuffer.size(); i++)
            mJob.exec(timestamp, newBuffer.get(i));

        for(Range r : mRanges ) {
            ArrayList<Color> b = new ArrayList<>(0);
            for(ArrayList<Color> a : newBuffer )
                if((b = a).size() == r.size())
                    break;

            //Put the results into the buffer.
            for(int c = 0; c < b.size(); c++) {
                if(b.get(c) != null) {
                    Color8Bit color = new Color8Bit(b.get(c));
                    buffer.setRGB(r.reversed ? r.end-(c+1) : c+r.start, color.red, color.green, color.blue);
                }
            }
        }
    }

    public synchronized void setJob(Job newJob) {
        mJob = newJob;
    }
}
