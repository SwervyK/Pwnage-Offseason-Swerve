package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;


/**
 * takes the buffer that would normaly go to one Job and splits it in oder to give it to two or more
 */
public class SplitJobContainer implements Job {
    private ArrayList<Job> jobs;
    private ArrayList<Float> splitPercentages;

    /**
     * contains jobs so that they can share the same strip
     * @param jobs the contained jobs
     * @param splitPercentages the pixel number (reletive to the container) to start the next job
     */
    public SplitJobContainer(Job[] jobs, float[] splitPercentages) {
        this.jobs = new ArrayList<Job>(jobs.length);
        this.splitPercentages = new ArrayList<Float>(splitPercentages.length);

        for(float i : splitPercentages )
            this.splitPercentages.add(i);
        for(Job j : jobs )
            this.jobs.add(j);
    }

    /**
     * {@inheritDoc}
     */
    public void exec(double timestamp, ArrayList<Color> buffer) {
        ArrayList<ArrayList<Color>> buffers = getBuffers(buffer.size());

        for(int c = 0; c < jobs.size() && c < buffers.size(); c++)
            if(jobs.get(c) != null)
                jobs.get(c).exec(timestamp, buffers.get(c));

        for(int c = 0, index = 0, previousSize = 0; c < buffer.size(); c++) {
            if(c-previousSize >= buffers.get(index).size()) {
                previousSize = c;
                index++;
            }
            buffer.set(c, buffers.get(index).get(c-previousSize));
        }
    }

    protected ArrayList<ArrayList<Color>> getBuffers(int bufferSize) {
        int buffersInitCap = splitPercentages.size()+1;
        ArrayList<ArrayList<Color>> buffers = new ArrayList<>(buffersInitCap);

        for(int c = 0; c < buffersInitCap; c++)
        {
            int tmpInitCap;
            if(c == 0)
                tmpInitCap = (int)(splitPercentages.get(0)*(double)bufferSize);
            else if(c == buffersInitCap - 1)
                tmpInitCap = bufferSize - (int)(splitPercentages.get(splitPercentages.size()-1)*(double)bufferSize);
            else
                tmpInitCap = (int)(splitPercentages.get(c)*(double)bufferSize - splitPercentages.get(c-1)*(double)bufferSize);

            buffers.add(new ArrayList<Color>(tmpInitCap));
            for(int x = 0; x < tmpInitCap; x++)
                buffers.get(c).add(null);
        }

        return buffers;
    }

    /**
     * {@inheritDoc}
     */
    public int getIterationCount() {
        int i = 0;
        for(Job j : jobs)
            i += j.getIterationCount();
        return i;
    }

    /**
     * {@inheritDoc}
     */
    public void expire(ArrayList<Color> buffer) { for(Job j : jobs) j.expire(buffer); }

    /**
     * {@inheritDoc}
     */
    public boolean isExpired() {
        for(int c = 0; c < jobs.size(); c++)
            if(jobs.get(c).isExpired()) {
                return true;
            }
        return (jobs.size() == 0);
    }
    
    /**
     * {@inheritDoc}
     */
    public void reset() { for(Job j : jobs ) j.reset(); }

    /**
     * removes all expired jobs from container
     */
    public void removeExpiredJobs() {
        for(int c = 0; c < jobs.size(); c++)
            if(jobs.get(c).isExpired()) {
                jobs.remove(c);
                splitPercentages.remove((c == 0) ? 0 : c-1);
            }
    }
}
