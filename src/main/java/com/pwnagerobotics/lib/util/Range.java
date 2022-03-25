package com.pwnagerobotics.lib.util;

public class Range
{
    public final int start;
    public final int end;
    public final boolean reversed;

    /**
     * constructs a range using 2 ends in any order
     * @param num1 one end of the range
     * @param num2 the other end
     */
    public Range(int num1, int num2)
    {
        if(num1 < num2){
            start = num1;
            end = num2;
            reversed = false;
        } else {
            start = num2;
            end = num1;
            reversed = true;
        }
    }

    /**
     * @param range the 2nd range to compair to
     * @return whether the 2 ranges overlap or not
     */
    public boolean hasIntersection(Range range) { return (getIntersection(range) != null); }

    /**
     * @param range the 2nd range to compair to
     * @return the overlap in the ranges
     */
    public Range getIntersection(Range range)
    {
        int s = Integer.MIN_VALUE;
        int e = Integer.MAX_VALUE;

        if(isInRange(range.start))
            s = range.start;
        else if(range.isInRange(start))
            s = start;

        if(isInRange(range.end))
            e = range.end;
        else if(range.isInRange(end))
            e = end;

        if(s != Integer.MIN_VALUE && e != Integer.MAX_VALUE)
            return new Range(s, e);
        else
            return null;
    }

    /**
     * @return whether the value is in this range
     */
    public boolean isInRange(int x)
    {
        if(x <= end && x >= start)
            return true;
        else
            return false;
    }

    /**
     * @return the number of values in this ranges domain
     */
    public int size() { return end - start; }
}
