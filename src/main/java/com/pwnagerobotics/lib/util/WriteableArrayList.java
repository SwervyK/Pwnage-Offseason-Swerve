package com.pwnagerobotics.lib.util;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.util.CSVWritable;

public class WriteableArrayList<T> extends ArrayList<T> implements CSVWritable {

    private static final long serialVersionUID = 5061474551535766793L;

    public WriteableArrayList(List<T> list) {
        super(list);
    }

    public WriteableArrayList(int initialSize){
        super(initialSize);
    }

    @Override
    public String toCSV() {

        StringBuilder sb = new StringBuilder();
        for (T element : this)
        {
            sb.append(element.toString());
            sb.append("|");
        }

        return sb.toString();
    }

}