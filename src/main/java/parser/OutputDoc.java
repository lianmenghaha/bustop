package parser;

import shapes.Keepout;
import shapes.Master;
import shapes.Slave;
import shapes.VirtualPoint;

import java.awt.*;
import java.util.ArrayList;

public class OutputDoc {

    private String I2Cname;
    private ArrayList<Keepout> keepouts;
    private Master master;
    private ArrayList<VirtualPoint> virtualPoints;
    private ArrayList<Slave> slaves;
    private int BusLength;
    private int SideBusLength;

    public OutputDoc(String i2Cname, ArrayList<Keepout> keepouts, Master master, ArrayList<Slave> slaves) {
        I2Cname = i2Cname;
        this.keepouts = keepouts;
        this.master = master;
        this.slaves = slaves;
        this.virtualPoints = new ArrayList<>();
    }

    public ArrayList<VirtualPoint> getVirtualPoints() {
        return virtualPoints;
    }

    public void addToVirtualPoints(VirtualPoint vp) {
        this.virtualPoints.add(vp);
    }

    public ArrayList<Slave> getSlaves() {
        return slaves;
    }

    public ArrayList<Keepout> getKeepouts() {
        return keepouts;
    }

    public Master getMaster() {
        return master;
    }

    public String getI2Cname() {
        return I2Cname;
    }

    public int getBusLength() {
        return BusLength;
    }

    public void setBusLength(int busLength) {
        BusLength = busLength;
    }

    public int getSideBusLength() {
        return SideBusLength;
    }

    public void setSideBusLength(int sideBusLength) {
        SideBusLength = sideBusLength;
    }
}
