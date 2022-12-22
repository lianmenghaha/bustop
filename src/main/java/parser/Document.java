package parser;

import shapes.Keepout;
import shapes.Master;
import shapes.Poly_Keepout;
import shapes.Slave;

import java.util.ArrayList;

public class Document {
    private String name;
    private Master master;
    private ArrayList<Slave> slaves;
    private ArrayList<Keepout> uni_keepouts;
    private ArrayList<Poly_Keepout> poly_keepouts;
    private int busC;
    private int slaveC;

    public Document() {
        this.slaves = new ArrayList<>();
        this.uni_keepouts = new ArrayList<>();
        this.poly_keepouts = new ArrayList<>();
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Master getMaster() {
        return master;
    }

    public void setMaster(Master master) {
        this.master = master;
    }

    public ArrayList<Slave> getSlaves() {
        return slaves;
    }

    public void addToSlaves(Slave slave) {
        this.slaves.add(slave);
    }

    public ArrayList<Keepout> getUni_keepouts() {
        return uni_keepouts;
    }

    public void addToUni_keepouts(Keepout o) {
        this.uni_keepouts.add(o);
    }

    public int getBusC() {
        return busC;
    }

    public void setBusC(int busC) {
        this.busC = busC;
    }

    public int getSlaveC() {
        return slaveC;
    }

    public void setSlaveC(int slaveC) {
        this.slaveC = slaveC;
    }

    public ArrayList<Poly_Keepout> getPoly_keepouts() {
        return poly_keepouts;
    }

    public void addtOPoly_keepouts(Poly_Keepout poly_o) {
        this.poly_keepouts.add(poly_o);
    }
}
