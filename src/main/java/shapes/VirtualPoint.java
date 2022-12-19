package shapes;

import java.util.ArrayList;

public class VirtualPoint extends Shape{
    private final ShapeType type = ShapeType.VirtualPoint;
    private Slave slave;
    private ArrayList<ObObC> rel_sl_Obs;
    private ArrayList<ObObC> rel_vp_Obs;
    private ArrayList<ObObC> rel_mv_Obs;
    private int dis_vp;
    private int dis_sl;


    public VirtualPoint(double x_ct, double y_ct, String name) {
        super();
        this.x_ct = x_ct;
        this.y_ct = y_ct;
        this.name = name;
        this.rel_sl_Obs = new ArrayList<>();
        this.rel_vp_Obs = new ArrayList<>();
        this.rel_mv_Obs = new ArrayList<>();
    }

    public Slave getSlave() {
        return slave;
    }

    public void setSlave(Slave slave) {
        this.slave = slave;
    }

    public ArrayList<ObObC> getRel_sl_Obs() {
        return rel_sl_Obs;
    }

    public void addToRel_sl_Obs(ObObC rel_sl_Ob) {
        this.rel_sl_Obs.add(rel_sl_Ob);
    }

    public ArrayList<ObObC> getRel_vp_Obs() {
        return rel_vp_Obs;
    }

    public void addToRel_vp_Obs(ObObC rel_vp_Ob) {
        this.rel_vp_Obs.add(rel_vp_Ob);
    }

    public ArrayList<ObObC> getRel_mv_Obs() {
        return rel_mv_Obs;
    }

    public void addToRel_mv_Obs(ObObC rel_mv_Ob) {
        this.rel_mv_Obs.add(rel_mv_Ob);
    }

    public int getDis_sl() {
        return dis_sl;
    }

    public void setDis_sl(int dis_sl) {
        this.dis_sl = dis_sl;
    }

    public int getDis_vp() {
        return dis_vp;
    }

    public void setDis_vp(int dis_vp) {
        this.dis_vp = dis_vp;
    }

    @Override
    public String toString() {
        return "VirtualPoint{" +
                "x_ct=" + x_ct +
                ", y_ct=" + y_ct +
                ", slave=" + slave +
                ", rel_sl_Obs=" + rel_sl_Obs +
                ", rel_vp_Obs=" + rel_vp_Obs +
                '}';
    }
}
