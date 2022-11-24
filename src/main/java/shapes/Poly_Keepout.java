package shapes;

import java.util.ArrayList;

public class Poly_Keepout extends Keepout{

    ArrayList<Keepout> inner_Keepouts_V;
    ArrayList<Keepout> inner_Keepouts_H;

    public Poly_Keepout(int minX, int maxX, int minY, int maxY) {
        super(minX, maxX, minY, maxY);//外框
        this.inner_Keepouts_V = new ArrayList<>();
        this.inner_Keepouts_H = new ArrayList<>();

    }

    public ArrayList<Keepout> getInner_Keepouts_V() {
        return inner_Keepouts_V;
    }

    public void setInner_Keepouts_V(ArrayList<Keepout> inner_Keepouts_V) {
        this.inner_Keepouts_V = inner_Keepouts_V;
    }

    public ArrayList<Keepout> getInner_Keepouts_H() {
        return inner_Keepouts_H;
    }

    public void setInner_Keepouts_H(ArrayList<Keepout> inner_Keepouts_H) {
        this.inner_Keepouts_H = inner_Keepouts_H;
    }
}
