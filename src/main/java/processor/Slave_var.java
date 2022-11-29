package processor;

import shapes.Keepout;
import shapes.Slave;

import java.util.HashMap;
import java.util.Map;

public class Slave_var extends Slave {


    /*
     * binary pseudo variables:
     * 0: Left
     * 1: Right
     * 2: Above
     * 3: Below
     */
    public Map<Keepout, int[]> pseudo_bVars;

    /*
     * pseudo_iVars
     * 0: |s_j.x - c^k_1.x|
     * 1: |s_j.y - c^k_1.y|
     * 2: |s_j.x - c^k_2.x|
     * 3: |s_j.y - c^k_2.y|
     */
    public Map<Keepout, int[]> pseudo_iVars;

    /*
     *
     */



    public Slave_var(double x_ct, double y_ct) {
        super(x_ct, y_ct);
        this.pseudo_bVars = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();


    }

    public void addToPseudo_bVars(Keepout k, int[] ints) {
        this.pseudo_bVars.put(k,ints);
    }

    public void addToPsedo_iVars(Keepout k, int[] ints) {
        this.pseudo_iVars.put(k,ints);
    }

    public Map<Keepout, int[]> getPseudo_bVars() {
        return pseudo_bVars;
    }

    public Map<Keepout, int[]> getPseudo_iVars() {
        return pseudo_iVars;
    }


    @Override
    public String toString() {
        return "Slave_var{" +
                "pseudo_bVars=" + pseudo_bVars.size() +
                ", pseudo_iVars=" + pseudo_iVars +
                ", x_ct=" + x_ct +
                ", y_ct=" + y_ct +
                ", name='" + name +
                '}';
    }
}
