package shapeVar;

import shapes.Keepout;
import shapes.Slave;

import java.util.HashMap;
import java.util.Map;

public class Slave_var extends Slave {


    /*
     * binary pseudo variables:
     * 0: L
     * 1: R
     * 2: A
     * 3: B
     * 4: Ld
     * 5: Rd
     * 6: Ad
     * 7: Bd
     */
    private final Map<Keepout, int[]> pseudo_bVars;

    /*
     * pseudo_iVars
     * 0: so_x_ll: |sx - ox_ll|
     * 1: so_y_ll: |sy - oy_ll|
     * 2: so_x_ur: |sx - ox_ur|
     * 3: so_y_ur: |sy - oy_ur|
     */
    private final Map<Keepout, int[]> pseudo_iVars;

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

    public void addToPseudo_iVars(Keepout k, int[] ints) {
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
