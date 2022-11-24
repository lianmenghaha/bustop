package shapes;

import java.util.ArrayList;

public class Value_S {

    /**
     * The value is determined by
     * d[I][J] + (S[E,J] + S[D-E, J])
     */
    private ArrayList<Integer> firstArg_Array;
    private int secondArg_Int;
    private double value;
    private int J_Num;
    private ArrayList<Integer> D_E;
    private ArrayList<Integer> D_without_E;


    public Value_S() {
        this.firstArg_Array = new ArrayList<>();
        this.value = Double.MAX_VALUE;
        this.J_Num = 0;
        this.D_E = new ArrayList<>();
        this.D_without_E = new ArrayList<>();
    }

    public ArrayList<Integer> getD_E() {
        return D_E;
    }

    public void setD_E(ArrayList<Integer> d_E) {
        D_E = d_E;
    }

    public ArrayList<Integer> getD_without_E() {
        return D_without_E;
    }

    public void setD_without_E(ArrayList<Integer> d_without_E) {
        D_without_E = d_without_E;
    }

    public int getJ_Num() {
        return J_Num;
    }

    public void setJ_Num(int j_Num) {
        J_Num = j_Num;
    }

    public ArrayList<Integer> getFirstArg_Array() {
        return firstArg_Array;
    }

    public void setFirstArg_Array(ArrayList<Integer> firstArg_Array) {
        this.firstArg_Array = firstArg_Array;
    }

    public void addToSetD(int d){
        firstArg_Array.add(d);
    }

    public int getSecondArg_Int() {
        return secondArg_Int;
    }

    public void setSecondArg_Int(int secondArg_Int) {
        this.secondArg_Int = secondArg_Int;
    }

    public double getValue() {
        return value;
    }

    public void setValue(double value) {
        this.value = value;
    }

    @Override
    public String toString() {
        return "S{" +
                "firstArg_Array=" + firstArg_Array +
                ", secondArg_Int=" + secondArg_Int +
                ", value=" + value +
                ", J_Num=" + J_Num +
                ", D_E=" + D_E +
                ", D_without_E=" + D_without_E +
                '}';
    }
}
