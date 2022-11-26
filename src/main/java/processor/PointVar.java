/*
 * Copyright (c) 2021. Meng Lian and Yushen Zhang
 *
 *   CONFIDENTIAL
 *   __________________
 *
 *   Meng Lian and Yushen Zhang
 *   All Rights Reserved.
 *
 *   NOTICE:  All information contained herein is, and remains
 *   the property of Meng Lian, Yushen Zhang and the Technical University
 *   of Munich, if applies. The intellectual and technical concepts contained
 *   herein are proprietary to Meng Lian, Yushen Zhang and/or the Technical University
 *   of Munich and may be covered by European and Foreign Patents,
 *   patents in process, and are protected by trade secret or copyright law.
 *   Dissemination of this information or reproduction of this material
 *   is strictly forbidden unless prior written permission is obtained
 *   from Meng Lian and Yushen Zhang.
 */

package processor;

import grb.GurobiExecutor;
import grb.GurobiVariable;
import shapes.Shape;

import java.util.HashMap;
import java.util.Map;

public class PointVar {

    public GurobiVariable x;//x-coordinate of v_point
    public GurobiVariable y;//y-coordinate of v_point
    public GurobiVariable distI;//distance between this and the next v_point
    public GurobiVariable distS;//distance between v_point and the corr. slave



    public Map<Shape,GurobiVariable> s_qVars;

    public GurobiVariable distM_x;//|delta_Mx_Var|
    public GurobiVariable distM_y;//|delta_My_Var|

    public GurobiVariable distI_x;//|delta_Ix_Var|
    public GurobiVariable distI_y;//|delta_Iy_Var|
    public Map<Shape,GurobiVariable> distS_xs;//|delta_Sx_Var|
    public Map<Shape,GurobiVariable> distS_ys;//|delta_Sy_Var|
    public Map<Shape,GurobiVariable> distSs;//|delta_Sx_Var| + |delta_Sy_Var|


    public PointVar() {

        this.s_qVars = new HashMap<>();
        this.distS_xs = new HashMap<>();
        this.distS_ys = new HashMap<>();
        this.distSs = new HashMap<>();

    }

    /**
     * This function will accumulate the given {@link GurobiVariable} object, and auto add the corresponding GRBVar to the passed {@link GurobiExecutor} object.
     * @param x_Var
     * @param exe The {@link GurobiExecutor} used for this optimization.
     */
    public void setX_Var(GurobiVariable x_Var, GurobiExecutor exe) {
        this.x = x_Var;
        exe.addVariable(x_Var);
    }

    /**
     * This function will accumulate the given {@link GurobiVariable} object, and auto add the corresponding GRBVar to the passed {@link GurobiExecutor} object.
     * @param y_Var
     * @param exe The {@link GurobiExecutor} used for this optimization.
     */
    public void setY_Var(GurobiVariable y_Var, GurobiExecutor exe) {
        this.y = y_Var;
        exe.addVariable(y_Var);
    }


    /**
     * This function will accumulate the given {@link GurobiVariable} object, and auto add the corresponding GRBVar to the passed {@link GurobiExecutor} object.
     * @param dist
     * @param exe The {@link GurobiExecutor} used for this optimization.
     */
    public void setDistI(GurobiVariable dist, GurobiExecutor exe) {
        this.distI = dist;
        exe.addVariable(dist);
    }

    public void setDistS(GurobiVariable dist, GurobiExecutor exe) {
        this.distS = dist;
        exe.addVariable(dist);
    }

    public void setDistI_x(GurobiVariable distI_x, GurobiExecutor exe) {
        this.distI_x = distI_x;
        exe.addVariable(distI_x);
    }

    public void setDistI_y(GurobiVariable distI_y, GurobiExecutor exe) {
        this.distI_y = distI_y;
        exe.addVariable(distI_y);
    }

    public void setDistM_x(GurobiVariable distM_x, GurobiExecutor exe) {
        this.distM_x = distM_x;
        exe.addVariable(distM_x);
    }

    public void setDistM_y(GurobiVariable distM_y, GurobiExecutor exe) {
        this.distM_y = distM_y;
        exe.addVariable(distM_y);
    }





    /**
     * This function will accumulate the given {@link GurobiVariable} object mapped with the {@link Shape}, and auto add the corresponding GRBVar to the passed {@link GurobiExecutor} object.
     * @param s
     * @param v
     * @param exe The {@link GurobiExecutor} used for this optimization.
     */
    public void addToSlave_Vars (Shape s, GurobiVariable v, GurobiExecutor exe){
        this.s_qVars.put(s,v);
        exe.addVariable(v);
    }

    /*public void addToDelta_Sx_Vars (Shape s, GurobiVariable v, GurobiExecutor exe){
        this.delta_Sx_Vars.put(s,v);
        exe.addVariable(v);
    }

    public void addToDelta_Sy_Vars (Shape s, GurobiVariable v, GurobiExecutor exe){
        this.delta_Sy_Vars.put(s,v);
        exe.addVariable(v);
    }*/

    public void addToDistS_xs (Shape s, GurobiVariable v, GurobiExecutor exe){
        this.distS_xs.put(s,v);
        exe.addVariable(v);
    }

    public void addToDistS_ys (Shape s, GurobiVariable v, GurobiExecutor exe){
        this.distS_ys.put(s,v);
        exe.addVariable(v);
    }

    public void addToDistSs (Shape s, GurobiVariable v, GurobiExecutor exe){
        this.distSs.put(s,v);
        exe.addVariable(v);
    }


}
