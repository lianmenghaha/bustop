package processor;

import grb.*;
import gurobi.GRB;
import gurobi.GRBException;
import shapes.*;

import java.math.BigDecimal;
import java.util.*;

public class Processor {

    private GurobiExecutor executor;
    private final static int M = 999999;

    /**
     * The ILP Model to solve I2C connection without Keepout
     * The coordinates are Int Type
     * use Math.rund()
     *
     * @param master MasterIC
     * @param slaves ArrayList of slaves
     * @param busC   Coefficient for busLength
     * @param slaveC Coefficient for sideBusLength
     * @throws GRBException GRBException
     */
    public void processToOutput_wo_KO(Master master, ArrayList<Slave> slaves, double busC, double slaveC) throws GRBException {

        int s_cnt = slaves.size();

        //initialize Objects
        executor = new GurobiExecutor("LinearBusRouting");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);
        //executor.setNonConvex(2);


        /*
         * setLBR_Vars
         */
        ArrayList<PointVar> pointVars = new ArrayList<>();
        buildVars_wo_KO(slaves, s_cnt, pointVars);

        /*
         * build global GurobiVariables
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
         */
        GurobiVariable busLength, sideBusLength;
        busLength = new GurobiVariable(GRB.INTEGER, 0, M, "bus_length");
        executor.addVariable(busLength);
        sideBusLength = new GurobiVariable(GRB.INTEGER, 0, M, "side_bus_length");
        executor.addVariable(sideBusLength);
        /*
         * AAAAAAAAAAAAAAAAAAAAAAAAAAAA
         * build global GurobiVariables
         */

        executor.updateModelWithVars();

        /*
         * addConsToLBR
         */
        //addConsToLBR_ABS(master, slaves, s_cnt, pointVars);
        buildCons_wo_KO(master, slaves, s_cnt, pointVars, busLength, sideBusLength);
        executor.updateModelWithCons();
        //System.out.println(executor.getConstraintByIndex(5));

        GurobiObjConstraint objCons = new GurobiObjConstraint();
        setObjCons(busLength, sideBusLength, objCons, busC, slaveC);
        executor.setObjConstraint(objCons);
        System.out.println(executor.getObjConstraint());

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        if (executor.getModel().get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
            executor.whenInfeasibleWriteOutputTo("inf.ilp");
            System.out.println("HH:IIS");
            //break;
        }

        /*
         * Retrieve from Gurobi
         */
        ArrayList<Point> iPoints = new ArrayList<>();
        //System.out.println("distM_x = " + pvF.distM_x.getContResult() + " distM_y = " + pvF.distM_y.getContResult());
        int cnt = 0;
        for (PointVar pv : pointVars) {
            ++cnt;
            System.out.println("IP" + cnt);
            Point ip = new Point(pv.x.getIntResult(), pv.y.getIntResult());
            System.out.println("(" + ip.getX_exact() + " , " + ip.getY_exact() + ")");
            ip.setLength_I(pv.distI.getContResult());
            //System.out.println("distI = " + pv.distI.getContResult());
            //System.out.println("distS = " + pv.distS.getContResult());
            for (Slave s : slaves) {
                if (pv.s_qVars.get(s).getIntResult() == 1) {
                    ip.setSlave(s);
                    System.out.println(s.getName() + " " + s.getX_ct() + " " + s.getY_ct());
                    ip.setLength_S(pv.distSs.get(s).getIntResult());

                    break;
                }
            }

            iPoints.add(ip);

        }


    }

    /**
     * The QP Model to solve I2C connection with Keepout
     * The coordinates are Int Type
     * Absolute values are presented by GRB.GenConstraints
     *
     * @param master MasterIC
     * @param slaves ArrayList of slaves
     * @throws GRBException GRBException
     */

    public void processToOutput_w_multiKO(Master master, ArrayList<Slave> slaves, ArrayList<Keepout> uni_keepouts, ArrayList<Poly_Keepout> poly_keepouts, double busC, double slaveC) throws GRBException {

        ArrayList<Keepout> all_keepouts = new ArrayList<>();
        all_keepouts.addAll(uni_keepouts);
        all_keepouts.addAll(poly_keepouts);
        update_LRAB_keepouts(uni_keepouts);
        System.out.println(all_keepouts);


        int s_cnt = slaves.size();

        //initialize Objects
        executor = new GurobiExecutor("LinearBusRouting_KO");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);
        //executor.setNonConvex(2);


        /*
         * setLBR_Vars
         */
        ArrayList<VP_var> VPvars = new ArrayList<>();
        Master_var mv = new Master_var(master.getX_ct(), master.getY_ct());
        ArrayList<Slave_var> slaveVars_ko = new ArrayList<>();


        //buildVars
        buildVars_w_multiKO(mv, slaves, slaveVars_ko, s_cnt, VPvars, uni_keepouts, poly_keepouts);
        //build global GurobiVariables
        GurobiVariable busLength, sideBusLength;
        busLength = new GurobiVariable(GRB.INTEGER, 0, M, "bus_length");
        executor.addVariable(busLength);
        sideBusLength = new GurobiVariable(GRB.INTEGER, 0, M, "side_bus_length");
        executor.addVariable(sideBusLength);
        //update Model with variables
        executor.updateModelWithVars();

        /*
         * addConsToLBR
         */
        buildCons_w_multiKO(mv, slaveVars_ko, VPvars, uni_keepouts, poly_keepouts, all_keepouts, busLength, sideBusLength);

        executor.updateModelWithCons();
//        System.out.println("808" + executor.getConstraintByIndex(808));
//        System.out.println("818" + executor.getConstraintByIndex(818));
//        System.out.println("963" + executor.getConstraintByIndex(963));
//        System.out.println("964" + executor.getConstraintByIndex(964));
        /*System.out.println(executor.getConstraintByIndex(23));*/


        GurobiObjConstraint objCons = new GurobiObjConstraint();
        setObjCons(busLength, sideBusLength, objCons, busC, slaveC);


        executor.setObjConstraint(objCons);
        //System.out.println(executor.getObjConstraint());

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        if (executor.getModel().get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
            executor.whenInfeasibleWriteOutputTo("inf.ilp");
            System.out.println("HH:IIS");
            //break;
        }

        /*
         * Retrieve from Gurobi
         */

        int cnt = 0;
        for (VP_var vp : VPvars) {
            ++cnt;
            System.out.println("sp_" + cnt);
            System.out.println("(" + vp.x.getIntResult() + " , " + vp.y.getIntResult() + ")");
            GurobiVariable[] sp_iq = vp.vp_iVars_abs;
            for (int i = 0; i < sp_iq.length; ++i) {
                System.out.print("sp_iq_" + i + "=" + sp_iq[i].getIntResult() + ";");
            }
            System.out.println();
            GurobiVariable[] sp_q = vp.vp_bVars;
            for (int i = 0; i < sp_q.length; ++i) {
                System.out.print("sp_q_" + i + "=" + sp_q[i].getIntResult() + ";");
            }
            System.out.println();

            if (cnt == 1) {
                GurobiVariable[] ms_sp_q = vp.ms_vp_bVars;
                GurobiVariable[] ms_sp_iq = vp.ms_vp_iVars;
                for (int i = 0; i < ms_sp_q.length; ++i) {
                    System.out.print("ms_sp_q_" + i + "=" + ms_sp_q[i].getIntResult() + ";");
                }
                System.out.println();
                for (int i = 0; i < ms_sp_iq.length; ++i) {
                    System.out.print("ms_sp_iq_" + i + "=" + ms_sp_iq[i].getIntResult() + ";");
                }
                System.out.println();
                for (Keepout k : all_keepouts) {
                    System.out.println(k);
                    GurobiVariable[] ms_ko_sp_q = vp.ms_ko_vp_bVars.get(k);
                    GurobiVariable[] ms_ko_sp_iq = vp.ms_ko_vp_iVars.get(k);
                    for (int i = 0; i < ms_ko_sp_q.length; ++i) {
                        System.out.print("ms_ko_sp_q_" + i + "=" + ms_ko_sp_q[i].getIntResult() + ";");
                    }
                    System.out.println();
                    for (int i = 0; i < ms_ko_sp_iq.length; ++i) {
                        System.out.print("ms_ko_sp_iq_" + i + "=" + ms_ko_sp_iq[i].getIntResult() + ";");
                    }
                    System.out.println();
                }
            }
            for (Slave_var sv : slaveVars_ko) {
                if (vp.sl_bVars.get(sv)[5].getIntResult() == 1) {
                    System.out.println(sv.getName() + " " + sv.getX_ct() + " " + sv.getY_ct());
                    GurobiVariable[] sl_q = vp.sl_bVars.get(sv);
                    for (int i = 0; i < sl_q.length; ++i) {
                        System.out.print("sl_q_" + i + "=" + sl_q[i].getIntResult() + ";");
                    }
                    System.out.println();
                    GurobiVariable[] sl_iq = vp.sl_iVars_abs.get(sv);
                    for (int i = 0; i < sl_iq.length; ++i) {
                        System.out.print("sl_iq_" + i + "=" + sl_iq[i].getIntResult() + ";");
                    }
                    System.out.println();
                    for (Keepout k : all_keepouts) {
                        System.out.println(Arrays.toString(sv.getPseudo_bVars().get(k)));
                        System.out.println(Arrays.toString(sv.getPseudo_iVars().get(k)));
                        GurobiVariable[] ko_sl_q = vp.ko_sl_bVars.get(k).get(sv);
                        for (int i = 0; i < ko_sl_q.length; ++i) {
                            System.out.print("ko_sl_q" + i + "=" + ko_sl_q[i].getIntResult() + ";");
                        }
                        System.out.println();
                    }
                }
            }
            for (Keepout k : all_keepouts) {
                System.out.println(k);
                GurobiVariable[] ko_sp_q = vp.ko_vp_bVars.get(k);
                for (int i = 0; i < ko_sp_q.length; ++i) {
                    System.out.print("ko_sp_q_" + i + "=" + ko_sp_q[i].getIntResult() + ";");
                }
                System.out.println();
                GurobiVariable[] ko_sp_iq = vp.ko_vp_iVars_abs.get(k);
                for (int i = 0; i < ko_sp_iq.length; ++i) {
                    System.out.print("ko_sp_iVars_" + i + "= " + ko_sp_iq[i].getIntResult() + ";");
                }
                System.out.println();
            }


        }
        //debug
//        gurobi.GRBVar[] vars = executor.getModel().getVars();
//        for (int i = 0; i < vars.length; ++i) {
//            double vars_i = vars[i].get(GRB.DoubleAttr.X);
//            //System.out.println(vars[i].get(GRB.StringAttr.VarName));
//            System.out.print("var[" + i + "]= " + vars_i + "; lb= " + vars[i].get(GRB.DoubleAttr.LB) + "; rb= " + vars[i].get(GRB.DoubleAttr.UB));
//            System.out.println();
//        }


    }

    private void update_LRAB_keepouts(ArrayList<Keepout> uni_keepouts) {

        /*
        initialize oL, oR, oA, oB
         */
        for (Keepout current_k : uni_keepouts) {
            for (Keepout other_k : uni_keepouts) {
                if (uni_keepouts.indexOf(current_k) != uni_keepouts.indexOf(other_k)) {
                    //oL
                    if (other_k.getMaxX() <= current_k.getMinX() && ((other_k.getMinY() < current_k.getMinY() && current_k.getMinY() < other_k.getMaxY()) || (other_k.getMinY() < current_k.getMaxY() && current_k.getMaxY() < other_k.getMaxY()))) {
                        current_k.addToLeft_os(other_k);
                    }
                    //oR
                    if (other_k.getMinX() >= current_k.getMaxX() && ((other_k.getMinY() < current_k.getMinY() && current_k.getMinY() < other_k.getMaxY()) || (other_k.getMinY() < current_k.getMaxY() && current_k.getMaxY() < other_k.getMaxY()))) {
                        current_k.addToRight_os(other_k);
                    }

                    //oA
                    if (other_k.getMinY() >= current_k.getMaxY() && ((other_k.getMinX() < current_k.getMinX() && current_k.getMinX() < other_k.getMaxX()) || (other_k.getMinX() < current_k.getMaxX() && current_k.getMaxX() < other_k.getMaxX()))) {
                        current_k.addToAbove_os(other_k);
                    }
                    //oB
                    if (other_k.getMaxY() <= current_k.getMinY() && ((other_k.getMinX() < current_k.getMinX() && current_k.getMinX() < other_k.getMaxX()) || (other_k.getMinX() < current_k.getMaxX() && current_k.getMaxX() < other_k.getMaxX()))) {
                        current_k.addToBelow_os(other_k);
                    }


                }
            }
        }

        /*
        update oL, oR, oA, oB
         */
        for (Keepout current_k : uni_keepouts) {
            //update oL
            if (current_k.getLeft_os().size() != 0) {
                boolean flag = true;
                while (flag) {
                    int cnt_k_oL = current_k.getLeft_os().size();
                    for (Keepout oL_k : current_k.getLeft_os()) {
                        if (!current_k.getLeft_os().containsAll(oL_k.getLeft_os())) {
                            current_k.getLeft_os().removeAll(oL_k.getLeft_os());
                            current_k.getLeft_os().addAll(oL_k.getLeft_os());
                            break;
                        }
                    }
                    if (cnt_k_oL == current_k.getLeft_os().size()) {
                        flag = false;
                    }
                }
            }

            //update oR
            if (current_k.getRight_os().size() != 0) {
                boolean flag = true;
                while (flag) {
                    int cnt_k_oR = current_k.getRight_os().size();
                    for (Keepout oL_k : current_k.getRight_os()) {
                        if (!current_k.getRight_os().containsAll(oL_k.getRight_os())) {
                            current_k.getRight_os().removeAll(oL_k.getRight_os());
                            current_k.getRight_os().addAll(oL_k.getRight_os());
                            break;
                        }
                    }
                    if (cnt_k_oR == current_k.getRight_os().size()) {
                        flag = false;
                    }
                }
            }

            //update oA
            if (current_k.getAbove_os().size() != 0) {
                boolean flag = true;
                while (flag) {
                    int cnt_k_oA = current_k.getAbove_os().size();
                    for (Keepout oL_k : current_k.getAbove_os()) {
                        if (!current_k.getAbove_os().containsAll(oL_k.getAbove_os())) {
                            current_k.getAbove_os().removeAll(oL_k.getAbove_os());
                            current_k.getAbove_os().addAll(oL_k.getAbove_os());
                            break;
                        }
                    }
                    if (cnt_k_oA == current_k.getAbove_os().size()) {
                        flag = false;
                    }
                }
            }

            //update oB
            if (current_k.getBelow_os().size() != 0) {
                boolean flag = true;
                while (flag) {
                    int cnt_k_oB = current_k.getBelow_os().size();
                    for (Keepout oL_k : current_k.getBelow_os()) {
                        if (!current_k.getBelow_os().containsAll(oL_k.getBelow_os())) {
                            current_k.getBelow_os().removeAll(oL_k.getBelow_os());
                            current_k.getBelow_os().addAll(oL_k.getBelow_os());
                            break;
                        }
                    }
                    if (cnt_k_oB == current_k.getBelow_os().size()) {
                        flag = false;
                    }
                }
            }


        }

        /*
        add itself
         */
        for (Keepout current_k : uni_keepouts) {
            current_k.addToLeft_os(current_k);
            current_k.addToRight_os(current_k);
            current_k.addToAbove_os(current_k);
            current_k.addToBelow_os(current_k);
        }

        /*
        update map_oo_dist
         */
        for (Keepout current_k : uni_keepouts) {
            for (Keepout other_k : uni_keepouts) {
                int[] dist = new int[4];
                //ll
                dist[0] = Math.abs(current_k.getMinX() - other_k.getMinX()) + Math.abs(current_k.getMinY() - other_k.getMinY());
                //ll,ur
                dist[1] = Math.abs(current_k.getMinX() - other_k.getMaxX()) + Math.abs(current_k.getMinY() - other_k.getMaxY());
                //ur,ll
                dist[2] = Math.abs(current_k.getMaxX() - other_k.getMinX()) + Math.abs(current_k.getMaxY() - other_k.getMinY());
                //ur
                dist[3] = Math.abs(current_k.getMaxX() - other_k.getMaxX()) + Math.abs(current_k.getMaxY() - other_k.getMaxY());
                current_k.addToMap_oo_dist(other_k, dist);
            }
        }
    }


    /**
     * The original Alg. of DW
     *
     * @param master Master of the network
     * @param slaves Slaves of the network
     */

    public void processToOutput_DW(Master master, ArrayList<Slave> slaves) {

        ArrayList<Node> terminals = new ArrayList<>();
        //Node_0 is the master
        Node ms_node = new Node(master.getX_ct(), master.getY_ct(), 0, NodeType.Master);

        for (int i = 0; i < slaves.size(); ++i) {
            Node n = new Node(slaves.get(i).getX_ct(), slaves.get(i).getY_ct(), i + 1, NodeType.Terminal);
            terminals.add(n);
        }

        int t_cnt = slaves.size(); //number of terminals

        /*
         * Graph G = (N,A)
         * nodes contains all nodes in the graph
         */
        ArrayList<Node> nodes = new ArrayList<>();

        /*
         * We generate all nodes besides the terminals
         */
        steinerNodesGenerator(terminals, t_cnt, nodes, ms_node);

        int node_cnt = nodes.size();
        System.out.println("node_cnt" + node_cnt);
        System.out.println(nodes);

        /*
         * The Matrix d of shortest lengths
         * d_i,j = d_j,i = length of the shortest path between nodes i and j in N
         */
        double[][] d = new double[node_cnt + 1][node_cnt + 1];
        for (int i = 1; i <= node_cnt; ++i) {
            for (int j = 1; j <= node_cnt; ++j) {
                d[i][j] = add_BD(Math.abs(sub_BD(nodes.get(i - 1).getX_exact(), nodes.get(j - 1).getX_exact())), Math.abs(sub_BD(nodes.get(i - 1).getY_exact(), nodes.get(j - 1).getY_exact())));

            }
        }
        d[0][0] = 0.0;
        for (int i = 1; i <= node_cnt; ++i) {
            d[i][0] = d[0][i] = add_BD(Math.abs(nodes.get(i - 1).getX_exact() - master.getX_ct()), Math.abs(nodes.get(i - 1).getY_exact() - master.getY_ct()));
        }

        //debug
        /*for (int i = 1; i <= node_cnt; ++i) {
            for (int j = 1; j <= node_cnt; ++j) {
                System.out.print(d[i][j] + " ");
            }
            System.out.println();
        }*/


        /*
         * Algorithm A: (Computes the length of the Steiner tree connecting Y, and assigns this length to variable "v")
         * v is our MasterIC
         * In the paper: Y = {terminals}, C = Y - {v};
         * here: C = {terminals}, v = MasterIC
         */
        ArrayList<Value_S> Value_Ss = new ArrayList<>();
        /*
         * For each t in C do
         *  For each J in N do
         *      S[{t},J] <- D(t,J);
         */
        for (Node t : terminals) {
            for (Node J : nodes) {

                Value_S tmpS = new Value_S();
                tmpS.addToSetD(t.getNum());
                tmpS.setSecondArg_Int(J.getNum());
                tmpS.setValue(d[t.getNum()][J.getNum()]);
                Value_Ss.add(tmpS);

            }
        }
        //System.out.println("Value_Ss = " + Value_Ss);


        /*
         * Generate all the subsets of C, with cardinality >= 2
         */
        ArrayList<Integer> terminal_Int = new ArrayList<>();
        for (int i = 0; i < t_cnt; ++i) {
            terminal_Int.add(i + 1);
        }
        Set<ArrayList<Integer>> Ds = subsetsGenerator(terminal_Int, 2, t_cnt);

        for (int m = 2; m <= t_cnt - 1; ++m) {
            System.out.println("m = " + m);
            for (ArrayList<Integer> D : Ds) {
                if (D.size() == m) {
                    System.out.println("D = " + D);
                    for (Node I : nodes) {
                        Value_S tmpS = new Value_S();
                        tmpS.setFirstArg_Array(D);
                        tmpS.setSecondArg_Int(I.getNum());
                        Value_Ss.add(tmpS);
                    }

                    Set<ArrayList<Integer>> Es = subsetsGenerator(D, 1, D.size() - 1);
                    //System.out.println("Es" + Es);
                    for (Node J : nodes) {
                        //System.out.println("Jnum= " + J.getNum());
                        double u = Double.MAX_VALUE;
                        /*
                         * for each E such that (D[1] in E) and (E proper subset of D) do
                         *      u <- min (u, S[E,J] + S[D-E,J])
                         */
                        ArrayList<Integer> tmpD_E = new ArrayList<>();
                        ArrayList<Integer> tmpD_without_E = new ArrayList<>();
                        for (ArrayList<Integer> E : Es) {
                            if (E.contains(D.get(0))) {
                                //System.out.println("E = " + E);
                                ArrayList<Integer> D_without_E = new ArrayList<>(D);
                                D_without_E.removeAll(E);
                                //System.out.println("D_without_E = " + D_without_E);
                                double S_E_J = Double.MAX_VALUE, S_D_without_E_J = Double.MAX_VALUE;
                                for (Value_S s : Value_Ss) {
                                    ArrayList<Integer> setOfs = new ArrayList<>(s.getFirstArg_Array());
                                    if (setOfs.containsAll(E) && E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                                        S_E_J = s.getValue();
                                        //System.out.println("S_E_J = " + S_E_J);
                                    } else if (setOfs.containsAll(D_without_E) && D_without_E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                                        S_D_without_E_J = s.getValue();
                                        //System.out.println("S_D_without_E_J = " + S_D_without_E_J);
                                    }
                                }
                                double previous_u = u;
                                u = Math.min(u, add_BD(S_E_J, S_D_without_E_J));
                                //System.out.println("u" + u);
                                if (u < previous_u) { //u is updated
                                    tmpD_E = new ArrayList<>(E);
                                    tmpD_without_E = new ArrayList<>(D_without_E);
                                }
                            }
                        }
                        /*
                         * for each I in N do
                         *      S[D, I] <- min (S[D,I], D(I,J) + u)
                         */
                        //System.out.println("Value_Ss = " + Value_Ss);
                        for (Node I : nodes) {
                            for (Value_S s : Value_Ss) {
                                ArrayList<Integer> setOfs = new ArrayList<>(s.getFirstArg_Array());
                                if (setOfs.containsAll(D) && D.containsAll(setOfs) && s.getSecondArg_Int() == I.getNum()) {
                                    double previous_sValue = s.getValue();
                                    s.setValue(Math.min(s.getValue(), add_BD(d[I.getNum()][J.getNum()], u)));
                                    if (s.getValue() < previous_sValue) { //s.Value updated
                                        s.setD_E(tmpD_E);
                                        s.setD_without_E(tmpD_without_E);
                                        s.setJ_Num(J.getNum());
                                    }

                                }
                            }

                        }
                        //System.out.println("Value_Ss = " + Value_Ss);


                    }

                }

            }


        }

        /*
         * (15) -- (20)
         */
        double v = Double.MAX_VALUE;
        //We retrieve the last J (we only give the last solution)
        int p_Num = 0;
        for (Node J : nodes) {
            //System.out.println("J.Num = " + J.getNum());
            double u = Double.MAX_VALUE;
            Set<ArrayList<Integer>> Es = subsetsGenerator(terminal_Int, 1, t_cnt - 1);
            for (ArrayList<Integer> E : Es) {
                if (E.contains(terminals.get(0).getNum())) {
                    /*
                     * u <- min (u, S[E,J] + S[C-E,J])
                     * for each J, we need to store the E and C-E such that the u achieves the Min.
                     */
                    //System.out.println("E = " + E);
                    ArrayList<Integer> C_without_E = new ArrayList<>(terminal_Int);
                    C_without_E.removeAll(E);

                    //System.out.println("C_without_E = " + C_without_E);

                    double S_E_J = Double.MAX_VALUE, S_C_without_E_J = Double.MAX_VALUE;
                    for (Value_S s : Value_Ss) {
                        ArrayList<Integer> setOfs = new ArrayList<>(s.getFirstArg_Array());
                        if (setOfs.containsAll(E) && E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                            S_E_J = s.getValue();
                            //System.out.println("S_E_J = " + S_E_J);
                        } else if (setOfs.containsAll(C_without_E) && C_without_E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                            S_C_without_E_J = s.getValue();
                            //System.out.println("S_C_without_E_J = " + S_C_without_E_J);

                        }

                    }

                    double previous_u = u;
                    u = Math.min(u, add_BD(S_E_J, S_C_without_E_J));
                    if (u < previous_u) {//u is updated
                        J.setC_E(E);
                        J.setC_without_E(C_without_E);
                    }

                    /*System.out.println("u" + u);
                    System.out.println("S+S = " + add_BD(S_E_J, S_C_without_E_J));
                    System.out.println("J.Num = " + J.getNum());
                    System.out.println("J.E = " + J.getC_E());
                    System.out.println("J.C_without_E = " + J.getC_without_E());*/

                }

            }

            /*
             * v <- min (v, D(q,J) + u)
             * here: q is our masterIC
             */
            //System.out.println("u = " + u);

            double previous_v = v;
            v = Math.min(v, add_BD(d[0][J.getNum()], u));

            if (v < previous_v) { //updated v
                p_Num = J.getNum();
            }
        }

        System.err.println("p_Num = " + p_Num);
        System.err.println("v = " + v);
        System.out.println("Value_Ss = " + Value_Ss);

        /*
         * Retrieve from the Algorithm-A
         */
        System.out.println(nodes);
        //p_node is the point that connect the MasterIC and the restGroup
        Node p_node = nodes.get(p_Num - 1);
        ArrayList<Integer> p_C_E = p_node.getC_E();
        ArrayList<Integer> p_C_without_E = p_node.getC_without_E();
        Path_Tree path_tree = new Path_Tree(ms_node);
        Path_Tree.T_Node p_Tnode = new Path_Tree.T_Node(p_node);
        path_tree.getMaster().addToChildren(p_Tnode);
        ArrayList<Path_Tree.T_Node> t_nodes = new ArrayList<>();
        retrieve_S_tree(nodes, Value_Ss, p_Num, p_C_E, path_tree, p_Tnode, t_nodes);
        retrieve_S_tree(nodes, Value_Ss, p_Num, p_C_without_E, path_tree, p_Tnode, t_nodes);
        /*
         * Retrieve Data from path_tree
         */
        for (Path_Tree.T_Node t_node : t_nodes) {
            System.out.println("t_node= " + t_node);
            if (t_node.getChildren() != null) {
                for (Path_Tree.T_Node child_tnode : t_node.getChildren()) {
                    System.out.println("child=" + child_tnode);
                }
            }
            if (t_node.getParent() != null) {
                System.out.println("parent=" + t_node.getParent());
            }


        }


    }

    /**
     * The modified Version ov DW, obtain the optimal solution w.r.t. the daisy-chained Network Topology
     * All the coordinates are DOUBLE Type
     *
     * @param master Master of the network
     * @param slaves Slaves of the network
     */
    public void processToOutput_DW_MD(Master master, ArrayList<Slave> slaves) {

        ArrayList<Node> terminals = new ArrayList<>();
        //Node_0 is the master
        Node ms_node = new Node(master.getX_ct(), master.getY_ct(), 0, NodeType.Master);

        for (int i = 0; i < slaves.size(); ++i) {
            Node n = new Node(slaves.get(i).getX_ct(), slaves.get(i).getY_ct(), i + 1, NodeType.Terminal);
            terminals.add(n);
        }

        int t_cnt = slaves.size(); //number of terminals

        /*
         * Graph G = (N,A)
         * nodes contains all nodes in the graph
         */
        ArrayList<Node> nodes = new ArrayList<>();

        /*
         * We generate all nodes besides the terminals
         */
        steinerNodesGenerator(terminals, t_cnt, nodes, ms_node);

        /*ArrayList<Node> steiner_nodes = new ArrayList<>(nodes);
        steiner_nodes.removeAll(terminals);*/

        int node_cnt = nodes.size();
        System.out.println("node_cnt" + node_cnt);
        System.out.println(nodes);

        /*
         * The Matrix d of shortest lengths
         * d_i,j = d_j,i = length of the shortest path between nodes i and j in N
         */
        double[][] d = new double[node_cnt + 1][node_cnt + 1];
        for (int i = 1; i <= node_cnt; ++i) {
            for (int j = 1; j <= node_cnt; ++j) {
                d[i][j] = add_BD(Math.abs(sub_BD(nodes.get(i - 1).getX_exact(), nodes.get(j - 1).getX_exact())), Math.abs(sub_BD(nodes.get(i - 1).getY_exact(), nodes.get(j - 1).getY_exact())));

            }
        }
        d[0][0] = 0.0;
        for (int i = 1; i <= node_cnt; ++i) {
            d[i][0] = d[0][i] = add_BD(Math.abs(nodes.get(i - 1).getX_exact() - master.getX_ct()), Math.abs(nodes.get(i - 1).getY_exact() - master.getY_ct()));
        }

        //debug
        /*for (int i = 1; i <= node_cnt; ++i) {
            for (int j = 1; j <= node_cnt; ++j) {
                System.out.print(d[i][j] + " ");
            }
            System.out.println();
        }*/


        /*
         * Algorithm A: (Computes the length of the Steiner tree connecting Y, and assigns this length to variable "v")
         * v is our MasterIC
         * In the paper: Y = {terminals}, C = Y - {v};
         * here: C = {terminals}, v = MasterIC
         */
        ArrayList<Value_S> Ss = new ArrayList<>();
        /*
         * For each t in C do
         *  For each J in N do
         *      S[{t},J] <- D(t,J);
         */
        for (Node t : terminals) {
            for (Node J : nodes) {
                Value_S tmpS = new Value_S();
                tmpS.addToSetD(t.getNum());
                tmpS.setSecondArg_Int(J.getNum());
                tmpS.setValue(d[t.getNum()][J.getNum()]);
                Ss.add(tmpS);

            }
        }
        //System.out.println("Ss = " + Ss);


        /*
         * Generate all the subsets of C, with cardinality >= 2
         */
        ArrayList<Integer> terminal_Int = new ArrayList<>();
        for (int i = 0; i < t_cnt; ++i) {
            terminal_Int.add(i + 1);
        }
        Set<ArrayList<Integer>> Ds = subsetsGenerator(terminal_Int, 2, t_cnt);

        for (int m = 2; m <= t_cnt - 1; ++m) {
            System.out.println("m = " + m);
            for (ArrayList<Integer> D : Ds) {
                if (D.size() == m) {
                    System.out.println("D = " + D);
                    for (Node I : nodes) {
                        Value_S tmpS = new Value_S();
                        tmpS.setFirstArg_Array(D);
                        tmpS.setSecondArg_Int(I.getNum());
                        Ss.add(tmpS);
                    }

                    Set<ArrayList<Integer>> Es = subsetsGenerator(D, 1, D.size() - 1);
                    System.out.println("Es=" + Es);
                    Set<ArrayList<Integer>> EsCopy = new HashSet<>(Es);
                    for (ArrayList<Integer> E : EsCopy) {
                        if (E.size() > 1 && E.size() < D.size() - 1) {
                            Es.remove(E);
                        }
                    }
                    System.out.println("Es.size()= " + Es.size());
                    //System.out.println("Es" + Es);
                    for (Node J : nodes) {
                        //System.out.println("Jnum= " + J.getNum());
                        double u = Double.MAX_VALUE;
                        /*
                         * for each E such that (D[1] in E) and (E proper subset of D) do
                         *      u <- min (u, S[E,J] + S[D-E,J])
                         */
                        ArrayList<Integer> tmpD_E = new ArrayList<>();
                        ArrayList<Integer> tmpD_without_E = new ArrayList<>();
                        for (ArrayList<Integer> E : Es) {
                            if (E.contains(D.get(0))) {
                                //System.out.println("E = " + E);
                                ArrayList<Integer> D_without_E = new ArrayList<>(D);
                                D_without_E.removeAll(E);
                                //System.out.println("D_without_E = " + D_without_E);
                                double S_E_J = Double.MAX_VALUE, S_D_without_E_J = Double.MAX_VALUE;
                                for (Value_S s : Ss) {
                                    ArrayList<Integer> setOfs = new ArrayList<>(s.getFirstArg_Array());
                                    if (setOfs.containsAll(E) && E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                                        S_E_J = s.getValue();
                                        //System.out.println("S_E_J = " + S_E_J);
                                    } else if (setOfs.containsAll(D_without_E) && D_without_E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                                        S_D_without_E_J = s.getValue();
                                        //System.out.println("S_D_without_E_J = " + S_D_without_E_J);
                                    }
                                }
                                double previous_u = u;
                                u = Math.min(u, add_BD(S_E_J, S_D_without_E_J));
                                //System.out.println("u" + u);
                                if (u < previous_u) { //u is updated
                                    tmpD_E = new ArrayList<>(E);
                                    tmpD_without_E = new ArrayList<>(D_without_E);
                                }
                            }
                        }
                        /*
                         * for each I in N do
                         *      S[D, I] <- min (S[D,I], D(I,J) + u)
                         */
                        //System.out.println("Ss = " + Ss);
                        for (Node I : nodes) {
                            for (Value_S s : Ss) {
                                ArrayList<Integer> setOfs = new ArrayList<>(s.getFirstArg_Array());
                                if (setOfs.containsAll(D) && D.containsAll(setOfs) && s.getSecondArg_Int() == I.getNum()) {
                                    double previous_sValue = s.getValue();
                                    s.setValue(Math.min(s.getValue(), add_BD(d[I.getNum()][J.getNum()], u)));
                                    if (s.getValue() < previous_sValue) { //s.Value updated
                                        s.setD_E(tmpD_E);
                                        s.setD_without_E(tmpD_without_E);
                                        s.setJ_Num(J.getNum());
                                    }

                                }
                            }

                        }
                        //System.out.println("Ss = " + Ss);


                    }

                }

            }


        }

        /*
         * (15) -- (20)
         */
        double v = Double.MAX_VALUE;
        //We retrieve the last J (we only give the last solution)
        int p_Num = 0;
        for (Node J : nodes) {
            //System.out.println("J.Num = " + J.getNum());
            double u = Double.MAX_VALUE;
            Set<ArrayList<Integer>> Es = subsetsGenerator(terminal_Int, 1, t_cnt - 1);
            for (ArrayList<Integer> E : Es) {
                if (E.contains(terminals.get(0).getNum())) {
                    /*
                     * u <- min (u, S[E,J] + S[C-E,J])
                     * for each J, we need to store the E and C-E such that the u achieves the Min.
                     */
                    //System.out.println("E = " + E);
                    ArrayList<Integer> C_without_E = new ArrayList<>(terminal_Int);
                    C_without_E.removeAll(E);

                    //System.out.println("C_without_E = " + C_without_E);

                    double S_E_J = Double.MAX_VALUE, S_C_without_E_J = Double.MAX_VALUE;
                    for (Value_S s : Ss) {
                        ArrayList<Integer> setOfs = new ArrayList<>(s.getFirstArg_Array());
                        if (setOfs.containsAll(E) && E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                            S_E_J = s.getValue();
                            //System.out.println("S_E_J = " + S_E_J);
                        } else if (setOfs.containsAll(C_without_E) && C_without_E.containsAll(setOfs) && s.getSecondArg_Int() == J.getNum()) {
                            S_C_without_E_J = s.getValue();
                            //System.out.println("S_C_without_E_J = " + S_C_without_E_J);

                        }

                    }

                    double previous_u = u;
                    u = Math.min(u, add_BD(S_E_J, S_C_without_E_J));
                    if (u < previous_u) {//u is updated
                        J.setC_E(E);
                        J.setC_without_E(C_without_E);
                    }

                    /*System.out.println("u" + u);
                    System.out.println("S+S = " + add_BD(S_E_J, S_C_without_E_J));
                    System.out.println("J.Num = " + J.getNum());
                    System.out.println("J.E = " + J.getC_E());
                    System.out.println("J.C_without_E = " + J.getC_without_E());*/

                }

            }

            /*
             * v <- min (v, D(q,J) + u)
             * here: q is our masterIC
             */
            //System.out.println("u = " + u);

            double previous_v = v;
            v = Math.min(v, add_BD(d[0][J.getNum()], u));

            if (v < previous_v) { //updated v
                p_Num = J.getNum();
            }
        }

        System.err.println("p_Num = " + p_Num);
        System.err.println("v = " + v);
        System.out.println("Ss = " + Ss);

        /*
         * Retrieve from the Algorithm-A
         */
        System.out.println(nodes);
        //p_node is the point that connect the MasterIC and the restGroup
        Node p_node = nodes.get(p_Num - 1);
        ArrayList<Integer> p_C_E = p_node.getC_E();
        System.out.println("p_C_E= " + p_C_E);
        ArrayList<Integer> p_C_without_E = p_node.getC_without_E();
        System.out.println("p_C_without_E= " + p_C_without_E);
        Path_Tree path_tree = new Path_Tree(ms_node);
        Path_Tree.T_Node p_Tnode = new Path_Tree.T_Node(p_node);
        path_tree.getMaster().addToChildren(p_Tnode);
        ArrayList<Path_Tree.T_Node> t_nodes = new ArrayList<>();
        retrieve_S_tree(nodes, Ss, p_Num, p_C_E, path_tree, p_Tnode, t_nodes);
        retrieve_S_tree(nodes, Ss, p_Num, p_C_without_E, path_tree, p_Tnode, t_nodes);
        /*
         * Retrieve Data from path_tree
         */
        for (Path_Tree.T_Node t_node : t_nodes) {
            System.out.println("t_node= " + t_node);
            if (t_node.getChildren() != null) {
                for (Path_Tree.T_Node child_tnode : t_node.getChildren()) {
                    System.out.println("child=" + child_tnode);
                }
            }
            if (t_node.getParent() != null) {
                System.out.println("parent=" + t_node.getParent());
            }


        }


    }


    private void retrieve_S_tree(ArrayList<Node> nodes, ArrayList<Value_S> Ss, int current_Num, ArrayList<Integer> set, Path_Tree path_tree, Path_Tree.T_Node current_Tnode, ArrayList<Path_Tree.T_Node> t_nodes) {
        if (set.size() > 1) {
            for (Value_S s : Ss) {
                if (s.getFirstArg_Array().containsAll(set) && set.containsAll(s.getFirstArg_Array()) && s.getSecondArg_Int() == current_Num) {

                    int next_Num_L = s.getJ_Num();
                    Path_Tree.T_Node nextNode = new Path_Tree.T_Node(nodes.get(next_Num_L - 1));
                    nextNode.setParent(current_Tnode);
                    t_nodes.add(nextNode);
                    ArrayList<Integer> S_L = s.getD_E();
                    ArrayList<Integer> S_R = s.getD_without_E();
                    retrieve_S_tree(nodes, Ss, next_Num_L, S_L, path_tree, nextNode, t_nodes);
                    retrieve_S_tree(nodes, Ss, next_Num_L, S_R, path_tree, nextNode, t_nodes);

                }
            }
        } else {
            if (set.get(0) != current_Num) {
                Node cildnode = nodes.get(set.get(0) - 1);
                Path_Tree.T_Node child = new Path_Tree.T_Node(cildnode.getNum(), cildnode.getX_exact(), cildnode.getY_exact(), current_Tnode, null);
                current_Tnode.addToChildren(child);
                t_nodes.add(child);
            }
        }
    }

    private Set<ArrayList<Integer>> subsetsGenerator(ArrayList<Integer> intArray, int min_cnt, int max_cnt) {
        Set<ArrayList<Integer>> subsets = new HashSet<>();
        int cnt = intArray.size();
        for (int i = 1; i < (1 << cnt); i++) {
            ArrayList<Integer> tmpArray = new ArrayList<>();
            // Print current subset
            for (int j = 0; j < cnt; j++) {
                if ((i & (1 << j)) > 0) {
                    tmpArray.add(intArray.get(j));
                }
            }
            if (tmpArray.size() >= min_cnt && tmpArray.size() <= max_cnt) {
                subsets.add(tmpArray);
            }
        }
        //System.out.println(subsets);
        return subsets;

    }


    private void steinerNodesGenerator(ArrayList<Node> set, int node_cnt, ArrayList<Node> nodes, Node ms_node) {
        for (Node tn : set) {
            nodes.add(tn);
        }
        ArrayList<Node> set_copy = new ArrayList<>(set);
        set_copy.add(ms_node);
        for (int i = 0; i < set_copy.size(); ++i) {
            double tmpX = set_copy.get(i).getX_exact();
            //System.out.println(tmpX);
            for (int j = 0; j < set_copy.size(); ++j) {
                if (j != i) {
                    double tmpY = set_copy.get(j).getY_exact();
                    Node tmpN = new Node(tmpX, tmpY, NodeType.SteinerNode);

                    if (!set_copy.contains(tmpN)) {
                        tmpN.setNum(nodes.size() + 1);
                        //System.out.println(tmpN);
                        nodes.add(tmpN);
                    } else {
                        System.out.println("This Steiner Point is in Terminals");
                    }
                }

            }
        }
    }

    private void setObjCons(GurobiVariable busLength, GurobiVariable sideBusLength, GurobiObjConstraint objCons, double busC, double slaveC) {

        /*PointVar pvF = pointVars.get(0);
        objCons.addToLHS(pvF.distM_x, busC);
        objCons.addToLHS(pvF.distM_y, busC);*/

        objCons.addToLHS(busLength, busC);
        objCons.addToLHS(sideBusLength, slaveC);

        /*for (PointVar pv : pointVars) {
            objCons.addToLHS(pv.distI, busC);
            for (Slave s : slaves) {
                objCons.addToLHS(pv.distSs.get(s), pv.slave_Vars.get(s), slaveC);
            }
        }*/

        objCons.setGoal(GRB.MINIMIZE);
    }

    /**
     * Build the constaints for ILP-Model without Keepout
     * The abs values are solved by GRB.GenConstraints
     *
     * @param master        Master of the network
     * @param slaves        Slaves of the network
     * @param pointVars_abs ArrayList of the pointVars for use abs
     * @throws GRBException Gurobi
     */
    private void buildCons_wo_KO_ABS(Master master, ArrayList<Slave> slaves, ArrayList<PointVar_Abs> pointVars_abs) throws GRBException {
        GurobiConstraint c;
        PointVar_Abs pv;
        PointVar_Abs pvNext;

        /*
         * SETUP CONSTRAINS FOR FIRST POINT
         *  VVVVVVVVVVVVVVVVVVVVVV
         */
        PointVar_Abs pvF = pointVars_abs.get(0);
        //dist(M, I1) = distM_x + distM_y


        //delta_Mx = x_1 - M_x
        //distM_x = |delta_Mx|
        c = new GurobiConstraint();
        c.addToLHS(pvF.delta_Mx_Var, 1);
        c.setSense(GRB.EQUAL);
        c.addToRHS(pvF.x_Var, 1);
        c.setRHSConstant(-master.getX_ct());
        executor.addConstraint(c);
        executor.addGenConstraintAbs(pvF.distM_x, pvF.delta_Mx_Var, "Master_x_abs");

        //delta_My = y_1 - M_y
        //distM_y = |delta_My|
        c = new GurobiConstraint();
        c.addToLHS(pvF.delta_My_Var, 1);
        c.setSense(GRB.EQUAL);
        c.addToRHS(pvF.y_Var, 1);
        c.setRHSConstant(-master.getY_ct());
        executor.addConstraint(c);
        executor.addGenConstraintAbs(pvF.distM_y, pvF.delta_My_Var, "Master_y_abs");

        /*
         * AAAAAAAAAAAAAAAAAAAAAAAAA
         * SETUP CONSTRAINS FOR FIRST POINT
         */


        for (int j = 0; j < pointVars_abs.size() - 1; j++) {

            pv = pointVars_abs.get(j);
            pvNext = pointVars_abs.get(j + 1);

            //distI = distI_x + distI_y
            c = new GurobiConstraint();
            c.addToLHS(pv.distI, 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.distI_x, 1);
            c.addToRHS(pv.distI_y, 1);
            executor.addConstraint(c);


            //delta_Ix_Var = x_i - x_{i+1}
            //distI_x = |delta_Ix_Var|
            c = new GurobiConstraint();
            c.addToLHS(pv.delta_Ix_Var, 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.x_Var, 1);
            c.addToRHS(pvNext.x_Var, -1);
            executor.addConstraint(c);
            executor.addGenConstraintAbs(pv.distI_x, pv.delta_Ix_Var, j + "_abs_Ix: distI_x = |delta_Ix_Var|");

            //delta_Iy_Var = y_i - y_{i+1}
            //distI_y = |delta_Iy_Var|
            c = new GurobiConstraint();
            c.addToLHS(pv.delta_Iy_Var, 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.y_Var, 1);
            c.addToRHS(pvNext.y_Var, -1);
            executor.addConstraint(c);
            executor.addGenConstraintAbs(pv.distI_y, pv.delta_Iy_Var, j + "_abs_Iy: distI_y = |delta_Iy_Var|");


            //each IP must connect one Slave
            GurobiConstraint cConnectSlave = new GurobiConstraint();
            cConnectSlave.setRHSConstant(1);
            cConnectSlave.setSense(GRB.EQUAL);


            for (Slave s : slaves) {

                cConnectSlave.addToLHS(pv.slave_Vars.get(s), 1);

                //distS_j = distS_xj + distS_yj
                c = new GurobiConstraint();
                c.addToLHS(pv.distSs.get(s), 1);
                c.setSense(GRB.EQUAL);
                c.addToRHS(pv.distS_xs.get(s), 1);
                c.addToRHS(pv.distS_ys.get(s), 1);
                executor.addConstraint(c);

                //delta_Sx = x_j - xs_j
                //distS_x = |delta_Sx|
                c = new GurobiConstraint();
                c.addToLHS(pv.delta_Sx_Vars.get(s), 1);
                c.setSense(GRB.EQUAL);
                c.addToRHS(pv.x_Var, 1);
                c.setRHSConstant(-s.getX_ct());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(pv.distS_xs.get(s), pv.delta_Sx_Vars.get(s), j + "distS_xs = |delta_Sx_Vars|");

                //delta_Sy = y_j - ys_j
                //distS_y = |delta_Sy|
                c = new GurobiConstraint();
                c.addToLHS(pv.delta_Sy_Vars.get(s), 1);
                c.setSense(GRB.EQUAL);
                c.addToRHS(pv.y_Var, 1);
                c.setRHSConstant(-s.getY_ct());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(pv.distS_ys.get(s), pv.delta_Sy_Vars.get(s), j + "distS_ys = |delta_Sy_Vars|");


            }
            executor.addConstraint(cConnectSlave);


        }

        /*
         * SETUP CONSTRAINS FOR LAST POINT
         *  VVVVVVVVVVVVVVVVVVVVVV
         */
        pv = pointVars_abs.get(pointVars_abs.size() - 1);


        //Each IP must connect one Slave
        GurobiConstraint cConnectSlave = new GurobiConstraint();
        cConnectSlave.setRHSConstant(1);
        cConnectSlave.setSense(GRB.EQUAL);


        for (Slave s : slaves) {

            cConnectSlave.addToLHS(pv.slave_Vars.get(s), 1);

            //distS_j = distS_xj + distS_yj
            c = new GurobiConstraint();
            c.addToLHS(pv.distSs.get(s), 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.distS_xs.get(s), 1);
            c.addToRHS(pv.distS_ys.get(s), 1);
            executor.addConstraint(c);


            //delta_Sx = x_j - xs_j
            //distS_x = |delta_Sx|
            c = new GurobiConstraint();
            c.addToLHS(pv.delta_Sx_Vars.get(s), 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.x_Var, 1);
            c.setRHSConstant(-s.getX_ct());
            executor.addConstraint(c);
            executor.addGenConstraintAbs(pv.distS_xs.get(s), pv.delta_Sx_Vars.get(s), pointVars_abs.size() + "distS_xs = |delta_Sx_Vars|");

            //delta_Sy = y_j - ys_j
            //distS_y = |delta_Sy|
            c = new GurobiConstraint();
            c.addToLHS(pv.delta_Sy_Vars.get(s), 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.y_Var, 1);
            c.setRHSConstant(-s.getY_ct());
            executor.addConstraint(c);
            executor.addGenConstraintAbs(pv.distS_ys.get(s), pv.delta_Sy_Vars.get(s), pointVars_abs.size() + "distS_ys = |delta_Sy_Vars|");


        }

        executor.addConstraint(cConnectSlave);

        /*
         * AAAAAAAAAAAAAAAAAAAAAAAAA
         * SETUP CONSTRAINS FOR LAST POINT
         */


        for (Slave s : slaves) {
            //Each Slave must connect one IP
            c = new GurobiConstraint();
            c.setRHSConstant(1);
            c.setSense(GRB.EQUAL);
            for (PointVar_Abs pvv : pointVars_abs) {
                c.addToLHS(pvv.slave_Vars.get(s), 1);
            }
            executor.addConstraint(c);

        }
    }

    /**
     * Build the constaints for ILP-Model without Keepout
     * The abs values are solved by Linearization
     *
     * @param master    Master of the network
     * @param slaves    Slaves of the network
     * @param n         number
     * @param pointVars pointVars
     * @throws GRBException Gurobi
     */
    private void buildCons_wo_KO(Master master, ArrayList<Slave> slaves, int n, ArrayList<PointVar> pointVars, GurobiVariable busLength, GurobiVariable sideBusLength) {
        GurobiConstraint c;
        PointVar pv;
        PointVar pvNext;

        GurobiConstraint c_busLength;
        GurobiQuadConstraint c_sideBusLength;

        //setup for buslength
        c_busLength = new GurobiConstraint();
        c_busLength.addToLHS(busLength, 1);
        c_busLength.setSense(GRB.EQUAL);

        //setup for sidebuslength
        c_sideBusLength = new GurobiQuadConstraint();
        c_sideBusLength.addToLHS(sideBusLength, 1);
        c_sideBusLength.setSense(GRB.EQUAL);


        /*
         * SETUP CONSTRAINS FOR FIRST POINT
         *  VVVVVVVVVVVVVVVVVVVVVV
         */
        PointVar pvF = pointVars.get(0);

        /*
         * dist(M, I1) = distM_x + distM_y
         */
        //distM_x >= Mx - x_1
        c = new GurobiConstraint();
        c.addToLHS(pvF.distM_x, 1);
        c.setSense(GRB.GREATER_EQUAL);
        c.addToRHS(pvF.x, -1);
        c.setRHSConstant(master.getX_ct());
        executor.addConstraint(c);
        //distM_x >= x_1 - M_x
        c = new GurobiConstraint();
        c.addToLHS(pvF.distM_x, 1);
        c.setSense(GRB.GREATER_EQUAL);
        c.addToRHS(pvF.x, 1);
        c.setRHSConstant(-master.getX_ct());
        executor.addConstraint(c);
        //distM_y >= My - y_1
        c = new GurobiConstraint();
        c.addToLHS(pvF.distM_y, 1);
        c.setSense(GRB.GREATER_EQUAL);
        c.addToRHS(pvF.y, -1);
        c.setRHSConstant(master.getY_ct());
        executor.addConstraint(c);
        //distM_y >= y_1 - My
        c = new GurobiConstraint();
        c.addToLHS(pvF.distM_y, 1);
        c.setSense(GRB.GREATER_EQUAL);
        c.addToRHS(pvF.y, 1);
        c.setRHSConstant(-master.getY_ct());
        executor.addConstraint(c);

        c_busLength.addToRHS(pvF.distM_x, 1);
        c_busLength.addToRHS(pvF.distM_y, 1);


        /*
         * AAAAAAAAAAAAAAAAAAAAAAAAA
         * SETUP CONSTRAINS FOR FIRST POINT
         */


        for (int j = 0; j < pointVars.size() - 1; j++) {

            pv = pointVars.get(j);
            pvNext = pointVars.get(j + 1);


            //distI = distI_x + distI_y
            c = new GurobiConstraint();
            c.addToLHS(pv.distI, 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.distI_x, 1);
            c.addToRHS(pv.distI_y, 1);
            executor.addConstraint(c);
            c_busLength.addToRHS(pv.distI, 1);

            //distI_x >= x_i - x_{i+1}
            c = new GurobiConstraint();
            c.addToLHS(pv.distI_x, 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pv.x, 1);
            c.addToRHS(pvNext.x, -1);
            executor.addConstraint(c);
            //distI_x >= x_{i+1} - x_i
            c = new GurobiConstraint();
            c.addToLHS(pv.distI_x, 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pvNext.x, 1);
            c.addToRHS(pv.x, -1);
            executor.addConstraint(c);
            //distI_y >= y_i - y_{i+1}
            c = new GurobiConstraint();
            c.addToLHS(pv.distI_y, 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pv.y, 1);
            c.addToRHS(pvNext.y, -1);
            executor.addConstraint(c);
            //distI_y >= y_{i+1} - y_i
            c = new GurobiConstraint();
            c.addToLHS(pv.distI_y, 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pvNext.y, 1);
            c.addToRHS(pv.y, -1);
            executor.addConstraint(c);


            //each IP must connect one Slave
            GurobiConstraint cConnectSlave = new GurobiConstraint();
            cConnectSlave.setRHSConstant(1);
            cConnectSlave.setSense(GRB.EQUAL);

            for (Slave s : slaves) {

                cConnectSlave.addToLHS(pv.s_qVars.get(s), 1);

                //distS_j = distS_xj + distS_yj
                c = new GurobiConstraint();
                c.addToLHS(pv.distSs.get(s), 1);
                c.setSense(GRB.EQUAL);
                c.addToRHS(pv.distS_xs.get(s), 1);
                c.addToRHS(pv.distS_ys.get(s), 1);
                executor.addConstraint(c);
                c_sideBusLength.addToRHS(pv.distSs.get(s), pv.s_qVars.get(s), 1);

                //distS_xj >= x_j - xs_j
                c = new GurobiConstraint();
                c.addToLHS(pv.distS_xs.get(s), 1);
                c.setSense(GRB.GREATER_EQUAL);
                c.addToRHS(pv.x, 1);
                c.setRHSConstant(-s.getX_ct());
                executor.addConstraint(c);
                //distS_xj >= xs_j - x_j
                c = new GurobiConstraint();
                c.addToLHS(pv.distS_xs.get(s), 1);
                c.setSense(GRB.GREATER_EQUAL);
                c.addToRHS(pv.x, -1);
                c.setRHSConstant(s.getX_ct());
                executor.addConstraint(c);
                //distS_yj >= y_j - ys_j
                c = new GurobiConstraint();
                c.addToLHS(pv.distS_ys.get(s), 1);
                c.setSense(GRB.GREATER_EQUAL);
                c.addToRHS(pv.y, 1);
                c.setRHSConstant(-s.getY_ct());
                executor.addConstraint(c);
                //distS_yj >= ys_j - y_j
                c = new GurobiConstraint();
                c.addToLHS(pv.distS_ys.get(s), 1);
                c.setSense(GRB.GREATER_EQUAL);
                c.addToRHS(pv.y, -1);
                c.setRHSConstant(s.getY_ct());
                executor.addConstraint(c);


            }
            executor.addConstraint(cConnectSlave);

        }

        /*
         * SETUP CONSTRAINS FOR LAST POINT
         *  VVVVVVVVVVVVVVVVVVVVVV
         */
        pv = pointVars.get(pointVars.size() - 1);

        //Each IP must connect one Slave
        GurobiConstraint c_ConnectSlave = new GurobiConstraint();
        c_ConnectSlave.setRHSConstant(1);
        c_ConnectSlave.setSense(GRB.EQUAL);


        for (Slave s : slaves) {

            c_ConnectSlave.addToLHS(pv.s_qVars.get(s), 1);

            //distS_j = distS_xj + distS_yj
            c = new GurobiConstraint();
            c.addToLHS(pv.distSs.get(s), 1);
            c.setSense(GRB.EQUAL);
            c.addToRHS(pv.distS_xs.get(s), 1);
            c.addToRHS(pv.distS_ys.get(s), 1);
            executor.addConstraint(c);
            c_sideBusLength.addToRHS(pv.distSs.get(s), pv.s_qVars.get(s), 1);

            //distS_xj >= x_j - xs_j
            c = new GurobiConstraint();
            c.addToLHS(pv.distS_xs.get(s), 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pv.x, 1);
            c.setRHSConstant(-s.getX_ct());
            executor.addConstraint(c);
            //distS_xj >= xs_j - x_j
            c = new GurobiConstraint();
            c.addToLHS(pv.distS_xs.get(s), 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pv.x, -1);
            c.setRHSConstant(s.getX_ct());
            executor.addConstraint(c);
            //distS_yj >= y_j - ys_j
            c = new GurobiConstraint();
            c.addToLHS(pv.distS_ys.get(s), 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pv.y, 1);
            c.setRHSConstant(-s.getY_ct());
            executor.addConstraint(c);
            //distS_yj >= ys_j - y_j
            c = new GurobiConstraint();
            c.addToLHS(pv.distS_ys.get(s), 1);
            c.setSense(GRB.GREATER_EQUAL);
            c.addToRHS(pv.y, -1);
            c.setRHSConstant(s.getY_ct());
            executor.addConstraint(c);


        }


        /*
         * AAAAAAAAAAAAAAAAAAAAAAAAA
         * SETUP CONSTRAINS FOR LAST POINT
         */

        executor.addConstraint(c_ConnectSlave);
        executor.addConstraint(c_busLength);
        executor.addConstraint(c_sideBusLength);


        /*
         * Each Slave must connect one IP
         */
        for (Slave s : slaves) {
            c = new GurobiConstraint();
            c.setRHSConstant(1);
            c.setSense(GRB.EQUAL);
            for (PointVar pvv : pointVars) {
                c.addToLHS(pvv.s_qVars.get(s), 1);
            }
            executor.addConstraint(c);
        }
    }

    /**
     * Build the constraints for Model with Keepout
     * The abs values are solved by GRB.GenConstraints
     * The coordinate are INT Type
     *
     * @param mv            pseudo_Var for master
     * @param slaveVars_ko  Arraylist of pseudo_Vars for slaves
     * @param s_cnt         number of slaves
     * @param pointVars_ko  Arraylist of all pointVars_ko
     * @param uni_keepouts  Arraylist of the single keepouts
     * @param poly_keepouts Arraylist of the merged keepouts
     * @param all_keepouts  Arraylist of all keepouts
     * @param busLength     Gurobi-variable of bus-length
     * @param sideBusLength Gurobi-varibale of branch-length
     * @throws GRBException throws GRBException
     */
    private void buildCons_w_KO(Master_var mv, ArrayList<Slave_var> slaveVars_ko, int s_cnt, ArrayList<PointVar_ko> pointVars_ko, ArrayList<Keepout> uni_keepouts, ArrayList<Poly_Keepout> poly_keepouts, ArrayList<Keepout> all_keepouts, GurobiVariable busLength, GurobiVariable sideBusLength) throws GRBException {
        double eps = 1;

        GurobiConstraint c;
        GurobiQuadConstraint qc;
        PointVar_ko pv_ko;
        PointVar_ko pvNext_ko;

        GurobiVariable[] sp_q;
        GurobiVariable[] sp_iq;
        GurobiVariable[] ko_sp_q;
        GurobiVariable[] ko_sp_qNext;
        GurobiVariable[] ko_sp_iq;
        GurobiVariable[] ko_sp_iqNext;
        GurobiVariable[] ko_sl_q;
        GurobiVariable[] ko_sl_iq;
        GurobiVariable[] sl_q;
        GurobiVariable[] sl_iq;
        //only for v1 <-> ms
        GurobiVariable[] ms_sp_q;
        GurobiVariable[] ms_sp_iq;
        GurobiVariable[] ms_ko_sp_q;
        GurobiVariable[] ms_ko_sp_iq;


        int[] sv_q;
        int[] sv_iq;
        int[] ms_q;
        int[] ms_iq;


        GurobiConstraint dq_i_j_greater_c;
        GurobiConstraint dq_i_sj_greater_c;
        GurobiConstraint dq_1_ms_greater_c;
        GurobiConstraint vs_connecting_c;
        GurobiConstraint c_busLength;
        GurobiConstraint c_sideBusLength;

        //setup for buslength
        c_busLength = new GurobiConstraint();
        c_busLength.addToLHS(busLength, 1.0);
        c_busLength.setSense('=');
        executor.addConstraint(c_busLength);

        //setup for sidebuslength
        c_sideBusLength = new GurobiConstraint();
        c_sideBusLength.addToLHS(sideBusLength, 1.0);
        c_sideBusLength.setSense('=');
        executor.addConstraint(c_sideBusLength);



        /*
         * Setup EXTRA constraints for the 1st steiner point
         * v_1 <-> ms
         * VVVV
         */
        {
            pv_ko = pointVars_ko.get(0);
            ms_sp_q = pv_ko.ms_sp_bVars;
            ms_sp_iq = pv_ko.ms_sp_iVars;

            //add d(v_1,ms) to busLength
            c_busLength.addToRHS(ms_sp_iq[0], 1.0);


            /*
             * Compute d(v_1, ms) w.r.t. Manhattan distance:
             * VVVV
             * d(v_1, ms) >= |v_1.x - ms.x| + |v_1.y - ms.y|
             */
            c = new GurobiConstraint();
            c.addToLHS(ms_sp_iq[0], 1.0);
            c.setSense('>');
            c.addToRHS(ms_sp_iq[1], 1.0);
            c.addToRHS(ms_sp_iq[2], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(ms_sp_iq[3], 1.0);
            c.setSense('=');
            c.addToRHS(pv_ko.x, 1.0);
            c.setRHSConstant(-mv.getX_ct());
            executor.addConstraint(c);
            executor.addGenConstraintAbs(ms_sp_iq[1], ms_sp_iq[3], "_abs_v_1.x - ms.x");
            c = new GurobiConstraint();
            c.addToLHS(ms_sp_iq[4], 1.0);
            c.setSense('=');
            c.addToRHS(pv_ko.y, 1.0);
            c.setRHSConstant(-mv.getY_ct());
            executor.addConstraint(c);
            executor.addGenConstraintAbs(ms_sp_iq[2], ms_sp_iq[4], "_abs_v_1.y - ms.y");

            /*
             * AAAA
             * Compute d(v_1, ms) w.r.t. Manhattan distance:
             */

            /*
            Orientation v1 <-> ms
            VVVV
             */
            {
                /*
                ms is on the LEFT to v_i
                ms.x <= v_i.x - eps + (1 - qL_i_ms) * M
                ms.x >= v_i.x - qL_i_ms * M
                 */
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getX_ct());
                c.setSense('<');
                c.addToRHS(pv_ko.x, 1.0);
                c.addToRHS(ms_sp_q[0], -M);
                c.setRHSConstant(-eps + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getX_ct());
                c.setSense('>');
                c.addToRHS(pv_ko.x, 1.0);
                c.addToRHS(ms_sp_q[0], -M);
                executor.addConstraint(c);


                /*
                ms is on the RIGHT to v_i
                ms.x >= v_i.x + eps - (1 - qR_i_ms) * M
                ms.x <= v_i.x + qR_i_ms * M
                 */
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getX_ct());
                c.setSense('>');
                c.addToRHS(pv_ko.x, 1.0);
                c.addToRHS(ms_sp_q[1], M);
                c.setRHSConstant(eps - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getX_ct());
                c.setSense('<');
                c.addToRHS(pv_ko.x, 1.0);
                c.addToRHS(ms_sp_q[1], M);
                executor.addConstraint(c);

                /*
                ms is on the ABOVE to v_i
                ms.y >= v_i.y + eps - (1 - qA_i_ms) * M
                ms.y <= v_i.y + qA_i_ms * M
                 */
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getY_ct());
                c.setSense('>');
                c.addToRHS(pv_ko.y, 1.0);
                c.addToRHS(ms_sp_q[2], M);
                c.setRHSConstant(eps - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getY_ct());
                c.setSense('<');
                c.addToRHS(pv_ko.y, 1.0);
                c.addToRHS(ms_sp_q[2], M);
                executor.addConstraint(c);

                /*
                ms is on the BELOW to v_i
                ms.y <= v_i.x - eps + (1 - qB_i_ms) * M
                ms.y >= v_i.x - qB_i_ms * M
                 */
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getY_ct());
                c.setSense('<');
                c.addToRHS(pv_ko.y, 1.0);
                c.addToRHS(ms_sp_q[3], -M);
                c.setRHSConstant(-eps + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setLHSConstant(mv.getY_ct());
                c.setSense('>');
                c.addToRHS(pv_ko.y, 1.0);
                c.addToRHS(ms_sp_q[3], -M);
                executor.addConstraint(c);

            }

            /*
            AAAA
            orientation v1 <-> ms
             */

            /*
             * detour trigger with next steiner point w.r.t. the unique keepout
             * VVVV
             * dq_1_ms >= sum_o oq^d_1_ms - (#o - 1)
             */
            dq_1_ms_greater_c = new GurobiConstraint();
            executor.addConstraint(dq_1_ms_greater_c);
            dq_1_ms_greater_c.addToLHS(ms_sp_q[4], 1.0);
            dq_1_ms_greater_c.setSense('>');
            for (Keepout all_k : all_keepouts) {
                ko_sp_q = pv_ko.ko_sp_bVars.get(all_k);
                ko_sp_iq = pv_ko.ko_sp_iVars.get(all_k);
                ms_ko_sp_q = pv_ko.ms_ko_sp_bVars.get(all_k);
                ms_ko_sp_iq = pv_ko.ms_ko_sp_iVars.get(all_k);

                ms_q = mv.pseudo_bVars.get(all_k);
                ms_iq = mv.pseudo_iVars.get(all_k);


                /*
                (ms: LR.1) 1_nonR + 1_nonA + 1_nonB + (ms)_nonL + (ms)_nonA + (ms)_nonB <= oq^LR_1_ms * M
                (ms: LR.2) 1_nonR + 1_nonA + 1_nonB + (ms)_nonL + (ms)_nonA + (ms)_nonB >= oq^LR_1_ms
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.setLHSConstant(ms_q[0] + ms_q[2] + ms_q[3]);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[0], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.setLHSConstant(ms_q[0] + ms_q[2] + ms_q[3]);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_q[0], 1.0);
                executor.addConstraint(c);

                /*
                (ms: RL.1) 1_nonL + 1_nonA + 1_nonB + (ms)_nonR + (ms)_nonA + (ms)_nonB <= oq^RL_1_ms * M
                (ms: RL.2) 1_nonL + 1_nonA + 1_nonB + (ms)_nonR + (ms)_nonA + (ms)_nonB >= oq^RL_1_ms
                */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.setLHSConstant(ms_q[1] + ms_q[2] + ms_q[3]);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[1], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.setLHSConstant(ms_q[1] + ms_q[2] + ms_q[3]);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_q[1], 1.0);
                executor.addConstraint(c);

                /*
                (ms: AB.1) 1_nonB + 1_nonL + 1_nonR + (ms)_nonA + (ms)_nonL + (ms)_nonR <= oq^AB_1_ms * M
                (ms: AB.2) 1_nonB + 1_nonL + 1_nonR + (ms)_nonA + (ms)_nonL + (ms)_nonR >= oq^AB_1_ms
                */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.setLHSConstant(ms_q[2] + ms_q[0] + ms_q[1]);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[2], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.setLHSConstant(ms_q[2] + ms_q[0] + ms_q[1]);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_q[2], 1.0);
                executor.addConstraint(c);

                /*
                (ms: BA.1) 1_nonA + 1_nonL + 1_nonR + (ms)_nonB + (ms)_nonL + (ms)_nonR <= oq^BA_1_ms * M
                (ms: BA.2) 1_nonA + 1_nonL + 1_nonR + (ms)_nonB + (ms)_nonL + (ms)_nonR >= oq^BA_1_ms
                */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.setLHSConstant(ms_q[3] + ms_q[0] + ms_q[1]);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[3], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.setLHSConstant(ms_q[3] + ms_q[0] + ms_q[1]);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_q[3], 1.0);
                executor.addConstraint(c);


                /*
                ms: detour trigger for each keepout oq^d_1_ms
                (oq^d_1_ms = oqLR_1_ms * oqRL_1_ms * oqAB_1_ms * oqBA_1_ms):
                oq^d_1_ms <= oq^LR_1_ms
                oq^d_1_ms <= oq^RL_1_ms
                oq^d_1_ms <= oq^AB_1_ms
                oq^d_1_ms <= oq^BA_1_ms
                oq^d_1_ms >= oq^LR_1_ms + oq^RL_1_ms + oq^AB_1_ms + oq^BA_1_ms - 3
                 */
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_q[4], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_q[4], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_q[4], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[2], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_q[4], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[3], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_q[4], 1.0);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_q[0], 1.0);
                c.addToRHS(ms_ko_sp_q[1], 1.0);
                c.addToRHS(ms_ko_sp_q[2], 1.0);
                c.addToRHS(ms_ko_sp_q[3], 1.0);
                c.setRHSConstant(-3);
                executor.addConstraint(c);

                /*
                dq_1_ms <= oq^d_1_ms
                 */
                c = new GurobiConstraint();
                c.addToLHS(ms_sp_q[4], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_q[4], 1.0);
                executor.addConstraint(c);

                /*
                dq_1_ms >= sum_o oq^d_1_ms - (#o - 1)
                 */
                dq_1_ms_greater_c.addToRHS(ms_ko_sp_q[4], 1.0);

                /*
                d(v_1, o^LL, ms) = vo^LL_1.x + vo^LL_1.y + |ms.x - c^k_LL.x| + |ms.y - c^k_LL.y|
                d(v_1, o^UR, ms) = vo^UR_1.x + vo^UR_1.y + |ms.x - c^k_UR.x| + |ms.y - c^k_UR.y|
                d(v_1, o, ms) >= d(v_1, o^LL, ms) - aux_oq_1_ms * M
                d(v_1, o, ms) <= d(v_1, o^LL, ms) + aux_oq_1_ms * M
                d(v_1, o, ms) >= d(v_1, o^UR, ms) - (1 - aux_oq_1_ms) * M
                d(v_1, o, ms) <= d(v_1, o^UR, ms) + (1 - aux_oq_1_ms) * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_iq[1], 1.0);
                c.setSense('=');
                c.addToRHS(ko_sp_iq[0], 1.0);
                c.addToRHS(ko_sp_iq[1], 1.0);
                c.setRHSConstant(ms_iq[0] + ms_iq[1]);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_iq[2], 1.0);
                c.setSense('=');
                c.addToRHS(ko_sp_iq[2], 1.0);
                c.addToRHS(ko_sp_iq[3], 1.0);
                c.setRHSConstant(ms_iq[2] + ms_iq[3]);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_iq[1], 1.0);
                c.addToRHS(ms_ko_sp_q[5], -M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_iq[0], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_iq[1], 1.0);
                c.addToRHS(ms_ko_sp_q[5], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_iq[2], 1.0);
                c.addToRHS(ms_ko_sp_q[5], M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ms_ko_sp_iq[0], 1.0);
                c.setSense('<');
                c.addToRHS(ms_ko_sp_iq[2], 1.0);
                c.addToRHS(ms_ko_sp_q[5], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);


                /*
                d(v_1, ms) >= d(v_1, o, ms) - oq^d_1_ms * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(ms_sp_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(ms_ko_sp_iq[0], 1.0);
                c.addToRHS(ms_ko_sp_q[4], -M);
                executor.addConstraint(c);


            }
            dq_1_ms_greater_c.setRHSConstant(-all_keepouts.size() + 1.0);


        }
        /*
         * AAAA
         * Setup EXTRA constraints for the 1st steiner point
         */


        /*
         * Setup constraints for 0 --- pv.size()-1
         * VVVV
         * steiner points <-> steiner points
         */
        for (int j = 0; j < pointVars_ko.size() - 1; j++) {

            pv_ko = pointVars_ko.get(j);
            pvNext_ko = pointVars_ko.get(j + 1);
            sp_iq = pv_ko.sp_iVars;
            sp_q = pv_ko.sp_bVars;

            //add d(v_i, v_i+1) to busLength
            c_busLength.addToRHS(sp_iq[0], 1.0);


            /*
            d(v_i, v_i+1) >= |v_i.x - v_i+1.x| + |v_i.y - v_i+1.y|
             */
            c = new GurobiConstraint();
            c.addToLHS(sp_iq[0], 1.0);
            c.setSense('>');
            c.addToRHS(sp_iq[2], 1.0);
            c.addToRHS(sp_iq[3], 1.0);
            executor.addConstraint(c);
            /*
            |v_i.x - v_i+1.x| >= v_i.x - v_i+1.x
            |v_i.x - v_i+1.x| >= v_i+1.x - v_i.x
             */
            c = new GurobiConstraint();
            c.addToLHS(sp_iq[2], 1.0);
            c.setSense('>');
            c.addToRHS(pv_ko.x, 1.0);
            c.addToRHS(pvNext_ko.x, -1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(sp_iq[2], 1.0);
            c.setSense('>');
            c.addToRHS(pv_ko.x, -1.0);
            c.addToRHS(pvNext_ko.x, 1.0);
            executor.addConstraint(c);

            /*
            |v_i.y - v_i+1.y| >= v_i.y - v_i+1.y
            |v_i.y - v_i+1.y| >= v_i+1.y - v_i.y
             */
            c = new GurobiConstraint();
            c.addToLHS(sp_iq[3], 1.0);
            c.setSense('>');
            c.addToRHS(pv_ko.y, 1.0);
            c.addToRHS(pvNext_ko.y, -1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(sp_iq[3], 1.0);
            c.setSense('>');
            c.addToRHS(pv_ko.y, -1.0);
            c.addToRHS(pvNext_ko.y, 1.0);
            executor.addConstraint(c);


            /*
             * detour trigger with next steiner point w.r.t. uni_keepout
             * VVVV
             * dq_ij >= sum_o oq^d_ij - (#o - 1)
             */
            dq_i_j_greater_c = new GurobiConstraint();
            dq_i_j_greater_c.addToLHS(sp_q[4], 1.0);
            dq_i_j_greater_c.setSense('>');
            executor.addConstraint(dq_i_j_greater_c);
            for (Keepout all_k : all_keepouts) {
                ko_sp_q = pv_ko.ko_sp_bVars.get(all_k);
                ko_sp_qNext = pvNext_ko.ko_sp_bVars.get(all_k);
                ko_sp_iq = pv_ko.ko_sp_iVars.get(all_k);
                ko_sp_iqNext = pvNext_ko.ko_sp_iVars.get(all_k);


                /*
                (LR.1) i_nonR + i_nonA + i_nonB + (i+1)_nonL + (i+1)_nonA + (i+1)_nonB <= oq^LR_ij * M
                (LR.2) i_nonR + i_nonA + i_nonB + (i+1)_nonL + (i+1)_nonA + (i+1)_nonB >= oq^LR_ij
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_qNext[0], 1.0);
                c.addToLHS(ko_sp_qNext[2], 1.0);
                c.addToLHS(ko_sp_qNext[3], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[4], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_qNext[0], 1.0);
                c.addToLHS(ko_sp_qNext[2], 1.0);
                c.addToLHS(ko_sp_qNext[3], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_q[4], 1.0);
                executor.addConstraint(c);

                /*
                (RL.1) i_nonL + i_nonA + i_nonB + (i+1)_nonR + (i+1)_nonA + (i+1)_nonB <= oq^RL_ij * M
                (RL.2) i_nonL + i_nonA + i_nonB + (i+1)_nonR + (i+1)_nonA + (i+1)_nonB >= oq^RL_ij
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_qNext[1], 1.0);
                c.addToLHS(ko_sp_qNext[2], 1.0);
                c.addToLHS(ko_sp_qNext[3], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[5], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_qNext[1], 1.0);
                c.addToLHS(ko_sp_qNext[2], 1.0);
                c.addToLHS(ko_sp_qNext[3], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_q[5], 1.0);
                executor.addConstraint(c);


                /*
                (AB.1) i_nonB + i_nonL + i_nonR + (i+1)_nonA + (i+1)_nonL + (i+1)_nonR <= oq^AB_ij * M
                (AB.2) i_nonB + i_nonL + i_nonR + (i+1)_nonA + (i+1)_nonL + (i+1)_nonR >= oq^AB_ij
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_qNext[2], 1.0);
                c.addToLHS(ko_sp_qNext[0], 1.0);
                c.addToLHS(ko_sp_qNext[1], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[6], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[3], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_qNext[2], 1.0);
                c.addToLHS(ko_sp_qNext[0], 1.0);
                c.addToLHS(ko_sp_qNext[1], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_q[6], 1.0);
                executor.addConstraint(c);

                /*
                (BA.1) i_nonA + i_nonL + i_nonR + (i+1)_nonB + (i+1)_nonL + (i+1)_nonR <= oq^BA_ij * M
                (BA.2) i_nonA + i_nonL + i_nonR + (i+1)_nonB + (i+1)_nonL + (i+1)_nonR >= oq^BA_ij
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_qNext[3], 1.0);
                c.addToLHS(ko_sp_qNext[0], 1.0);
                c.addToLHS(ko_sp_qNext[1], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[7], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_qNext[3], 1.0);
                c.addToLHS(ko_sp_qNext[0], 1.0);
                c.addToLHS(ko_sp_qNext[1], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_q[7], 1.0);
                executor.addConstraint(c);

                /*
                detour trigger for each keepout [8] oq^d_ij
                (oq^LR_ij * oq^RL_ij * oq^AB_ij * oq^BA_ij = oq^d_ij):
                oq^d_ij <= oq^LR_ij
                oq^d_ij <= oq^RL_ij
                oq^d_ij <= oq^AB_ij
                oq^d_ij <= oq^BA_ij
                oq^d_ij >= oq^LR_ij + oq^RL_ij + oq^AB_ij + oq^BA_ij - 3
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[8], 1);
                c.setSense('<');
                c.addToRHS(ko_sp_q[4], 1);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[8], 1);
                c.setSense('<');
                c.addToRHS(ko_sp_q[5], 1);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[8], 1);
                c.setSense('<');
                c.addToRHS(ko_sp_q[6], 1);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[8], 1);
                c.setSense('<');
                c.addToRHS(ko_sp_q[7], 1);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[8], 1);
                c.setSense('>');
                c.addToRHS(ko_sp_q[4], 1);
                c.addToRHS(ko_sp_q[5], 1);
                c.addToRHS(ko_sp_q[6], 1);
                c.addToRHS(ko_sp_q[7], 1);
                c.setRHSConstant(-3);
                executor.addConstraint(c);

                /*
                dq_ij <= oq^d_ij
                 */
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.sp_bVars[4], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[8], 1.0);
                executor.addConstraint(c);

                /*
                dq_ij >= sum_o oq^d_ij - (#o - 1)
                 */
                dq_i_j_greater_c.addToRHS(ko_sp_q[8], 1.0);





                /*
                aux_oq_ij : ko_sp_bVars[9]
                d(v_i, o, v_i+1): ko_sp_iq[4]
                d(v_i, o^LL, v_i+1) = vo^LL_i.x + vo^LL_i.y + vo^LL_i+1.x + vo^LL_i+1.y
                d(v_i, o^UR, v_i+1) = vo^UR_i.x + vo^UR_i.y + vo^UR_i+1.x + vo^UR_i+1.y
                d(v_i, o, v_i+1) >= d(v_i, o^LL, v_i+1) - aux_oq_ij * M
                d(v_i, o, v_i+1) <= d(v_i, o^LL, v_i+1) + aux_oq_ij * M
                d(v_i, o, v_i+1) >= d(v_i, o^UR, v_i+1) - (1 - aux_oq_ij) * M
                d(v_i, o, v_i+1) <= d(v_i, o^UR, v_i+1) + (1 - aux_oq_ij) * M
                d(v_i, o, v_i+1) = d(v_i, o^LL, v_i+1) * (1 - aux_oq_ij) + d(v_i, o^UR, v_i+1) * aux_oq_ij
                                 = d(v_i, o^LL, v_i+1) - d(v_i, o^LL, v_i+1) * aux_oq_ij + d(v_i, o^UR, v_i+1) * aux_oq_ij
                 */

                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[4], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_iq[0], 1.0);
                c.addToRHS(ko_sp_iqNext[0], 1.0);
                c.addToRHS(ko_sp_iq[1], 1.0);
                c.addToRHS(ko_sp_iqNext[1], 1.0);
                c.addToRHS(ko_sp_q[9], -M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[4], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_iq[0], 1.0);
                c.addToRHS(ko_sp_iqNext[0], 1.0);
                c.addToRHS(ko_sp_iq[1], 1.0);
                c.addToRHS(ko_sp_iqNext[1], 1.0);
                c.addToRHS(ko_sp_q[9], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[4], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_iq[2], 1.0);
                c.addToRHS(ko_sp_iqNext[2], 1.0);
                c.addToRHS(ko_sp_iq[3], 1.0);
                c.addToRHS(ko_sp_iqNext[3], 1.0);
                c.addToRHS(ko_sp_q[9], M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[4], 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_iq[2], 1.0);
                c.addToRHS(ko_sp_iqNext[2], 1.0);
                c.addToRHS(ko_sp_iq[3], 1.0);
                c.addToRHS(ko_sp_iqNext[3], 1.0);
                c.addToRHS(ko_sp_q[9], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);


                /*
                d(v_i, v_i+1) >= d(v_i, o, v_i+1) - oq^d_ij * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(sp_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_iq[4], 1.0);
                c.addToRHS(ko_sp_q[8], -M);
                executor.addConstraint(c);

            }
            dq_i_j_greater_c.setRHSConstant(-all_keepouts.size() + 1.0);

            /*
             * AAAA
             * detour trigger with next steiner point w.r.t. uni_keepout
             */


        }
        /*
         * AAAA
         * Setup constraints for 0 --- pv.size()-1
         */

        /*
         * steiner points <-> slaves
         * Setup constraints for 0 --- pv.size()
         * VVVV
         * Each steiner point connects only one slave (vs_connecting_c): sum_j q_i_sj = 1
         */

        for (PointVar_ko pointVar_ko : pointVars_ko) {
            vs_connecting_c = new GurobiConstraint();
            vs_connecting_c.setRHSConstant(1.0);
            vs_connecting_c.setSense('=');
            executor.addConstraint(vs_connecting_c);

            pv_ko = pointVar_ko;
            sp_iq = pv_ko.sp_iVars;

            //add d(v_i, corrS) to sidebusLength
            c_sideBusLength.addToRHS(sp_iq[1], 1.0);

            /*
             * Non-overlapping for each uni_keepouts
             * VVVV
             */
            for (Keepout o : uni_keepouts) {
                ko_sp_q = pv_ko.ko_sp_bVars.get(o);

                //nonL + nonR + nonA + nonB >= 1
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_q[0], 1.0);
                c.addToLHS(ko_sp_q[1], 1.0);
                c.addToLHS(ko_sp_q[2], 1.0);
                c.addToLHS(ko_sp_q[3], 1.0);
                c.setSense('>');
                c.setRHSConstant(1.0);
                executor.addConstraint(c);

                /*
                LEFT:
                v_i.x <= o.minx - eps + (1 - nonL) * M
                      <= -M * nonL + (o.minx - eps + M)
                v_i.x >= o.minx - nonL * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.x, 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[0], -M);
                c.setRHSConstant(o.getMinX() - eps + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.x, 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_q[0], -M);
                c.setRHSConstant(o.getMinX());
                executor.addConstraint(c);

                /*
                Right:
                v_i.x >= o.maxx + eps - (1 - nonR) * M
                v_i.x <= o.maxx + nonR * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.x, 1);
                c.setSense('>');
                c.addToRHS(ko_sp_q[1], M);
                c.setRHSConstant(o.getMaxX() + eps - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.x, 1);
                c.setSense('<');
                c.addToRHS(ko_sp_q[1], M);
                c.setRHSConstant(o.getMaxX());
                executor.addConstraint(c);

                /*
                Above:
                v_i.y >= o.maxy + eps - (1 - nonA) * M
                      >= M * nonA + (o.maxy + eps - M)
                v_i.y <= o.maxy + nonA * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.y, 1);
                c.setSense('>');
                c.addToRHS(ko_sp_q[2], M);
                c.setRHSConstant(o.getMaxY() + eps - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.y, 1);
                c.setSense('<');
                c.addToRHS(ko_sp_q[2], M);
                c.setRHSConstant(o.getMaxY());
                executor.addConstraint(c);

                /*
                Below:
                v_i.y <= o.miny - eps + (1 - nonB)M
                    <= -M * nonB + o.miny - eps + M
                v_i.y >= o.miny - nonB * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.y, 1.0);
                c.setSense('<');
                c.addToRHS(ko_sp_q[3], -M);
                c.setRHSConstant(o.getMinY() - eps + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(pv_ko.y, 1.0);
                c.setSense('>');
                c.addToRHS(ko_sp_q[3], -M);
                c.setRHSConstant(o.getMinY());
                executor.addConstraint(c);


            }
            /*
             * AAAA
             * Non-overlapping for each uni_keepouts
             */

            /*
             * Compute the distance of steiner point and each keepout.LL and keepout.UR
             */
            for (Keepout all_k : all_keepouts) {
                ko_sp_iq = pv_ko.ko_sp_iVars.get(all_k);
                /*
                0: |v_i.x - c^k_1.x| (vo^LL_i.x)
                vo^LL_i.x >= v_i.x - c^k_LL.x
                vo^LL_i.x >= c^k_LL.x - v_i.x
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(pv_ko.x, 1.0);
                c.setRHSConstant(-all_k.getMinX());
                //executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(pv_ko.x, -1.0);
                c.setRHSConstant(all_k.getMinX());
                //executor.addConstraint(c);
                /*
                0: |v_i.x - c^k_1.x| (vo^LL_i.x)
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[5], 1);
                c.setSense('=');
                c.addToRHS(pv_ko.x, 1);
                c.setRHSConstant(-all_k.getMinX());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_sp_iq[0], ko_sp_iq[5], "vo^LL_i.x");

                /*
                1: |v_i.y - c^k_1.y| (vo^LL_i.y)
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[6], 1);
                c.setSense('=');
                c.addToRHS(pv_ko.y, 1);
                c.setRHSConstant(-all_k.getMinY());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_sp_iq[1], ko_sp_iq[6], "vo^LL_i.y");

                /*
                2: |v_i.x - c^k_UR.x| (vo^UR_i.x)
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[7], 1);
                c.setSense('=');
                c.addToRHS(pv_ko.x, 1);
                c.setRHSConstant(-all_k.getMaxX());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_sp_iq[2], ko_sp_iq[7], "vo^UR_i.x");

                /*
                3: |v_i.y - c^k_UR.y| (vo^UR_i.y)
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_sp_iq[8], 1);
                c.setSense('=');
                c.addToRHS(pv_ko.y, 1);
                c.setRHSConstant(-all_k.getMaxY());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_sp_iq[3], ko_sp_iq[8], "vo^LL_i.y");
            }

            /*
             * steiner points vs. slaves
             * VVVV
             */
            /*
            d(v_i, corrS) = sum_j d(v_i, s_j) * q_i_sj
             */


            for (Slave_var sv : slaveVars_ko) {
                sl_q = pv_ko.sl_bVars.get(sv);
                sl_iq = pv_ko.sl_iVars.get(sv);
                vs_connecting_c.addToLHS(sl_q[5], 1.0);

                /*
                d(v_i, corr.S) >= d(v_i, s_j) - (1 - q_i_sj) * M
                d(v_i, corr.S) <= d(v_i, s_j) + (1 - q_i_sj) * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(sp_iq[1], 1.0);
                c.setSense('>');
                c.addToRHS(sl_iq[0], 1.0);
                c.addToRHS(sl_q[5], M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(sp_iq[1], 1.0);
                c.setSense('<');
                c.addToRHS(sl_iq[0], 1.0);
                c.addToRHS(sl_q[5], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);


                /*
                d(v_i, s_j) >= |v_i.x - s_j.x| + |v_i.y - s_j.y|
                 */
                c = new GurobiConstraint();
                c.addToLHS(sl_iq[0], 1.0);
                c.setSense('>');
                c.addToRHS(sl_iq[1], 1.0);
                c.addToRHS(sl_iq[2], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(sl_iq[1], sl_iq[3], "abs_v_s_x");
                executor.addGenConstraintAbs(sl_iq[2], sl_iq[4], "abs_v_s_y");

                /*
                v_i.x - s_j.x
                v_i.y - s_j.y
                 */
                c = new GurobiConstraint();
                c.addToLHS(sl_iq[3], 1.0);
                c.setSense('=');
                c.addToRHS(pv_ko.x, 1.0);
                c.setRHSConstant(-sv.getX_ct());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(sl_iq[4], 1.0);
                c.setSense('=');
                c.addToRHS(pv_ko.y, 1.0);
                c.setRHSConstant(-sv.getY_ct());
                executor.addConstraint(c);




                /*
                Orientation of steiner point with each slaves
                 */

                {
                    /*
                    s_j is on the LEFT to v_i
                    s_j.x <= v_i.x - eps + (1 - qL_i_sj) * M
                    s_j.x >= v_i.x - qL_i_sj * M
                     */
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getX_ct());
                    c.setSense('<');
                    c.addToRHS(pv_ko.x, 1.0);
                    c.addToRHS(sl_q[0], -M);
                    c.setRHSConstant(-eps + M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getX_ct());
                    c.setSense('>');
                    c.addToRHS(pv_ko.x, 1.0);
                    c.addToRHS(sl_q[0], -M);
                    executor.addConstraint(c);

                    /*
                    s_j is on the RIGHT to v_i
                    s_j.x >= v_i.x + eps - (1 - qR_i_sj) * M
                    s_j.x <= v_i.x + qR_i_sj * M
                     */
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getX_ct());
                    c.setSense('>');
                    c.addToRHS(pv_ko.x, 1.0);
                    c.addToRHS(sl_q[1], M);
                    c.setRHSConstant(eps - M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getX_ct());
                    c.setSense('<');
                    c.addToRHS(pv_ko.x, 1.0);
                    c.addToRHS(sl_q[1], M);
                    executor.addConstraint(c);

                    /*
                    s_j is on the ABOVE to v_i
                    s_j.y >= v_i.y + eps - (1 - qA_i_sj) * M
                    s_j.y <= v_i.y + qA_i_sj * M
                     */
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getY_ct());
                    c.setSense('>');
                    c.addToRHS(pv_ko.y, 1.0);
                    c.addToRHS(sl_q[2], M);
                    c.setRHSConstant(eps - M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getY_ct());
                    c.setSense('<');
                    c.addToRHS(pv_ko.y, 1.0);
                    c.addToRHS(sl_q[2], M);
                    executor.addConstraint(c);

                    /*
                    s_j is on the BELOW to v_i
                    s_j.y <= v_i.x - eps + (1 - qB_i_sj) * M
                    s_j.y >= v_i.x - qB_i_sj * M
                     */
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getY_ct());
                    c.setSense('<');
                    c.addToRHS(pv_ko.y, 1.0);
                    c.addToRHS(sl_q[3], -M);
                    c.setRHSConstant(-eps + M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setLHSConstant(sv.getY_ct());
                    c.setSense('>');
                    c.addToRHS(pv_ko.y, 1.0);
                    c.addToRHS(sl_q[3], -M);
                    executor.addConstraint(c);

                }







                /*todo

                 */
                dq_i_sj_greater_c = new GurobiConstraint();
                dq_i_sj_greater_c.addToLHS(sl_q[4], 1.0);
                dq_i_sj_greater_c.setSense('>');
                executor.addConstraint(dq_i_sj_greater_c);
                for (Keepout all_k : all_keepouts) {

                    ko_sp_q = pv_ko.ko_sp_bVars.get(all_k);
                    ko_sp_iq = pv_ko.ko_sp_iVars.get(all_k);
                    ko_sl_q = pv_ko.ko_sl_bVars.get(all_k).get(sv);
                    ko_sl_iq = pv_ko.ko_sl_iVars.get(all_k).get(sv);

                    sv_q = sv.pseudo_bVars.get(all_k);
                    sv_iq = sv.pseudo_iVars.get(all_k);


                    /*
                    (vs: LR.1) i_nonR + i_nonA + i_nonB + [(s_j)_nonL + (s_j)_nonA + (s_j)_nonB] <= oq^LR_i_sj * M
                    (vs: LR.2) i_nonR + i_nonA + i_nonB + [(s_j)_nonL + (s_j)_nonA + (s_j)_nonB] >= oq^LR_i_sj
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[1], 1.0);
                    c.addToLHS(ko_sp_q[2], 1.0);
                    c.addToLHS(ko_sp_q[3], 1.0);
                    c.setLHSConstant(sv_q[0] + sv_q[2] + sv_q[3]);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[0], M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[1], 1.0);
                    c.addToLHS(ko_sp_q[2], 1.0);
                    c.addToLHS(ko_sp_q[3], 1.0);
                    c.setLHSConstant(sv_q[0] + sv_q[2] + sv_q[3]);
                    c.setSense('>');
                    c.addToRHS(ko_sl_q[0], 1.0);
                    executor.addConstraint(c);

                    /*
                    (vs: RL.1) i_nonL + i_nonA + i_nonB + (s_j)_nonR + (s_j)_nonA + (s_j)_nonB <= oq^RL_i_sj * M
                    (vs: RL.2) i_nonL + i_nonA + i_nonB + (s_j)_nonR + (s_j)_nonA + (s_j)_nonB >= oq^RL_i_sj
                    */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[0], 1.0);
                    c.addToLHS(ko_sp_q[2], 1.0);
                    c.addToLHS(ko_sp_q[3], 1.0);
                    c.setLHSConstant(sv_q[1] + sv_q[2] + sv_q[3]);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[1], M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[0], 1.0);
                    c.addToLHS(ko_sp_q[2], 1.0);
                    c.addToLHS(ko_sp_q[3], 1.0);
                    c.setLHSConstant(sv_q[1] + sv_q[2] + sv_q[3]);
                    c.setSense('>');
                    c.addToRHS(ko_sl_q[1], 1.0);
                    executor.addConstraint(c);

                    /*
                    (vs: AB.1) i_nonB + i_nonL + i_nonR + (s_j)_nonA + (s_j)_nonL + (s_j)_nonR <= oq^AB_i_sj * M
                    (vs: AB.2) i_nonB + i_nonL + i_nonR + (s_j)_nonA + (s_j)_nonL + (s_j)_nonR >= oq^AB_i_sj
                    */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[3], 1.0);
                    c.addToLHS(ko_sp_q[0], 1.0);
                    c.addToLHS(ko_sp_q[1], 1.0);
                    c.setLHSConstant(sv_q[2] + sv_q[0] + sv_q[1]);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[2], M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[3], 1.0);
                    c.addToLHS(ko_sp_q[0], 1.0);
                    c.addToLHS(ko_sp_q[1], 1.0);
                    c.setLHSConstant(sv_q[2] + sv_q[0] + sv_q[1]);
                    c.setSense('>');
                    c.addToRHS(ko_sl_q[2], 1.0);
                    executor.addConstraint(c);

                    /*
                    (vs: BA.1) i_nonA + i_nonL + i_nonR + (s_j)_nonB + (s_j)_nonL + (s_j)_nonR <= oq^BA_i_sj * M
                    (vs: BA.2) i_nonA + i_nonL + i_nonR + (s_j)_nonB + (s_j)_nonL + (s_j)_nonR >= oq^BA_i_sj
                    */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[2], 1.0);
                    c.addToLHS(ko_sp_q[0], 1.0);
                    c.addToLHS(ko_sp_q[1], 1.0);
                    c.setLHSConstant(sv_q[3] + sv_q[0] + sv_q[1]);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[3], M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sp_q[2], 1.0);
                    c.addToLHS(ko_sp_q[0], 1.0);
                    c.addToLHS(ko_sp_q[1], 1.0);
                    c.setLHSConstant(sv_q[3] + sv_q[0] + sv_q[1]);
                    c.setSense('>');
                    c.addToRHS(ko_sl_q[3], 1.0);
                    executor.addConstraint(c);

                    /*
                    vs: detour trigger for each keepout oq^d_i_sj
                    oq^d_i_sj <= oq^LR_i_sj
                    oq^d_i_sj <= oq^RL_i_sj
                    oq^d_i_sj <= oq^AB_i_sj
                    oq^d_i_sj <= oq^BA_i_sj
                    oq^d_i_sj >= oq^LR_i_sj + oq^RL_i_sj + oq^AB_i_sj + oq^BA_i_sj - 3
                    */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_q[4], 1);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[0], 1);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_q[4], 1);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[1], 1);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_q[4], 1);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[2], 1);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_q[4], 1);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[3], 1);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_q[4], 1);
                    c.setSense('>');
                    c.addToRHS(ko_sl_q[0], 1);
                    c.addToRHS(ko_sl_q[1], 1);
                    c.addToRHS(ko_sl_q[2], 1);
                    c.addToRHS(ko_sl_q[3], 1);
                    c.setRHSConstant(-3);
                    executor.addConstraint(c);


                    /*
                    dq_ij <= oq^d_i_sj
                    */
                    c = new GurobiConstraint();
                    c.addToLHS(sl_q[4], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_sl_q[4], 1.0);
                    executor.addConstraint(c);

                    /*
                    dq_i_sj >= sum_o oq^d_i_sj - (#o - 1)
                    */
                    dq_i_sj_greater_c.addToRHS(ko_sl_q[4], 1.0);

                    /*
                    d(v_i, o^LL, s_j) = vo^LL_i.x + vo^LL_i.y + (|s_j.x - c^k_LL.x| + |s_j.y - c^k_LL.y|)
                    d(v_i, o^UR, s_j) = vo^UR_i.x + vo^UR_i.y + (|s_j.x - c^k_UR.x| + |s_j.y - c^k_UR.y|)
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_iq[1], 1);
                    c.setSense('=');
                    c.addToRHS(ko_sp_iq[0], 1);
                    c.addToRHS(ko_sp_iq[1], 1);
                    c.setRHSConstant(sv_iq[0] + sv_iq[1]);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_iq[2], 1);
                    c.setSense('=');
                    c.addToRHS(ko_sp_iq[2], 1);
                    c.addToRHS(ko_sp_iq[3], 1);
                    c.setRHSConstant(sv_iq[2] + sv_iq[3]);
                    executor.addConstraint(c);

                    /*
                    d(v_i, o, s_j) >= d(v_i, o^LL, s_j) - aux_oq_i_sj * M
                                   >= vo^LL_i.x + vo^LL_i.y - aux_oq_i_sj * M + (|s_j.x - c^k_LL.x| + |s_j.y - c^k_LL.y|)
                    d(v_i, o, s_j) <= d(v_i, o^LL, s_j) + aux_oq_i_sj * M
                                   <= vo^LL_i.x + vo^LL_i.y + aux_oq_i_sj * M + (|s_j.x - c^k_LL.x| + |s_j.y - c^k_LL.y|)
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_iq[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_sl_iq[1], 1);
                    c.addToRHS(ko_sl_q[5], -M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_iq[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_sl_iq[1], 1);
                    c.addToRHS(ko_sl_q[5], M);
                    executor.addConstraint(c);

                    /*
                    d(v_i, o, s_j) >= d(v_i, o^UR, s_j) - (1 - aux_oq_i_sj) * M
                                   >= vo^UR_i.x + vo^UR_i.y + M * aux_oq_i_sj - M + (|s_j.x - c^k_UR.x| + |s_j.y - c^k_UR.y|)
                    d(v_i, o, s_j) <= d(v_i, o^UR, s_j) + (1 - aux_oq_i_sj) * M
                                   <= vo^UR_i.x + vo^UR_i.y - M * aux_oq_i_sj + M + (|s_j.x - c^k_UR.x| + |s_j.y - c^k_UR.y|)
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_iq[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_sl_iq[2], 1.0);
                    c.addToRHS(ko_sl_q[5], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_sl_iq[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_sl_iq[2], 1.0);
                    c.addToRHS(ko_sl_q[5], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);


                    /*
                    d(v_i, s_j) >= d(v_i, o, s_j) - oq^d_i_sj * M
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(sl_iq[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_sl_iq[0], 1.0);
                    c.addToRHS(ko_sl_q[4], -M);
                    executor.addConstraint(c);


                }
                dq_i_sj_greater_c.setRHSConstant(-all_keepouts.size() + 1.0);

            }

            /*
             * AAAA
             * steiner points vs. slaves
             */


        }
        /*
         * AAAA
         * Setup constraints for 0 --- pv.size()
         */


        /*
         * Each Slave must connect one IP
         */
        for (Slave_var sv : slaveVars_ko) {
            c = new GurobiConstraint();
            c.setRHSConstant(1);
            c.setSense('=');
            for (PointVar_ko pv_ko_2 : pointVars_ko) {
                c.addToLHS(pv_ko_2.sl_bVars.get(sv)[5], 1);
            }
            executor.addConstraint(c);
        }


    }

    private void buildCons_w_multiKO(Master_var mv, ArrayList<Slave_var> slaveVars, ArrayList<VP_var> VPvars, ArrayList<Keepout> uni_keepouts, ArrayList<Poly_Keepout> poly_keepouts, ArrayList<Keepout> all_keepouts, GurobiVariable busLength, GurobiVariable sideBusLength) throws GRBException {
        double eps = 1;

        GurobiConstraint c;
        GurobiQuadConstraint qc;


        GurobiVariable[] vp_q;
        GurobiVariable[] vp_iq_abs;
        GurobiVariable[] vp_iq;


        GurobiVariable[] ko_vp_q;
        GurobiVariable[] ko_vp_iq_abs;
        GurobiVariable[] ko_vp_iq;


        GurobiVariable[] ko_vp_qNext;
        GurobiVariable[] ko_vp_iqNext_abs;
        GurobiVariable[] ko_vp_iqNext;


        GurobiVariable[] sl_q;
        GurobiVariable[] sl_iq_abs;
        GurobiVariable[] sl_iq;

        GurobiVariable[] ko_sl_q;
        GurobiVariable[] ko_sl_iq_abs;
        GurobiVariable[] ko_sl_iq;


        //only for v1 <-> ms
        GurobiVariable[] ms_vp_q;
        GurobiVariable[] ms_vp_iq;
        GurobiVariable[] ms_ko_vp_q;
        GurobiVariable[] ms_ko_vp_iq;


        int[] sv_q;
        int[] sv_iq;
        int[] ms_q;
        int[] ms_iq;


        GurobiConstraint c_busLength;
        GurobiConstraint c_sideBusLength;

        //setup for buslength
        c_busLength = new GurobiConstraint();
        c_busLength.addToLHS(busLength, 1.0);
        c_busLength.setSense('=');
        executor.addConstraint(c_busLength);

        //setup for sidebuslength
        c_sideBusLength = new GurobiConstraint();
        c_sideBusLength.addToLHS(sideBusLength, 1.0);
        c_sideBusLength.setSense('=');
        executor.addConstraint(c_sideBusLength);



        /*
         * Setup EXTRA constraints for the 1st steiner point
         * v_1 <-> ms
         * VVVV
         */
//        {
//            vp = VPvars.get(0);
//            ms_vp_q = vp.ms_vp_bVars;
//            ms_vp_iq = vp.ms_vp_iVars;
//
//            //add d(v_1,ms) to busLength
//            c_busLength.addToRHS(ms_vp_iq[0], 1.0);
//
//
//            /*
//             * Compute d(v_1, ms) w.r.t. Manhattan distance:
//             * VVVV
//             * d(v_1, ms) >= |v_1.x - ms.x| + |v_1.y - ms.y|
//             */
//            c = new GurobiConstraint();
//            c.addToLHS(ms_vp_iq[0], 1.0);
//            c.setSense('>');
//            c.addToRHS(ms_vp_iq[1], 1.0);
//            c.addToRHS(ms_vp_iq[2], 1.0);
//            executor.addConstraint(c);
//            c = new GurobiConstraint();
//            c.addToLHS(ms_vp_iq[3], 1.0);
//            c.setSense('=');
//            c.addToRHS(vp.x, 1.0);
//            c.setRHSConstant(-mv.getX_ct());
//            executor.addConstraint(c);
//            executor.addGenConstraintAbs(ms_vp_iq[1], ms_vp_iq[3], "_abs_v_1.x - ms.x");
//            c = new GurobiConstraint();
//            c.addToLHS(ms_vp_iq[4], 1.0);
//            c.setSense('=');
//            c.addToRHS(vp.y, 1.0);
//            c.setRHSConstant(-mv.getY_ct());
//            executor.addConstraint(c);
//            executor.addGenConstraintAbs(ms_vp_iq[2], ms_vp_iq[4], "_abs_v_1.y - ms.y");
//
//            /*
//             * AAAA
//             * Compute d(v_1, ms) w.r.t. Manhattan distance:
//             */
//
//
//
//
//        }
        /*
         * AAAA
         * Setup EXTRA constraints for the 1st steiner point
         */


        /*
         * Setup constraints for 0 --- pv.size()-1
         * VVVV
         * virtual points <-> virtual points
         */
        for (int j = 0; j < VPvars.size() - 1; j++) {

            VP_var vp = VPvars.get(j);
            VP_var vpNext = VPvars.get(j + 1);

            vp_q = vp.vp_bVars;
            vp_iq = vp.vp_iVars;
            vp_iq_abs = vp.vp_iVars_abs;


            //add d(v_i, v_i+1) to busLength
            c_busLength.addToRHS(vp_iq_abs[0], 1.0);


            /*
            d(v_i, v_i+1) >= |v_i.x - v_i+1.x| + |v_i.y - v_i+1.y|
             */
            c = new GurobiConstraint();
            c.addToLHS(vp_iq_abs[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp_iq_abs[1], 1.0);
            c.addToRHS(vp_iq_abs[2], 1.0);
            executor.addConstraint(c);
            //|v_i.x - v_i+1.x| = abs(v_i.x - v_i+1.x)
            c = new GurobiConstraint();
            c.addToLHS(vp_iq[0], 1.0);
            c.setSense('=');
            c.addToRHS(vp.x, 1.0);
            c.addToRHS(vpNext.x, -1.0);
            executor.addConstraint(c);
            executor.addGenConstraintAbs(vp_iq_abs[1], vp_iq[0], "vv.x_abs");
            //|v_i.y - v_i+1.y| = abs(v_i.y - v_i+1.y)
            c = new GurobiConstraint();
            c.addToLHS(vp_iq[1], 1.0);
            c.setSense('=');
            c.addToRHS(vp.y, 1.0);
            c.addToRHS(vpNext.y, -1.0);
            executor.addConstraint(c);
            executor.addGenConstraintAbs(vp_iq_abs[2], vp_iq[1], "vv.y_abs");
            /*
            DT.7 (1/3)
             */
            GurobiQuadConstraint d_vv = new GurobiQuadConstraint();
            d_vv.addToLHS(vp_iq_abs[3], 1.0);
            d_vv.setSense('=');
            executor.addConstraint(d_vv);
            /*
            DT.8
             */
            c = new GurobiConstraint();
            c.addToLHS(vp_iq_abs[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp_iq_abs[3], 1.0);
            c.addToRHS(vp_q[4], M);
            executor.addConstraint(c);


            /*
            vp <-> vp
             */
            //(DT.1)
            c = new GurobiConstraint();
            c.addToLHS(vp_q[4], 1.0);
            c.setSense('<');
            c.addToRHS(vp_q[0], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(vp_q[4], 1.0);
            c.setSense('<');
            c.addToRHS(vp_q[1], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(vp_q[4], 1.0);
            c.setSense('<');
            c.addToRHS(vp_q[2], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(vp_q[4], 1.0);
            c.setSense('<');
            c.addToRHS(vp_q[3], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(vp_q[4], 1.0);
            c.setSense('>');
            c.addToRHS(vp_q[0], 1.0);
            c.addToRHS(vp_q[1], 1.0);
            c.addToRHS(vp_q[2], 1.0);
            c.addToRHS(vp_q[3], 1.0);
            c.setRHSConstant(-3.0);
            executor.addConstraint(c);


            //(RL.3)
            GurobiConstraint cRL_leq = new GurobiConstraint();
            cRL_leq.setSense('<');
            cRL_leq.addToRHS(vp_q[0], -M);
            cRL_leq.setRHSConstant(M);
            executor.addConstraint(cRL_leq);
            GurobiConstraint cRL_geq = new GurobiConstraint();
            cRL_geq.setSense('>');
            cRL_geq.addToRHS(vp_q[0], -1.0);
            cRL_geq.setRHSConstant(1.0);
            executor.addConstraint(cRL_geq);
            //(LR.3)
            GurobiConstraint cLR_leq = new GurobiConstraint();
            cLR_leq.setSense('<');
            cLR_leq.addToRHS(vp_q[1], -M);
            cLR_leq.setRHSConstant(M);
            executor.addConstraint(cLR_leq);
            GurobiConstraint cLR_geq = new GurobiConstraint();
            cLR_geq.setSense('>');
            cLR_geq.addToRHS(vp_q[1], -1.0);
            cLR_geq.setRHSConstant(1.0);
            executor.addConstraint(cLR_geq);
            //(AB.3)
            GurobiConstraint cAB_leq = new GurobiConstraint();
            cAB_leq.setSense('<');
            cAB_leq.addToRHS(vp_q[2], -M);
            cAB_leq.setRHSConstant(M);
            executor.addConstraint(cAB_leq);
            GurobiConstraint cAB_geq = new GurobiConstraint();
            cAB_geq.setSense('>');
            cAB_geq.addToRHS(vp_q[2], -1.0);
            cAB_geq.setRHSConstant(1.0);
            executor.addConstraint(cAB_geq);
            //(BA.3)
            GurobiConstraint cBA_leq = new GurobiConstraint();
            cBA_leq.setSense('<');
            cBA_leq.addToRHS(vp_q[2], -M);
            cBA_leq.setRHSConstant(M);
            executor.addConstraint(cBA_leq);
            GurobiConstraint cBA_geq = new GurobiConstraint();
            cBA_geq.setSense('>');
            cBA_geq.addToRHS(vp_q[2], -1.0);
            cBA_geq.setRHSConstant(1.0);
            executor.addConstraint(cBA_geq);

            /*
            DT.5 (3&4/4)
             */
            GurobiQuadConstraint dt_rule_out = new GurobiQuadConstraint();
            dt_rule_out.setSense('=');
            dt_rule_out.addToRHS(vp_q[4], -1.0);
            dt_rule_out.setRHSConstant(1.0);
            executor.addConstraint(dt_rule_out);
            GurobiQuadConstraint dt_rule_in = new GurobiQuadConstraint();
            dt_rule_in.setSense('=');
            dt_rule_in.addToRHS(vp_q[4], -1.0);
            dt_rule_in.setRHSConstant(1.0);
            executor.addConstraint(dt_rule_in);


            for (Keepout o : uni_keepouts) {
                ko_vp_q = vp.ko_vp_bVars.get(o);
                ko_vp_iq_abs = vp.ko_vp_iVars_abs.get(o);

                //(RL.3)
                cRL_leq.addToLHS(ko_vp_q[12], 1.0);
                cRL_geq.addToLHS(ko_vp_q[12], 1.0);
                //(LR.3)
                cLR_leq.addToLHS(ko_vp_q[13], 1.0);
                cLR_geq.addToLHS(ko_vp_q[13], 1.0);
                //(AB.3)
                cAB_leq.addToLHS(ko_vp_q[14], 1.0);
                cAB_geq.addToLHS(ko_vp_q[14], 1.0);
                //(BA.3)
                cBA_leq.addToLHS(ko_vp_q[15], 1.0);
                cBA_geq.addToLHS(ko_vp_q[15], 1.0);


                GurobiConstraint cLeq;
                GurobiConstraint cGeq;
                /*
                RL-relation: q(RL) = 0
                 */
                if (o.getLeft_os().size() > 1) {
                    /*
                    (RL.1)
                     */
                    cLeq = new GurobiConstraint();
                    cLeq.setSense('<');
                    cLeq.addToRHS(ko_vp_q[8], M);
                    executor.addConstraint(cLeq);
                    cGeq = new GurobiConstraint();
                    cGeq.setSense('>');
                    cGeq.addToRHS(ko_vp_q[8], 1.0);
                    executor.addConstraint(cGeq);
                    for (Keepout oL : o.getLeft_os()) {
                        cLeq.addToLHS(vp.ko_vp_bVars.get(oL)[4], 1.0);
                        cGeq.addToLHS(vp.ko_vp_bVars.get(oL)[4], 1.0);
                    }
                    /*
                    (RL.2)
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[12], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[5], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[12], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[8], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[12], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_vp_q[5], 1.0);
                    c.addToRHS(ko_vp_q[8], 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);

                } else {
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[12], 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);
                }

                /*
                LR-relation: q(LR) = 0
                 */
                if (o.getRight_os().size() > 1) {
                    /*
                    LR.1
                     */
                    cLeq = new GurobiConstraint();
                    cLeq.setSense('<');
                    cLeq.addToRHS(ko_vp_q[9], M);
                    executor.addConstraint(cLeq);
                    cGeq = new GurobiConstraint();
                    cGeq.setSense('>');
                    cGeq.addToRHS(ko_vp_q[9], 1.0);
                    executor.addConstraint(cGeq);
                    for (Keepout oR : o.getRight_os()) {
                        cLeq.addToLHS(vp.ko_vp_bVars.get(oR)[5], 1.0);
                        cGeq.addToLHS(vp.ko_vp_bVars.get(oR)[5], 1.0);
                    }
                    /*
                    LR.2
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[13], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[4], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[13], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[9], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[13], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_vp_q[4], 1.0);
                    c.addToRHS(ko_vp_q[9], 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);


                } else {
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[13], 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);
                }

                /*
                AB-relation: q(AB) = 0
                 */
                if (o.getBelow_os().size() > 1) {
                    /*
                    AB.1
                     */
                    cLeq = new GurobiConstraint();
                    cLeq.setSense('<');
                    cLeq.addToRHS(ko_vp_q[11], M);
                    executor.addConstraint(cLeq);
                    cGeq = new GurobiConstraint();
                    cGeq.setSense('>');
                    cGeq.addToRHS(ko_vp_q[11], 1.0);
                    executor.addConstraint(cGeq);
                    for (Keepout oB : o.getBelow_os()) {
                        cLeq.addToLHS(vp.ko_vp_bVars.get(oB)[7], 1.0);
                        cGeq.addToLHS(vp.ko_vp_bVars.get(oB)[7], 1.0);
                    }
                    /*
                    AB.2
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[14], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[6], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[14], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[11], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[14], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_vp_q[6], 1.0);
                    c.addToRHS(ko_vp_q[11], 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);

                } else {
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[14], 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);
                }

                /*
                BA-relation: q(BA) = 0
                 */
                if (o.getAbove_os().size() > 1) {
                    /*
                    BA.1
                     */
                    cLeq = new GurobiConstraint();
                    cLeq.setSense('<');
                    cLeq.addToRHS(ko_vp_q[10], M);
                    executor.addConstraint(cLeq);
                    cGeq = new GurobiConstraint();
                    cGeq.setSense('>');
                    cGeq.addToRHS(ko_vp_q[10], 1.0);
                    executor.addConstraint(cGeq);
                    for (Keepout oA : o.getAbove_os()) {
                        cLeq.addToLHS(vp.ko_vp_bVars.get(oA)[6], 1.0);
                        cGeq.addToLHS(vp.ko_vp_bVars.get(oA)[6], 1.0);
                    }
                    /*
                    BA.2
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[15], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[7], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[15], 1.0);
                    c.setSense('<');
                    c.addToRHS(ko_vp_q[10], 1.0);
                    executor.addConstraint(c);
                    c.addToLHS(ko_vp_q[15], 1.0);
                    c.setSense('>');
                    c.addToRHS(ko_vp_q[7], 1.0);
                    c.addToRHS(ko_vp_q[10], 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);


                } else {
                    c = new GurobiConstraint();
                    c.addToLHS(ko_vp_q[15], 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);
                }

                /*
                DT.2
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[12], 1.0);
                c.addToLHS(ko_vp_q[13], 1.0);
                c.addToLHS(ko_vp_q[14], 1.0);
                c.addToLHS(ko_vp_q[15], 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[16], M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[12], 1.0);
                c.addToLHS(ko_vp_q[13], 1.0);
                c.addToLHS(ko_vp_q[14], 1.0);
                c.addToLHS(ko_vp_q[15], 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[16], 1.0);
                executor.addConstraint(c);

                /*
                DT.3
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[16], 1.0);
                c.setSense('=');
                c.addToRHS(ko_vp_q[17], 1.0);
                c.addToRHS(ko_vp_q[18], 1.0);
                executor.addConstraint(c);


                /*
                DT.5 (1&2/4)
                 */
                GurobiQuadConstraint dt_rule_ur = new GurobiQuadConstraint();
                dt_rule_ur.addToLHS(vpNext.ko_vp_bVars.get(o)[22], 1.0);
                dt_rule_ur.setSense('=');
                dt_rule_ur.addToRHS(ko_vp_q[18], 1.0);
                executor.addConstraint(dt_rule_ur);
                GurobiQuadConstraint dt_rule_ll = new GurobiQuadConstraint();
                dt_rule_ll.addToLHS(vpNext.ko_vp_bVars.get(o)[21], 1.0);
                dt_rule_ll.setSense('=');
                dt_rule_ll.addToRHS(ko_vp_q[17], 1.0);
                executor.addConstraint(dt_rule_ll);

                /*
                DT.5 (3&4/4)
                 */
                //i->out
                dt_rule_out.addToLHS(ko_vp_q[19], ko_vp_q[16], 1.0);
                dt_rule_out.addToLHS(ko_vp_q[20], ko_vp_q[16], 1.0);
                //i+1->in
                dt_rule_in.addToLHS(vpNext.ko_vp_bVars.get(o)[21], ko_vp_q[16], 1.0);
                dt_rule_in.addToLHS(vpNext.ko_vp_bVars.get(o)[22], ko_vp_q[16], 1.0);

                for (Keepout other_o : uni_keepouts) {
                    GurobiVariable[] oo_vp_q = vp.oo_vp_bVars.get(o).get(other_o);
                    GurobiVariable other_oq_d_ll_in = vpNext.ko_vp_bVars.get(other_o)[17];
                    GurobiVariable other_oq_d_ur_in = vpNext.ko_vp_bVars.get(other_o)[18];



                    /*
                    DT.4
                     */
                    if (!o.getName().equals(other_o.getName())) {
                        c = new GurobiConstraint();
                        c.addToLHS(oo_vp_q[0], 1.0);
                        c.setSense('=');
                        c.addToRHS(oo_vp_q[1], 1.0);
                        c.addToRHS(oo_vp_q[2], 1.0);
                        c.addToRHS(oo_vp_q[3], 1.0);
                        c.addToRHS(oo_vp_q[4], 1.0);
                        executor.addConstraint(c);
                    } else {
                        c = new GurobiConstraint();
                        c.addToLHS(oo_vp_q[0], 1.0);
                        c.addToLHS(oo_vp_q[1], 1.0);
                        c.addToLHS(oo_vp_q[2], 1.0);
                        c.addToLHS(oo_vp_q[3], 1.0);
                        c.addToLHS(oo_vp_q[4], 1.0);
                        c.setSense('=');
                        c.setRHSConstant(0.0);
                        executor.addConstraint(c);
                    }

                    /*
                    DT.5.(1&2/4)
                     */
                    dt_rule_ur.addToLHS(oo_vp_q[3], other_oq_d_ll_in, 1.0);
                    dt_rule_ur.addToLHS(oo_vp_q[4], other_oq_d_ur_in, 1.0);

                    dt_rule_ll.addToLHS(oo_vp_q[1], other_oq_d_ll_in, 1.0);
                    dt_rule_ll.addToLHS(oo_vp_q[2], other_oq_d_ur_in, 1.0);

                    /*
                    DT.6
                     */
                    c = new GurobiConstraint();
                    c.addToLHS(oo_vp_q[0], 1.0);
                    c.addToLHS(vp.oo_vp_bVars.get(other_o).get(o)[0], 1.0);
                    c.setSense('<');
                    c.setRHSConstant(1.0);
                    executor.addConstraint(c);

                    /*
                    DT.7 (3/3)
                     */
                    int[] cnm = o.getMap_oo_dist().get(other_o);
                    d_vv.addToRHS(oo_vp_q[1], cnm[0]);
                    d_vv.addToRHS(oo_vp_q[2], cnm[1]);
                    d_vv.addToRHS(oo_vp_q[3], cnm[2]);
                    d_vv.addToRHS(oo_vp_q[4], cnm[3]);

                }

                /*
                DT.7 (2/3)
                 */
                GurobiVariable oq_d_ll_in = vpNext.ko_vp_bVars.get(o)[21];
                GurobiVariable oq_d_ur_in = vpNext.ko_vp_bVars.get(o)[22];
                GurobiVariable[] ko_vp_iq_absNext = vpNext.ko_vp_iVars_abs.get(o);
                d_vv.addToRHS(ko_vp_iq_abs[0], ko_vp_q[19], 1.0);
                d_vv.addToRHS(ko_vp_iq_abs[1], ko_vp_q[19], 1.0);
                d_vv.addToRHS(ko_vp_iq_abs[2], ko_vp_q[20], 1.0);
                d_vv.addToRHS(ko_vp_iq_abs[3], ko_vp_q[20], 1.0);
                d_vv.addToRHS(ko_vp_iq_absNext[0], oq_d_ll_in, 1.0);
                d_vv.addToRHS(ko_vp_iq_absNext[1], oq_d_ll_in, 1.0);
                d_vv.addToRHS(ko_vp_iq_absNext[2], oq_d_ur_in, 1.0);
                d_vv.addToRHS(ko_vp_iq_absNext[3], oq_d_ur_in, 1.0);

            }


        }
        /*
         * AAAA
         * Setup constraints for 0 --- pv.size()-1
         */

        /*
         * for each virtual point
         * Setup constraints for 0 --- pv.size()
         * VVVV
         *
         */

        for (VP_var vp : VPvars) {
            vp_iq_abs = vp.vp_iVars_abs;

            /*
            Each steiner point connects only one slave (vs_connecting_c): sum_j q_i_sj = 1
             */
            GurobiConstraint vs_connecting_c = new GurobiConstraint();
            vs_connecting_c.setRHSConstant(1.0);
            vs_connecting_c.setSense('=');
            executor.addConstraint(vs_connecting_c);


            //add d(v_i, corrS) to sidebusLength
            c_sideBusLength.addToRHS(vp_iq_abs[3], 1.0);


            /*
             * virtual point vs. obstacles
             * VVVV
             */
            for (Keepout o : uni_keepouts) {
                ko_vp_q = vp.ko_vp_bVars.get(o);
                ko_vp_iq_abs = vp.ko_vp_iVars_abs.get(o);
                ko_vp_iq = vp.ko_vp_iVars.get(o);

                /*
                 * Non-overlapping and dSet
                 * VVVV
                 */

                //oqL + oqR + oqA + oqB >= 1
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[0], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[2], 1.0);
                c.addToLHS(ko_vp_q[3], 1.0);
                c.setSense('>');
                c.setRHSConstant(1.0);
                executor.addConstraint(c);


                /*
                LEFT:
                v_i.x <= o.minX - eps + (1 - oqL) * M
                      <= -M * oqL + (o.minx - eps + M)
                v_i.x >= o.minX - oqL * M
                Ld:
                oqA + oqB + oqR <= M - M * oqLd
                oqA + oqB + oqR >= 1 - oqLd
                 */
                c = new GurobiConstraint();
                c.addToLHS(vp.x, 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[0], -M);
                c.setRHSConstant(o.getMinX() - eps + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.x, 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[0], -M);
                c.setRHSConstant(o.getMinX());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[2], 1.0);
                c.addToLHS(ko_vp_q[3], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[4], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[2], 1.0);
                c.addToLHS(ko_vp_q[3], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[4], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);

                /*
                Right:
                v_i.x >= o.maxX + eps - (1 - oqR) * M
                v_i.x <= o.maxX + oqR * M
                Rd:
                oqA + oqB + oqL <= M - M * oqRd
                oqA + oqB + oqL >= 1 - oqRd
                 */
                c = new GurobiConstraint();
                c.addToLHS(vp.x, 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[1], M);
                c.setRHSConstant(o.getMaxX() + eps - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.x, 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[1], M);
                c.setRHSConstant(o.getMaxX());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[2], 1.0);
                c.addToLHS(ko_vp_q[0], 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[5], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[2], 1.0);
                c.addToLHS(ko_vp_q[0], 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[5], -1);
                c.setRHSConstant(1);
                executor.addConstraint(c);


                /*
                Above:
                v_i.y >= o.maxY + eps - (1 - oqA) * M
                      >= M * nonA + (o.maxY + eps - M)
                v_i.y <= o.maxY + oqA * M
                Ad:
                oqL + oqR + oqB <= M - M * oqAd
                oqL + oqR + oqB >= 1 - oqAd
                 */
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1);
                c.setSense('>');
                c.addToRHS(ko_vp_q[2], M);
                c.setRHSConstant(o.getMaxY() + eps - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1);
                c.setSense('<');
                c.addToRHS(ko_vp_q[2], M);
                c.setRHSConstant(o.getMaxY());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[0], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[3], 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[6], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[0], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[3], 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[6], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);

                /*
                Below:
                v_i.y <= o.minY - eps + (1 - oqB)M
                      <= -M * nonB + o.minY - eps + M
                v_i.y >= o.minY - oqB * M
                Bd:
                oqL + oqR + oqA <= M - M * oqBd
                oqL + oqR + oqA >= 1 - oqBd
                 */
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[3], -M);
                c.setRHSConstant(o.getMinY() - eps + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[3], -M);
                c.setRHSConstant(o.getMinY());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[0], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[2], 1.0);
                c.setSense('<');
                c.addToRHS(ko_vp_q[7], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_q[0], 1.0);
                c.addToLHS(ko_vp_q[1], 1.0);
                c.addToLHS(ko_vp_q[2], 1.0);
                c.setSense('>');
                c.addToRHS(ko_vp_q[7], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                /*
                 * AAAA
                 * Non-overlapping and dSet
                 */

                /*
                 vo_ll.x, vo_ll.y, vo_ur.x, vo_ur.y
                 vo_ll.x = abs(v.x - o_ll.x)
                 vo_ll.y = abs(v.y - o_ll.y)
                 vo_ur.x = abs(v.x - o_ur.x)
                 vo_ur.y = abs(v.y - o_ur.y)
                 */
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_iq[0], 1.0);
                c.setSense('=');
                c.addToRHS(vp.x, 1.0);
                c.setRHSConstant(-o.getMinX());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_vp_iq_abs[0], ko_vp_iq[0], "vo_ll.x_abs");
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_iq[1], 1.0);
                c.setSense('=');
                c.addToRHS(vp.y, 1.0);
                c.setRHSConstant(-o.getMinY());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_vp_iq_abs[1], ko_vp_iq[1], "vo_ll.y_abs");
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_iq[2], 1.0);
                c.setSense('=');
                c.addToRHS(vp.x, 1.0);
                c.setRHSConstant(-o.getMaxX());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_vp_iq_abs[2], ko_vp_iq[2], "vo_ur.x_abs");
                c = new GurobiConstraint();
                c.addToLHS(ko_vp_iq[3], 1.0);
                c.setSense('=');
                c.addToRHS(vp.y, 1.0);
                c.setRHSConstant(-o.getMaxY());
                executor.addConstraint(c);
                executor.addGenConstraintAbs(ko_vp_iq_abs[3], ko_vp_iq[3], "vo_ur.x_abs");


            }
            /*
             * AAAA
             * virtual point vs. obstacles
             */



            /*
             * virtual points vs. slaves
             * VVVV
             */

            for (Slave_var sv : slaveVars) {
                sl_q = vp.sl_bVars.get(sv);
                sl_iq_abs = vp.sl_iVars_abs.get(sv);
                vs_connecting_c.addToLHS(sl_q[5], 1.0);

                /*
                d_vs(d) >= d(v_i, s_j) - (1 - q_i_sj) * M
                d_vs(d) <= d(v_i, s_j) + (1 - q_i_sj) * M
                 */
                c = new GurobiConstraint();
                c.addToLHS(vp_iq_abs[1], 1.0);
                c.setSense('>');
                c.addToRHS(sl_iq_abs[0], 1.0);
                c.addToRHS(sl_q[5], M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp_iq_abs[1], 1.0);
                c.setSense('<');
                c.addToRHS(sl_iq_abs[0], 1.0);
                c.addToRHS(sl_q[5], -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);


                /*
                d(v_i, s_j) >= |v_i.x - s_j.x| + |v_i.y - s_j.y|
                 */
                c = new GurobiConstraint();
                c.addToLHS(sl_iq_abs[0], 1.0);
                c.setSense('>');
                c.addToRHS(sl_iq_abs[1], 1.0);
                c.addToRHS(sl_iq_abs[2], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(sl_iq_abs[1], sl_iq_abs[3], "abs_v_s_x");
                executor.addGenConstraintAbs(sl_iq_abs[2], sl_iq_abs[4], "abs_v_s_y");

                /*
                v_i.x - s_j.x
                v_i.y - s_j.y
                 */
                c = new GurobiConstraint();
                c.addToLHS(sl_iq_abs[3], 1.0);
                c.setSense('=');
                c.addToRHS(vp.x, 1.0);
                c.setRHSConstant(-sv.getX_ct());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(sl_iq_abs[4], 1.0);
                c.setSense('=');
                c.addToRHS(vp.y, 1.0);
                c.setRHSConstant(-sv.getY_ct());
                executor.addConstraint(c);


                for (Keepout o : uni_keepouts) {

                    ko_vp_q = vp.ko_vp_bVars.get(o);
                    ko_vp_iq_abs = vp.ko_vp_iVars_abs.get(o);
                    ko_sl_q = vp.ko_sl_bVars.get(o).get(sv);


                    sv_q = sv.pseudo_bVars.get(o);
                    sv_iq = sv.pseudo_iVars.get(o);





                }


            }

            /*
             * AAAA
             * steiner points vs. slaves
             */


        }
        /*
         * AAAA
         * Setup constraints for 0 --- pv.size()
         */


        /*
         * Each Slave must connect one IP
         */
        for (Slave_var sv : slaveVars) {
            c = new GurobiConstraint();
            c.setRHSConstant(1);
            c.setSense('=');
            for (VP_var vp : VPvars) {
                c.addToLHS(vp.sl_bVars.get(sv)[5], 1);
            }
            executor.addConstraint(c);
        }


    }


    private void buildVars_wo_KO_ABS(ArrayList<Slave> slaves, int n, ArrayList<PointVar_Abs> pointVars_abs) {

        for (int i = 0; i < n; ++i) {
            PointVar_Abs pv = new PointVar_Abs();

            if (i == 0) {
                pv.setDistM_x(new GurobiVariable(GRB.CONTINUOUS, 0, M, "distM_x"), executor);
                pv.setDistM_y(new GurobiVariable(GRB.CONTINUOUS, 0, M, "distM_y"), executor);
                pv.setDelta_Mx_Var(new GurobiVariable(GRB.CONTINUOUS, -M, M, "delta_Mx_Var"), executor);
                pv.setDelta_My_Var(new GurobiVariable(GRB.CONTINUOUS, -M, M, "delta_My_Var"), executor);
            }

            pv.setX_Var(new GurobiVariable(GRB.CONTINUOUS, -M, M, "x" + i), executor);
            pv.setY_Var(new GurobiVariable(GRB.CONTINUOUS, -M, M, "y" + i), executor);

            pv.setDelta_Ix_Var(new GurobiVariable(GRB.CONTINUOUS, -M, M, "delta_Ix_Var"), executor);
            pv.setDelta_Iy_Var(new GurobiVariable(GRB.CONTINUOUS, -M, M, "delta_Iy_Var"), executor);


            pv.setDistI(new GurobiVariable(GRB.CONTINUOUS, 0, M, "distI" + i), executor);
            pv.setDistS(new GurobiVariable(GRB.CONTINUOUS, 0, M, "distS" + i), executor);


            pv.setDistI_x(new GurobiVariable(GRB.CONTINUOUS, 0, M, "distI_x" + i), executor);
            pv.setDistI_y(new GurobiVariable(GRB.CONTINUOUS, 0, M, "distI_y" + i), executor);

            for (Slave s : slaves) {
                pv.addToSlave_Vars(s, new GurobiVariable(GRB.BINARY, 0, 1, "s"), executor);
                pv.addToDelta_Sx_Vars(s, new GurobiVariable(GRB.CONTINUOUS, -M, M, "delta_Sx_Vars"), executor);
                pv.addToDelta_Sy_Vars(s, new GurobiVariable(GRB.CONTINUOUS, -M, M, "delta_Sy_Vars"), executor);
                pv.addToDistS_xs(s, new GurobiVariable(GRB.CONTINUOUS, 0, M, "distS_xs"), executor);
                pv.addToDistS_ys(s, new GurobiVariable(GRB.CONTINUOUS, 0, M, "distS_ys"), executor);
                pv.addToDistSs(s, new GurobiVariable(GRB.CONTINUOUS, 0, M, "distSs"), executor);
            }
            pointVars_abs.add(pv);
        }
    }

    /**
     * Build the variables for Model without Keepouts
     * The abs values are solved by Linearization
     * All coordinates are INT Type
     *
     * @param slaves
     * @param n
     * @param pointVars
     */
    private void buildVars_wo_KO(ArrayList<Slave> slaves, int n, ArrayList<PointVar> pointVars) {

        for (int i = 0; i < n; ++i) {
            PointVar pv = new PointVar();

            if (i == 0) {
                pv.setDistM_x(new GurobiVariable(GRB.INTEGER, 0, M, "distM_x"), executor);
                pv.setDistM_y(new GurobiVariable(GRB.INTEGER, 0, M, "distM_y"), executor);
            }

            pv.setX_Var(new GurobiVariable(GRB.INTEGER, -M, M, "x" + i), executor);
            pv.setY_Var(new GurobiVariable(GRB.INTEGER, -M, M, "y" + i), executor);


            pv.setDistI(new GurobiVariable(GRB.INTEGER, 0, M, "distI" + i), executor);


            pv.setDistI_x(new GurobiVariable(GRB.INTEGER, 0, M, "distI_x" + i), executor);
            pv.setDistI_y(new GurobiVariable(GRB.INTEGER, 0, M, "distI_y" + i), executor);

            for (Slave s : slaves) {
                pv.addToSlave_Vars(s, new GurobiVariable(GRB.BINARY, 0, 1, "s"), executor);
                pv.addToDistS_xs(s, new GurobiVariable(GRB.INTEGER, 0, M, "distS_xs"), executor);
                pv.addToDistS_ys(s, new GurobiVariable(GRB.INTEGER, 0, M, "distS_ys"), executor);
                pv.addToDistSs(s, new GurobiVariable(GRB.INTEGER, 0, M, "distSs"), executor);
            }
            pointVars.add(pv);
        }
    }


    /**
     * Build the variables for Model with Keepouts
     * The abs values are solved by GRB.GenConstraints
     * All coordinates are INT Type
     *
     * @param mv             pseudoVar of master
     * @param slaves         ArrayList of slaves
     * @param slaveVars_ko   ArrayList of pseudoVar of slaves
     * @param n
     * @param pointVar_w_kos ArrayList of pointVar_w_ko
     * @param keepouts
     */
    private void buildVars_w_KO(Master_var mv, ArrayList<Slave> slaves, ArrayList<Slave_var> slaveVars_ko, int n, ArrayList<PointVar_ko> pointVar_w_kos, ArrayList<Keepout> keepouts) {


        int ub;

        int ub_x = 0;
        int ub_y = 0;
        int lb_x = 0;
        int lb_y = 0;

        if (mv.getX_ct() < lb_x) {
            lb_x = mv.getX_ct();
        }
        if (mv.getX_ct() > ub_x) {
            ub_x = mv.getX_ct();
        }
        if (mv.getY_ct() < lb_y) {
            lb_y = mv.getY_ct();
        }
        if (mv.getY_ct() > ub_y) {
            ub_y = mv.getY_ct();
        }


        /*
         * Set bVars and iVars for Master
         */
        for (Keepout k : keepouts) {
            int[] nqM = new int[4];
            //nonL
            if (mv.getX_ct() < k.getMinX()) {
                nqM[0] = 1;
            } else {
                nqM[0] = 0;
            }
            //nonR
            if (mv.getX_ct() > k.getMaxX()) {
                nqM[1] = 1;
            } else {
                nqM[1] = 0;
            }
            //nonA
            if (mv.getY_ct() > k.getMaxY()) {
                nqM[2] = 1;
            } else {
                nqM[2] = 0;
            }
            //nonB
            if (mv.getY_ct() < k.getMinY()) {
                nqM[3] = 1;
            } else {
                nqM[3] = 0;
            }
            mv.addToPseudo_bVars(k, nqM);

            int[] dM = new int[4];
            /*
             * 0: |ms.x - c^k.LL.x|
             * 1: |ms.y - c^k.LL.y|
             * 2: |ms.x - c^k.UR.x|
             * 3: |ms.y - c^k.UR.y|
             */
            dM[0] = Math.abs(mv.getX_ct() - k.getMinX());
            dM[1] = Math.abs(mv.getY_ct() - k.getMinY());
            dM[2] = Math.abs(mv.getX_ct() - k.getMaxX());
            dM[3] = Math.abs(mv.getY_ct() - k.getMaxY());
            mv.addToPsedo_iVars(k, dM);

        }

        /*
         * Set bVars and iVars for Slaves
         */

        for (Slave s : slaves) {

            Slave_var sv = new Slave_var(s.getX_ct(), s.getY_ct());
            slaveVars_ko.add(sv);
            sv.setName(s.getName());

            if (sv.getX_ct() < lb_x) {
                lb_x = sv.getX_ct();
            }
            if (sv.getX_ct() > ub_x) {
                ub_x = sv.getX_ct();
            }
            if (sv.getY_ct() < lb_y) {
                lb_y = sv.getY_ct();
            }
            if (sv.getY_ct() > ub_y) {
                ub_y = sv.getY_ct();
            }


            for (Keepout k : keepouts) {

                if (k.getMinX() < lb_x) {
                    lb_x = k.getMinX();
                }
                if (k.getMaxX() > ub_x) {
                    ub_x = k.getMaxX();
                }
                if (k.getMinY() < lb_y) {
                    lb_y = k.getMinY();
                }
                if (k.getMaxY() > ub_y) {
                    ub_y = k.getMaxY();
                }


                int[] sv_q = new int[4];
                //nonL
                if (sv.getX_ct() < k.getMinX()) {
                    sv_q[0] = 1;
                } else {
                    sv_q[0] = 0;
                }
                //nonR
                if (sv.getX_ct() > k.getMaxX()) {
                    sv_q[1] = 1;
                } else {
                    sv_q[1] = 0;
                }
                //nonA
                if (sv.getY_ct() > k.getMaxY()) {
                    sv_q[2] = 1;
                } else {
                    sv_q[2] = 0;
                }
                //nonB
                if (sv.getY_ct() < k.getMinY()) {
                    sv_q[3] = 1;
                } else {
                    sv_q[3] = 0;
                }
                sv.addToPseudo_bVars(k, sv_q);

                int[] sv_iq = new int[4];
                /*
                 * 0: |s_j.x - c^k.LL.x|
                 * 1: |s_j.y - c^k.LL.y|
                 * 2: |s_j.x - c^k.UR.x|
                 * 3: |s_j.y - c^k.UR.y|
                 */
                sv_iq[0] = Math.abs(sv.getX_ct() - k.getMinX());
                sv_iq[1] = Math.abs(sv.getY_ct() - k.getMinY());
                sv_iq[2] = Math.abs(sv.getX_ct() - k.getMaxX());
                sv_iq[3] = Math.abs(sv.getY_ct() - k.getMaxY());
                sv.addToPsedo_iVars(k, sv_iq);
            }


        }

        //lb_x
        if (lb_x <= 0) {
            lb_x *= 2;
        } else {
            lb_x *= 0.5;
        }
        //lb_y
        if (lb_y <= 0) {
            lb_y *= 2;
        } else {
            lb_y *= 0.5;
        }
        //ub_x
        if (ub_x >= 0) {
            ub_x *= 2;
        } else {
            ub_x *= 0.5;
        }
        //ub_y
        if (ub_y >= 0) {
            ub_y *= 2;
        } else {
            ub_y *= 0.5;
        }

        ub = 2 * ((ub_x - lb_x) + (ub_y - lb_y));


        for (int i = 0; i < n; ++i) {
            PointVar_ko pv_ko = new PointVar_ko();

            pv_ko.setX_Var(new GurobiVariable(GRB.INTEGER, lb_x, ub_x, "x" + i), executor);
            pv_ko.setY_Var(new GurobiVariable(GRB.INTEGER, lb_y, ub_y, "y" + i), executor);

            /*
            the 1st steiner point
             */
            if (i == 0) {
                /*
                 * ONLY for v1 <-> ms (5)
                 * 0: Left (q^L_1_ms)
                 * 1: Right (q^R_1_ms)
                 * 2: Up (q^U_1_ms)
                 * 3: Down (q^D_1_ms)
                 * 4: detour trigger with masterIC w.r.t. all keepouts (dq_1_ms)
                 */
                GurobiVariable[] ms_bVars = new GurobiVariable[5];
                for (int var_cnt = 0; var_cnt < 5; ++var_cnt) {
                    ms_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, i + "_ms_sp_bVars_" + var_cnt);
                    executor.addVariable(ms_bVars[var_cnt]);
                }
                pv_ko.ms_sp_bVars = ms_bVars;

                /*
                 * ONLY for v1 <-> ms (5)
                 * 0: |v_1.x - ms.x|
                 * 1: |v_1.y - ms.y|
                 * 2: d(v_1, ms): >= d(v_1, o, ms) - oq^d_i_ms * M, for each o
                 * 3: v_1.x - ms.x
                 * 4: v_1.y - ms.y
                 */
                GurobiVariable[] sp_iVars = new GurobiVariable[5];
                for (int var_cnt = 0; var_cnt < 5; ++var_cnt) {
                    sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, 0, ub, i + "_ms_sp_iVars_" + var_cnt);
                    executor.addVariable(sp_iVars[var_cnt]);
                }
                pv_ko.ms_sp_iVars = sp_iVars;

                for (Keepout k : keepouts) {
                    /*
                     * ONLY for v1 <-> ms (6)
                     * 0: left-right opposite with masterIC (oq^LR_1_ms)
                     * 1: right-left  ----------||--------- (oq^RL_1_ms)
                     * 2: above-below ----------||--------- (oq^AB_1_ms)
                     * 3: below-above ----------||--------- (oq^BA_1_ms)
                     * 4: for each o: detour trigger with masterIC (oq^d_1_ms)
                     * 5: choose path for d(v_1, ms) (aux_oq_1_ms)
                     */
                    GurobiVariable[] ko_sp_bVars = new GurobiVariable[6];
                    for (int var_cnt = 0; var_cnt < 6; ++var_cnt) {
                        ko_sp_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + k.getName() + "_ko_sp_bVars_" + var_cnt);
                        executor.addVariable(ko_sp_bVars[var_cnt]);
                    }
                    pv_ko.ms_ko_sp_bVars.put(k, ko_sp_bVars);

                    /*
                     * ONLY for v1 <-> ms
                     * d(v_1, o, ms)
                     * v_1.x - ms.x
                     * v_1.y - ms.y
                     */
                    GurobiVariable[] ko_sp_iVars = new GurobiVariable[3];
                    for (int var_cnt = 0; var_cnt < 1; ++var_cnt) {
                        ko_sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub, "v_" + i + ";" + k.getName() + "_ms_ko_sp_iVars_" + var_cnt);
                        executor.addVariable(ko_sp_iVars[var_cnt]);
                    }
                    for (int var_cnt = 1; var_cnt < 3; ++var_cnt) {
                        ko_sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub, ub, "v_" + i + ";" + k.getName() + "_ms_ko_sp_iVars_" + var_cnt);
                        executor.addVariable(ko_sp_iVars[var_cnt]);
                    }
                    pv_ko.ms_ko_sp_iVars.put(k, ko_sp_iVars);


                }


            }

            /*
             * * sp_bVars
             * Binary variables (5/10):
             * orientation with next steiner point:
             * 0: Left (q^L_ij)
             * 1: Right (q^R_ij)
             * 2: Up (q^U_ij)
             * 3: Down (q^D_ij)
             * 4: detour trigger with next steiner point w.r.t. all keepouts (dq_ij)
             */

            GurobiVariable[] sp_bVars = new GurobiVariable[5];
            for (int var_cnt = 0; var_cnt < 5; ++var_cnt) {
                sp_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, i + "_sp_bVars_" + var_cnt);
                executor.addVariable(sp_bVars[var_cnt]);
            }
            pv_ko.sp_bVars = sp_bVars;

            /*
             * sp_iVars
             * Integer variables (4):
             * 0: d(v_i, v_i+1): >= d(v_i, o, v_i+1) - oq^d_ij * M, for each o
             * 1: d(v_i, corr.S) <=/>= d(v_i, s_j) +/- (1 - q_i_sj) * M, for each slave
             * 2: |v_i.x - v_i+1.x|
             * 3: |v_i.y - v_i+1.y|
             *
             */

            GurobiVariable[] sp_iVars = new GurobiVariable[4];
            for (int var_cnt = 0; var_cnt < 4; ++var_cnt) {
                sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, 0, ub, i + "_sp_iVars_" + var_cnt);
                executor.addVariable(sp_iVars[var_cnt]);
            }
            pv_ko.sp_iVars = sp_iVars;






            /*
            ko_sp_bVars
            in_ko_sp_bVars
            ko_sp_iVars
            ko_sl_bVars
            ko_sl_iVars
             */
            for (Keepout k : keepouts) {

                /*
                 * ko_sp_bVars
                 * Binary variables (10/16):
                 * Non-overlapping with keepouts
                 * 0: Left (nonL)
                 * 1: Right (nonR)
                 * 2: Above (nonA)
                 * 3: Below (nonB)
                 * 4: left-right opposite with next steiner point (oq^LR_ij)
                 * 5: right-left  --------------||--------------- (oq^RL_ij)
                 * 6: above-below --------------||--------------- (oq^AB_ij)
                 * 7: below-above --------------||--------------- (oq^BA_ij)
                 * 8: for each o: detour trigger with next steiner point (oq^d_ij)
                 * 9: choose path for d(vi, o, v_i+1) (aux_oq_ij)
                 */

                GurobiVariable[] ko_sp_bVars = new GurobiVariable[10];
                for (int var_cnt = 0; var_cnt < 10; ++var_cnt) {
                    ko_sp_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + k.getName() + "_ko_sp_bVars_" + var_cnt);
                    executor.addVariable(ko_sp_bVars[var_cnt]);
                }
                pv_ko.ko_sp_bVars.put(k, ko_sp_bVars);




                /*
                 * ko_sp_iVars
                 * Integer variables (5/6):
                 * 0: |v_i.x - c^k_LL.x| (vo^LL_i.x)
                 * 1: |v_i.y - c^k_LL.y| (vo^LL_i.y)
                 * 2: |v_i.x - c^k_UR.x| (vo^UR_i.x)
                 * 3: |v_i.y - c^k_UR.y| (vo^UR_i.y)
                 * (0. 1. 2. 3) also for connecting with next steiner point, slaves, and master
                 * //
                 * sum of i.(0. 1.) + i+1.(0. 1.): d(v_i, o.LL, v_i+1)
                 * sum of i.(2. 3.) + i+1.(2. 3.): d(v_i, o.UR, v_i+1)
                 * 4: d(v_i, o, v_i+1)
                 *
                 * 5: v_i.x - c^k_LL.x
                 * 6: v_i.y - c^k_LL.y
                 * 7: v_i.x - c^k_UR.x
                 * 8: v_i.y - c^k_UR.y
                 */

                GurobiVariable[] ko_sp_iVars = new GurobiVariable[9];
                for (int var_cnt = 0; var_cnt < 5; ++var_cnt) {
                    ko_sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub, "v_" + i + ";" + k.getName() + "_ms_ko_sp_iVars_" + var_cnt);
                    executor.addVariable(ko_sp_iVars[var_cnt]);
                }
                for (int var_cnt = 5; var_cnt < 9; ++var_cnt) {
                    ko_sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub, ub, "v_" + i + ";" + k.getName() + "_ms_ko_sp_iVars_" + var_cnt);
                    executor.addVariable(ko_sp_iVars[var_cnt]);
                }
                pv_ko.ko_sp_iVars.put(k, ko_sp_iVars);


                Map<Slave_var, GurobiVariable[]> ko_sl_bVars = new HashMap<>();
                Map<Slave_var, GurobiVariable[]> ko_sl_iVars = new HashMap<>();

                for (Slave_var sv : slaveVars_ko) {
                    /*
                     * ko_sl_bVars
                     * Binary variables (6):
                     * 0: left-right opposite with each slave with each ko (oq^LR_i_sj)
                     * 1: right-left  ---------||------------ (oq^RL_i_sj)
                     * 2: above-below ---------||------------ (oq^AB_i_sj)
                     * 3: below-above ---------||------------ (oq^BA_i_sj)
                     * 4: detour trigger with each slave with each ko (oq^d_i_sj)
                     * 5: choose path for d(v_i, o, s_j) (aux_oq_i_sj)
                     */
                    GurobiVariable[] bVars = new GurobiVariable[6];
                    for (int var_cnt = 0; var_cnt < 6; ++var_cnt) {
                        bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + k.getName() + "_ko_sl_bVars_" + var_cnt);
                        executor.addVariable(bVars[var_cnt]);
                    }
                    ko_sl_bVars.put(sv, bVars);

                    /*
                     * ko_sl_iVars
                     * Integer varibales (1):
                     * 0: d(v_i, o, s_j) = d(v_i, o.LL, s_j) * aux_oq_i_sj + d(v_i, o.UR, s_j) * (1 - aux_oq_i_sj)
                     * 1 : d(v_i, o.LL, s_j)
                     * 2: d(v_i, o.UR, s_j)
                     */
                    GurobiVariable[] ko_sl_iq = new GurobiVariable[3];
                    for (int var_cnt = 0; var_cnt < 3; ++var_cnt) {
                        ko_sl_iq[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub, "v_" + i + ";" + k.getName() + "_ko_sl_iVars_" + var_cnt);
                        executor.addVariable(ko_sl_iq[var_cnt]);
                    }
                    ko_sl_iVars.put(sv, ko_sl_iq);

                }
                pv_ko.ko_sl_bVars.put(k, ko_sl_bVars);
                pv_ko.ko_sl_iVars.put(k, ko_sl_iVars);


            }

            /*
            sl_bVars
            sl_iVars
             */
            for (Slave_var sv : slaveVars_ko) {

                /*
                 * sl_bVars
                 * Binary variables:
                 * 0: connection binary var
                 * orientation with slaves
                 * 1: Left
                 * 2: Right
                 * 3: Up
                 * 4: Down
                 * 5: for each slave, detour trigger with all keepouts
                 */
                GurobiVariable[] sl_bVars = new GurobiVariable[6];
                for (int var_cnt = 0; var_cnt < 6; ++var_cnt) {
                    sl_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + sv.getName() + "_sl_bVars_" + var_cnt);
                    executor.addVariable(sl_bVars[var_cnt]);
                }
                pv_ko.sl_bVars.put(sv, sl_bVars);

                /*
                 * sl_iVars
                 * Integer variables (5):
                 * 0: d(v_i, s_j): >= d (v_i, o, s_j) - oq^d_i_sj * M
                 * 1: |v_i.x - s_j.x|
                 * 2: |v_i.y - s_j.y|
                 * 3: v_i.x - s_j.x
                 * 4: v_i.y - s_j.y
                 */
                GurobiVariable[] sl_iVars = new GurobiVariable[5];
                for (int var_cnt = 0; var_cnt < 3; ++var_cnt) {
                    sl_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub, "v_" + i + ";" + sv.getName() + "_sl_iVars_" + 0);
                    executor.addVariable(sl_iVars[var_cnt]);
                }
                for (int var_cnt = 3; var_cnt < 5; ++var_cnt) {
                    sl_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub, ub, "v_" + i + ";" + sv.getName() + "_sl_iVars_" + 0);
                    executor.addVariable(sl_iVars[var_cnt]);
                }
                pv_ko.sl_iVars.put(sv, sl_iVars);


            }


            pointVar_w_kos.add(pv_ko);
        }
    }

    private void buildVars_w_multiKO(Master_var mv, ArrayList<Slave> slaves, ArrayList<Slave_var> slave_vars, int n, ArrayList<VP_var> VPvars, ArrayList<Keepout> uni_keepouts, ArrayList<Poly_Keepout> poly_keepouts) {


        int ub_abs;

        int ub_x = 0;
        int ub_y = 0;
        int lb_x = 0;
        int lb_y = 0;


        if (mv.getX_ct() < lb_x) {
            lb_x = mv.getX_ct();
        }
        if (mv.getX_ct() > ub_x) {
            ub_x = mv.getX_ct();
        }
        if (mv.getY_ct() < lb_y) {
            lb_y = mv.getY_ct();
        }
        if (mv.getY_ct() > ub_y) {
            ub_y = mv.getY_ct();
        }


        /*
         * Set bVars and iVars for Master
         */
        for (Keepout k : uni_keepouts) {
            int[] nqM = new int[4];
            //nonL
            if (mv.getX_ct() < k.getMinX()) {
                nqM[0] = 1;
            } else {
                nqM[0] = 0;
            }
            //nonR
            if (mv.getX_ct() > k.getMaxX()) {
                nqM[1] = 1;
            } else {
                nqM[1] = 0;
            }
            //nonA
            if (mv.getY_ct() > k.getMaxY()) {
                nqM[2] = 1;
            } else {
                nqM[2] = 0;
            }
            //nonB
            if (mv.getY_ct() < k.getMinY()) {
                nqM[3] = 1;
            } else {
                nqM[3] = 0;
            }
            mv.addToPseudo_bVars(k, nqM);

            int[] dM = new int[4];
            /*
             * 0: |ms.x - c^k.LL.x|
             * 1: |ms.y - c^k.LL.y|
             * 2: |ms.x - c^k.UR.x|
             * 3: |ms.y - c^k.UR.y|
             */
            dM[0] = Math.abs(mv.getX_ct() - k.getMinX());
            dM[1] = Math.abs(mv.getY_ct() - k.getMinY());
            dM[2] = Math.abs(mv.getX_ct() - k.getMaxX());
            dM[3] = Math.abs(mv.getY_ct() - k.getMaxY());
            mv.addToPsedo_iVars(k, dM);

        }

        /*
         * Set bVars and iVars for Slaves
         */

        for (Slave s : slaves) {

            Slave_var sv = new Slave_var(s.getX_ct(), s.getY_ct());
            slave_vars.add(sv);
            sv.setName(s.getName());

            if (sv.getX_ct() < lb_x) {
                lb_x = sv.getX_ct();
            }
            if (sv.getX_ct() > ub_x) {
                ub_x = sv.getX_ct();
            }
            if (sv.getY_ct() < lb_y) {
                lb_y = sv.getY_ct();
            }
            if (sv.getY_ct() > ub_y) {
                ub_y = sv.getY_ct();
            }


            for (Keepout k : uni_keepouts) {
                int[] sv_q = new int[4];
                //nonL
                if (sv.getX_ct() < k.getMinX()) {
                    sv_q[0] = 1;
                } else {
                    sv_q[0] = 0;
                }
                //nonR
                if (sv.getX_ct() > k.getMaxX()) {
                    sv_q[1] = 1;
                } else {
                    sv_q[1] = 0;
                }
                //nonA
                if (sv.getY_ct() > k.getMaxY()) {
                    sv_q[2] = 1;
                } else {
                    sv_q[2] = 0;
                }
                //nonB
                if (sv.getY_ct() < k.getMinY()) {
                    sv_q[3] = 1;
                } else {
                    sv_q[3] = 0;
                }
                sv.addToPseudo_bVars(k, sv_q);

                int[] sv_iq = new int[4];
                /*
                 * 0: |s_j.x - c^k.LL.x|
                 * 1: |s_j.y - c^k.LL.y|
                 * 2: |s_j.x - c^k.UR.x|
                 * 3: |s_j.y - c^k.UR.y|
                 */
                sv_iq[0] = Math.abs(sv.getX_ct() - k.getMinX());
                sv_iq[1] = Math.abs(sv.getY_ct() - k.getMinY());
                sv_iq[2] = Math.abs(sv.getX_ct() - k.getMaxX());
                sv_iq[3] = Math.abs(sv.getY_ct() - k.getMaxY());
                sv.addToPsedo_iVars(k, sv_iq);
            }


        }

        for (Keepout k : uni_keepouts) {
            if (k.getMinX() < lb_x) {
                lb_x = k.getMinX();
            }
            if (k.getMaxX() > ub_x) {
                ub_x = k.getMaxX();
            }
            if (k.getMinY() < lb_y) {
                lb_y = k.getMinY();
            }
            if (k.getMaxY() > ub_y) {
                ub_y = k.getMaxY();
            }
        }
        for (Keepout k : poly_keepouts) {
            if (k.getMinX() < lb_x) {
                lb_x = k.getMinX();
            }
            if (k.getMaxX() > ub_x) {
                ub_x = k.getMaxX();
            }
            if (k.getMinY() < lb_y) {
                lb_y = k.getMinY();
            }
            if (k.getMaxY() > ub_y) {
                ub_y = k.getMaxY();
            }
        }

        //lb_x
        if (lb_x <= 0) {
            lb_x *= 2;
        } else {
            lb_x *= 0.5;
        }
        //lb_y
        if (lb_y <= 0) {
            lb_y *= 2;
        } else {
            lb_y *= 0.5;
        }
        //ub_x
        if (ub_x >= 0) {
            ub_x *= 2;
        } else {
            ub_x *= 0.5;
        }
        //ub_y
        if (ub_y >= 0) {
            ub_y *= 2;
        } else {
            ub_y *= 0.5;
        }

        ub_abs = 2 * ((ub_x - lb_x) + (ub_y - lb_y));


        /*
        initialize virtual points
         */
        for (int i = 0; i < n; ++i) {

            VP_var vp = new VP_var();

            vp.x = new GurobiVariable(GRB.INTEGER, lb_x, ub_x, "x" + i);
            executor.addVariable(vp.x);
            vp.y = new GurobiVariable(GRB.INTEGER, lb_y, ub_y, "y" + i);
            executor.addVariable(vp.y);


            /*
            the 1st steiner point
             */
            if (i == 0) {
                /*
                 * ONLY for v1 <-> ms (5)

                 */
                GurobiVariable[] ms_bVars = new GurobiVariable[5];
                for (int var_cnt = 0; var_cnt < 5; ++var_cnt) {
                    ms_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, i + "_ms_vp_bVars_" + var_cnt);
                    executor.addVariable(ms_bVars[var_cnt]);
                }
                vp.ms_vp_bVars = ms_bVars;

                /*
                 * ONLY for v1 <-> ms
                 *
                 */
                GurobiVariable[] sp_iVars = new GurobiVariable[5];
                for (int var_cnt = 0; var_cnt < 5; ++var_cnt) {
                    sp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, 0, ub_abs, i + "_ms_vp_iVars_" + var_cnt);
                    executor.addVariable(sp_iVars[var_cnt]);
                }
                vp.ms_vp_iVars = sp_iVars;

                for (Keepout k : uni_keepouts) {
                    /*
                     * ONLY for v1 <-> ms
                     */
                    GurobiVariable[] ko_vp_bVars = new GurobiVariable[6];
                    for (int var_cnt = 0; var_cnt < 6; ++var_cnt) {
                        ko_vp_bVars[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + k.getName() + "_ko_vp_bVars_" + var_cnt);
                        executor.addVariable(ko_vp_bVars[var_cnt]);
                    }
                    vp.ms_ko_vp_bVars.put(k, ko_vp_bVars);

                    /*
                     * ONLY for v1 <-> ms
                     */
                    GurobiVariable[] ko_vp_iVars = new GurobiVariable[3];
                    for (int var_cnt = 0; var_cnt < 1; ++var_cnt) {
                        ko_vp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub_abs, "v_" + i + ";" + k.getName() + "_ms_ko_vp_iVars_" + var_cnt);
                        executor.addVariable(ko_vp_iVars[var_cnt]);
                    }
                    for (int var_cnt = 1; var_cnt < 3; ++var_cnt) {
                        ko_vp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub_abs, ub_abs, "v_" + i + ";" + k.getName() + "_ms_ko_vp_iVars_" + var_cnt);
                        executor.addVariable(ko_vp_iVars[var_cnt]);
                    }
                    vp.ms_ko_vp_iVars.put(k, ko_vp_iVars);


                }


            }

            /*
             * vp_bVars regarding next vp
             * (0) q(RL)
             * (1) q(LR)
             * (2) q(AB)
             * (3) q(BA)
             * (4) q(d): detour trigger
             */
            int cnt_vp_q = 5;
            GurobiVariable[] vp_q = new GurobiVariable[cnt_vp_q];
            for (int var_cnt = 0; var_cnt < cnt_vp_q; ++var_cnt) {
                vp_q[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, i + "_vp_bVars_" + var_cnt);
                executor.addVariable(vp_q[var_cnt]);
            }
            vp.vp_bVars = vp_q;

            /*
             * vp_iVars_abs
             *
             * (0) d_vv
             * (1) |v_i.x - v_i+1.x|
             * (2) |v_i.y - v_i+1.y|
             * (3) d_vv(d)
             * (4) d_v_S(d)
             *
             */
            int cnt_vp_iq_abs = 5;
            GurobiVariable[] vp_iq_abs = new GurobiVariable[cnt_vp_iq_abs];
            for (int var_cnt = 0; var_cnt < cnt_vp_iq_abs; ++var_cnt) {
                vp_iq_abs[var_cnt] = new GurobiVariable(GRB.INTEGER, 0, ub_abs, i + "_vp_iVars_ABS_" + var_cnt);
                executor.addVariable(vp_iq_abs[var_cnt]);
            }
            vp.vp_iVars_abs = vp_iq_abs;

            /*
             * vp_iVars
             * (0) v_i.x - v_i+1.x
             * (1) v_i.y - v_i+1.y
             */
            int cnt_vp_iq = 2;
            GurobiVariable[] vp_iq = new GurobiVariable[cnt_vp_iq];
            for (int var_cnt = 0; var_cnt < cnt_vp_iq; ++var_cnt) {
                vp_iq[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub_abs, ub_abs, i + "_vp_iVars_" + var_cnt);
                executor.addVariable(vp_iq[var_cnt]);
            }
            vp.vp_iVars = vp_iq;


            for (Keepout k : uni_keepouts) {

                /*
                 * ko_sp_bVars
                 * (0) oqL
                 * (1) oqR
                 * (2) oqA
                 * (3) oqB
                 * (4) oqLd
                 * (5) oqRd
                 * (6) oqAd
                 * (7) oqBd
                 * (8) oq(L): oq_ij^L = 1
                 * (9) oq(R)
                 * (10) oq(A)
                 * (11) oq(B)
                 * (12) oq(RL)
                 * (13) oq(LR)
                 * (14) oq(AB)
                 * (15) oq(BA)
                 * (16) oq(d): indicate relative obstacle regarding next vp
                 * (17) oq(d,ll): indicate route pass by lower-left corner
                 * (18) oq(d,ur)
                 * (19) oq(d,ll,out)
                 * (20) oq(d,ur,out)
                 * (21) oq(d,ll,in)
                 * (22) oq(d,ur,in)
                 */

                int cnt_ko_vp_q = 23;
                GurobiVariable[] ko_vp_q = new GurobiVariable[cnt_ko_vp_q];
                for (int var_cnt = 0; var_cnt < cnt_ko_vp_q; ++var_cnt) {
                    ko_vp_q[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + k.getName() + "_ko_vp_bVars_" + var_cnt);
                    executor.addVariable(ko_vp_q[var_cnt]);
                }
                vp.ko_vp_bVars.put(k, ko_vp_q);




                /*
                 *ko_vp_iVars_abs
                 * (0) vo_x_ll
                 * (1) vo_y_ll
                 * (2) vo_x_ur
                 * (3) vo_y_ur
                 */
                int cnt_ko_vp_iq_abs = 4;
                GurobiVariable[] ko_vp_iVars_abs = new GurobiVariable[cnt_ko_vp_iq_abs];
                for (int var_cnt = 0; var_cnt < cnt_ko_vp_iq_abs; ++var_cnt) {
                    ko_vp_iVars_abs[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub_abs, "v_" + i + ";" + k.getName() + "_ko_sp_iVars_ABS_" + var_cnt);
                    executor.addVariable(ko_vp_iVars_abs[var_cnt]);
                }
                vp.ko_vp_iVars_abs.put(k, ko_vp_iVars_abs);
                /*
                 * ko_vp_iVars
                 * (0) vx - ox_ll
                 * (1) vy - oy_ll
                 * (2) vx - ox_ur
                 * (3) vy - oy_ur
                 */
                int cnt_ko_vp_iq = 4;
                GurobiVariable[] ko_vp_iVars = new GurobiVariable[cnt_ko_vp_iq];
                for (int var_cnt = 0; var_cnt < cnt_ko_vp_iq; ++var_cnt) {
                    ko_vp_iVars[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub_abs, ub_abs, "v_" + i + ";" + k.getName() + "_ko_sp_iVars_" + var_cnt);
                    executor.addVariable(ko_vp_iVars[var_cnt]);
                }
                vp.ko_vp_iVars.put(k, ko_vp_iVars);


                Map<Slave_var, GurobiVariable[]> ko_sl_bVars = new HashMap<>();
                for (Slave_var sv : slave_vars) {
                    /*
                     * ko_sl_bVars (vp <-> slaves)
                     * (0) oq_vs(RL)
                     * (1) oq_vs(LR)
                     * (2) oq_vs(AB)
                     * (3) oq_vs(BA)
                     * (4) oq_vs(d)
                     * (5) oq_vs(d,ll,in)
                     * (6) oq_vs(d,ur,in)
                     */
                    int cnt_ko_sl_q = 7;
                    GurobiVariable[] ko_sl_q = new GurobiVariable[cnt_ko_sl_q];
                    for (int var_cnt = 0; var_cnt < cnt_ko_sl_q; ++var_cnt) {
                        ko_sl_q[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + k.getName() + "_ko_sl_bVars_" + var_cnt);
                        executor.addVariable(ko_sl_q[var_cnt]);
                    }
                    ko_sl_bVars.put(sv, ko_sl_q);


                }
                vp.ko_sl_bVars.put(k, ko_sl_bVars);


                //ooq
                Map<Keepout, GurobiVariable[]> oo_vp_bVars = new HashMap<>();
                for (Keepout otherK : uni_keepouts) {
                    /*
                     * oo_vp_bVars
                     * (0) ooq(d)
                     * (1) ooq(d,ll)
                     * (2) ooq(d,ll,ur)
                     * (3) ooq(d,ur,ll)
                     * (4) ooq(d,ur)
                     */

                    int cnt_oo_vp_q = 5;
                    GurobiVariable[] oo_vp_q = new GurobiVariable[cnt_oo_vp_q];
                    for (int var_cnt = 0; var_cnt < cnt_oo_vp_q; ++var_cnt) {
                        oo_vp_q[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, k.getName() + "-" + otherK.getName() + "_vp" + i + "_bVars_" + var_cnt);
                        executor.addVariable(oo_vp_q[var_cnt]);
                    }
                    oo_vp_bVars.put(otherK, oo_vp_q);
                }
                vp.oo_vp_bVars.put(k, oo_vp_bVars);
            }

            /*
            sl_bVars
            sl_iVars
             */
            for (Slave_var sv : slave_vars) {

                /*
                 * sl_bVars (vp <-> slaves)
                 * (0) q_vs(RL)
                 * (1) q_vs(LR)
                 * (2) q_vs(AB)
                 * (3) q_vs(BA)
                 * (4) q_vs(d)
                 * (5) q_vs: connection binary var
                 */
                int cnt_sl_q = 6;
                GurobiVariable[] sl_q = new GurobiVariable[cnt_sl_q];
                for (int var_cnt = 0; var_cnt < cnt_sl_q; ++var_cnt) {
                    sl_q[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + sv.getName() + "_sl_bVars_" + var_cnt);
                    executor.addVariable(sl_q[var_cnt]);
                }
                vp.sl_bVars.put(sv, sl_q);

                /*
                 * sl_iVars_abs
                 * (0) d_vs(d)
                 * (1) |v_i.x - s_j.x|
                 * (2) |v_i.y - s_j.y|
                 */
                int cnt_sl_iq_abs = 3;
                GurobiVariable[] sl_iq_abs = new GurobiVariable[cnt_sl_iq_abs];
                for (int var_cnt = 0; var_cnt < cnt_sl_iq_abs; ++var_cnt) {
                    sl_iq_abs[var_cnt] = new GurobiVariable(GRB.INTEGER, +0, ub_abs, "v_" + i + ";" + sv.getName() + "_sl_iVars_ABS_" + 0);
                    executor.addVariable(sl_iq_abs[var_cnt]);
                }
                vp.sl_iVars_abs.put(sv, sl_iq_abs);
                /*
                 * sl_iVars
                 * (0) v_i.x - s_j.x
                 * (1) v_i.y - s_j.y
                 */
                int cnt_sl_iq = 2;
                GurobiVariable[] sl_iq = new GurobiVariable[cnt_sl_iq];
                for (int var_cnt = 0; var_cnt < cnt_sl_iq; ++var_cnt) {
                    sl_iq[var_cnt] = new GurobiVariable(GRB.INTEGER, -ub_abs, ub_abs, "v_" + i + ";" + sv.getName() + "_sl_iVars_" + 0);
                    executor.addVariable(sl_iq[var_cnt]);
                }
                vp.sl_iVars.put(sv, sl_iq);


            }


            VPvars.add(vp);
        }
    }


    /**
     * 提供精確的加法運算。
     *
     * @param value1 被加數
     * @param value2 加數
     * @return 兩個引數的和
     */
    public static Double add_BD(Double value1, Double value2) {
        BigDecimal b1 = new BigDecimal(Double.toString(value1));
        BigDecimal b2 = new BigDecimal(Double.toString(value2));
        return b1.add(b2).doubleValue();
    }

    /**
     * 提供精確的減法運算。
     *
     * @param value1 被減數
     * @param value2 減數
     * @return 兩個引數的差
     */
    public static double sub_BD(Double value1, Double value2) {
        BigDecimal b1 = new BigDecimal(Double.toString(value1));
        BigDecimal b2 = new BigDecimal(Double.toString(value2));
        return b1.subtract(b2).doubleValue();
    }

    /**
     * 提供精確的乘法運算。
     *
     * @param value1 被乘數
     * @param value2 乘數
     * @return 兩個引數的積
     */
    public static Double mul_BD(Double value1, Double value2) {
        BigDecimal b1 = new BigDecimal(Double.toString(value1));
        BigDecimal b2 = new BigDecimal(Double.toString(value2));
        return b1.multiply(b2).doubleValue();
    }


}

