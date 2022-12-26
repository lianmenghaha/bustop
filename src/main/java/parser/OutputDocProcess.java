package parser;

import org.bouncycastle.asn1.esf.SPuri;
import processor.DrawPDF;
import shapes.*;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.util.ArrayList;

public class OutputDocProcess {

    private OutputDoc outputDoc;

    public OutputDocProcess(OutputDoc outputDoc, String path) throws IOException {
        this.outputDoc = outputDoc;

        this.screenMessage();
        this.writeFile(path);
        new DrawPDF(outputDoc, path).outputToPdf();
    }

    public void screenMessage() {

        System.out.println(outputDoc.getI2Cname());
        System.out.println("BusLength = " + outputDoc.getBusLength());
        System.out.println("SideBusLength = " + outputDoc.getSideBusLength());
        System.out.println(outputDoc.getMaster().getName() + "(" + outputDoc.getMaster().getX_ct() + ", " + outputDoc.getMaster().getY_ct() + ")");
        ArrayList<VirtualPoint> vps = outputDoc.getVirtualPoints();
        for (int i = 0; i < vps.size(); ++i) {
            VirtualPoint vp = vps.get(i);
            System.out.println(
                    vp.getName() + "(" + vp.getX_ct() + ", " + vp.getY_ct() + ")" + " ---- " + vp.getSlave().getName() + "(" + vp.getSlave().getX_ct() + ", " + vp.getSlave().getY_ct() + ")"
            );
        }
        System.out.println("---------------------------------");
        System.out.println("BusLine");
        VirtualPoint vp1 = vps.get(0);
        System.out.println(outputDoc.getMaster().getName() + " and vp1:");
        if (vp1.isDetour_mv()) {
            ArrayList<ObObC> rel_mv_obs = vp1.getRel_mv_Obs();
            for (ObObC connect : rel_mv_obs) {
                if (connect.type == ConnectionType.URtoLL) {
                    System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName() + ".LL");
                }
                if (connect.type == ConnectionType.URtoUR) {
                    System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName() + ".UR");
                }
                if (connect.type == ConnectionType.LLtoUR) {
                    System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName() + ".UR");
                }
                if (connect.type == ConnectionType.LLtoLL) {
                    System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName() + ".LL");
                }
                if (connect.type == ConnectionType.VPtoLL) {
                    System.out.println(connect.start.getName() + " ---- " + connect.end.getName() + ".LL");
                }
                if (connect.type == ConnectionType.VPtoUR) {
                    System.out.println(connect.start.getName() + " ---- " + connect.end.getName() + ".UR");
                }
                if (connect.type == ConnectionType.URtoMS) {
                    System.out.println(connect.end.getName() + " ---- " + connect.start.getName() + ".UR");
                }
                if (connect.type == ConnectionType.LLtoMS) {
                    System.out.println(connect.end.getName() + " ---- " + connect.start.getName() + ".LL");
                }

            }
        } else {
            System.out.println(outputDoc.getMaster().getName() + " ---- vp1");
        }
        System.out.println();
        for (int vp_cnt = 0; vp_cnt < vps.size() - 1; ++vp_cnt) {
            VirtualPoint vp = vps.get(vp_cnt);
            System.out.println("vp" + (vp_cnt + 1) + " and vp" + (vp_cnt + 2) + ":");
            if (vp.isDetour_vp()) {
                ArrayList<ObObC> rel_vp_obs = vp.getRel_vp_Obs();
                for (ObObC connect : rel_vp_obs) {
                    System.out.println(connect);
                    if (connect.type == ConnectionType.URtoLL) {
                        System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.URtoUR) {
                        System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.LLtoUR) {
                        System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.LLtoLL) {
                        System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.VPtoLL) {
                        System.out.println(connect.start.getName() + " ---- " + connect.end.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.VPtoUR) {
                        System.out.println(connect.start.getName() + " ---- " + connect.end.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.LLtoVP) {
                        //System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName());
                        System.out.println(connect.end.getName() + " ---- " + connect.start.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.URtoVP) {
                        //System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName());
                        System.out.println(connect.end.getName() + " ---- " + connect.start.getName() + ".UR");
                    }


                }
            } else {
                System.out.println("vp" + (vp_cnt + 1) + " ---- vp" + (vp_cnt + 2));
            }
            System.out.println();
        }

        System.out.println("---------------------------------");
        System.out.println("SideBusLine");
        for (VirtualPoint vp : vps) {
            Slave sv = vp.getSlave();
            System.out.println(vp.getName() + " and " + sv.getName() + ":");
            if (vp.isDetour_sl()) {
                ArrayList<ObObC> rel_sl_obs = vp.getRel_sl_Obs();
                for (ObObC connect : rel_sl_obs) {
                    if (connect.type == ConnectionType.URtoLL) {
                        System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.URtoUR) {
                        System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.LLtoUR) {
                        System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.LLtoLL) {
                        System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.VPtoLL) {
                        System.out.println(connect.start.getName() + " ---- " + connect.end.getName() + ".LL");
                    }
                    if (connect.type == ConnectionType.VPtoUR) {
                        System.out.println(connect.start.getName() + " ---- " + connect.end.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.URtoSL) {
                        //System.out.println(connect.start.getName() + ".UR ---- " + connect.end.getName());
                        System.out.println(connect.end.getName() + " ---- " + connect.start.getName() + ".UR");
                    }
                    if (connect.type == ConnectionType.LLtoSL) {
                        //System.out.println(connect.start.getName() + ".LL ---- " + connect.end.getName());
                        System.out.println(connect.end.getName() + " ---- " + connect.start.getName() + ".LL");
                    }


                }

            } else {
                System.out.println(vp.getName() + " ---- " + sv.getName());
            }
            System.out.println();
        }


    }

    public void writeFile(String path) throws IOException {
        String drawPath = path + ".txt ";
        File file = new File(drawPath);
        FileWriter writer = new FileWriter(file);

        ArrayList<VirtualPoint> vps = outputDoc.getVirtualPoints();
        ArrayList<Keepout> keepouts = outputDoc.getKeepouts();
        ArrayList<Slave> slaves = outputDoc.getSlaves();
        Master master = outputDoc.getMaster();
        writer.write("Master\n" + master.getName() + " " + master.getX_ct() + " " + master.getY_ct() + "\n");
        writer.write("Uni_Keepout\n");
        for (Keepout o : keepouts){
            writer.write(o.getMinX() + " " + o.getMaxX() + " " + o.getMinY() + " " + o.getMaxY() + "\n");
        }
        writer.write("FIN\n");
        writer.write("Slave\n");
        for (Slave sv : slaves){
            writer.write(sv.getName() + " " + sv.getX_ct() + " " + sv.getY_ct() + "\n");
        }
        writer.write("FIN\n");
        VirtualPoint vp1 = vps.get(0);
        writer.write("Connection\n");
        if (vp1.isDetour_mv()) {
            ArrayList<ObObC> rel_mv_obs = vp1.getRel_mv_Obs();
            for (ObObC connect : rel_mv_obs) {
                if (connect.type == ConnectionType.URtoLL) {

                    writer.write(((Keepout) connect.start).getMaxX() + "," + ((Keepout) connect.start).getMaxY() + " ");
                    writer.write(((Keepout) connect.end).getMinX() + "," + ((Keepout) connect.end).getMinY() + "\n");

                }
                if (connect.type == ConnectionType.URtoUR) {

                    writer.write(((Keepout) connect.start).getMaxX() + "," + ((Keepout) connect.start).getMaxY() + " ");
                    writer.write(((Keepout) connect.end).getMaxX() + "," + ((Keepout) connect.end).getMaxY() + "\n");

                }
                if (connect.type == ConnectionType.LLtoUR) {

                    writer.write(((Keepout) connect.start).getMinX() + "," + ((Keepout) connect.start).getMinY() + " ");
                    writer.write(((Keepout) connect.end).getMaxX() + "," + ((Keepout) connect.end).getMaxY() + "\n");

                }
                if (connect.type == ConnectionType.LLtoLL) {

                    writer.write(((Keepout) connect.start).getMinX() + "," + ((Keepout) connect.start).getMinY() + " ");
                    writer.write(((Keepout) connect.end).getMinX() + "," + ((Keepout) connect.end).getMinY() + "\n");

                }
                if (connect.type == ConnectionType.VPtoLL) {

                    writer.write(connect.start.getX_ct() + "," + connect.start.getY_ct() + " ");
                    writer.write(((Keepout) connect.end).getMinX() + "," + ((Keepout) connect.end).getMinY() + "\n");

                }
                if (connect.type == ConnectionType.VPtoUR) {

                    writer.write(connect.start.getX_ct() + "," + connect.start.getY_ct() + " ");
                    writer.write(((Keepout) connect.end).getMaxX() + "," + ((Keepout) connect.end).getMaxY() + "\n");

                }
                if (connect.type == ConnectionType.URtoMS) {
                    writer.write(((Keepout) connect.start).getMaxX() + "," + ((Keepout) connect.start).getMaxY() + " ");
                    writer.write(outputDoc.getMaster().getX_ct() + "," + outputDoc.getMaster().getY_ct() + "\n");

                }
                if (connect.type == ConnectionType.LLtoMS) {

                    writer.write(((Keepout) connect.start).getMinX() + "," + ((Keepout) connect.start).getMinY() + " ");
                    writer.write(outputDoc.getMaster().getX_ct() + "," + outputDoc.getMaster().getY_ct() + "\n");
                }

            }
        } else {
            writer.write(master.getX_ct() + "," + master.getY_ct() + " ");
            writer.write(vp1.getX_ct() + ", " + vp1.getY_ct() + "\n");
        }
        //writer.write("FIN\n");
        //writer.write("virtualPoints:\n");
        for (int vp_cnt = 0; vp_cnt < vps.size() - 1; ++vp_cnt) {
            VirtualPoint vp = vps.get(vp_cnt);
            VirtualPoint vpNext = vps.get(vp_cnt + 1);
            //writer.write(vp.getName() + "&" + vpNext.getName() + "\n");
            if (vp.isDetour_vp()) {
                ArrayList<ObObC> rel_vp_obs = vp.getRel_vp_Obs();
                for (ObObC connect : rel_vp_obs) {
                    if (connect.type == ConnectionType.URtoLL) {

                        writer.write(((Keepout) connect.start).getMaxX() + "," + ((Keepout) connect.start).getMaxY() + " ");
                        writer.write(((Keepout) connect.end).getMinX() + "," + ((Keepout) connect.end).getMinY() + "\n");

                    }
                    if (connect.type == ConnectionType.URtoUR) {

                        writer.write(((Keepout) connect.start).getMaxX() + "," + ((Keepout) connect.start).getMaxY() + " ");
                        writer.write(((Keepout) connect.end).getMaxX() + "," + ((Keepout) connect.end).getMaxY() + "\n");

                    }
                    if (connect.type == ConnectionType.LLtoUR) {

                        writer.write(((Keepout) connect.start).getMinX() + "," + ((Keepout) connect.start).getMinY() + " ");
                        writer.write(((Keepout) connect.end).getMaxX() + "," + ((Keepout) connect.end).getMaxY() + "\n");

                    }
                    if (connect.type == ConnectionType.LLtoLL) {

                        writer.write(((Keepout) connect.start).getMinX() + "," + ((Keepout) connect.start).getMinY() + " ");
                        writer.write(((Keepout) connect.end).getMinX() + "," + ((Keepout) connect.end).getMinY() + "\n");

                    }
                    if (connect.type == ConnectionType.VPtoLL) {

                        writer.write(connect.start.getX_ct() + "," + connect.start.getY_ct() + " ");
                        writer.write(((Keepout) connect.end).getMinX() + "," + ((Keepout) connect.end).getMinY() + "\n");

                    }
                    if (connect.type == ConnectionType.VPtoUR) {

                        writer.write(connect.start.getX_ct() + "," + connect.start.getY_ct() + " ");
                        writer.write(((Keepout) connect.end).getMaxX() + "," + ((Keepout) connect.end).getMaxY() + "\n");

                    }
                    if (connect.type == ConnectionType.LLtoVP) {
                        writer.write(((Keepout) connect.start).getMaxX() + "," + ((Keepout) connect.start).getMaxY() + " ");
                        writer.write(connect.end.getX_ct() + "," + connect.end.getY_ct() + "\n");

                    }
                    if (connect.type == ConnectionType.URtoVP) {

                        writer.write(((Keepout) connect.start).getMinX() + "," + ((Keepout) connect.start).getMinY() + " ");
                        writer.write(connect.end.getX_ct() + "," + connect.end.getY_ct() + "\n");
                    }


                }
            } else {
                writer.write(vp.getX_ct() + "," + vp.getY_ct() + " ");
                writer.write(vpNext.getX_ct() + "," + vpNext.getY_ct() + "\n");
            }
            //writer.write("FIN\n");
        }
        //writer.write("slaves:\n");
        for (VirtualPoint vp : vps) {
            Slave sv = vp.getSlave();
            //writer.write(vp.getName() + "&" + sv.getName() + "\n");
            if (vp.isDetour_sl()) {
                ArrayList<ObObC> rel_sl_obs = vp.getRel_sl_Obs();
                for (ObObC connect : rel_sl_obs) {
                    if (connect.type == ConnectionType.URtoLL){

                        writer.write(((Keepout)connect.start).getMaxX()+ "," + ((Keepout)connect.start).getMaxY()+ " ");
                        writer.write(((Keepout)connect.end).getMinX()+ "," + ((Keepout)connect.end).getMinY()+ "\n");

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        writer.write(((Keepout)connect.start).getMaxX()+ "," +  ((Keepout)connect.start).getMaxY()+ " ");
                        writer.write(((Keepout)connect.end).getMaxX()+ "," + ((Keepout)connect.end).getMaxY()+ "\n");

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        writer.write(((Keepout)connect.start).getMinX()+ "," + ((Keepout)connect.start).getMinY()+ " ");
                        writer.write(((Keepout)connect.end).getMaxX()+ "," + ((Keepout)connect.end).getMaxY()+ "\n");

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        writer.write(((Keepout)connect.start).getMinX()+ "," + ((Keepout)connect.start).getMinY()+ " ");
                        writer.write(((Keepout)connect.end).getMinX()+ "," + ((Keepout)connect.end).getMinY()+ "\n");

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        writer.write(connect.start.getX_ct()+ "," +connect.start.getY_ct()+ " ");
                        writer.write(((Keepout)connect.end).getMinX()+ "," +((Keepout)connect.end).getMinY()+ "\n");

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        writer.write(connect.start.getX_ct()+ "," + connect.start.getY_ct()+ " ");
                        writer.write(((Keepout)connect.end).getMaxX()+ "," +((Keepout)connect.end).getMaxY()+ "\n");

                    }
                    if (connect.type == ConnectionType.URtoSL){
                        writer.write(((Keepout)connect.start).getMaxX()+ "," + ((Keepout)connect.start).getMaxY()+ " ");
                        writer.write(connect.end.getX_ct()+ "," + connect.end.getY_ct()+ "\n");

                    }
                    if (connect.type == ConnectionType.LLtoSL){

                        writer.write(((Keepout)connect.start).getMinX()+ "," +  ((Keepout)connect.start).getMinY()+ " ");
                        writer.write(connect.end.getX_ct()+ "," +  connect.end.getY_ct()+ "\n");
                    }
                }

            } else {
                writer.write(vp.getX_ct() + "," + vp.getY_ct() + " ");
                writer.write(sv.getX_ct() + "," + sv.getY_ct() + "\n");
            }
            //writer.write("FIN\n");
        }

        writer.write("FIN\n");
        writer.close();

    }

}
