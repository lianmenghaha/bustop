package processor;

import com.itextpdf.kernel.colors.DeviceCmyk;
import com.itextpdf.kernel.colors.DeviceRgb;
import com.itextpdf.kernel.pdf.PdfDocument;
import com.itextpdf.kernel.pdf.PdfWriter;
import com.itextpdf.kernel.pdf.canvas.PdfCanvas;
import com.itextpdf.kernel.colors.Color;
import com.itextpdf.kernel.geom.Rectangle;
import parser.OutputDoc;
import shapes.*;

import java.awt.*;
import java.io.FileNotFoundException;
import java.util.ArrayList;

import static com.itextpdf.kernel.colors.Color.convertRgbToCmyk;

public class DrawPDF {
    private OutputDoc outputDoc;
    private final String path;

    public DrawPDF(OutputDoc outputDoc, String path) {
        this.outputDoc = outputDoc;
        this.path = path;
    }

    public void outputToPdf() throws FileNotFoundException {
        String pdfPath = this.path + "_" +outputDoc.getI2Cname();
        PdfDocument pdfDoc = new PdfDocument(new PdfWriter(pdfPath));

        ArrayList<VirtualPoint> virtualPoints = outputDoc.getVirtualPoints();
        ArrayList<Slave> slaves = outputDoc.getSlaves();
        ArrayList<Keepout> uni_keepouts = outputDoc.getKeepouts();
        Master master = outputDoc.getMaster();


        PdfCanvas canvas = new PdfCanvas(pdfDoc.addNewPage());
        canvas.setLineWidth((float) 0.2);

        /*
        draw Keepouts
         */
        int bias = 200;
        for (Keepout o : uni_keepouts){
            Rectangle rect = new Rectangle(o.getMinX() + bias, o.getMinY() + bias, o.getMaxX() - o.getMinX(), o.getMaxY() - o.getMinY());
            Color Blue = convertRgbToCmyk(new DeviceRgb(0,0,255));
            canvas.setColor(Blue, true)
                    .rectangle(rect)
                    .fill()
                    .stroke();
        }
        /*
        draw Master
         */
        Color PINK = convertRgbToCmyk(new DeviceRgb(255,182,193));
        canvas.setColor(PINK, false)
                .circle(master.getX_ct() + bias, master.getY_ct() + bias, 0.2)
                .stroke();


        for (VirtualPoint vp : virtualPoints){
            /*
            draw Vp
             */
            Color CYAN = convertRgbToCmyk(new DeviceRgb(0,255,255));
            canvas.setColor(CYAN, false)
                    .circle(vp.getX_ct()+ bias, vp.getY_ct()+ bias, 0.15)
                    .stroke();
            ArrayList<ObObC> rel_mv_obs = vp.getRel_mv_Obs();
            ArrayList<ObObC> rel_vp_obs = vp.getRel_vp_Obs();
            ArrayList<ObObC> rel_sl_obs = vp.getRel_sl_Obs();
            if (rel_mv_obs.size() != 0){
                Color LIME = convertRgbToCmyk(new DeviceRgb(0, 255,0));
                canvas.setColor(LIME, false);
                for (ObObC connect : rel_mv_obs){
                    if (connect.type == ConnectionType.URtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.URtoMS){
                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(master.getX_ct() + bias, master.getY_ct() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoMS){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(master.getX_ct() + bias, master.getY_ct() + bias);
                    }
                    if (connect.type == ConnectionType.NoDetour){
                        canvas.moveTo(connect.start.getX_ct() + bias, connect.end.getY_ct() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);
                    }
                    canvas.closePathStroke();


                }
            }

            if (rel_vp_obs.size() != 0){
                Color LIME = convertRgbToCmyk(new DeviceRgb(0, 255,0));
                canvas.setColor(LIME, false);
                for (ObObC connect : rel_vp_obs){
                    if (connect.type == ConnectionType.URtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoVP){
                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);

                    }
                    if (connect.type == ConnectionType.URtoVP){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);
                    }
                    if (connect.type == ConnectionType.NoDetour){
                        canvas.moveTo(connect.start.getX_ct() + bias, connect.end.getY_ct() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);
                    }

                    canvas.closePathStroke();


                }
            }

            if (rel_sl_obs.size() != 0){
                Color ORANGE = convertRgbToCmyk(new DeviceRgb(255,165,0));
                canvas.setColor(ORANGE, false);
                for (ObObC connect : rel_sl_obs){
                    System.out.println(connect);
                    if (connect.type == ConnectionType.URtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMinX() + bias, ((Keepout)connect.end).getMinY() + bias);

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(((Keepout)connect.end).getMaxX() + bias, ((Keepout)connect.end).getMaxY() + bias);

                    }
                    if (connect.type == ConnectionType.URtoSL){
                        canvas.moveTo(((Keepout)connect.start).getMaxX() + bias, ((Keepout)connect.start).getMaxY() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);

                    }
                    if (connect.type == ConnectionType.LLtoSL){

                        canvas.moveTo(((Keepout)connect.start).getMinX() + bias, ((Keepout)connect.start).getMinY() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);
                    }
                    if (connect.type == ConnectionType.NoDetour){
                        canvas.moveTo(connect.start.getX_ct() + bias, connect.start.getY_ct() + bias);
                        canvas.lineTo(connect.end.getX_ct() + bias, connect.end.getY_ct() + bias);
                    }

                    canvas.closePathStroke();


                }
            }


        }

        /*
        draw Slaves
         */
        for (Slave sv : slaves){
            Color RED = convertRgbToCmyk(new DeviceRgb(255,0,0));
            canvas.setColor(RED, false)
                    .circle(sv.getX_ct()+ bias, sv.getY_ct()+ bias, 0.15)
                    .stroke();
        }



        pdfDoc.close();




    }







}
