import gurobi.GRBException;
import parser.OutputDoc;
import parser.OutputDocProcess;
import processor.Processor;
import shapes.Keepout;
import shapes.Master;
import shapes.Poly_Keepout;
import shapes.Slave;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class Test_KO_hw_test {

    public static void main (String[] args){
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: "+ DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));

        Processor processor = new Processor();

        Master master = new Master(883, 1537);

        Slave R704 = new Slave(976, 1228);
        R704.setName("R704");
        Slave U5701 = new Slave(-562, 1393);
        U5701.setName("U5701");
        Slave U1701 = new Slave(-813, 1433);
        U1701.setName("U1701");
        Slave U2501 = new Slave(-652, 1576);
        U2501.setName("U2501");
        Slave U1901 = new Slave(-214, 2012);
        U1901.setName("U1901");
        Slave U2500 = new Slave(-382, 2300);
        U2500.setName("U2500");
        Slave U1702 = new Slave(-1014, 1427);
        U1702.setName("U1702");
        Slave R2912 = new Slave(-140, 2385);
        R2912.setName("R2912");

        ArrayList<Slave> slaves = new ArrayList<>();
        slaves.add(R704);
        slaves.add(U5701);
        slaves.add(U1701);
        slaves.add(U2501);
        slaves.add(U1901);
        slaves.add(U2500);
        slaves.add(U1702);
        slaves.add(R2912);

        ArrayList<Poly_Keepout> poly_keepouts = new ArrayList<>();



        Keepout v_ko1 = new Keepout("o1",-451,-39,1455,1992);
        Keepout v_ko2 = new Keepout("o2", -39,461,1455,2352);
        ArrayList<Keepout> uni_keepouts = new ArrayList<>();
        uni_keepouts.add(v_ko1);
        uni_keepouts.add(v_ko2);






        try {
            OutputDoc outputDoc = processor.processToOutput_w_multiKO("hwTest",master, slaves, uni_keepouts, poly_keepouts,1, 1);
            LocalDateTime end = LocalDateTime.now();
            OutputDocProcess finOutput = new OutputDocProcess(outputDoc, "Result/test_hw");
            Duration duration = Duration.between(start,end);
            LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
            System.out.println("Program Ends at: "+ DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
            System.out.println("Program Run Time is: "+ DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));
        } catch (GRBException | FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
