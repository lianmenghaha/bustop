import processor.Processor;
import shapes.Master;
import shapes.Slave;

import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class Test_DW {

    public static void main (String[] args){
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: "+ DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));

        Processor processor = new Processor();

        Master master = new Master(883.0, 1537.0);
        Master master_test = new Master(3,17);


        Slave s1 = new Slave(0.0,1.0);
        s1.setName("s1");
        Slave s2 = new Slave(1.0,0.0);
        s2.setName("s2");
        Slave s3 = new Slave(7.0, 2.2);
        s3.setName("s3");
        Slave s4 = new Slave(6.0, 1.2);
        s4.setName("s4");
        Slave s5 = new Slave(4.0, 4.2);
        s5.setName("s5");

        Slave R704 = new Slave(975.5, 1228.0);
        R704.setName("R704");
        Slave U5701 = new Slave(-562.5, 1393.0);
        U5701.setName("U5701");
        Slave U1701 = new Slave(-813.0, 1432.5);
        U1701.setName("U1701");
        Slave U2501 = new Slave(-652.5, 1575.5);
        U2501.setName("U2501");
        Slave U1901 = new Slave(-214, 2012.0);
        U1901.setName("U1901");
        Slave U2500 = new Slave(-382.5, 2299.5);
        U2500.setName("U2500");
        Slave U1702 = new Slave(-1014, 1426.5);
        U1702.setName("U1702");
        Slave R2912 = new Slave(-140.50, 2385);
        R2912.setName("R2912");


        ArrayList<Slave> slaves = new ArrayList<>();
        /*slaves.add(s1);
        slaves.add(s2);
        slaves.add(s3);
        slaves.add(s4);
        slaves.add(s5);*/

        slaves.add(R704);
        slaves.add(U5701);
        slaves.add(U1701);
        slaves.add(U2501);
        slaves.add(U1901);
        slaves.add(U2500);
        slaves.add(U1702);
        slaves.add(R2912);



        processor.processToOutput_DW_MD(master,slaves);
        LocalDateTime end = LocalDateTime.now();
        Duration duration = Duration.between(start,end);
        LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
        System.out.println("Program Ends at: "+ DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
        System.out.println("Program Run Time is: "+ DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));
    }
}
