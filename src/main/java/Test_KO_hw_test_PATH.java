import gurobi.GRBException;
import parser.OutputDoc;
import parser.OutputDocProcess;
import processor.Processor;

import java.io.FileNotFoundException;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

public class Test_KO_hw_test_PATH {

    public static void main(String[] args) throws FileNotFoundException, GRBException {
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));
        Processor processor = new Processor();

        OutputDoc outputDoc = processor.processToOutput_w_multiKO("Input/case1_I2C0_SCL");
        LocalDateTime end = LocalDateTime.now();
        OutputDocProcess finOutput = new OutputDocProcess(outputDoc, "Result/test_hw");
        Duration duration = Duration.between(start, end);
        LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
        System.out.println("Program Ends at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
        System.out.println("Program Run Time is: " + DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));

    }
}
