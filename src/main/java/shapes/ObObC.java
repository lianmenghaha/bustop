package shapes;

public class ObObC {
    public Shape start;
    public Shape end;
    public ConnectionType type;

    public ObObC(Shape start, Shape end, ConnectionType type) {
        this.start = start;
        this.end = end;
        this.type = type;
    }

    @Override
    public String toString() {
        String typeName;
        if (type == ConnectionType.LLtoLL){
            typeName = "LLtoLL";
        }else if (type == ConnectionType.LLtoUR){
            typeName = "LLtoUR";
        }else if (type == ConnectionType.URtoLL){
            typeName = "URtoLL";
        }else if (type == ConnectionType.URtoUR){
            typeName = "URtoUR";
        }else if (type == ConnectionType.URtoMS){
            typeName = "URtoMS";
        }else if (type == ConnectionType.LLtoMS){
            typeName = "LLtoMS";
        }else if (type == ConnectionType.URtoSL){
            typeName = "URtoSL";
        }else if (type == ConnectionType.LLtoSL){
            typeName = "LLtoSL";
        }else if (type == ConnectionType.VPtoLL){
            typeName = "VPtoLL";
        }else if (type == ConnectionType.VPtoUR){
            typeName = "VPtoUR";
        }else if (type == ConnectionType.LLtoVP) {
            typeName = "LLtoVP";
        }else if (type == ConnectionType.URtoVP) {
            typeName = "URtoVP";
        }else if (type == ConnectionType.NoDetour){
            typeName = "NoDetour";
        }
        else {
            typeName = "UNKNOWN";
        }

        return "ObObC{" +
                "startOb=" + start.getName() +
                ", endOb=" + end.getName() +
                ", type=" + typeName +
                '}';
    }
}
