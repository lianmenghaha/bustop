package shapes;

import java.util.ArrayList;

public class Node extends Point{

    private int num; //the index of Node
    private NodeType type;//the type of Node: Mater, Terminal, SterinNode
    private ArrayList<Integer> C_E;
    private ArrayList<Integer> C_without_E;



    public Node(int num, NodeType type) {
        this.num = num;
        this.type = type;
        this.C_E = new ArrayList<>();
        this.C_without_E = new ArrayList<>();
    }

    public Node(double x, double y, NodeType type) {
        super(x, y);
        this.type = type;
        this.C_E = new ArrayList<>();
        this.C_without_E = new ArrayList<>();
    }

    public Node(double x, double y) {
        super(x, y);
        this.C_E = new ArrayList<>();
        this.C_without_E = new ArrayList<>();
    }

    public Node(double x, double y, int num, NodeType type) {
        super(x, y);
        this.num = num;
        this.type = type;
        this.C_E = new ArrayList<>();
        this.C_without_E = new ArrayList<>();
    }

    public ArrayList<Integer> getC_E() {
        return C_E;
    }

    public void setC_E(ArrayList<Integer> c_E) {
        C_E = c_E;
    }

    public ArrayList<Integer> getC_without_E() {
        return C_without_E;
    }

    public void setC_without_E(ArrayList<Integer> c_without_E) {
        C_without_E = c_without_E;
    }

    public int getNum() {
        return num;
    }

    public void setNum(int num) {
        this.num = num;
    }

    public NodeType getType() {
        return type;
    }

    public void setType(NodeType type) {
        this.type = type;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null) return false;
        Node node = (Node) o;
        return Double.compare(node.x, x) == 0 && Double.compare(node.y, y) == 0;
    }


    @Override
    public String toString() {
        return "Node{" +
                "num=" + num +
                ", type=" + type +
                ", C_E=" + C_E +
                ", C_without_E=" + C_without_E +
                ", x=" + x +
                ", y=" + y +
                '}';
    }
}
