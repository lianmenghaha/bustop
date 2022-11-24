package shapes;

import java.util.ArrayList;

public class Path_Tree {

    private T_Node master;

    public Path_Tree(Node node) {
        this.master = new T_Node(node);
    }

    public static class T_Node {
        private int num;
        private double x;
        private double y;
        private T_Node parent;
        private ArrayList<T_Node> children;


        public T_Node(int num, double x, double y, T_Node parent, ArrayList<T_Node> children) {
            this.num = num;
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.children = children;
        }

        public T_Node(Node node) {
            this.num = node.getNum();
            this.x = node.getX_exact();
            this.y = node.getY_exact();
            this.children = new ArrayList<>();
        }

        public int getNum() {
            return num;
        }

        public void setNum(int num) {
            this.num = num;
        }

        public T_Node getParent() {
            return parent;
        }

        public void setParent(T_Node parent) {
            this.parent = parent;
        }

        public ArrayList<T_Node> getChildren() {
            return children;
        }

        public void setChildren(ArrayList<T_Node> children) {
            this.children = children;
        }

        public void addToChildren(T_Node node) {
            this.children.add(node);
        }

        @Override
        public String toString() {
            return "T_Node{" +
                    "num=" + num +
                    ", x=" + x +
                    ", y=" + y +
                    '}';
        }
    }

    public T_Node getMaster() {
        return master;
    }

    public void setMaster(T_Node master) {
        this.master = master;
    }

    @Override
    public String toString() {
        return "Path_Tree{" +
                "master=" + master +
                '}';
    }
}
