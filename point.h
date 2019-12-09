#include <string>


class Point{
    private:
        double x;
        double y;
        double z;
    public:
        Point();
        Point(double X, double Y, double Z);
        void display(std::ostream& str);
        void set(double X, double Y, double Z);
        void setX(double X);
        void setY(double Y);
        void setZ(double Z);
};


