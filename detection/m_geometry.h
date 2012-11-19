#ifndef M_GEOMETRY_H

#define M_GEOMETRY_H
    struct Point {
        int x;
        int y;
        //
        Point():x(0),y(0) { }
        Point(int x0, int y0){ x = x0; y = y0; }

        Point( const Point &other ){ *this = other; }   
        Point& operator = ( const Point &other ) { x = other.x; y = other.y; return *this; }
        Point operator * (const Point &other) const{
            return Point(x * other.x, y * other.y);
        }
        Point& operator *= (int num){
            x *= num;
            y *= num;
            return *this;
        }
        Point operator + (const Point &other) const{
            return Point(x + other.x, y + other.y);
        }
        Point& operator += (const Point &other){
            x += other.x;
            y += other.y;
            return *this;
        }
        //! do support string like weak order comparing
        bool operator < (const Point &other) const{
            if (x < other.x)
                return true;
            else if (x > other.x)
                return false;
            else if (y < other.y)
                return true;
            else 
                return false;
        };
    }; 

    struct PointF {
        double x;
        double y;
        //
        PointF():x(0),y(0) { }
        PointF(int x0, int y0):x(x0), y(y0){ }
        PointF(const Point &other):x(other.x), y(other.y){ }

        PointF( const PointF &other ){ *this = other; }   
        PointF& operator = ( const PointF &other ) { x = other.x; y = other.y; return *this; }

        PointF operator + (const Point &other) const{
            return PointF(x + other.x, y + other.y);
        }
        PointF& operator += (const PointF &other){
            x += other.x;
            y += other.y;
            return *this;
        }

        PointF operator * (const Point &other) const{
            return PointF(x * other.x, y * other.y);
        }
        PointF& operator *= (double num){
            x *= num;
            y *= num;
            return *this;
        }
        //! do support string like weak order comparing
        bool operator < (const PointF &other) const{
            if (x < other.x)
                return true;
            else if (x > other.x)
                return false;
            else if (y < other.y)
                return true;
            else 
                return false;
        };

    }; 

    std::ostream& operator<<(std::ostream& out, const Point &t);
    std::ostream& operator<<(std::ostream& out, const PointF &t);

#endif /* end of include guard: M_GEOMETRY_H */
