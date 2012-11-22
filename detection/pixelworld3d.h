/**
 * @file pixelworld.h
 * @brief compute attraction force
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */
#ifndef PIXElWORLD3D_H
#define PIXELWORLD3D_H

template < typename T >
class PixelWorld3D {


    /*---------------------------  lifecycle  ------------------------------------------------ */
    public:
        typedef std::set<T> PixelSet;
        explicit PixelWorld3D(bool isGrid = true){ };
        virtual ~PixelWorld3D(){ };

        /*------------------------------------------------------------------------------------ */
        T* operator[](int rowIndx) { return _pixels[rowIndx]; }
        const T* operator[](int rowIndx)const { return _pixels[rowIndx]; }

    protected:

        /*typename PixelSet::iterator get_pixel(int x, int y, int z) const;*/
        std::vector<T*> get_neighbors(const T &t) const;
        std::vector<T*> _grid_neighbors(const T &t) const;
        std::vector<T*> _feature_neighbors(const T &t) const{ };

        /*--------------------  accessor methods  -------------------------------------------- */
    public:

    protected:
        bool            _isGrid;
        PixelSet     _pixels;    


}; 

//
template < typename T >
inline std::vector<T*> PixelWorld3D<T>::get_neighbors(const T &t) const{
    return _isGrid ? _grid_neighbors(t) :  _feature_neighbors(t);
}
/*template < typename T >*/
/*inline typename PixelSet::iterator Segment3D<T>::get_pixel(int x, int y, int z) const{*/
/*return _pixels.find(T(x, y, z));*/
/*}*/
template < typename T >
inline std::vector<T*> PixelWorld3D<T>::_grid_neighbors(const T &t) const{
    // push 8 nearest neighbors
    std::vector<T*> neighbors;
    typename PixelSet::iterator i;
    // the middle plane
    i = _pixels.find(T(t._x - 1, t._y - 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y - 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y - 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y + 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y + 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y + 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    // the top plane

    i = _pixels.find(T(t._x - 1, t._y - 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y - 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y - 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x , t._y, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y + 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y + 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y + 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    // the bottom plane

    i = _pixels.find(T(t._x - 1, t._y - 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y - 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y - 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x , t._y, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y + 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y + 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y + 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    return neighbors;

}

#endif /* end of include guard: PIXELWORLD3D.H */
