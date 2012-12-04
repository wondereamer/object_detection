#ifndef COMPONENT_H


#define COMPONENT_H
#include <type_traits>
#include <cassert>
static const double SCALE = 0;

/**
 * @brief a set of pixel pointers
 *
 * @tparam T a pointer to pixel
 */
template < typename T >
class Component {
    public:
        typedef typename std::remove_pointer<T>::type Pixel;
        typedef typename Pixel::ColorType Color;
    public:
        Component( ): _max_weight(0) { }
        Component( T first_pixel): _max_weight(0) {
            // an single pixel componnet
            add_member(first_pixel);
        }
        // compare operation for componets that only have one pixel
        bool operator < (const Component &r) const{
            assert(r.get_members().size() == 1 && _members.size() == 1);
            return *(r.get_members().begin()) < *_members.begin();
        }
        // @speed this function may not be inlined!
        /**
         * @brief merge small componet to bigger component  
         *
         * @return 0: merge t to s; 1: merge s to t; -1: can't merge 
         */
        static int merge(double component_diff,   Component &s, Component &t){
            if(_if_merge(component_diff, s, t)){
                // merge to bigger component
                if( t.size() > s.size()){
                    // merge s to t
                    for(T m : s.get_members()){
                        // move r' member to s
                        t.add_member(m);
                    }
                    t.set_max_weight(component_diff);
                    return 1;
                }else{
                    for(T m : t.get_members()){
                        s.add_member(m);
                    }
                    s.set_max_weight(component_diff);
                    return 0;
                }
            }
            return -1;
        }    
        void merge_from(const Component &r){
            for(T pixel: r.get_members()){
                _members.insert(pixel);
            }
        }
        //! check if two component close enough to merge
        static bool _if_merge(double comp_diff, const Component &a, const Component &b) {
            //            std::cout<<"comp_diff: "<<comp_diff<<" a.diff: "<<a.internal_diff()<<" b.diff: "<<b.internal_diff()<<std::endl;
            return  comp_diff <= std::min(a.internal_diff(), b.internal_diff());
        }
        //! the internal difference of the component
        double internal_diff() const{
            //            std::cout<<"weight: "<<_max_weight<< " size: "<<_members.size()<<std::endl;
            return _max_weight + K / _members.size();
        }
        bool contains(T pixel) const{
            return true ? _members.find(pixel) != _members.end() : false; 
        }
        void add_member(T mem){
            _members.insert(mem);
        }
        std::set<T>& get_members(){
            return _members;
        }

        const std::set<T>& get_members() const {
            return _members;
        }
        void set_max_weight(double w){
            _max_weight = w;
        }
        int size() const{
            return _members.size();
        }
        // return the average value of density
        // which is going to be used as the color
        // of the component in segmented picture.
        /// @todo return an color rather than density
        /*Color compute_average_color();*/
    public:
        static double K;          // the larger K, the larger component
    private:
        //! pixels in the component
        std::set<T> _members;
        //! the latest pixel edge triggering merge
        double _max_weight;

};
template < typename T >
double Component<T>::K = SCALE;

template < typename T >
class WeightEdge {
    public:
        WeightEdge (double weight, T b, T e):_weight(weight), _b(b), _e(e){ };
        bool operator < (const WeightEdge &r) const{
            if(_weight < r._weight)
                return true;
            else if(_weight > r._weight)
                return false;
            else if(_b < r._b)
                return true;
            else if(_b > r._b)
                return false;
            else
                return _e < r._e;
        }
    public:
        double _weight;
        T _b;
        T _e;
        int m;
};

/*template < typename T >*/
/*typename Component<T>::Color Component<T>::compute_average_color(){*/
    /*double sum = 0;*/
    /*for(T p : _members){*/
    /*// so T must be a  pointer!*/
    /*sum = sum + p->get_density();*/
    /*}*/
    /*double average_density = sum/_members.size();*/
    /*_component_color.v = average_density;*/
    /*return **_members.begin();*/
/*}*/
#endif /* end of include guard: COMPONENT_H */
