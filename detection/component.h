#ifndef COMPONENT_H


#define COMPONENT_H
#include <type_traits>
#include <cassert>
#include <set>
#include <vector>
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
        Component( ): _max_weight(0), _next(NULL) { }
        Component( const Component &other ):_max_weight(other._max_weight)
                                           ,_next(other._next){
            //            assert(other.size() == 2);
            //            std::cout<<other.size()<<std::endl;
            //            _members.clear();
            //            add_member(other.get_members()[0]);
            //            add_member(other.get_members()[1]);
            for(auto p : other.get_members()){
                add_member(p); 
//                std::cout<<p<<std::endl;
            }
//            std::cout<<_members.size()<<std::endl;
//            assert(_members.size() == 2);

        }   
        Component(T first_pixel): _max_weight(0), _next(NULL) {
            // an single pixel componnet
//            add_member(first_pixel);
            _members.insert(first_pixel);
        }
        // compare operation for componets that only have one pixel
        inline bool operator < (const Component &r) const{
            assert(r.get_members().size() == 1 && _members.size() == 1);
            return *(r.get_members().begin()) < *_members.begin();
        }
        // @speed this function may not be inlined!
        /**
         * @brief merge small componet to bigger component  
         *
         * @return 0: merge t to s; 1: merge s to t; -1: can't merge 
         */
         inline static int merge(double component_diff,   Component &s, Component &t){
            if(_if_merge(component_diff, s, t)){
                // merge to bigger component
                if( t.size() > s.size()){
                    // merge s to t
                    for(T m : s.get_members()){
                        // move r' member to s
//                        std::cout<<m<<std::endl;
                        t.add_member(m);
                        MOVE_NUM++;
                        //                        s._next = &t;
                    }
                    s.clear();
                    t.set_max_weight(component_diff);
                    
                    return 1;
                }
                else{
                        for(T m : t.get_members()){
                            s.add_member(m);
                            MOVE_NUM++;
                        }
                        s.set_max_weight(component_diff);
                        t.clear();
                        return 0;
                }
            }
            return -1;
        }    

        inline void merge_from(const Component &r){
            for(T pixel: r.get_members()){
                add_member(pixel);
            }
        }
        //! check if two component close enough to merge
        inline static bool _if_merge(double comp_diff, const Component &a, const Component &b) {
            //            std::cout<<"comp_diff: "<<comp_diff<<" a.diff: "<<a.internal_diff()<<" b.diff: "<<b.internal_diff()<<std::endl;
            return  comp_diff <= std::min(a.internal_diff(), b.internal_diff());
        }
        //! the internal difference of the component
        inline double internal_diff() const{
            //            std::cout<<"weight: "<<_max_weight<< " size: "<<_members.size()<<std::endl;
            return _max_weight + K / _members.size();
        }
        inline bool contains(T pixel) const{
//            for (int i = 0; i < _members.size(); i++) 
//                if(_members[i] == pixel)
//                    return true;
//            return false;

            return true ? _members.find(pixel) != _members.end() : false; 
        }
        inline void add_member(T elem){
            elem->_parent = this;
            _members.insert(elem);
        }
        inline std::set<T>& get_members(){
            return _members;
        }

        inline const std::set<T>& get_members() const {
            return _members;
        }
        inline void set_max_weight(double w){
            _max_weight = w;
        }
        inline int size() const{
            return _members.size();
        }
        inline Component* right_one(){
            Component *rst = this;
            while(_next){
                rst = _next;
                _next = _next->_next;
            }
            return rst;
        }
        inline void clear(){
            _members.clear();
        }
        // return the average value of density
        // which is going to be used as the color
        // of the component in segmented picture.
        /// @todo return an color rather than density
        /*Color compute_average_color();*/
    public:
        static double K;          // the larger K, the larger component
        static double MOVE_NUM;
        Component *_next;
    private:
        //! pixels in the component
        std::set<T> _members;
        //! the latest pixel edge triggering merge
        double _max_weight;

};
template < typename T >
double Component<T>::K = SCALE;

template < typename T >
double Component<T>::MOVE_NUM = 0;

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
