/**
 * @file m_util.h
 * @brief some useful functions
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */

#ifndef M_UTIL_H 

#define M_UTIL_H 

#include <cmath>
#include <vector>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>
#include <string>
#include <sstream>
#include <boost/regex.hpp>
#include <set>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

//#include <boost/foreach.hpp>
//namespace boost
//{
//
//    // Suggested work-around for https://svn.boost.org/trac/boost/ticket/6131
//    namespace BOOST_FOREACH =  foreach;
//    //    namespace BOOST_REVERSE_FOREACH = reverse_each;
//};
#define foreach   BOOST_FOREACH
#ifdef SPECIAL_LEXICAL_CAST
namespace boost {
    template<>
        inline int lexical_cast(const std::string& arg)
        {
            char* stop;
            int res = strtol( arg.c_str(), &stop, 10 );
            if ( *stop != 0 ) throw_exception(bad_lexical_cast(typeid(int), typeid(std::string)));
            return res;
        }
    template<>
        inline std::string lexical_cast(const int& arg)
        {
            char buffer[20]; 
            //            itoa( arg, buffer, 10 );
            sprintf(buffer, "%i", arg);
            return std::string( buffer );
        }
}
#endif

namespace m_util {
    using std::string;
    inline void msg_assert(bool flag, const std::string &msg){
        if(!flag){
            std::cerr<<msg <<std::endl;
            assert(flag);
        }
    }

    /*template < typename T >*/
    /*xmlrpc_c::paramList param_list(const std::vector<T> &y){*/
    /*xmlrpc_c::paramList params;*/
    /*std::vector<xmlrpc_c::value> x_array;*/
    /*for(const T &val : y){*/
    /*x_array.push_back(xmlrpc_c::value_double(val));*/
    /*}*/
    /*params.add(xmlrpc_c::value_array(x_array));*/
    /*return params;*/
    /*}*/

    //! call remote functions through xmlrpc
    xmlrpc_c::value call_py( const string &method_name, xmlrpc_c::paramList params,
            int port = 8000, const string &server_url = "http://localhost" );
    //! a c++ sprintf 
    std::string string_format(const std::string &fmt, ...);
    //! an simple function convert other types to string, not very efficient, but good enough 
    template < typename T >
        inline string sth2string(T i){
            std::stringstream ss;
            ss<<i;
            return ss.str();
        }
    //! the inverse of #sth2string
    template < typename T >
        inline T string2sth(const std::string& str){
            T val;
            std::stringstream ss(str);
            ss>>val;
            return val;
        }


    inline std::vector<std::string> split(std::string s, const string &symbol){ 
        std::vector<std::string> rst;
        boost::regex e(symbol);
        boost::regex_split(std::back_inserter(rst), s, e);
        return rst;
    }
    template<typename T>
        struct remove_pointer
        {

            typedef T type;
        };
    // higher priority
    template<typename T>
        struct remove_pointer<T*>
        {

            typedef typename remove_pointer<T>::type type;
        };

    //    object rst = foo(aa);
    //    stl_input_iterator<int> begin(rst);
    //    stl_input_iterator<int> end;
    //    std::copy(begin, end, back_inserter(o));
    //    std::cout<<extract<int>(rst[0][0])<<std::endl;

    class RpcServer {
        public:
            RpcServer (int port = DATA_SERVER_PORT){
                _port = port;
            }

            void run(){
                xmlrpc_c::serverAbyss myAbyssServer(
                        _myRegistry,
                        _port,              // TCP port on which to listen
                        "./xmlrpc_log"  // Log file
                        );
                std::cout<<"listening on port: "<<_port<<std::endl;
                myAbyssServer.run();
                // xmlrpc_c::serverAbyss.run() never returns
            }

            void register_method(const xmlrpc_c::methodPtr func, std::string funcName){
                /*xmlrpc_c::methodPtr const sampleAddMethodP(new sampleAddMethod);*/
                _myRegistry.addMethod(funcName, func);
            }
        private:
            xmlrpc_c::registry _myRegistry;
            int _port;
    };

    //! output every #every loop
    class EveryDisplay{
        public:
            EveryDisplay(int every):_every(every), _count(0), _display(0){ }
            void every_display (){
                _display++;
                _count++;
                if(_display >= _every){
                    // output the number of edges dealed with every 1000
                    std::cout<<_count<<std::endl;
                    _display = 0;
                }
            }
            void reset(){
                _count = 0;
                _display = 0;
            }
        private:
            double _count;
            double _display;
            double _every;
    };
    //
    void files_in_path(const std::string &fname);

    template< class MapType >
        void print_map(const MapType & m)
        {
            typedef typename MapType::const_iterator const_iterator;
            for( const_iterator iter =  m.begin(), iend =  m.end(); iter != iend; ++iter )
            {
                std::cout << iter->first << "-->" << iter->second << std::endl;
            }
        }

    template< class T >
        void print_list(const  T& m)
        {
            typedef typename T::const_iterator const_iterator;
            for( const_iterator iter =  m.begin(), iend =  m.end(); iter != iend; ++iter )
            {
                std::cout << *iter<<std::endl;
            }
        }
    template < typename T >
    void rank(const std::vector<T> &input, std::map<T, int> *rst){
        std::set<T> temp;
        for(auto &v : input){
            temp.insert(v); 
        }
        int rank = 0;
        // T -> int (rank)
        for(auto &v : temp){
            rst->insert(make_pair(v, rank++));
        }

    }
    
    bool is_nun(double a);
    double create_nun();

class PyModule {
public:
    typedef boost::python::list list;
    typedef boost::python::object object;
    PyModule (){
        Py_Initialize();
        _mainModule =  boost::python::import("__main__");
        _mainNamespace = _mainModule.attr("__dict__");
    };
    object load_file(const std::string &fname){
        return  boost::python::exec_file(fname.c_str(), _mainNamespace, _mainNamespace);
    }
    object funObj(const std::string &fname){
        return _mainNamespace[fname.c_str()];
    }
    template < typename T >
    inline void vec2list(const std::vector<T> &vec, list *t){
        for(auto &v : vec)
            t->append(v);
    }
    template < typename T >
    void vecs2lists(const std::vector<std::vector<T>> &vecs,
                                                  list *t){
        for(auto &vec : vecs){
            list row;
            vec2list(vec, &row);
            t->append(row);
        }
    }


    public:
    object _mainModule;
    object _mainNamespace;
    
};

}; /* m_lib */
#endif /* end of include guard: M_UTIL */
