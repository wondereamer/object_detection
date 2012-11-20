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
namespace m_util {
    using std::string;

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



}; /* m_lib */
#endif /* end of include guard: M_UTIL */