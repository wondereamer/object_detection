
#include <getopt.h>
#include <langinfo.h>
#include <locale.h>

#include <boost/filesystem/operations.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <utility>
#include <string>

#include <opencog/atomspace/atom_types.h>
#include <opencog/guile/load-file.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/misc.h>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>
#include <vector>
#include <cassert>
#include "m_opencv.h" 
#include "colormap.h"
#include <map>
using namespace m_opencv;
using namespace opencog;
using namespace std;
template < typename T >
T string2sth(const std::string& str){
    T val;
    std::stringstream ss(str);
    ss>>val;
    return val;
}
class AtomSpaceMethod : public xmlrpc_c::method {
    public:
        AtomSpaceMethod(CogServer &cogserve){
            _a = &cogserve.getAtomSpace();
            /*load_scm_file(*_a, "./data.scm");*/
            /*std::cout<<"Have loaded data!"<<std::endl;*/
            std::vector<Handle> handles;
            _a->getHandleSet(back_inserter(handles), STRUCTURE_NODE);

            std::vector<xmlrpc_c::value> position_array;
            std::vector<xmlrpc_c::value> size_array;
            std::vector<xmlrpc_c::value> color_array;
            std::vector<xmlrpc_c::value> material_array;
            std::map<std::string, std::string> mAndc;

            // for every block
            foreach(Handle h, handles){
                // initial block properties
                position_array.push_back(xmlrpc_c::value_string(""));
                size_array.push_back(xmlrpc_c::value_int(1));
                material_array.push_back(xmlrpc_c::value_string(""));
                color_array.push_back(xmlrpc_c::value_string(""));

                HandleSeq parents = _a->getIncoming(h);
                // find their properties
                foreach(Handle p, parents){
                    if(_a->getType(p) == LIST_LINK)
                    {
                        // LIST_LINK parent
                        HandleSeq grandParent = _a->getIncoming(p);
                        foreach(Handle g, grandParent){
                            // EVALUATION_LINK grandparent
                            if(_a->getType(g) == EVALUATION_LINK)
                            {
                                HandleSeq uncles = _a->getOutgoing(g);
                                foreach(Handle u, uncles){
                                    std::string name = _a->getName(u);
                                    if(name == "material")
                                    {
                                        HandleSeq proppertys = _a->getOutgoing(p);
                                        // in the order appeared in scheme file!
                                        material_array.pop_back();
                                        std::string material = xmlrpc_c::value_string(_a->getName(proppertys[1]));
                                        material_array.push_back(xmlrpc_c::value_string(material));
                                        // @bug colors, tempornal code until scheme file have color attribute
                                        color_array.pop_back();
                                        color_array.push_back(xmlrpc_c::value_string(material2color(material)));
                                        mAndc.insert(make_pair(material, material2color(material)));
                                    }else if(name == "AGISIM_position"){
                                        HandleSeq proppertys = _a->getOutgoing(p);
                                        std::string x = _a->getName(proppertys[1]);
                                        std::string y = _a->getName(proppertys[2]);
                                        std::string z = _a->getName(proppertys[3]);
                                        position_array.pop_back();
                                        position_array.push_back(xmlrpc_c::value_string(x + "," + y + "," + z ));
                                    }else if(name == "size"){
                                        HandleSeq proppertys = _a->getOutgoing(p);
                                        std::string size = _a->getName(proppertys[1]);
                                        size_array.pop_back();
                                        size_array.push_back(xmlrpc_c::value_int(string2sth<int>(size)));
                                    } //else if(name == "color"){
                                    /*HandleSeq proppertys = _a->getOutgoing(p);*/
                                    /*// in the order appeared in scheme file!*/
                                    /*color_array.pop_back();*/
                                    /*//color_array.push_back(xmlrpc_c::value_string(_a->getName(proppertys[1])));*/
                                    /*color_array.push_back(xmlrpc_c::value_string("255, 0, 0"));*/

                                    /*}*/
                                }
                            }
                        }
                    }
                }
            };
            assert(position_array.size() == color_array.size() && color_array.size() == size_array.size());
            assert( position_array.size() == material_array.size());
            // array to array
            std::vector<xmlrpc_c::value> ttt;

            ttt.push_back(xmlrpc_c::value_array(position_array));
            ttt.push_back(xmlrpc_c::value_array(size_array));
            ttt.push_back(xmlrpc_c::value_array(material_array));
            ttt.push_back(xmlrpc_c::value_array(color_array));
            _rst = xmlrpc_c::value_array(ttt);
            // blocks info
            std::cout<<"**********************************************"<<std::endl;
            std::cout<<"Size of AtomSpace: "<< _a->getSize()<<std::endl;
            std::cout<<"Size of BlockWorlds: "<< handles.size()<<std::endl;

            std::cout<<"materials and colors:"<<std::endl;
            for(auto v : mAndc){
                std::cout<<v.first<<": "<<v.second<<std::endl;
            }
            std::cout<<"**********************************************"<<std::endl;

        }
        void execute(xmlrpc_c::paramList const& paramList,
                xmlrpc_c::value *   const  retvalP) {
            *retvalP = _rst;
            std::cout<<"Having answered one request."<<std::endl;

        }
        AtomSpace *_a;
        xmlrpc_c::value _rst;
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
            std::cout<<"listening on port: 8000"<<std::endl;
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

