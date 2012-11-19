#include "server.h"

void parse_args(int argc, char *argv[]);
// sceme file path
static const char* DEFAULT_MODULE_PATHS[] = 
{
    DATADIR,
    ".",
#ifndef WIN32
    "/usr/share/opencog",
    "/usr/local/share/opencog",
#endif // !WIN32
    NULL
};
int main(int argc, char *argv[])
{
    parse_args(argc, argv);
    CogServer& cogserve = cogserver();
    // Open database *before* loading modules, since the modules
    // might create atoms, and we can't have that happen until 
    // storage is open, as otherwise, there will be handle conflicts.
    cogserve.openDatabase(); 

    // Load modules specified in config
    cogserve.loadModules(); 
    cogserve.loadSCMModules(DEFAULT_MODULE_PATHS);

    RpcServer server;
    xmlrpc_c::methodPtr const method(new AtomSpaceMethod(cogserve));
    server.register_method(method, "get_block_attrs");
    server.run();
    // enable the network server and run the server's main loop
    //    cogserve.enableNetworkServer();
    //   cogserve.serverLoop();

    exit(0);
}
//-------------------------------------------------------------------------------------------------------------------------------------
static const char* DEFAULT_CONFIG_FILENAME = "opencog.conf";
static const char* DEFAULT_CONFIG_PATHS[] = 
{
    //CONFDIR,
    "/home/opencog/lib" ,
#ifndef WIN32
    "/etc",
#endif // !WIN32
    "../lib",
    NULL
};


static void usage(const char* progname)
{
    std::cerr << "Usage: " << progname << " [[-c <config-file>]..] [[-DOPTION=\"VALUE\"]..]\n\n";
    std::cerr << "Each config file is loaded sequentially, with the values in \n"
        << " later files overwriting earlier. Then each singular option overrides \n" 
        << " options in config files. " << std::endl;
}
void parse_args(int argc, char *argv[])
{
    // Get the locale from the environment... 
    // Perhaps we should someday get it from the config file ???
    setlocale(LC_ALL, "");

    // Check to make sure the current locale is UTF8; if its not,
    // then force-set this to the english utf8 locale 
    const char * codeset = nl_langinfo(CODESET);
    if (!strstr(codeset, "UTF") && !strstr(codeset, "utf"))
    {
        fprintf(stderr,
                "%s: Warning: locale %s was not UTF-8; force-setting to en_US.UTF-8\n",
                argv[0], codeset);
        setlocale(LC_CTYPE, "en_US.UTF-8");
    }

    static const char *optString = "c:D:h";
    int c = 0;
    vector<string> configFiles;
    vector< pair<string,string> > configPairs;
    string progname = argv[0];

    // parse command line
    while (1) {
        c = getopt (argc, argv, optString);
        /* Detect end of options */
        if (c == -1) {
            break;
        } else if (c == 'c') {
            configFiles.push_back(optarg);
        } else if (c == 'D') {
            // override all previous options, e.g.
            // -DLOG_TO_STDOUT=TRUE
            string text = optarg; 
            string value, optionName;
            vector<std::string> strs;
            boost::split(strs, text, boost::is_any_of("=:"));
            optionName = strs[0];
            if (strs.size() > 2) {
                // merge end tokens if more than one separator found
                for (uint i = 1; i < strs.size(); i++)
                    value += strs[i];
            } else if (strs.size() == 1) {
                std::cerr << "No value given for option " << strs[0] << endl;
            } else {
                value = strs[1];
            }
            configPairs.push_back( pair<string,string>(optionName, value) );
        } else {
            // unknown option (or help)
            usage(progname.c_str());
            if (c == 'h')
                exit(0);
            else
                exit(1);
        }

    }

    if (configFiles.size() == 0) {
        // search for configuration file on default locations
        for (int i = 0; DEFAULT_CONFIG_PATHS[i] != NULL; ++i) {
            boost::filesystem::path configPath(DEFAULT_CONFIG_PATHS[i]);
            configPath /= DEFAULT_CONFIG_FILENAME;
            if (boost::filesystem::exists(configPath)) {
                cerr << "Using default config at " << configPath.string() << endl;
                configFiles.push_back(configPath.string());
            }
        }
    }
    config().reset();
    if (configFiles.size() == 0) {
        cerr << "No config files could be found!" << endl;
        exit(-1);
    }
    // Each config file sequentially overwrites the next
    BOOST_FOREACH (string configFile, configFiles) {
        try {
            std::cout<<"Loaded configure file: "<<configFile<<std::endl;
            config().load(configFile.c_str(), false);
            break;
        } catch (RuntimeException &e) {
            std::cerr << e.getMessage() << std::endl;
            exit(1);
        }
    }
    // Each specific option
    pair<string,string> optionPair;
    BOOST_FOREACH (optionPair, configPairs) {
        //cerr << optionPair.first << " = " << optionPair.second << endl;
        config().set(optionPair.first, optionPair.second);
    }


}
