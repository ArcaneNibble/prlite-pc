#include "ros/ros.h"
#include "pr2lite_actuators/LinearActuator.hpp"

#include <boost/tokenizer.hpp>

#include <vector>
#include <map>

#include <sstream>
#include <cstdlib>
#include <cmath>
#include <dlfcn.h>

#define BASE_LENGTH 9.9f
#define SWING 4.0f
#define LINACT_TORSO_ID     0x0C 
#define LINACT_RIGHT_ARM_ID 0x0E 
#define LINACT_LEFT_ARM_ID  0x0F 
#define LINACT_WHEELS_ID    0xAA 

typedef std::string                                    (*lookupFunction)(void);
typedef pr2lite::actuators::LinearActuator*            (*actuatorAllocator)(ros::NodeHandle&, std::string);
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

int main (int argc, char** argv)
{
    // Initalize as a ROS node
    ros::init(argc, argv, "linact_server");
    ros::NodeHandle nh;
    
    // Plugin map
    std::map<std::string, actuatorAllocator> plugin_allocators; 
    std::vector<void *>                      plugins;
    std::vector<pr2lite::actuators::LinearActuator *>            actuators;
    
    // Fetch the plugins list to load
    std::string plugin_list;
    nh.getParam("/linact_server/plugins", plugin_list);
    
    // Break down the string and load in the plugins
    boost::char_separator<char> sep(";");
    tokenizer tokens(plugin_list, sep);
    for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
    {
        // Alert the log of a loading plugin
        ROS_INFO("Loading Plugin \"%s\"", tok_iter->c_str());
        
        // Variables
        void             *plugin    = NULL;
        actuatorAllocator allocator = NULL;
        lookupFunction    lookup    = NULL;
        
        // Attempt to load the library
        if(!(plugin = dlopen(tok_iter->c_str(), RTLD_LAZY)))
        {
            ROS_WARN("[Plugin %s] %s", tok_iter->c_str(), dlerror());
            continue;
        }
        dlerror();
        
        // Lookup the allocator and naming function
        *(void **)(&lookup) = dlsym(plugin, "pluginType");
        assert(lookup != NULL);
        *(void **)(&allocator) = dlsym(plugin, "createActuator");
        assert(allocator != NULL);
        
        // Check if we encountered an error in the lookup
        char *error =  NULL;
        if((error = dlerror()) != NULL)
        {
            ROS_WARN("[Plugin %s] %s", tok_iter->c_str(), dlerror());
            dlclose(plugin);
            continue;
        }
        
        // Lookup plugin name
        std::string pluginName = (*lookup)();
        ROS_INFO("[Plugin %s] fetched actuator = %s", tok_iter->c_str(), pluginName.c_str());
        
        // Store stuff
        plugin_allocators[pluginName] = allocator;
        plugins.push_back(plugin);
    }

    // Load actuators
    std::string actuator_list;
    nh.getParam("/linact_server/actuators", actuator_list);
    
    // Break down the string and load in the actuators
    tokenizer actuator_tokenizer(actuator_list, sep);
    for(tokenizer::iterator act_iter = actuator_tokenizer.begin(); act_iter != actuator_tokenizer.end(); ++act_iter)
    {
        // Variables
        std::string name = act_iter->c_str();
        std::string pluginType;
        
        // Get the type of actuator
        nh.getParam(name + "/type", pluginType);
        ROS_INFO("Starting Actuator \"%s\" (%s)", act_iter->c_str(), pluginType.c_str());
        actuatorAllocator allocator = plugin_allocators[pluginType];
        assert(allocator != NULL);
        
        // Create an instance of it
        pr2lite::actuators::LinearActuator *actuator = (*allocator)(nh, name);
        actuators.push_back(actuator);
    }

    // Execute forever
    ros::spin();
    
    // Unload plugins
    for(std::vector<void *>::iterator it = plugins.begin(); it != plugins.end(); ++it)
        dlclose(*it);

    return 0;
}

/*

int main (int argc, char** argv)
{
    // Check to see if a plugin library was provides
    if(argc < 2)
    {
        std::cerr << "No dynamic library provided" << std::endl;
        exit(1);
    }
    
    // Variables
    void* plugin = NULL;
    BasicService* (*registerService)(void);
    
    // Attempt to load the plugin library
    if(!(plugin = dlopen(argv[1], RTLD_LAZY)))
    {
        std::cerr << dlerror() << std::endl;
        exit(1);
    }
    dlerror();
    
    *(void **)(&registerService) = dlsym(plugin, "registerService");
    char *error = NULL;
    if((error = dlerror()) != NULL)
    {
        std::cerr << error << std::endl;
        exit(1);
    }

    BasicService *service = (*registerService)();
    
    std::cout << "Service\'s \"description()\" function returns: " << service->description() << std::endl;
    dlclose(plugin);
    
    return 0;
}
*/

