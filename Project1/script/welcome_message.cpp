#include <gazebo/gazebo.hh>

namespace gazebo
{
    class HelloWorldPlugin : public WorldPlugin
    {
        public: HelloWorldPlugin() : WorldPlugin()
        {
            printf("Welcome to Alex’s World!\n");
        }

        public: void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
        {
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(HelloWorldPlugin)
}