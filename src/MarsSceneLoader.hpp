#pragma once
#include <mars_utils/Vector.h>


#include <cfg_manager/CFGManagerInterface.h>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_ode_physics/WorldPhysicsLoader.hpp>
#include <mars_ode_collision/CollisionSpaceLoader.hpp>
#include <cfg_manager/CFGManagerInterface.h>

#include <mars_interfaces/sim/LoadSceneInterface.h>

#include <iostream>

namespace mars
{
    namespace mars_scene_loader
    {

        class MarsSceneLoader : public interfaces::LoadSceneInterface,
                                public cfg_manager::CFGClient
        {

        public:
            MarsSceneLoader(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~MarsSceneLoader();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string("mars_scene_loader");
            }

            bool loadFile(std::string filename, std::string tmpPath,
                                    std::string robotname) override;
            bool loadFile(std::string filename, std::string tmpPath,
                                    std::string robotname, utils::Vector pos, utils::Vector rot) override;
            int saveFile(std::string filename, std::string tmpPath) override;

            CREATE_MODULE_INFO();

            virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) override;

        private:
            ode_collision::CollisionSpaceLoader *collisionSpaceLoader;
            std::shared_ptr<interfaces::CollisionInterface> globalCollisionSpace;
            interfaces::GraphicsManagerInterface *graphics;
            interfaces::SimulatorInterface *sim;
            cfg_manager::CFGManagerInterface *cfg;
            std::vector<unsigned long> drawIDs;
            bool showViz;

            void loadYamlMarsScene(const std::string &path, const std::string &file, const std::string &robotname, utils::Vector pos, utils::Quaternion rot);
            void loadConfiguration(const std::string &path, configmaps::ConfigMap &scene);

            void loadSmurfsScene(const std::string &filePath);
            void loadSmurfScene(const std::string &filePath, std::string robotname, utils::Vector pos, utils::Quaternion rot);
        };

    } // end of namespace mars_scene_loader
} // end of namespace mars
