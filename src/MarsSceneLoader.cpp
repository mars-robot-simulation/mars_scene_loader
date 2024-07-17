/* Conventions:
 *   - the includes should be defined in the header file
 *   - atomic variables shoud use snake_case
 *   - instances of classes should use camel_case
 *   - method names are camel_case
 *   - always use braces
 *   - braces start in new line
 *   - indent with four tabs
 */

#include "MarsSceneLoader.hpp"

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>
#include <mars_ode_collision/objects/Object.hpp>
#include <mars_ode_collision/objects/Heightfield.hpp>
#include <mars_ode_collision/objects/Mesh.hpp>
#include <mars_interfaces/terrainStruct.h>

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/MARSDefs.h>

#include <envire_core/graph/GraphDrawing.hpp>

#include <envire_core/graph/GraphDrawing.hpp>

#include <envire_smurf_loader/Model.hpp>

#include <base-logging/Logging.hpp>
#include <envire_types/registration/TypeCreatorFactory.hpp>



typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

namespace mars
{
    namespace mars_scene_loader
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;
        using namespace envire::smurf_loader;

        MarsSceneLoader::MarsSceneLoader(lib_manager::LibManager *theManager) :
            interfaces::LoadSceneInterface{theManager}
        {
            cfg = libManager->getLibraryAs<cfg_manager::CFGManagerInterface>("cfg_manager");
            collisionSpaceLoader = libManager->getLibraryAs<ode_collision::CollisionSpaceLoader>("mars_ode_collision");
            if(!collisionSpaceLoader)
            {
                LOG_ERROR("MarsSceneLoader: cannot find CollisionSpaceLoader!");
                return;
            }
            graphics = libManager->getLibraryAs<GraphicsManagerInterface>("mars_graphics");
            sim = libManager->getLibraryAs<SimulatorInterface>("mars_core");
            if(!sim)
            {
                LOG_ERROR("MarsSceneLoader: cannot find SimulatorInterface!");
                return;
            }

            // add this plugin as loader to LoadCenter
            // so we can access the loader over LoadCenter according to the supported file extension
            auto* const control = sim->getControlCenter();
            if (!control)
            {
                LOG_ERROR("MarsSceneLoader: Control center is not set");
                return;
            }
            ControlCenter::loadCenter->loadScene[".smurfs"] = this; // smurf scene
            std::cout << "MarsSceneLoader added as SMURF loader to LoadCenter" << std::endl;

            // search for root collision space that we expect as item under the root frame
            globalCollisionSpace = nullptr;
            try
            {
                auto it = ControlCenter::envireGraph->getItem<envire::core::Item<CollisionInterfaceItem>>(SIM_CENTER_FRAME_NAME);
                globalCollisionSpace = it->getData().collisionInterface.get();
            }
            catch (...)
            {
                globalCollisionSpace = nullptr;
            }
            if(!globalCollisionSpace)
            {
                LOG_ERROR("MarsSceneLoader: cannot find collisionInterfaceItem at root frame in envire graph!");
            }
            showViz = true;
            if(cfg)
            {
                showViz = cfg->getOrCreateProperty("Simulator", "visual rep.", (int)1, this).iValue & 1;
            }
        }

        MarsSceneLoader::~MarsSceneLoader()
        {
            if(collisionSpaceLoader)
            {
                libManager->releaseLibrary("mars_ode_collision");
            }
            if(graphics)
            {
                libManager->releaseLibrary("mars_graphics");
            }
            if(sim)
            {
                libManager->releaseLibrary("mars_core");
            }
            if(cfg)
            {
                libManager->releaseLibrary("cfg_manager");
            }
        }

        bool MarsSceneLoader::loadFile(std::string fileName, std::string tmpPath, std::string robotname)
        {
            std::cout << "MarsSceneLoader::loadFile(std::string fileName, std::string tmpPath, std::string robotname)" << std::endl;
            std::cout << "fileName: " << fileName << std::endl;
            std::cout << "tmpPath: " << tmpPath << std::endl;
            std::cout << "robotname: " << robotname << std::endl;

            const auto fileExtension = utils::getFilenameSuffix(fileName);
            const auto path = utils::getPathOfFile(fileName);
            utils::removeFilenamePrefix(&fileName);
            const auto absoluteFilePath = path + fileName;

            std::cout << "absoluteFilePath: " << absoluteFilePath << std::endl;

            // currently we handle only yml
            if(fileExtension != ".smurfs")
            {
                const auto errmsg = std::string{"The MarsSceneLoader does not support the file format: "} + fileExtension;
                LOG_ERROR("%s", errmsg.c_str());
                return false;
            }

            auto smurfsMap = configmaps::ConfigMap::fromYamlFile(absoluteFilePath, true);
            std::cout << "smurfsMap: " << smurfsMap.toJsonString() << std::endl;

            // parse the smurfs entries and load them
            configmaps::ConfigVector::iterator it;
            for (it = smurfsMap["entities"].begin(); it != smurfsMap["entities"].end(); ++it)
            {
                auto entityFile = (*it)["file"].toString();
                auto entityFileExtension = utils::getFilenameSuffix(entityFile);
                auto entityFileName = entityFile;
                utils::removeFilenamePrefix(&entityFileName);
                // handle absolute paths
                std::string absoluteEntityFilePath;
                if(entityFile.substr(0,1) == "/")
                {
                    absoluteEntityFilePath = utils::getPathOfFile(entityFile) + entityFileName;
                }
                else
                {
                    absoluteEntityFilePath = path + utils::getPathOfFile(entityFile) + entityFileName;
                }
                std::cout << "--- Load: " << std::endl;
                std::cout << "absoluteEntityFilePath: " << absoluteEntityFilePath << std::endl;
                std::string robotname;
                if(it->hasKey("name"))
                {
                    robotname = (*it)["name"].toString();
                }

                auto pos = utils::Vector{utils::Vector::Zero()};
                if(it->hasKey("position"))
                {
                    pos.x() = (*it)["position"][0];
                    pos.y() = (*it)["position"][1];
                    pos.z() = (*it)["position"][2];
                }
                auto rot = utils::Quaternion{utils::Quaternion::Identity()};
                if(it->hasKey("rotation"))
                {
                    switch ((*it)["rotation"].size())
                    {
                    case 1:
                    {
                        utils::Vector tmpV;
                        tmpV[0] = 0;
                        tmpV[1] = 0;
                        tmpV[2] = (*it)["rotation"][0];
                        rot = utils::eulerToQuaternion(tmpV);
                        break;
                    }
                    case 3:
                    {
                        utils::Vector tmpV;
                        tmpV[0] = (*it)["rotation"][0];
                        tmpV[1] = (*it)["rotation"][1];
                        tmpV[2] = (*it)["rotation"][2];
                        rot = utils::eulerToQuaternion(tmpV);
                        break;
                    }
                    case 4:
                    {
                        rot.x() = static_cast<sReal>((*it)["rotation"][1]);
                        rot.y() = static_cast<sReal>((*it)["rotation"][2]);
                        rot.z() = static_cast<sReal>((*it)["rotation"][3]);
                        rot.w() = static_cast<sReal>((*it)["rotation"][0]);
                        break;
                    }
                    }
                }

                std::cout << "robotname: " << robotname << std::endl;
                std::cout << "pos: " << pos << std::endl;
                std::cout << "rot: " << rot.w() << " " << rot.vec() << std::endl;

                if(entityFileExtension == ".smurfs")
                {
                    std::cout << "load SMURFs" << std::endl;
                    loadSmurfsScene(absoluteEntityFilePath);
                }
                else if(entityFileExtension == ".smurf")
                {
                    std::cout << "load SMURF" << std::endl;
                    loadSmurfScene(absoluteEntityFilePath, robotname, pos, rot);
                }
                else if(entityFileExtension == ".yml")
                {
                    loadYamlMarsScene(utils::getPathOfFile(absoluteEntityFilePath), entityFileName, robotname, pos, rot);
                }
                else
                {
                    const std::string errmsg = "The MarsSceneLoader does not support the file format: " + entityFileExtension;
                    LOG_ERROR("%s", errmsg.c_str());
                    return false;
                }
            }

            std::string out_dot_file = "test.dot";
            envire::core::GraphDrawing::write(*(ControlCenter::envireGraph), out_dot_file);

            return true;
        }

        bool MarsSceneLoader::loadFile(std::string filename, std::string tmpPath,
                                std::string robotname, utils::Vector pos, utils::Vector rot)
        {
            std::cout << "MarsSceneLoader::loadFile(std::string filename, std::string tmpPath, std::string robotname, utils::Vector pos, utils::Vector rot))" << std::endl;
            return false;
        }

        int MarsSceneLoader::saveFile(std::string filename, std::string tmpPath)
        {
            const std::string errmsg = "__PRETTY_FUNCTION__ not implemented";
            LOG_ERROR("%s", errmsg.c_str());
            return -1;
        }

        void MarsSceneLoader::loadSmurfsScene(const std::string &filePath)
        {
            // first we load a mars terrain
            /*char *envText = getenv("AUTOPROJ_CURRENT_ROOT");
            std::string path = envText;
            path += "/models/environments/mars";
            // first load smurfs file
            std::string file = path + "/terrain.smurfs";*/
            auto scene = ConfigMap::fromYamlFile(filePath);
            auto path = utils::getPathOfFile(filePath);

            // load configuration
            loadConfiguration(path, scene);

            // load plugins
            if(scene.hasKey("plugins"))
            {
                for(auto nt: scene["plugins"])
                {
                    const auto libName = nt.toString();
                    const auto libInfo = libManager->getLibraryInfo(libName);
                    if(libInfo.name != libName)
                    {
                        libManager->loadLibrary(libName, nullptr, false);
                    }
                }
            }

            // load entities
            for(auto &filename: scene["smurfs"])
            {
                // currently we handle only yml
                if(getFilenameSuffix(filename["file"]) == ".yml")
                {
                    const auto frameName = filename.hasKey("name") ? filename["name"].toString() : filename["file"].toString();
                    loadYamlMarsScene(path, filename["file"], frameName, Vector(0.0, 0.0, 0.0), Quaternion(1.0, 0.0, 0.0, 0.0));
                }
            }
        }

        void MarsSceneLoader::loadSmurfScene(const std::string &filePath, std::string robotname,
                                            utils::Vector pos, utils::Quaternion rot)
        {
            if(robotname.empty())
            {
                const std::string errmsg = "Can not load the smurf scene  " + filePath + ", since the robot name is not given";
                LOG_ERROR("%s", errmsg.c_str());
            }
            const auto& prefix = robotname;

            // TODO: use envire_base_loader
            // add envire_smurf_loader
            // then a coyote
            {
                const auto parentFrameId = envire::core::FrameId{SIM_CENTER_FRAME_NAME};
                std::cout << "LOAD OVER ENVIRE BASE LOADER" << std::endl;
                // ########################################## test loading of graph
                // load example robot
                envire::smurf_loader::Model model;
                model.loadFromSmurf(ControlCenter::envireGraph, parentFrameId, filePath, pos, rot, prefix);
            }

            //std::string out_dot_file = "test_envire_base_loader.dot";
            //envire::core::GraphDrawing::write(*(ControlCenter::envireGraph.get()), out_dot_file);
        }

        void MarsSceneLoader::loadYamlMarsScene(const std::string &path, const std::string &file, const std::string &robotname,
                                                utils::Vector pos, utils::Quaternion rot)
        {
            const auto filepath = pathJoin(path, file);
            auto model = ConfigMap::fromYamlFile(filepath);

            const auto parentFrameId = envire::core::FrameId{SIM_CENTER_FRAME_NAME};

            std::string prefix = file;

            envire::core::FrameId worldFrame = "World::" + robotname;
            ControlCenter::envireGraph->addFrame(worldFrame);
            envire::core::Transform worldPose(pos, rot);
            ControlCenter::envireGraph->addTransform(parentFrameId, worldFrame, worldPose);

            configmaps::ConfigMap worldMap;
            worldMap["name"] = worldFrame;
            worldMap["prefix"] = prefix;
            std::string className(base_types_namespace + std::string("World"));
            envire::core::ItemBase::Ptr item = envire::types::TypeCreatorFactory::createItem(className, worldMap);
            ControlCenter::envireGraph->addItemToFrame(worldFrame, item);

            for(auto& node: model["nodelist"])
            {
                auto config = static_cast<ConfigMap>(node);
                config["filePrefix"] = path;

                if(robotname != "")
                {
                    config["name"] = robotname + "." + config["name"].getString();
                }

                for(auto& it: model["materiallist"])
                {
                    if(config["material_id"] == it["id"])
                    {
                        it["loadPath"] = path;
                        config["material"] = it;
                        break;
                    }
                }

                interfaces::NodeData nodeData;
                nodeData.fromConfigMap(&config, "");
                nodeData.pos += pos;
                nodeData.rot *= rot;
                envire::core::Transform framePose(nodeData.pos, nodeData.rot);
          
                std::string objectType = "";
              
                if (config["physicmode"].toString() == "plane")
                {
                    objectType = "Plane";
                    config["size"]["x"] = config["extend"]["x"];
                    config["size"]["y"] = config["extend"]["y"];
                }
                else if (config["physicmode"].toString() == "box")
                {
                    objectType = "Box";
                    config["size"]["x"] = config["extend"]["x"];
                    config["size"]["y"] = config["extend"]["y"];
                    config["size"]["z"] = config["extend"]["z"];
                }
                else if (config["physicmode"].toString() == "cylinder")
                {
                    objectType = "Cylinder";
                }
                else if (config["physicmode"].toString() == "sphere")
                {
                    objectType = "Sphere";
                }
                else if (config["physicmode"].toString() == "capsule")
                {
                    objectType = "Capsule";
                }
                else if (config["physicmode"].toString() == "mesh")
                {
                    objectType = "Mesh";
                }
                else if (config["physicmode"].toString() == "heightfield")
                {
                    objectType = "Heightfield";
                }
                
                if (objectType == ""){
                    LOG_ERROR_S << "Can not add object " << objectType
                                << ", because the object type " << objectType << " is not known.";
                    return;              
                }

                // create and add into the graph envire item with the object corresponding to config type
                std::string visualClassName(geometry_namespace + objectType);
                envire::core::ItemBase::Ptr visualItem = envire::types::TypeCreatorFactory::createItem(visualClassName, config);
                if (!visualItem) {
                    LOG_ERROR_S << "Can not add visual " << objectType
                                << ", probably the visual type " << objectType << " is not registered.";
                    return;
                }
                visualItem->setTag("visual");

                envire::core::FrameId visualFrame = config["name"].getString() + "_" + "visual";
                ControlCenter::envireGraph->addFrame(visualFrame);
                ControlCenter::envireGraph->addTransform(worldFrame, visualFrame, framePose);
                ControlCenter::envireGraph->addItemToFrame(visualFrame, visualItem);

                std::cout << "Added visual!" << std::endl;

                // TODO: load the node as base type into the graph, add HeightMap type into base type
                if(nodeData.noPhysical == false)
                {
                    // create and add into the graph envire item with the object corresponding to config type
                    std::string collisionClassName(geometry_namespace + objectType);
                    envire::core::ItemBase::Ptr colisionItem = envire::types::TypeCreatorFactory::createItem(collisionClassName, config);
                    if (!colisionItem) {
                        LOG_ERROR_S << "Can not add collision " << objectType
                                    << ", probably the collision type " << objectType << " is not registered.";
                        return;
                    }
                    colisionItem->setTag("collision");
                    
                    envire::core::FrameId collisionFrame = config["name"].getString() + "_" + "collision";
                    ControlCenter::envireGraph->addFrame(collisionFrame);
                    ControlCenter::envireGraph->addTransform(worldFrame, collisionFrame, framePose);
                    ControlCenter::envireGraph->addItemToFrame(collisionFrame, colisionItem);
                    std::cout << "Added collision!" << std::endl;
                }
            }
        }

        void MarsSceneLoader::loadConfiguration(const std::string &path, ConfigMap &scene)
        {
            if(scene.hasKey("environment"))
            {
                auto envmap = static_cast<configmaps::ConfigMap>(scene["environment"]);
                if(cfg)
                {
                    if (envmap.hasKey("skybox"))
                    {
                        cfg->createParam("Scene","skydome_path", cfg_manager::stringParam);
                        cfg->createParam("Scene","skydome_enabled", cfg_manager::boolParam);
                        // check if path is relative to smurfs scene
                        auto skyboxPath = envmap["skybox"]["path"].toString();
                        skyboxPath = pathJoin(path, skyboxPath);
                        if(!pathExists(skyboxPath))
                        {
                            skyboxPath << envmap["skybox"]["path"];
                        }
                        cfg->setPropertyValue("Scene", "skydome_path", "value", skyboxPath);
                        cfg->setPropertyValue("Scene", "skydome_enabled", "value", true);
                    }
                    if(envmap.hasKey("animations"))
                    {
                        if(envmap["animations"].hasKey("config_path"))
                        {
                            cfg->createParam("Scene","animations_path", cfg_manager::stringParam);
                            cfg->setPropertyValue("Scene", "animations_path", "value", std::string(envmap["animations"]["config_path"]));
                        }
                    }
                    // TODO: I think the terrain propertiens are deprecated
                    //       -> terrain_path is used by particle system to search for configuration file
                    //       -> that will become deprecated with the new particle_system implementation
                    if(envmap.hasKey("terrain"))
                    {
                        cfg->createParam("Scene","terrain_path", cfg_manager::stringParam);
                        cfg->setPropertyValue("Scene", "terrain_path", "value", std::string(envmap["terrain"]["path"]));
                        if(envmap["terrain"].hasKey("mesh"))
                        {
                            cfg->createParam("Scene","mesh", cfg_manager::stringParam);
                            cfg->setPropertyValue("Scene", "mesh", "value", std::string(envmap["terrain"]["mesh"]));
                        }
                        if(envmap["terrain"].hasKey("mesh_material"))
                        {
                            cfg->createParam("Scene","mesh_material", cfg_manager::stringParam);
                            cfg->setPropertyValue("Scene", "mesh_material", "value", std::string(envmap["terrain"]["mesh_material"]));
                        }
                        if(envmap["terrain"].hasKey("mesh_scale"))
                        {
                            cfg->createParam("Scene","mesh_scale", cfg_manager::doubleParam);
                            cfg->setPropertyValue("Scene", "mesh_scale", "value", (double)(envmap["terrain"]["mesh_scale"]));
                        }
                    }
                }
                if(graphics)
                {
                    auto goptions = graphics->getGraphicOptions();
                    if(envmap.hasKey("background"))
                    {
                        Color bgcol;
                        bgcol.fromConfigItem(envmap["background"]);
                        goptions.clearColor = bgcol;
                    }
                    if(envmap.hasKey("fog"))
                    {
                        goptions.fogEnabled = true;
                        goptions.fogDensity = envmap["fog"]["density"];
                        goptions.fogStart = envmap["fog"]["start"];
                        goptions.fogEnd = envmap["fog"]["end"];
                        Color fogcol;
                        fogcol.fromConfigItem(envmap["fog"]["color"]);
                        goptions.fogColor = fogcol;
                    }
                    else
                    {
                        goptions.fogEnabled = false;
                    }
                    graphics->setGraphicOptions(goptions);
                }
            }
            if(scene.hasKey("physics"))
            {
                auto physicsmap = static_cast<configmaps::ConfigMap>(scene["physics"]);
                if(physicsmap.hasKey("gravity"))
                {
                    utils::Vector gravvec;
                    //configmaps::ConfigMap gravmap = physicsmap["gravity"];
                    //gravvec.x() = gravmap["x"];
                    gravvec.x() = physicsmap["gravity"]["x"];
                    gravvec.y() = physicsmap["gravity"]["y"];
                    gravvec.z() = physicsmap["gravity"]["z"];
                    sim->setGravity(gravvec);
                }
                if(cfg)
                {
                    if(physicsmap.hasKey("ode"))
                    {
                        if(physicsmap["ode"].hasKey("cfm"))
                        {
                            cfg->setPropertyValue("Simulator", "world cfm", "value", (sReal)(physicsmap["ode"]["cfm"]));
                        }
                        if(physicsmap["ode"].hasKey("erp"))
                        {
                            cfg->setPropertyValue("Simulator", "world erp", "value", (sReal)(physicsmap["ode"]["erp"]));
                        }
                        if(physicsmap["ode"].hasKey("stepsize"))
                        {
                            cfg->setPropertyValue("Simulator", "calc_ms", "value", (sReal)(physicsmap["ode"]["stepsize"]));
                        }
                    }
                }
            }
        }

        void MarsSceneLoader::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property)
        {
            showViz = _property.iValue & 1;
            if(graphics)
            {
                for(const auto& it: drawIDs)
                {
                    graphics->setDrawObjectShow(it, showViz);
                }
            }
        }

    } // end of namespace mars_scene_loader

} // end of namespace mars

DESTROY_LIB(mars::mars_scene_loader::MarsSceneLoader);
CREATE_LIB(mars::mars_scene_loader::MarsSceneLoader);
