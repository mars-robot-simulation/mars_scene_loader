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

typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

namespace mars
{
    namespace mars_scene_loader
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        MarsSceneLoader::MarsSceneLoader(lib_manager::LibManager *theManager) :
            interfaces::LoadSceneInterface(theManager)
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
            interfaces::ControlCenter *control = sim->getControlCenter();
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
                envire::core::EnvireGraph::ItemIterator<envire::core::Item<CollisionInterfaceItem>> it = ControlCenter::envireGraph->getItem<envire::core::Item<CollisionInterfaceItem>>(SIM_CENTER_FRAME_NAME);
                globalCollisionSpace = it->getData().collisionInterface;
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
                showViz = cfg->getOrCreateProperty("Simulator", "visual rep.",
                                                   (int)1, this).iValue & 1;;
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

            std::string fileExtension = utils::getFilenameSuffix(fileName);
            std::string path = utils::getPathOfFile(fileName);
            utils::removeFilenamePrefix(&fileName);
            std::string absoluteFilePath(path + fileName);

            std::cout << "absoluteFilePath: " << absoluteFilePath << std::endl;

            // currently we handle only yml
            if(fileExtension != ".smurfs")
            {
                const std::string errmsg = "The MarsSceneLoader does not support the file format: " + fileExtension;
                LOG_ERROR("%s", errmsg.c_str());
                return false;
            }

            configmaps::ConfigMap smurfsMap;
            smurfsMap = configmaps::ConfigMap::fromYamlFile(absoluteFilePath, true);
            std::cout << "smurfsMap: " << smurfsMap.toJsonString() << std::endl;

            // parse the smurfs entries and load them
            configmaps::ConfigVector::iterator it;
            for (it = smurfsMap["entities"].begin(); it != smurfsMap["entities"].end(); ++it)
            {
                std::string entityFile = (*it)["file"].toString();
                std::string entityFileExtension = utils::getFilenameSuffix(entityFile);
                std::string entityFileName = entityFile;
                utils::removeFilenamePrefix(&entityFileName);
                // handle absolute paths
                std::string absoluteEntityFilePath;
                if(entityFile.substr(0,1) == "/")
                {
                    absoluteEntityFilePath = utils::getPathOfFile(entityFile) + entityFileName;
                } else
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

                utils::Vector pos(0,0,0);
                utils::Quaternion rot(1,0,0,0);
                if(it->hasKey("position"))
                {
                    pos.x() = (*it)["position"][0];
                    pos.y() = (*it)["position"][1];
                    pos.z() = (*it)["position"][2];
                }
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
                        rot.x() = (sReal)(*it)["rotation"][1];
                        rot.y() = (sReal)(*it)["rotation"][2];
                        rot.z() = (sReal)(*it)["rotation"][3];
                        rot.w() = (sReal)(*it)["rotation"][0];
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
                } else if(entityFileExtension == ".smurf")
                {
                    std::cout << "load SMURF" << std::endl;
                    loadSmurfScene(absoluteEntityFilePath, robotname, pos, rot);
                } else if(entityFileExtension == ".yml")
                {
                    loadYamlMarsScene(utils::getPathOfFile(absoluteEntityFilePath), entityFileName, robotname, pos, rot);
                } else
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
            ConfigMap scene = ConfigMap::fromYamlFile(filePath);
            std::string path = utils::getPathOfFile(filePath);

            // load configuration
            loadConfiguration(path, scene);

            // load plugins
            std::string libName = "";
            lib_manager::LibInfo libInfo;
            if(scene.hasKey("plugins"))
            {
                for(auto nt: scene["plugins"])
                {
                    libName << nt;
                    libInfo = libManager->getLibraryInfo(libName);
                    if(libInfo.name != libName)
                    {
                        libManager->loadLibrary(libName, NULL, false);
                    }
                }
            }

            // load entities
            for(auto &filename: scene["smurfs"])
            {
                // currently we handle only yml
                if(getFilenameSuffix(filename["file"]) == ".yml")
                {
                    loadYamlMarsScene(path, filename["file"], "", Vector(0.0, 0.0, 0.0), Quaternion(1.0, 0.0, 0.0, 0.0));
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
            std::string prefix = robotname;

            // TODO: use envire_base_loader
            // add envire_smurf_loader
            // then a coyote
            {
                envire::core::FrameId parentFrameId = SIM_CENTER_FRAME_NAME;
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
            std::string filepath = pathJoin(path, file);
            ConfigMap model = ConfigMap::fromYamlFile(filepath);
            for(auto& node: model["nodelist"])
            {
                // TODO: create envire_types and visual and collision items in graph instead of directly creating
                //       visual and collision objects
                ConfigMap config = node;
                if(robotname != "")
                {
                    config["name"] = robotname + "." + config["name"].getString();
                }
                interfaces::NodeData nodeData;
                config["filePrefix"] = path;
                nodeData.fromConfigMap(&config, "");
                nodeData.pos += pos;
                nodeData.rot *= rot;
                ConfigMap material;
                for(auto& it: model["materiallist"])
                {
                    if(config["material_id"] == it["id"])
                    {
                        material = it;
                        material["loadPath"] = path;
                        break;
                    }
                }
                if(nodeData.terrain && !nodeData.terrain->pixelData)
                {
                    LOG_ERROR("Load heightmap pixelData...");
                    //nodeData.terrain = new(terrainStruct);
                    // TODO: add proper path handling
                    nodeData.terrain->srcname = path + "/" + nodeData.terrain->srcname;
                    LOG_ERROR(nodeData.terrain->srcname.c_str());
                    ControlCenter::loadCenter->loadHeightmap->readPixelData(nodeData.terrain);
                    if(!nodeData.terrain->pixelData)
                    {
                        LOG_ERROR("NodeManager::addNode: could not load image for terrain");
                    }
                }
                nodeData.material.fromConfigMap(&material, "");
                unsigned long drawID = graphics->addDrawObject(nodeData, showViz);
                drawIDs.push_back(drawID);
                ode_collision::Object *collisionObject;
                // TODO: load the node as base type into the graph, add HeightMap type into base type
                if(nodeData.noPhysical == false)
                {
                    // check physics type:
                    if(nodeData.terrain)
                    {
                        config["type"] = "heightfield";
                        // TODO: this is converstion from old mars config to new
                        // sould be done in loader later
                        config["size"]["x"] = config["extend"]["x"];
                        config["size"]["y"] = config["extend"]["y"];
                        config["size"]["z"] = config["extend"]["z"];

                        ode_collision::Heightfield* collision = (ode_collision::Heightfield*)(globalCollisionSpace->createObject(config, NULL));
                        collisionObject = collision;
                        if(!collision)
                        {
                            LOG_ERROR("Error creating collision object!");
                            return;
                        }
                        collision->setTerrainStrcut(nodeData.terrain);
                        if(!collision->createGeom())
                        {
                            LOG_ERROR("Error creating heightfield geom!");
                            return;
                        }
                    }
                    else
                    {
                        // TODO: create collision item in graph instead of collision object directly
                        ConfigMap tmpMap;
                        nodeData.toConfigMap(&tmpMap);
                        tmpMap["type"] = tmpMap["physicmode"];
                        ode_collision::Object* collision = ControlCenter::collision->createObject(tmpMap);
                        collisionObject = collision;
                        if(!collision)
                        {
                            LOG_ERROR("Error creating mars_yaml collision object!");
                            return;
                        }
                        if(tmpMap["type"] == "mesh")
                        {
                            nodeData.filename = pathJoin(path, nodeData.filename);
                            ControlCenter::loadCenter->loadMesh->getPhysicsFromMesh(&nodeData);
                            ((ode_collision::Mesh*)collision)->setMeshData(nodeData.mesh);
                            collision->createGeom();
                        }
                    }
                    collisionObject->setPosition(nodeData.pos);
                    collisionObject->setRotation(nodeData.rot);
                    // the position update is applied in updateTransform which is not
                    // called automatically for static objects
                    collisionObject->updateTransform();
                }
            }
        }

        void MarsSceneLoader::loadConfiguration(const std::string &path, ConfigMap &scene)
        {
            if(scene.hasKey("environment"))
            {
                configmaps::ConfigMap envmap = scene["environment"];
                if(cfg)
                {
                    if (envmap.hasKey("skybox"))
                    {
                        cfg->createParam("Scene","skydome_path", cfg_manager::stringParam);
                        cfg->createParam("Scene","skydome_enabled", cfg_manager::boolParam);
                        // check if path is relative to smurfs scene
                        std::string skyboxPath = envmap["skybox"]["path"];
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
                    interfaces::GraphicData goptions = graphics->getGraphicOptions();
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
                    } else
                    {
                        goptions.fogEnabled = false;
                    }
                    graphics->setGraphicOptions(goptions);
                }
            }
            if(scene.hasKey("physics"))
            {
                configmaps::ConfigMap physicsmap = scene["physics"];
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
                for(auto &it: drawIDs)
                {
                    graphics->setDrawObjectShow(it, showViz);
                }
            }
        }

    } // end of namespace mars_scene_loader

} // end of namespace mars

DESTROY_LIB(mars::mars_scene_loader::MarsSceneLoader);
CREATE_LIB(mars::mars_scene_loader::MarsSceneLoader);
