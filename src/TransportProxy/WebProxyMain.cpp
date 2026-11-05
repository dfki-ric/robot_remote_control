#include <unistd.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <experimental/filesystem>

#include <boost/program_options.hpp>

#include "GenericProxy.hpp"
#include "../RobotController/RobotController.hpp"
#include "../Transports/ZeroMQ/TransportZmq.hpp"
#include "../Transports/Http/TransportHttp.hpp"
#include "../Transports/WebSocket/TransportWebSocket.hpp"

using namespace robot_remote_control;
namespace po = boost::program_options;
namespace fs = std::experimental::filesystem;

/**
 * @brief folder name for a robot name, the web interface uses the same rule (modelFolderName() in model-loader.js):
 * characters other than A-Z a-z 0-9 _ . - are replaced by a single _
 */
std::string modelFolderName(const std::string &robotName) {
    std::string name;
    for (const char &c : robotName) {
        const unsigned char byte = static_cast<unsigned char>(c);
        if (std::isalnum(byte) || c == '_' || c == '.' || c == '-') {
            name += c;
        } else if ((byte & 0xC0) != 0x80) {
            // one _ for each (utf-8 encoded) character, continuation bytes are skipped
            name += '_';
        }
    }
    return (name.empty() || name == "." || name == "..") ? "robot" : name;
}

std::string jsonString(const std::string &text) {
    std::string json = "\"";
    for (const char &c : text) {
        if (c == '"' || c == '\\') {
            json += '\\';
            json += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            char escaped[8];
            snprintf(escaped, sizeof(escaped), "\\u%04x", c);
            json += escaped;
        } else {
            json += c;
        }
    }
    return json + "\"";
}

/**
 * @brief writes model.json into the model folder: the urdf and all files of the model, the web interface
 * needs it to map the mesh paths of the urdf to the files
 */
bool writeModelIndex(const std::string &folder, const std::string &path, const std::string &modelfile) {
    std::vector<std::string> files;
    for (const auto &entry : fs::recursive_directory_iterator(folder)) {
        const std::string relative = entry.path().string().substr(folder.size() + 1);
        if (fs::is_regular_file(entry.path()) && relative != "model.json") {
            files.push_back(relative);
        }
    }
    std::sort(files.begin(), files.end());
    std::ofstream index(folder + "/model.json");
    index << "{\n    \"path\": " << jsonString(path) << ",\n    \"modelfilename\": " << jsonString(modelfile) << ",\n    \"files\": [";
    for (size_t i = 0; i < files.size(); ++i) {
        index << (i ? "," : "") << "\n        " << jsonString(files[i]);
    }
    index << "\n    ]\n}\n";
    return index.good();
}

/**
 * @brief downloads the model of the robot into <modelsFolder>/<robot name>, if that folder does not exist (or force is set),
 * the web interface loads it from there instead of transferring it from the robot each time
 */
void cacheRobotModel(const std::string &commandAddress, const std::string &modelsFolder, bool force) {
    try {
        RobotController controller(std::make_shared<TransportZmq>(commandAddress, TransportZmq::REQ));
        // waiting for the robot is reported below
        controller.setupDisconnectedCallback([](const float &) {});

        RobotName name;
        bool waiting = false;
        while (true) {
            controller.requestRobotName(&name);
            // also connected if the robot has no name (empty reply)
            if (controller.isConnected()) {
                break;
            }
            if (!waiting) {
                printf("robot model: waiting for the robot\n");
                waiting = true;
            }
            sleep(2);
        }

        const std::string folder = modelsFolder + "/" + modelFolderName(name.value());
        if (force && fs::exists(folder)) {
            fs::remove_all(folder);
            printf("robot model: deleted %s\n", folder.c_str());
        }

        std::string path;
        std::string modelfile;
        if (!fs::exists(folder)) {
            printf("robot model: downloading into %s\n", folder.c_str());
            // download into a temporary folder, an interrupted download does not leave an incomplete model
            const std::string download = folder + ".download";
            fs::remove_all(download);
            std::tie(path, modelfile) = controller.requestRobotModel(download, 300);
            if (path.empty()) {
                fs::remove_all(download);
                printf("robot model: the robot has no model or the download failed\n");
                return;
            }
            fs::rename(download, folder);
        } else {
            // the index of a downloaded model needs the model information
            RobotModelInformation info;
            if (!controller.requestTelemetry(ROBOT_MODEL_INFORMATION, &info, 0) || !info.filedef().file_size()) {
                printf("robot model: %s exists, but the robot has no model information\n", folder.c_str());
                return;
            }
            path = info.filedef().file(0).path();
            modelfile = info.modelfilename();
            printf("robot model: using %s (--force-model-download downloads it again)\n", folder.c_str());
        }
        if (writeModelIndex(folder, path, modelfile)) {
            printf("robot model: served at /model/%s/\n", modelFolderName(name.value()).c_str());
        }
    } catch (const std::exception &error) {
        printf("robot model: %s\n", error.what());
    }
}

/**
 * @brief the web interface of robot_remote_control: installed, or in the source folder
 */
std::string defaultWebFolder() {
    for (const std::string folder : {RRC_WEB_INSTALL_DIR, RRC_WEB_SOURCE_DIR}) {
        if (std::ifstream(folder + "/index.html")) {
            return folder;
        }
    }
    return "";
}

/**
 * Connects to a robot via zmq and offers it to browsers: the web interface and the http api for commands,
 * and a websocket for telemetry (both json), so the web interface can be used with robots that only use zmq.
 */
int main(int argc, char** argv) {
    // keep the output readable when redirected to a file
    setvbuf(stdout, nullptr, _IOLBF, 0);

    std::string robot = "localhost";
    int robotCommandPort = 7001;
    int robotTelemetryPort = 7002;
    int httpPort = 8001;
    int websocketPort = 8002;
    std::string webFolder = defaultWebFolder();
    std::string modelsFolder = "models";
    bool forceModelDownload = false;
    float timeout = 2;

    po::options_description options("Options");
    options.add_options()
        ("help,h", "show this help")
        ("robot,r", po::value(&robot)->default_value(robot), "host of the robot")
        ("commandport,c", po::value(&robotCommandPort)->default_value(robotCommandPort), "zmq command port of the robot")
        ("telemetryport,t", po::value(&robotTelemetryPort)->default_value(robotTelemetryPort), "zmq telemetry port of the robot")
        ("httpport", po::value(&httpPort)->default_value(httpPort), "local port of the web interface and the http api (commands)")
        ("wsport", po::value(&websocketPort)->default_value(websocketPort), "local port of the telemetry websocket")
        ("web", po::value(&webFolder)->default_value(webFolder), "folder of the web interface, served at /html/")
        ("models", po::value(&modelsFolder)->default_value(modelsFolder),
            "folder for the downloaded robot models (<models>/<robot name>), served at /model/")
        ("force-model-download", po::bool_switch(&forceModelDownload),
            "delete the downloaded model of the robot and download it again")
        ("timeout", po::value(&timeout)->default_value(timeout), "seconds to wait for replies of the robot");
    po::positional_options_description positional;
    positional.add("robot", 1);

    po::variables_map arguments;
    try {
        po::store(po::command_line_parser(argc, argv).options(options).positional(positional).run(), arguments);
        po::notify(arguments);
    } catch (const po::error &error) {
        std::cerr << error.what() << std::endl << std::endl << options << std::endl;
        return 1;
    }
    if (arguments.count("help")) {
        std::cout << "usage: " << argv[0] << " [options] [robot]" << std::endl << std::endl << options << std::endl;
        return 0;
    }

    try {
        const std::string robotAddress = "tcp://" + robot + ":";
        TransportSharedPtr robotCommands = std::make_shared<TransportZmq>(robotAddress + std::to_string(robotCommandPort), TransportZmq::REQ);
        TransportSharedPtr robotTelemetry = std::make_shared<TransportZmq>(robotAddress + std::to_string(robotTelemetryPort), TransportZmq::SUB);

        auto http = std::make_shared<TransportHttp>("http://0.0.0.0:" + std::to_string(httpPort));
        if (webFolder.size()) {
            http->serveFolder(webFolder, "html");
            // the web interface connects to the telemetry port given as parameter
            http->setDefaultPage("/html/index.html?telemetryport=" + std::to_string(websocketPort));
            if (!std::ifstream(webFolder + "/lib/VERSIONS")) {
                printf("warning: the javascript libraries of the web interface are missing, run download_libraries.sh in %s\n", webFolder.c_str());
            }
        } else {
            printf("no web interface found (use --web <folder>), only the http api is available\n");
        }
        // the model appears in the served folder when it is downloaded
        fs::create_directories(modelsFolder);
        modelsFolder = fs::canonical(modelsFolder).string();
        http->serveFolder(modelsFolder, "model");
        auto websocket = std::make_shared<TransportWebSocket>(TransportWebSocket::SERVER_TEXT, websocketPort);

        GenericProxy commands(GenericProxy::COMMANDS, robotCommands, http);
        GenericProxy telemetry(GenericProxy::TELEMETRY, robotTelemetry, websocket);
        commands.setTimeout(timeout);
        commands.startUpdateThread(5);
        telemetry.startUpdateThread(5);

        printf("robot: %s%d (commands), %s%d (telemetry)\n", robotAddress.c_str(), robotCommandPort, robotAddress.c_str(), robotTelemetryPort);
        printf("http api: http://0.0.0.0:%d/api/, telemetry websocket: ws://0.0.0.0:%d\n", httpPort, websocketPort);
        if (webFolder.size()) {
            printf("web interface: http://localhost:%d/ (%s)\n", httpPort, webFolder.c_str());
        }

        // in the background, the web interface uses the model when it is downloaded
        std::thread(cacheRobotModel, robotAddress + std::to_string(robotCommandPort), modelsFolder, forceModelDownload).detach();

        while (true) {
            sleep(1);
        }
    } catch (const std::exception &error) {
        printf("%s\n", error.what());
        return 1;
    }
    return 0;
}
