
#include "DataStreamRRC.hpp"

#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QCheckBox>
#include <QLineEdit>
#include <QDebug>

#include <thread>
#include <mutex>
#include <chrono>
#include <list>
#include <vector>
#include <math.h>

#include <google/protobuf/message.h>

#include "../../Transports/ZeroMQ/TransportZmq.hpp"

using namespace PJ;
using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

DataStreamRRC::DataStreamRRC(): newData(false) {
    // ns = std::make_unique<orocos_cpp::CorbaNameService>();

    // config.load_all_packages = true;
    // config.load_typekits = true;  // this fails because of qt4 linked into some typekits
    // orocos = std::make_shared<orocos_cpp::OrocosCpp>();
    // orocos->initialize(config, false);
    // OrocosHelpers::initClientTask("plotjuggler");
    // localtask = OrocosHelpers::getClientTask();


    google::protobuf::LinkMessageReflection<robot_remote_control::JointState>();
}



bool DataStreamRRC::start(QStringList*) {
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
     _running = true;

    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));

    // TransportSharedPtr commands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7001, "localhost"));
    // TransportSharedPtr telemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7002, "localhost"));
    
    // TransportSharedPtr commands = TransportSharedPtr(new TransportHttp("http://localhost:7001", TransportHttp::CLIENT));

    robotdata = std::make_shared<robot_remote_control::RobotController>(commands, telemetry);
    robotdata->startUpdateThread(0);

    robotdata->setHeartBeatDuration(1);

    while (!robotdata->isConnected()){
        printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        sleep(1);
    }
    robotdata->requestChannelsDefinition(&channeldef);

    channeldef.PrintDebugString();

    pushSingleCycle();
    _thread = std::thread([this]() { this->loop(); });
    return true;
}

void DataStreamRRC::selectionChanged(QTreeWidgetItem * item, int column) {
    // // if (column == 2) {
    //     // check if a connection was changed
    //     auto entry = conenctions.find(item);
    //     if (entry != conenctions.end()) {
    //         ConnectionData& conndata = entry->second;
    //         if (item->checkState(2) == Qt::Checked) {
    //             connectRockPort(conndata);
    //         } else {
    //             printf("disconnect: %s\n", conndata.identifier.c_str());
    //             conndata.input_port->disconnect(conndata.output_port);
    //             // remove from evaluated ports
    //             input_ports[conndata.identifier] = nullptr;
    //         }
    //     }
    // // } else if (column == 1) {
    //     // check if a use_data_timestamp was changes
    //     auto use_ts = options_use_data_timestamp.find(item);
    //     if (use_ts != options_use_data_timestamp.end()) {
    //         if (item->checkState(1) == Qt::Checked) {
    //             use_ts->second->use_data_timestamp = true;
    //         } else {
    //             use_ts->second->use_data_timestamp = false;
    //         }
    //     }
    //     // check if a use_data_timestamp was changes
    //     auto ts_field_name = options_timestamp_field_name.find(item);
    //     if (ts_field_name != options_timestamp_field_name.end()) {
    //         ts_field_name->second->timestamp_field_name = item->text(1).toStdString();
    //     }
    // // }
}

// bool DataStreamRRC::connectRockPort(const ConnectionData& conndata) {
//     printf("connect: %s\n", conndata.identifier.c_str());
//     // add to the evaluated ports
//     input_ports[conndata.identifier] = conndata.input_port;
//     return conndata.output_port->connectTo(conndata.input_port);
// }

void DataStreamRRC::shutdown() {
    // treewidget->close();
    _running = false;
    if (_thread.joinable()) {
        _thread.join();
    }
    // localtask->stop();
}

bool DataStreamRRC::isRunning() const {
    return _running;
}

DataStreamRRC::~DataStreamRRC() {
    shutdown();
}

bool DataStreamRRC::xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const {
    return true;
}

bool DataStreamRRC::xmlLoadState(const QDomElement& parent_element) {
    return true;
}

void DataStreamRRC::pushSingleCycle() {
    std::lock_guard<std::mutex> lock(mutex());
    newData = false;

    // todo: satistics
    auto now = std::chrono::high_resolution_clock::now();
    double stamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
    

    for (size_t dataindex = 0; dataindex < robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER; ++dataindex) {
        switch (dataindex) {
            case robot_remote_control::CURRENT_POSE : handleChannels<robot_remote_control::Pose>(robot_remote_control::CURRENT_POSE, stamp);
            case robot_remote_control::JOINT_STATE : handleChannels<robot_remote_control::JointState>(robot_remote_control::JOINT_STATE, stamp);
        }
    }

    newData = true;

    printf("%s:%i %i\n", __PRETTY_FUNCTION__, __LINE__, newData);

    // double c = rand()/(double)RAND_MAX;
    // static int count = 0;
    // count++;
    //  printf("%s:%i %i %.2f\n", __PRETTY_FUNCTION__, __LINE__, count, c);
    // auto& plot = dataMap().getOrCreateNumeric("test");    
    // plot.pushBack(PJ::PlotData::Point(count, c));


}

void DataStreamRRC::loop() {
  _running = true;
  while (_running) {
    auto prev = std::chrono::high_resolution_clock::now();
    pushSingleCycle();
    if (newData) {
        emit dataReceived();
    }
    // std::this_thread::sleep_until(prev + std::chrono::milliseconds(20));  // 50 Hz
    std::this_thread::sleep_until(prev + std::chrono::milliseconds(100));  // 
  }
}
