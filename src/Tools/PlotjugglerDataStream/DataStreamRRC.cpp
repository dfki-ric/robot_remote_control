
#include "DataStreamRRC.hpp"

// #include <QTextStream>
// #include <QFile>
// #include <QMessageBox>
#include <QVBoxLayout>
// #include <QTreeWidget>
// #include <QCheckBox>
#include <QLineEdit>
// #include <QDebug>
#include <QDialog>
#include <QSettings>
#include <QSpinBox>
#include <QPushButton>

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

DataStreamRRC::DataStreamRRC() {
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


    QDialog *w = new QDialog();

    QVBoxLayout* vlayout = new QVBoxLayout();
    w->setLayout(vlayout);
    QLineEdit* ip = new QLineEdit();
    vlayout->addWidget(ip);

    QSpinBox* cmd = new QSpinBox();
    cmd->setMaximum(65535);
    vlayout->addWidget(cmd);
    
    QSpinBox* tele = new QSpinBox();
    tele->setMaximum(65535);
    vlayout->addWidget(tele);
    
    QPushButton* button = new QPushButton("connect");
    vlayout->addWidget(button);

    QSettings settings;
    QString address = settings.value("RRC::address", "localhost").toString();
    QString commandport = settings.value("RRC::commandport", "7001").toString();
    QString telemetryport = settings.value("RRC::telemetryport", "7002").toString();

    ip->setText(address);
    cmd->setValue(std::stoi(commandport.toStdString()));
    tele->setValue(std::stoi(telemetryport.toStdString()));
    

    connect(button, &QPushButton::pressed, [&]() {
         w->accept();
    });

    // w->show();
    w->exec();
    // w->hide();

    address = ip->text();
    commandport = cmd->text();
    telemetryport = tele->text();


    settings.setValue("RRC::address", address);
    settings.setValue("RRC::commandport", commandport);
    settings.setValue("RRC::telemetryport", telemetryport);


    TransportSharedPtr commands =  TransportSharedPtr(new TransportZmq(("tcp://"+address+":"+commandport).toStdString(), TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq(("tcp://"+address+":"+telemetryport).toStdString(), TransportZmq::SUB));

    // TransportSharedPtr commands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7001, "localhost"));
    // TransportSharedPtr telemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7002, "localhost"));
    
    // TransportSharedPtr commands = TransportSharedPtr(new TransportHttp("http://localhost:7001", TransportHttp::CLIENT));

    robotdata = std::make_shared<robot_remote_control::RobotController>(commands, telemetry);
    robotdata->startUpdateThread(0);

    robotdata->setHeartBeatDuration(1);
    robotdata->waitForConnection();
    robotdata->requestChannelsDefinition(&channeldef);


    // robotdata->getStatistics().setRunningAverageSamples(100);

    // channeldef.PrintDebugString();

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

    // todo: satistics
    auto now = std::chrono::high_resolution_clock::now();
    double stamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();

    // printf("%s:%i %f\n", __PRETTY_FUNCTION__, __LINE__, stamp);
    

    for (size_t dataindex = 0; dataindex < robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER; ++dataindex) {
        switch (dataindex) {
            case robot_remote_control::CURRENT_POSE :           newData = handleChannels<robot_remote_control::Pose>(robot_remote_control::CURRENT_POSE, stamp);
            case robot_remote_control::JOINT_STATE :            newData = handleChannels<robot_remote_control::JointState>(robot_remote_control::JOINT_STATE, stamp);
            case robot_remote_control::ROBOT_STATE :            newData = handleChannels<robot_remote_control::RobotState>(robot_remote_control::ROBOT_STATE, stamp);
            // case robot_remote_control::LOG_MESSAGE :            newData = handleChannels<robot_remote_control::LogMessage>(robot_remote_control::LOG_MESSAGE, stamp);
            case robot_remote_control::SIMPLE_SENSOR :          newData = handleChannels<robot_remote_control::SimpleSensor>(robot_remote_control::SIMPLE_SENSOR, stamp);
            case robot_remote_control::WRENCH_STATE :           newData = handleChannels<robot_remote_control::WrenchState>(robot_remote_control::WRENCH_STATE, stamp);
            case robot_remote_control::POSES :                  newData = handleChannels<robot_remote_control::Poses>(robot_remote_control::POSES, stamp);
            case robot_remote_control::IMU_VALUES :             newData = handleChannels<robot_remote_control::IMU>(robot_remote_control::IMU_VALUES, stamp);
            case robot_remote_control::CURRENT_TWIST :          newData = handleChannels<robot_remote_control::Twist>(robot_remote_control::CURRENT_TWIST, stamp);
            case robot_remote_control::CURRENT_ACCELERATION :   newData = handleChannels<robot_remote_control::Acceleration>(robot_remote_control::CURRENT_ACCELERATION, stamp);
            // case robot_remote_control::ODOMETRY :               newData = handleChannels<robot_remote_control::Odometry>(robot_remote_control::ODOMETRY, stamp);
            // case robot_remote_control::LASER_SCAN :  newData = handleChannels<robot_remote_control::JointState>(robot_remote_control::LASER_SCAN, stamp);
        }
    }
    //TODO displawrenchstate

    robot_remote_control::Statistics& stats = robotdata->getStatistics();
    stats.calculate();
    for (size_t i = 0; i < robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER; ++i) {
        dataMap().getOrCreateNumeric("ConnectionStatistics/"+ stats.names[i] + "/kBps").pushBack(PJ::PlotData::Point(stamp, stats.stat_per_type[i].getStats().bpsAvg/1024.0));
        dataMap().getOrCreateNumeric("ConnectionStatistics/"+ stats.names[i] + "/total kB").pushBack(PJ::PlotData::Point(stamp, stats.stat_per_type[i].getStats().bytesTotal/1024.0));
        dataMap().getOrCreateNumeric("ConnectionStatistics/"+ stats.names[i] + "/freq").pushBack(PJ::PlotData::Point(stamp, stats.stat_per_type[i].getStats().frequencyAvg));
    }


    // printf("%s:%i %i\n", __PRETTY_FUNCTION__, __LINE__, newData);

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
    if (newData.needsUpdate()) {
        emit dataReceived();
        newData.reset();
    }
    
    // std::this_thread::sleep_until(prev + std::chrono::milliseconds(20));  // 50 Hz
    std::this_thread::sleep_until(prev + std::chrono::milliseconds(100));  // 
  }
}

void DataStreamRRC::addToPlot(const robot_remote_control::TimeStamp& ts, const double stamp, const std::string prefix) {    
    dataMap().getOrCreateNumeric(prefix + "timestamp").pushBack(PJ::PlotData::Point(stamp, ts.secs() + ts.nsecs() / 1000000000.0 ));
}

void DataStreamRRC::addToPlot(const robot_remote_control::Header& header, const double stamp, const std::string prefix) {
    std::string entryname = prefix + "/";
    addToPlot(header.timestamp(),stamp,entryname);

    dataMap().getOrCreateStringSeries(entryname + "frame").pushBack(PJ::StringSeries::Point(stamp, header.frame()));
    dataMap().getOrCreateNumeric(entryname + "seq").pushBack(PJ::PlotData::Point(stamp, header.seq()));
}

void DataStreamRRC::addToPlot(const robot_remote_control::JointState& proto, const double stamp, const std::string prefix) {
    std::string headername = prefix + "/header";
    addToPlot(proto.header(), stamp, headername);

    for (int i = 0; i < proto.name().size(); ++i) {
        std::string entryname = prefix + "/" + proto.name(i);;// + std::to_string(i);
        std::string fieldname;
        // fieldname = entryname +
        //dataMap().getOrCreateStringSeries(fieldname).pushBack(PJ::StringSeries::Point(stamp, ));


        if (i < proto.position().size()) {
            fieldname = entryname + "/position";
            dataMap().getOrCreateNumeric(fieldname).pushBack(PJ::PlotData::Point(stamp, proto.position(i)));
        }
        if (i < proto.velocity().size()) {
            fieldname = entryname + "/velocity";
            dataMap().getOrCreateNumeric(fieldname).pushBack(PJ::PlotData::Point(stamp, proto.velocity(i)));
        }
        if (i < proto.effort().size()) {
            fieldname = entryname + "/effort";
            dataMap().getOrCreateNumeric(fieldname).pushBack(PJ::PlotData::Point(stamp, proto.effort(i)));
        }
        if (i < proto.acceleration().size()) {
            fieldname = entryname + "/acceleration";
            dataMap().getOrCreateNumeric(fieldname).pushBack(PJ::PlotData::Point(stamp, proto.acceleration(i)));
        }
        if (i < proto.tics().size()) {
            fieldname = entryname + "/tics";
            dataMap().getOrCreateNumeric(fieldname).pushBack(PJ::PlotData::Point(stamp, proto.tics(i)));
        }
    }
}

